#include "ACFDetector.h"
#include "imResample.h"

#include <opencv2/opencv.hpp>

enum ConvolutionType {
  CONVOLUTION_FULL,
  CONVOLUTION_SAME,
  CONVOLUTION_VALID
};

void conv2(const cv::Mat &A, const cv::Mat &B, ConvolutionType type, cv::Mat &dst) {
  cv::Mat src = A, kernel;
  cv::flip(B, kernel, -1);
  if (type == CONVOLUTION_FULL) {
    src = cv::Mat();
    const int ar = kernel.rows - 1, ac = kernel.cols - 1;
    cv::copyMakeBorder(A, src, (ar + 1) / 2, ar / 2, (ac + 1) / 2, ac / 2, cv::BORDER_CONSTANT, cv::Scalar(0));
  }
  cv::Point anchor(kernel.cols - kernel.cols / 2 - 1, kernel.rows - kernel.rows / 2 - 1);
  int borderMode = cv::BORDER_CONSTANT;
  cv::filter2D(src, dst, src.depth(), kernel, anchor, 0, borderMode);
  if (type == CONVOLUTION_VALID) {
    dst = dst.colRange((kernel.cols - 1) / 2, dst.cols - kernel.cols / 2)
             .rowRange((kernel.rows - 1) / 2, dst.rows - kernel.rows / 2);
  }
}

ACFDetector::ACFDetector():clf(), pPyramid(), filters() {

}

void ACFDetector::loadModel(const std::string &filepath) {
  FILE *fp = fopen(filepath.c_str(), "rb");
  if (fp == NULL) {
    wrError("cannot open model");
  }
  // get pPyramid
  // TODO: WARNING!! hard code for chnsInput
  pPyramid.chnsInput.shrink = 2;
  pPyramid.chnsInput.pColor = (ColorParam){true, 0.f, CS_LUV};
  pPyramid.chnsInput.pGradMag = (GradMagParam){true, 0, 5.f, .005f, false};
  pPyramid.chnsInput.pGradHist = (GradHistParam){true, pPyramid.chnsInput.shrink, 6, true, false, .2f};
  pPyramid.chnsInput.complete = true;

  fread(&pPyramid.nPerOct, sizeof(int), 1, fp);
  fread(&pPyramid.nOctUp, sizeof(int), 1, fp);
  fread(&pPyramid.nApprox, sizeof(int), 1, fp);
  uint32_t nTypes; fread(&nTypes, sizeof(uint32_t), 1, fp);
  pPyramid.lambdas.resize(nTypes);
  for (size_t i = 0; i < nTypes; ++i) {
    fread(&pPyramid.lambdas[i], sizeof(float), 1, fp);
  }
  fread(pPyramid.pad, sizeof(int), 2, fp);
  fread(pPyramid.minDs, sizeof(float), 2, fp);
  fread(&pPyramid.smooth, sizeof(float), 1, fp);
  fread(&pPyramid.complete, sizeof(bool), 1, fp);
  if (pPyramid.nApprox < 0) pPyramid.nApprox = pPyramid.nPerOct - 1;

  // get filters
  uint32_t rows, cols, channels;
  fread(&rows, sizeof(uint32_t), 1, fp);
  fread(&cols, sizeof(uint32_t), 1, fp);
  fread(&channels, sizeof(uint32_t), 1, fp);
  filters.create(rows, cols, channels);
  fread(filters.data, sizeof(float), rows * cols * channels, fp);

  // get modelDs and modelDsPad
  fread(modelDs, sizeof(float), 2, fp);
  fread(modelDsPad, sizeof(float), 2, fp);

  // get pNms, TODO: WARNING!! hard code
  pNms.type = "maxg";
  pNms.overlap = .65f;
  pNms.ovrDnm = "min";

  // get other parameters
  fread(&stride, sizeof(uint32_t), 1, fp);
  fread(&cascThr, sizeof(float), 1, fp);
  fread(&cascCal, sizeof(float), 1, fp);

  // get Forest
  clf.clear();
  fread(&clf.nTreeNodes, sizeof(uint32_t), 1, fp);
  fread(&clf.nTrees, sizeof(uint32_t), 1, fp);
  fread(&clf.treeDepth, sizeof(uint32_t), 1, fp);
  int total = clf.nTrees * clf.nTreeNodes;
  clf.fids = new uint32_t[total];
  clf.thrs = new float[total];
  clf.child = new uint32_t[total];
  clf.hs = new float[total];
  //clf.weights = new float[total];
  //clf.depth = new uint32_t[total];
  fread(clf.fids, sizeof(uint32_t), total, fp);
  fread(clf.thrs, sizeof(float), total, fp);
  fread(clf.child, sizeof(uint32_t), total, fp);
  fread(clf.hs, sizeof(float), total, fp);
  //fread(clf.weights, sizeof(float), total, fp);
  //fread(clf.depth, sizeof(uint32_t), total, fp);
}

Boxes ACFDetector::acfDetect(uint8_t *I, int h, int w, int d) {
  int shrink = pPyramid.chnsInput.shrink;
  int *pad = pPyramid.pad;
  // compute features (including optionally applying filters)
  PyramidOutput P;
  chnsPyramid(I, h, w, d, pPyramid, P);
  if (filters.total()) {
    // TODO; apply filters
    shrink *= 2;
    for (int i = 0; i < P.nScales; ++i) {
      CellArray C(P.data[i].rows, P.data[i].cols, filters.channels);
      for (int j = 0; j < filters.channels; ++j) {
        cv::Mat src(P.data[i].cols, P.data[i].rows, CV_32F, P.data[i].chn(j % 10));
        cv::Mat kernel(filters.cols, filters.rows, CV_32F, filters.chn(j));
        cv::Mat dst;
        conv2(src.t(), kernel.t(), CONVOLUTION_SAME, dst);
        float *A = C.chn(j), *B = (float*)dst.data;
        for (int c = 0; c < C.cols; ++c) {
          for (int r = 0; r < C.rows; ++r) {
            A[c * C.rows + r] = B[r * C.cols + c];
          }
        }
      }
      imResample(C, P.data[i], cv::Size(0, 0), 0.5f, 0.5f);
    }
  }
  // apply sliding window classifiers
  Boxes res;
  for (int i = 0; i < P.nScales; ++i) {
    Boxes bb = detect(P.data[i], shrink, modelDsPad[0], modelDsPad[1], stride, cascThr);
    float sh = (modelDsPad[0] - modelDs[0]) / 2 - pad[0];
    float sw = (modelDsPad[1] - modelDs[1]) / 2 - pad[1];
    for (size_t j = 0; j < bb.size(); ++j) {
      bb[j].c = (bb[j].c + sw) / P.scaleshw[i * 2 + 1];
      bb[j].r = (bb[j].r + sh) / P.scaleshw[i * 2 + 0];
      bb[j].w = modelDs[1] / P.scales[i];
      bb[j].h = modelDs[0] / P.scales[i];
      res.push_back(bb[j]);
    }
  }
  return bbNms(res, pNms);
}

Boxes ACFDetector::detect(CellArray &chns_data, int shrink, int modelHt, int modelWd, int stride, float cascThr) {
  // extract relevant fields from trees
  float* chns = chns_data.data;
  float *thrs = clf.thrs;
  float *hs = clf.hs;
  uint32_t *fids = clf.fids;
  uint32_t *child = clf.child;
  int treeDepth = clf.treeDepth;

  // get dimensions and constants
  const int height = chns_data.rows;
  const int width = chns_data.cols;
  const int nChns = chns_data.channels;
  const int nTreeNodes = clf.nTreeNodes;
  const int nTrees = clf.nTrees;
  const int height1 = (int)ceil(float(height * shrink - modelHt + 1) / stride);
  const int width1 = (int)ceil(float(width * shrink - modelWd + 1) / stride);

  // construct cids array
  int nFtrs = modelHt / shrink * modelWd / shrink * nChns;
  uint32_t *cids = new uint32_t[nFtrs]; int m=0;
  for (int z = 0; z < nChns; ++z)
    for (int c = 0; c < modelWd / shrink; ++c)
      for (int r = 0; r < modelHt / shrink; ++r)
        cids[m++] = z * width * height + c * height + r;

  // apply classifier to each patch
  Boxes res;
  for (int c = 0; c < width1; ++c) for (int r = 0; r < height1; ++r) {
    float h = 0, *chns1 = chns + (r * stride / shrink) + (c * stride / shrink) * height;
    if (treeDepth == 1) { // specialized case for treeDepth==1
      for (int t = 0; t < nTrees; ++t) {
        uint32_t offset = t * nTreeNodes, k = offset, k0 = 0;
        getChild(chns1, cids, fids, thrs, offset, k0, k);
        h += hs[k]; if (h <= cascThr) break;
      }
    } else if (treeDepth == 2) { // specialized case for treeDepth==2
      for (int t = 0; t < nTrees; ++t) {
        uint32_t offset = t * nTreeNodes, k = offset, k0 = 0;
        getChild(chns1, cids, fids, thrs, offset, k0, k);
        getChild(chns1, cids, fids, thrs, offset, k0, k);
        h += hs[k]; if (h <= cascThr) break;
      }
    } else if (treeDepth > 2) { // specialized case for treeDepth>2
      for (int t = 0; t < nTrees; ++t) {
        uint32_t offset = t * nTreeNodes, k = offset, k0 = 0;
        for (int i = 0; i < treeDepth; ++i)
          getChild(chns1, cids, fids, thrs, offset, k0, k);
        h += hs[k]; if (h <= cascThr) break;
      }
    } else { // general case (variable tree depth)
      for (int t = 0; t < nTrees; ++t) {
        uint32_t offset = t * nTreeNodes, k = offset, k0 = k;
        while (child[k]) {
          float ftr = chns1[cids[fids[k]]];
          k = (ftr < thrs[k]) ? 1 : 0;
          k0 = k = child[k0] - k + offset;
        }
        h += hs[k]; if (h <= cascThr) break;
      }
    }
    if (h > cascThr) res.push_back((Box){c * stride, r * stride, modelWd, modelHt, h});
  }
  delete [] cids;
  return res;
}
