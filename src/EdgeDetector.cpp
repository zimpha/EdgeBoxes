#include "EdgeDetector.h"
#include "convUtil.h"
#include "gradientUtil.h"
#include "imageUtil.h"
#include "wrappers.h"

#define PI 3.14159265f

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

EdgeDetector::EdgeDetector() {
}

EdgeDetector::~EdgeDetector() {
  clear();
}

void EdgeDetector::featureExtract(cv::Mat &I, cv::Mat &chnsReg, cv::Mat &chnsSim, DetectParam &opts) {
  if (I.channels() != 3) {
    wrError("input image must have 3 channels");
  }
  int shrink = opts.shrink;
  cv::Mat luv, Ishrink;
  rgbConvert(I, luv);
  imResample(luv, Ishrink, cv::Size(0, 0), 1.0 / shrink, 1.0 / shrink);
  cv::Mat chns[opts.nChns];
  int k = 0;
  chns[k++] = Ishrink;
  for (int i = 1, s = 1; i <= 2; ++i, s <<= 1) {
    cv::Mat I1, I2;
    if (s == shrink) I1 = Ishrink;
    else imResample(luv, I1, cv::Size(0, 0), 1.0 / s, 1.0 / s);
    convTri(I1, I2, opts.grdSmooth);
    cv::Mat M, O, H;
    gradientMag(I2, M, O, 0, opts.normRad, .01);
    gradientHist(M, O, H, std::max(1, shrink / s), opts.nOrients, 0);
    imResample(M, chns[k++], cv::Size(0, 0), (double)s / shrink, (double)s / shrink);
    imResample(H, chns[k++], cv::Size(0, 0), std::max(1.0, (double)s / shrink), std::max(1.0, (double)s / shrink));
  }
  cv::Mat tmp;
  cv::merge(chns, opts.nChns, tmp);
  assert(tmp.channels() == opts.nChns);
  float chnSm = opts.chnSmooth / shrink;
  float simSm = opts.simSmooth / shrink;
  convTri(tmp, chnsReg, chnSm);
  convTri(tmp, chnsSim, simSm);
}

void EdgeDetector::edgesDetect(cv::Mat &I, cv::Mat &E, cv::Mat &O, DetectParam &opts) {
  // get parameters
  opts.stride = std::max(opts.stride, opts.shrink);
  opts.nTreesEval = std::min(opts.nTreesEval, opts.nTrees);

  // pad image, making divisible by 4
  int r = opts.imWidth / 2;
  std::vector<int> p = {r, r, r, r};
  p[1] += (4 - (I.rows + r * 2) % 4) % 4;
  p[3] += (4 - (I.cols + r * 2) % 4) % 4;
  I = imPad(I, p, "symmetric");

  // get feature
  cv::Mat chnsReg, chnsSim;
  featureExtract(I, chnsReg, chnsSim, opts);
  if (opts.sharpen) {
    I = convTri(rgbConvert(I, "rgb"), 1);
  }

  // detect edges
  const int shrink = opts.shrink;
  const int imWidth = opts.imWidth;
  const int gtWidth = opts.gtWidth;
  const int nChns = opts.nChns;
  const int nCells = opts.nCells;
  const uint32_t nChnFtrs = opts.nChnFtrs;
  const int stride = opts.stride;
  const int nTreesEval = opts.nTreesEval;
  const int sharpen = opts.sharpen;
  const int nThreads = opts.nThreads;

  const int h = I.rows, w = I.cols, Z = I.channels();
  const int h1 = (int)ceil(double(h - imWidth) / stride);
  const int w1 = (int)ceil(double(w - imWidth) / stride);
  const int h2 = h1 * stride + gtWidth;
  const int w2 = w1 * stride + gtWidth;
  const int imgDims[3] = {h, w, Z};
  const int chnDims[3] = {h / shrink, w / shrink, nChns};
  const int indDims[3] = {h1, w1, nTreesEval};
  const int outDims[3] = {h2, w2, 1};
  const int segDims[5] = {gtWidth, gtWidth, h1, w1, nTreesEval};

  // construct lookup tables
  uint32 *iids, *eids, *cids, *cids1, *cids2;
  iids = buildLookup((int*)imgDims, gtWidth);
  eids = buildLookup((int*)outDims, gtWidth);
  cids = buildLookup((int*)chnDims, imWidth / shrink );
  buildLookupSs(cids1, cids2, (int*)chnDims, imWidth / shrink, nCells);

  // free memory
  delete [] iids; delete [] eids;
  delete [] cids; delete [] cids1; delete [] cids2;

  // normalize and finalize edge maps
  float t = 1.0f * opts.stride * opts.stride / (opts.gtWidth * opts.gtWidth) / opts.nTreesEval;
  r = opts.gtWidth / 2;

  if (opts.sharpen == 0) t *= 2;
  else if (opts.sharpen == 1) t *= 1.8;
  else t *= 1.66;
  E *= t;
  E = convTri(E, 1);

  // compute approximate orientation O from edges E
  E = convTri(E, 4);
  cv::Mat Ox, Oy, Oxx, Oxy, Oyy;
  gradient(E, Ox, Oy);
  gradient(Ox, Oxx, Oxy);
  gradient(Oy, Oxy, Oyy);
  wrCreateCVMat(E.size(), E.type(), O);
  for (int y = 0; y < O.rows; ++y) {
    for (int x = 0; x < O.cols; ++x) {
      float val = Oyy.at<float>(y, x) * sgn(-Oxy.at<float>(y, x)) / (Oxx.at<float>(y, x) + 1e-5);
      val = fmod(atan(val), PI);
      if (val < -1e-8) val += PI;
      O.at<float>(y, x) = val;
    }
  }
  // perform nms
  if (opts.nms > 0) {

  }
}

uint32_t EdgeDetector::buildLookup(int *dims, int w) {
  int c, r, z, n=w*w*dims[2];
  uint32 *cids=new uint32[n]; n=0;
  for(z=0; z<dims[2]; z++) for(c=0; c<w; c++) for(r=0; r<w; r++)
    cids[n++] = z*dims[0]*dims[1] + c*dims[0] + r;
  return cids;
}

void EdgeDetector::buildLookupSs(uint32_t *&cids1, uint32_t *&cids2, int *dims, int w, int m) {
  int i, j, z, z1, c, r; int locs[1024];
  int m2=m*m, n=m2*(m2-1)/2*dims[2], s=int(w/m/2.0+.5);
  cids1 = new uint32[n]; cids2 = new uint32[n]; n=0;
  for(i=0; i<m; i++) locs[i]=uint32((i+1)*(w+2*s-1)/(m+1.0)-s+.5);
  for(z=0; z<dims[2]; z++) for(i=0; i<m2; i++) for(j=i+1; j<m2; j++) {
    z1=z*dims[0]*dims[1]; n++;
    r=i%m; c=(i-r)/m; cids1[n-1]= z1 + locs[c]*dims[0] + locs[r];
    r=j%m; c=(j-r)/m; cids2[n-1]= z1 + locs[c]*dims[0] + locs[r];
  }
}

void EdgeDetector::clear() {
  wrFree(thrs); thrs = NULL;
  wrFree(fids); fids = NULL;
  wrFree(child); child = NULL;
  wrFree(segs); segs = NULL;
  wrFree(nSegs); nSegs = NULL;
  wrFree(eBins); eBins = NULL;
  wrFree(eBnds); eBnds = NULL;
}
