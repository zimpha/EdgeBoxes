#include "chnsPyramid.h"
#include "wrappers.h"
#include "rgbConvert.h"
#include "imResample.h"
#include "convUtil.h"
#include "imPad.h"

#include <cmath>
#include <algorithm>

int getScales(int nPerOct, int nOctUp, float* minDs, int shrink, int h, int w, float* &scales, float* &scaleshw) {
  if (h == 0 || w == 0) {
    scales = NULL, scaleshw = NULL;
    return 0;
  }
  int nScales = floor(nPerOct * (nOctUp + log2(std::min(h / minDs[0], w / minDs[1]))) + 1);
  float scalesTemp[nScales];
  for (int i = 0; i < nScales; ++i) {
    scalesTemp[i] = pow(2.f, -1.f * i / nPerOct + nOctUp);
  }
  int d0 = std::min(h, w), d1 = std::max(h, w);
  static float ss[101], es0[101], es1[101];
  for (int i = 0; i < nScales; ++i) {
    float s = scalesTemp[i];
    float s0 = (round(d0 * s / shrink) * shrink - .25f * shrink) / d0;
    float s1 = (round(d0 * s / shrink) * shrink + .25f * shrink) / d0;
    float minMaxV; int idx;
    for (int j = 0; j < 101; ++j) {
      float ssj = .01f * j * (s1 - s0) + s0;
      float es0j = d0 * ssj, es1j = d1 * ssj;
      ss[j] = ssj;
      es0[j] = fabs(es0j - round(es0j / shrink) * shrink);
      es1[j] = fabs(es1j - round(es1j / shrink) * shrink);
      float maxV = std::max(es0[j], es1[j]);
      if (j == 0 || minMaxV > maxV) {
        minMaxV = maxV;
        idx = j;
      }
    }
    scalesTemp[i] = ss[idx];
  }
  int count = 0, keep[nScales];
  for (int i = 0; i < nScales - 1; ++i) {
    if (scalesTemp[i] != scalesTemp[i + 1]) {
      ++count; keep[i] = true;
    } else keep[i] = false;
  }
  ++count; keep[nScales - 1] = true;
  scales = (float*)wrCalloc(count, sizeof(float));
  scaleshw = (float*)wrCalloc(count * 2, sizeof(float));
  for (int i = 0, j = 0; i < nScales; ++i) if (keep[i]) {
    scales[j] = scalesTemp[i];
    scaleshw[j * 2 + 0] = round(h * scales[j] / shrink) * shrink / h;
    scaleshw[j * 2 + 1] = round(w * scales[j] / shrink) * shrink / w;
    ++j;
  }
  return count;
}

void chnsPyramid(uint8_t *image, int h, int w, int d, PyramidInput &pyramidInput, PyramidOutput &pyramidOutput) {
  if (d != 3) {
    wrError("input image must have 3 channels");
    // TODO: conver to color image
  }
  if (!pyramidInput.complete) {
    pyramidInput = PyramidInput();
  }
  ChnsInput &chnsInput = pyramidInput.chnsInput;
  int shrink = chnsInput.shrink;
  int nPerOct = pyramidInput.nPerOct;
  int nOctUp = pyramidInput.nOctUp;
  int nApprox = pyramidInput.nApprox;
  std::vector<float> &lambdas = pyramidInput.lambdas;
  int *pad = pyramidInput.pad;
  float* minDs = pyramidInput.minDs;
  float smooth = pyramidInput.smooth;

  std::vector<ChnsOutput::Info> &infos = pyramidOutput.infos;
  std::vector<CellArray> &data = pyramidOutput.data;

  // convert image to appropriate color space (or simply normalize)
  CellArray I, I1, tmp;
  I.rows = h, I.cols = w, I.channels = d;
  I.data = rgbConvert(image, h, w, d, chnsInput.pColor.colorSpace);
  chnsInput.pColor.colorSpace = CS_ORIG;

  // get scales at which to compute features and list of real/approx scales
  float *scales, *scaleshw;
  int nScales = getScales(nPerOct, nOctUp, minDs, shrink, h, w, scales, scaleshw);
  std::vector<int> isR, isN, isA;
  for (int i = 1; i <= nScales; i += nApprox + 1) isR.push_back(i);
  for (size_t i = 1; i < isR.size(); ++i) {
    for (int j = isR[i - 1] + 1; j < isR[i]; ++j) isA.push_back(j);
  }
  for (int i = isR.back() + 1; i <= nScales; ++i) isA.push_back(i);
  for (int i = 1; i <= nScales; ++i) isN.push_back(i);
  std::vector<int> idx; idx.push_back(0);
  for (int i = 1; i < isR.size(); ++i) {
    idx.push_back((isR[i] + isR[i - 1]) / 2);
  }
  idx.push_back(nScales);
  for (size_t i = 0; i < isR.size(); ++i) {
    for (int j = idx[i]; j < idx[i + 1]; ++j) isN[j] = isR[i];
  }

  int nTypes = 3;
  std::vector<ChnsOutput> chns(nScales);
  // compute image pyramid [real scales]
  for (size_t j = 0; j < isR.size(); ++j) {
    int i = isR[j] - 1;
    float s = scales[i];
    int h1 = round(h * s / shrink) * shrink;
    int w1 = round(w * s / shrink) * shrink;
    if (h == h1 && w1 == w) I1 = I;
    else imResample(I, I1, cv::Size(w1, h1));
    if (s == .5f && (nApprox > 0 || nPerOct == 1)) I = I1;
    chnsCompute(I1, chnsInput, chns[i]);
    if (j == 0) infos = chns[i].infos, nTypes = chns[i].nTypes;
  }

  // if lambdas not specified compute image specific lambdas
  if (nScales > 0 && nApprox > 0 && lambdas.empty()) {
    // TODO
  }

  // compute image pyramid [approximated scales]
  for (size_t j = 0; j < isA.size(); ++j) {
    int i = isA[j] - 1, iR = isN[i] - 1;
    int h1 = round(h * scales[i] / shrink);
    int w1 = round(w * scales[i] / shrink);
    chns[i].data.resize(nTypes);
    for (int k = 0; k < nTypes; ++k) {
      float ratio = pow(scales[i] / scales[iR], -lambdas[k]);
      imResample(chns[iR].data[k], chns[i].data[k], cv::Size(w1, h1), 0, 0, "bilinear", ratio);
    }
  }

  // smooth channels, optionally pad and concatenate channels
  for (int i = 0; i < nScales; ++i) {
    for (int j = 0; j < nTypes; ++ j) {
      convTri(chns[i].data[j], tmp, smooth);
      chns[i].data[j].swap(tmp);
    }
  }
  if (pad[0] || pad[1]) {
    for (int i = 0; i < nScales; ++i) {
      for (int j = 0; j < nTypes; ++ j) {
        std::string padtype;
        if (infos[j].padwith == 0) padtype = "";
        else padtype = "replicate";
        std::vector<int> p(pad, pad + 2);
        p[0] /= shrink; p[1] /= shrink;
        imPad(chns[i].data[j], tmp, p, padtype);
        chns[i].data[j].swap(tmp);
      }
    }
  }
  data.resize(nScales);
  for (int i = 0; i < nScales; ++i) {
    mergeCellArray(chns[i].data, nTypes, data[i]);
  }
  pyramidOutput.lambdas = lambdas;
  pyramidOutput.scales = scales;
  pyramidOutput.scaleshw = scaleshw;
  pyramidOutput.nTypes = nTypes;
  pyramidOutput.nScales = nScales;
}
