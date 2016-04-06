#include "gradientUtil.h"
#include "convUtil.h"
#include "gradientMex.h"
#include <cmath>
#include <cstring>

void gradient(CellArray& I, CellArray& Gx, CellArray& Gy) {
  int h = I.rows, w = I.cols, d = I.channels;
  if (h < 2 || w < 2) wrError("I must be at least 2x2.");
  Gx.create(h, w, d); Gy.create(h, w, d);
  grad2(I.data, Gx.data, Gy.data, h, w, d);
}

void gradientMag(CellArray& I, CellArray& M, CellArray& O, int channel, float normRad, float normConst, bool full) {
  int h = I.rows, w = I.cols, d = I.channels;
  if (h < 2 || w < 2) wrError("I must be at least 2x2.");
  M.create(h, w, 1); O.create(h, w, 1);

  float* input = I.data;
  if (channel > 0 && channel <= d) {
    input += h * w * (channel - 1);
    d = 1;
  }
  gradMag(input, M.data, O.data, h, w, d, full);

  if (normRad == 0) return;
  // normalization
  CellArray S;
  convTri(M, S, normRad);
  gradMagNorm(M.data, S.data, h, w, normConst);
}

void gradientHist(CellArray& M, CellArray &O, CellArray& H, int binSize, int nOrients, int softBin, int useHog, float clipHog, bool full) {
  int h = M.rows, w = M.cols, d = M.channels;
  if (O.rows != h || O.cols != w || O.channels != 1 || d != 1) {
    wrError("M or O has incorrect size.");
  }

  // create output matrix
  int hb = h / binSize, wb = w / binSize;
  int nChns = useHog == 0 ? nOrients : (useHog == 1 ? nOrients * 4 : nOrients * 3 + 5);
  H.create(hb, wb, nChns);

  if (nOrients == 0) return;
  if (useHog == 0) {
    gradHist(M.data, O.data, H.data, h, w, binSize, nOrients, softBin, full);
  } else if (useHog == 1) {
    hog(M.data, O.data, H.data, h, w, binSize, nOrients, softBin, full, clipHog);
  } else {
    fhog(M.data, O.data, H.data, h, w, binSize, nOrients, softBin, clipHog);
  }
}
