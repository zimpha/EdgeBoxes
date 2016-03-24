#include "EdgeDetector.h"
#include "convUtil.h"
#include "gradientUtil.h"
#include "imageUtil.h"
#include "wrappers.h"

EdgeDetector::EdgeDetector() {
}

EdgeDetector::~EdgeDetector() {
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
}
