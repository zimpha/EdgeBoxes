#include "chnsCompute.h"
#include "convUtil.h"
#include "imResample.h"
#include "gradientUtil.h"

#include <opencv2/opencv.hpp>

void addChns(int k, ChnsOutput &chns, CellArray &data, const std::string &name, int padwith, int h, int w) {
  int h1 = data.rows, w1 = data.cols;
  if (h1 != h || w1 != w) {
    imResample(data, chns.data[k], cv::Size(w, h));
  } else chns.data[k] = data;
  chns.infos[k].name = name;
  chns.infos[k].nChannels = data.channels;
  chns.infos[k].padwith = padwith;
}

void chnsCompute(CellArray &I, ChnsInput &chnsInput, ChnsOutput &chnsOutput) {
  if (!chnsInput.complete) {
    chnsInput = ChnsInput();
  }
  if (I.total() == 0) return;
  chnsOutput.nTypes = chnsInput.pColor.enabled + chnsInput.pGradMag.enabled + chnsInput.pGradHist.enabled;
  chnsOutput.infos.resize(chnsOutput.nTypes);
  chnsOutput.data.resize(chnsOutput.nTypes);

  // crop I so divisible by shrink and get target dimensions
  int shrink = chnsInput.shrink;
  int h = I.rows, w = I.cols;
  int crh = h % shrink, crw = w % shrink;
  if (crh || crw) {
    h -= crh; w -= crw;
    I.crop(0, h, 0, w);
  }
  h /= shrink, w /= shrink;

  int k = 0;
  // compute color channels
  // I=rgbConvert(I,p.colorSpace); converted outside before
  CellArray tmp; convTri(I, tmp, chnsInput.pColor.smooth); I.swap(tmp);
  if (chnsInput.pColor.enabled) {
    addChns(k++, chnsOutput, I, "color channels", (int)replicate, h, w);
  }

  // compute gradient magnitude channel
  CellArray M, O, H;
  bool full = chnsInput.pGradMag.full;
  if (chnsInput.pGradMag.enabled || chnsInput.pGradHist.enabled) {
    GradMagParam &p = chnsInput.pGradMag;
    gradientMag(I, M, O, p.colorChannel, p.normRad, p.normConst, full);
    addChns(k++, chnsOutput, M, "gradient magnitude", 0, h, w);
  }

  // compute gradient histgoram channels
  if (chnsInput.pGradHist.enabled) {
    GradHistParam &p = chnsInput.pGradHist;
    gradientHist(M, O, H, p.binSize, p.nOrients, p.softBin, p.useHog, p.clipHog, full);
    addChns(k++, chnsOutput, H, "gradient histogram", 0, h, w);
  }
  assert(k == chnsOutput.nTypes);
}
