#include "imResample.h"
#include <cstring>
#include <cmath>
#include <typeinfo>

void imResample(CellArray& input, CellArray& output, cv::Size dsize, double fx, double fy, const std::string& method, float norm) {
  bool useBilinear = method == "bilinear";
  if (dsize.width == 0 || dsize.height == 0) {
    if (fx == 0 || fy == 0) {
      wrError("when dsize is zero, fx and fy must be non-zero");
    }
    dsize = cv::Size(round(fx * input.cols), round(fy * input.rows));
  }
  if (dsize.height == input.rows && dsize.width == input.cols && norm == 1) {
    output = input;
    return;
  }

  if (useBilinear) {
    output.create(dsize.height, dsize.width, input.channels);
    int nChannels = input.channels;
    int ns[2] = {input.rows, input.cols};
    int ms[2] = {output.rows, output.cols};
    if (ms[0] <= 0 || ms[1] <= 0) {
      wrError("downsampling factor too small.");
    }
    float *A = input.data;
    float *B = output.data;
    resample(A, B, ns[0], ms[0], ns[1], ms[1], nChannels, norm);
  } else {
    cv::Mat A = input.toCvMat(), B;
    cv::resize(A, B, dsize, 0, 0, cv::INTER_NEAREST);
    if (norm != 1.0f) B *= norm;
    output.fromCvMat(B);
  }
}

CellArray imResample(CellArray& input, cv::Size dsize, double fx, double fy, const std::string& method, float norm) {
  CellArray output;
  imResample(input, output, dsize, fx, fy, method, norm);
  return output;
}
