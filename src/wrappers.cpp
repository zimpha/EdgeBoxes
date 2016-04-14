#include "wrappers.h"
#include <iostream>
#include <cstdlib>
#include <string>
#include <malloc.h>
#include <opencv2/opencv.hpp>

void wrCreateCVMat(int rows, int cols, int type, cv::Mat& O) {
  if (O.rows != rows || O.cols != cols || O.type() != type) {
    O.release();
    O.create(rows, cols, type);
  } else {
    O = cv::Scalar(0);
  }
}

void wrCreateCVMat(cv::Size size, int type, cv::Mat& O) {
  if (O.size() != size || O.type() != type) {
    O.release();
    O.create(size, type);
  } else {
    O = cv::Scalar(0);
  }
}

void wrError(const std::string &errorMsg) {
  // TODO: deal with error message
  std::cerr << errorMsg << std::endl;
  throw errorMsg;
}

void* wrCalloc(size_t num, size_t size) {
  return calloc(num, size);
}

void* wrMalloc(size_t size) {
  return malloc(size);
}

void wrFree(void* ptr) {
  free(ptr);
}

// platform independent aligned memory allocation (see also alFree)
void* alMalloc(size_t size, int alignment) {
  return memalign(alignment, size);
}

// platform independent alignned memory de-allocation (see also alMalloc)
void alFree(void* aligned) {
  free(aligned);
}
