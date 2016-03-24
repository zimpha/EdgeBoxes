#include "wrappers.h"
#include <iostream>
#include <cstdlib>
#include <string>
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
  const size_t pSize = sizeof(void*), a = alignment - 1;
  void *raw = wrMalloc(size + a + pSize);
  void *aligned = (void*)(((size_t)raw + pSize + a) & ~a);
  *(void**)((size_t)aligned - pSize) = raw;
  return aligned;
}

// platform independent alignned memory de-allocation (see also alMalloc)
void alFree(void* aligned) {
  void* raw = *(void**)((char*)aligned - sizeof(void*));
  wrFree(raw);
}
