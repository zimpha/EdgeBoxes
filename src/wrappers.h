#ifndef _WRAPPERS_H
#define _WRAPPERS_H

#include <cstdlib>
#include <string>
#include <opencv2/opencv.hpp>

void wrCreateCVMat(int rows, int cols, int type, cv::Mat& O);

void wrCreateCVMat(cv::Size size, int type, cv::Mat& O);

void wrError(const std::string &errorMsg);

void* wrCalloc(size_t num, size_t size);

void* wrMalloc(size_t size);

void wrFree(void* ptr);

// platform independent aligned memory allocation (see also alFree)
void* alMalloc(size_t size, int alignment);

// platform independent alignned memory de-allocation (see also alMalloc)
void alFree(void* aligned);

#endif
