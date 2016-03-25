#include "CellArray.h"
#include "wrappers.h"

CellArray::CellArray():type(-1), rows(0), cols(0), channels(0), data(NULL) {
}

CellArray::CellArray(int _rows, int _cols, int _type, int _channels) {
  create(_rows, _cols, _channels, _type);
}

CellArray::CellArray(const CellArray &ca) {
  rows = ca.rows;
  cols = ca.cols;
  channels = ca.channels;
  step = ca.step;
  data = new uint8_t[rows * cols * channels * step];
  memcpy(data, ca.data, sizeof(uint8_t) * (rows * cols * channels * step));
}

CellArray::CellArray(const cv::Mat &m) {
  rows = m.rows;
  cols = m.cols;
  channels = m.channels();
  if (m.depth() == CV_8U || m.depth() == CV_8S) step = sizeof(uint8_t);
  else if (m.depth() == CV_16U || m.depth() == CV_16S) step = sizeof(uint16_t);
  else if (m.depth() == CV_32S) step = sizeof(int32_t);
  else if (m.depth() == CV_32F) step = sizeof(float);
  else if (m.depth() == CV_64F) step = sizeof(double);
  else wrError("Unsupported type");
  data = new uint8_t[rows * cols * channels * step];
}

void CellArray::release() {
  if (data == NULL) return;
  delete[] data;
  rows = cols = channels = step = 0;
}

void CellArray::create(int _rows, int _cols, int _channels, int _type) {
  switch (_type) {
  case UINT8_CLASS, INT8_CLASS:
    step = sizeof(uint8_t);
    break;
  case UINT16_CLASS, INT16_CLASS:
    step = sizeof(uint16_t);
    brea;
  case UINT32_CLASS, INT32_CLASS:
    step = sizeof(uint32_t);
    break;
  case SINGLE_CLASS:
    step = sizeof(float);
    break;
  case DOUBLE_CLASS:
    step = sizeof(double);
    break;
  default:
    wrError("Unsupported type.")
    break;
  }
  data = new uint8_t[rows * cols * channels * step];
}

CellArray::~CellArray() {
  release();
}
