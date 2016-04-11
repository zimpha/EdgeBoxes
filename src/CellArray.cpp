#include "CellArray.h"
#include "wrappers.h"

CellArray::CellArray(): rows(0), cols(0), channels(0), data(NULL) {
}

CellArray::CellArray(int _rows, int _cols, int _channels) {
  data = NULL;
  create(_rows, _cols, _channels);
}

CellArray::CellArray(const CellArray &ca) {
  rows = ca.rows;
  cols = ca.cols;
  channels = ca.channels;
  data = (float*)wrCalloc(total(), sizeof(float));
  memcpy(data, ca.data, sizeof(float) * total());
}

CellArray::CellArray(const cv::Mat &m) {
  data = NULL;
  fromCvMat(m);
}

void CellArray::fromCvMat(const cv::Mat &m) {
  if (m.depth() != CV_32F) {
    wrError("Unsupported type");
  }
  if (!m.isContinuous()) {
    wrError("Mat is not continuous");
  }
  release();
  rows = m.rows;
  cols = m.cols;
  channels = m.channels();
  data = (float*)wrCalloc(total(), sizeof(float));
  copyFromMatRawData((float*)m.data);
}

cv::Mat CellArray::toCvMat() const {
  cv::Mat res;
  res.create(rows, cols, CV_32FC(channels));
  copyToMatRawData((float*)res.data);
  return res;
}

void CellArray::crop(int r1, int r2, int c1, int c2) {
  int tr = rows, tc = cols;
  rows = r2 - r1;
  cols = c2 - c1;
  float *u = (float*)wrCalloc(total(), sizeof(float));
  for (int k = 0; k < channels; ++k) {
    float *v = u + k * cols * rows;
    for (int c = 0; c < cols; ++c) {
      memcpy(v, data + k * tc * tr + (c1 + c) * tr + r1, sizeof(float) * rows);
      v += rows;
    }
  }
  wrFree(data);
  data = u;
}

void CellArray::swap(CellArray &ca) {
  std::swap(rows, ca.rows);
  std::swap(cols, ca.cols);
  std::swap(channels, ca.channels);
  std::swap(data, ca.data);
}

void CellArray::release() {
  if (data == NULL) return;
  wrFree(data);
  rows = cols = channels = 0;
  data = NULL;
}

void CellArray::create(int _rows, int _cols, int _channels) {
  release();
  rows = _rows;
  cols = _cols;
  channels = _channels;
  data = (float*)wrCalloc(total(), sizeof(float));
}

CellArray::~CellArray() {
  release();
}

CellArray& CellArray::operator=(const CellArray& ca) {
  if (total() != ca.total()) {
    wrFree(data);
    data = (float*)wrCalloc(ca.total(), sizeof(float));
  }
  rows = ca.rows;
  cols = ca.cols;
  channels = ca.channels;
  if (ca.data == NULL) data = NULL;
  else memcpy(data, ca.data, sizeof(float) * total());
  return *this;
}

void mergeCellArray(CellArray a[], int num, CellArray &output) {
  if (num == 0) return;
  int total = 0;
  for (int i = 0; i < num; ++i) total += a[i].channels;
  output.create(a[0].rows, a[0].cols, total);
  float *u = output.data;
  for (int i = 0; i < num; ++i) {
    total = a[i].total();
    memcpy(u, a[i].data, sizeof(float) * total);
    u += total;
  }
}

void mergeCellArray(std::vector<CellArray> &a, int num, CellArray &output) {
  if (num == 0) return;
  int total = 0;
  for (int i = 0; i < num; ++i) total += a[i].channels;
  output.create(a[0].rows, a[0].cols, total);
  float *u = output.data;
  for (int i = 0; i < num; ++i) {
    total = a[i].total();
    memcpy(u, a[i].data, sizeof(float) * total);
    u += total;
  }
}
