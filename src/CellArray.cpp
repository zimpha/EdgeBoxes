#include "CellArray.h"
#include "wrappers.h"

CellArray::CellArray():type(-1), rows(0), cols(0), channels(0), data(NULL) {
}

CellArray::CellArray(int _rows, int _cols, int _type, int _channels) {
  data = NULL;
  create(_rows, _cols, _channels, _type);
}

CellArray::CellArray(const CellArray &ca) {
  rows = ca.rows;
  cols = ca.cols;
  channels = ca.channels;
  step = ca.step;
  type = ca.type;
  data = new uint8_t[rows * cols * channels * step];
  memcpy(data, ca.data, sizeof(uint8_t) * (rows * cols * channels * step));
}

CellArray::CellArray(const cv::Mat &m) {
  data = NULL;
  fromCvMat(m);
}

void CellArray::fromCvMat(const cv::Mat &m) {
  release();
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
  if (m.isContinuous()) {
    if (m.depth() == CV_8U) {
      type = UINT8_CLASS;
      copyFromMatRawData((uint8_t*)m.data);
    }
    else if (m.depth() == CV_8S) {
      type = INT8_CLASS;
      copyFromMatRawData((int8_t*)m.data);
    }
    else if (m.depth() == CV_16U) {
      type = UINT16_CLASS;
      copyFromMatRawData((uint16_t*)m.data);
    }
    else if (m.depth() == CV_16S) {
      type = INT16_CLASS;
      copyFromMatRawData((int16_t*)m.data);
    }
    else if (m.depth() == CV_32S) {
      type = INT32_CLASS;
      copyFromMatRawData((int32_t*)m.data);
    }
    else if (m.depth() == CV_32F) {
      type = SINGLE_CLASS;
      copyFromMatRawData((float*)m.data);
    }
    else if (m.depth() == CV_64F) {
      type = DOUBLE_CLASS;
      copyFromMatRawData((double*)m.data);
    }
  } else {
    wrError("current not support uncontinuous memory");
  }
}

cv::Mat CellArray::toCvMat() const {
  cv::Mat res;
  if (type == UINT8_CLASS) {
    res.create(rows, cols, CV_8UC(channels));
    copyToMatRawData((uint8_t*)res.data);
  } else if (type == INT8_CLASS) {
    res.create(rows, cols, CV_8SC(channels));
    copyToMatRawData((int8_t*)res.data);
  } else if (type == UINT16_CLASS) {
    res.create(rows, cols, CV_16UC(channels));
    copyToMatRawData((uint16_t*)res.data);
  } else if (type == INT16_CLASS) {
    res.create(rows, cols, CV_16SC(channels));
    copyToMatRawData((int16_t*)res.data);
  } else if (type == INT32_CLASS) {
    res.create(rows, cols, CV_32SC(channels));
    copyToMatRawData((int32_t*)res.data);
  } else if (type == SINGLE_CLASS) {
    res.create(rows, cols, CV_32FC(channels));
    copyToMatRawData((float*)res.data);
  } else if (type == DOUBLE_CLASS) {
    res.create(rows, cols, CV_64FC(channels));
    copyToMatRawData((double*)res.data);
  } else {
    wrError("Unsupported type for cvMat");
  }
  return res;
}

void CellArray::release() {
  if (data == NULL) return;
  delete[] data;
  rows = cols = channels = step = 0;
  type = -1;
  data = NULL;
}

void CellArray::create(int _rows, int _cols, int _channels, int _type) {
  release();
  switch (_type) {
  case UINT8_CLASS: case INT8_CLASS:
    step = sizeof(uint8_t);
    break;
  case UINT16_CLASS: case INT16_CLASS:
    step = sizeof(uint16_t);
    break;
  case UINT32_CLASS: case INT32_CLASS:
    step = sizeof(uint32_t);
    break;
  case SINGLE_CLASS:
    step = sizeof(float);
    break;
  case DOUBLE_CLASS:
    step = sizeof(double);
    break;
  default:
    wrError("Unsupported type.");
    break;
  }
  rows = _rows;
  cols = _cols;
  channels = _channels;
  type = _type;
  data = new uint8_t[rows * cols * channels * step];
}

CellArray::~CellArray() {
  release();
}

CellArray& CellArray::operator=(const CellArray& ca) {
  if (total() != ca.total()) {
    delete[] data;
    data = new uint8_t[ca.total()];
  }
  rows = ca.rows;
  cols = ca.cols;
  channels = ca.channels;
  step = ca.step;
  type = ca.type;
  memcpy(data, ca.data, sizeof(uint8_t) * (rows * cols * channels * step));
  return *this;
}

void mergeCellArray(CellArray a[], int num, CellArray &output) {
  if (num == 0) return;
  int total = 0;
  for (int i = 0; i < num; ++i) {
    if (a[i].rows != a[0].rows || a[i].cols != a[0].cols || a[i].type != a[0].type) {
      wrError("the size and type should be the same");
    }
    total += a[i].channels;
  }
  output.create(a[0].rows, a[0].cols, total, a[0].type);
  uint8_t *u = output.data;
  for (int i = 0; i < num; ++i) {
    total = a[i].total();
    memcpy(u, a[i].data, sizeof(uint8_t) * total);
    u += total;
  }
}

void mergeCellArray(std::vector<CellArray> &a, int num, CellArray &output) {
  if (num == 0) return;
  int total = 0;
  for (int i = 0; i < num; ++i) {
    if (a[i].rows != a[0].rows || a[i].cols != a[0].cols || a[i].type != a[0].type) {
      wrError("the size and type should be the same");
    }
    total += a[i].channels;
  }
  output.create(a[0].rows, a[0].cols, total, a[0].type);
  uint8_t *u = output.data;
  for (int i = 0; i < num; ++i) {
    total = a[i].total();
    memcpy(u, a[i].data, sizeof(uint8_t) * total);
    u += total;
  }
}
