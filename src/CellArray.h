#ifndef _CELL_ARRAY_H
#define _CELL_ARRAY_H

#include "global.h"
#include "opencv2/opencv.hpp"
#include <vector>

/**
 * Matlab Matrix like data structure, elements are stored in Column-major order
 * only support three dimension matrix now, [rows x cols x channels]
 */
class CellArray {
public:
  CellArray();
  CellArray(int _rows, int _cols, int _channels = 1);
  CellArray(const CellArray &ca);
  CellArray(const cv::Mat &m);

  ~CellArray();

  cv::Mat toCvMat() const;
  void fromCvMat(const cv::Mat &m);

  void crop(int r1, int r2, int c1, int c2);

  void release();
  void create(int _rows, int _cols, int _channels = 1);

  CellArray& operator=(const CellArray& ca);

  int total() const;

  //! access matrix elements, i0-th row, i1-th col, i2-th channel
  float* chn(int i = 0);
  float& at(int i0 = 0, int i1 = 0, int i2 = 0);
  const float& at(int i0 = 0, int i1 = 0, int i2 = 0) const;
  void multiply(float k);
  void swap(CellArray &ca);

  //! the number of rows, columns and channels
  int rows, cols, channels;
  //! pointer to the data
  float* data;

private:
  void copyFromMatRawData(const float *input);
  void copyToMatRawData(float *output) const;
};

inline int CellArray::total() const {
  return rows * cols * channels;
}

inline float* CellArray::chn(int i) {
  return data + i * rows * cols;
}

inline float& CellArray::at(int i0, int i1, int i2) {
  return data[i2 * rows * cols + i1 * rows + i0];
}

inline const float& CellArray::at(int i0, int i1, int i2) const {
  return data[i2 * rows * cols + i1 * rows + i0];
}

inline void CellArray::copyFromMatRawData(const float *input) {
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      for (int k = 0; k < channels; ++k) {
        data[k * rows * cols + j * rows + i] = input[i * cols * channels + j * channels + k];
      }
    }
  }
}

inline void CellArray::copyToMatRawData(float *output) const {
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      for (int k = 0; k < channels; ++k) {
        output[i * cols * channels + j * channels + k] = data[k * rows * cols + j * rows + i];
      }
    }
  }
}

inline void CellArray::multiply(float k) {
  float*u = data;
  int total = rows * cols * channels;
  while (total--) *(u++) *= k;
}

void mergeCellArray(CellArray a[], int num, CellArray &output);
void mergeCellArray(std::vector<CellArray> &a, int num, CellArray &output);

#endif
