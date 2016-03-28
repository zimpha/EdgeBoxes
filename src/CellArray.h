#ifndef _CELL_ARRAY_H
#define _CELL_ARRAY_H

#include "global.h"
#include "opencv2/opencv.hpp"

/**
 * Matlab Matrix like data structure, elements are stored in Column-major order
 * only support three dimension matrix now, [rows x cols x channels]
 */
class CellArray {
public:
  CellArray();
  CellArray(int _rows, int _cols, int _channels = 1, int _type = UINT8_CLASS);
  CellArray(const CellArray &ca);
  CellArray(const cv::Mat &m);

  ~CellArray();

  cv::Mat toCvMat() const;
  void fromCvMat(const cv::Mat &m);

  void release();
  void create(int _rows, int _cols, int _channels = 1, int _type = UINT8_CLASS);

  CellArray operator=(const CellArray& ca);

  //! access matrix elements, i0-th row, i1-th col, i2-th channel
  template<typename _Tp> _Tp& at(int i0 = 0, int i1 = 0, int i2 = 0);
  template<typename _Tp> const _Tp& at(int i0 = 0, int i1 = 0, int i2 = 0) const;
  template<typename T> void multiply(T k);

  //! the element type of matrix
  int type;
  //! the number of rows, columns and channels
  int rows, cols, channels;
  //! pointer to the data
  uint8_t* data;

private:
  // implemented
  int* refcount;
  int step;

  template<typename T> void copyFromMatRawData(T *input);
  template<typename T> void copyToMatRawData(T *output) const;
};

template<typename _Tp> inline
_Tp& CellArray::at(int i0, int i1, int i2) {
  const _Tp* u = (_Tp*)data;
  return u[i2 * rows * cols + i1 * rows + i0];
}

template<typename _Tp> inline
const _Tp& CellArray::at(int i0, int i1, int i2) const {
  const _Tp* u = (_Tp*)data;
  return u[i2 * rows * cols + i1 * rows + i0];
}

template<typename T> inline
void CellArray::copyFromMatRawData(T *input) {
  T* u = (T*)data;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      for (int k = 0; k < channels; ++k) {
        u[k * rows * cols + j * rows + i] = input[i * cols * channels + j * channels + k];
      }
    }
  }
}

template<typename T> inline
void CellArray::copyToMatRawData(T *output) const {
  T* u = (T*)data;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      for (int k = 0; k < channels; ++k) {
        output[i * cols * channels + j * channels + k] = u[k * rows * cols + j * rows + i];
      }
    }
  }
}

template<typename T> inline
void CellArray::multiply(T k) {
  T* u = (T*)data;
  int total = rows * cols * channels;
  while (total--) *(u++) *= k;
}

void mergeCellArray(CellArray a[], int num, CellArray &output);

#endif
