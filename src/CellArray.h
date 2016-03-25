#ifndef _CELL_ARRAY_H
#define _CELL_ARRAY_H

#include "global.h"

/**
 * Matlab Matrix like data structure, elements are stored in Column-major order
 * only support three dimension matrix now, [rows x cols x channels]
 */
class CellArray {
public:
  CellArray();
  CellArray(int _rows, int _cols, int _channels = 1, int _type = UINT8_CLASS);
  CellArray(const CellArray &ca, bool CopyData = true);
  CellArray(const cv::Mat &m);

  ~CellArray();

  cv::Mat toCvMat() const;

  void release();
  void create(int _rows, int _cols, int _channels = 1, int _type = UINT8_CLASS);


  // access matrix elements, i0-th row, i1-th col, i2-th channel
  template<typename _Tp> _Tp& at(int i0 = 0, int i1 = 0, int i2 = 0);
  template<typename _Tp> const _Tp& at(int i0 = 0, int i1 = 0, int i2 = 0) const;

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
};

template<typename _Tp> inline
_Tp& CellArray::at(int i0 = 0, int i1 = 0, int i2 = 0) {
  const _Tp* u = (_Tp*)data;
  return u[i2 * rows * cols + i1 * rows + i0];
}

template<typename _Tp> inline
const _Tp& CellArray::at(int i0 = 0, int i1 = 0, int i2 = 0) const {
  const _Tp* u = (_Tp*)data;
  return u[i2 * rows * cols + i1 * rows + i0];
}

#endif
