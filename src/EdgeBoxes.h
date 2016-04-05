#ifndef _EDGE_BOXES_H
#define _EDGE_BOXES_H

#include "CellArray.h"
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>

#define PI 3.14159265f

template <class T>
class Array {
public:
  Array(): h(0), w(0), data(NULL), isFree(1) {}
  Array(int _h, int _w): h(_h), w(_w), data(new T[h * w]()), isFree(0) {}
  void init(int _h, int _w) {
    if (!isFree && h * w == _h * _w) {
      h = _h, w = _w;
      memset(data, 0, sizeof(T) * h * w);
    } else {
      clear();
      h = _h;
      w = _w;
      data = new T[h * w]();
      isFree = false;
    }
  }
  void init(int _h, int _w, T *_data) {
    clear();
    h = _h;
    w = _w;
    data = _data;
    isFree = false;
  }
  T& at(size_t c, size_t r) {
    return data[c * h + r];
  }
  void clear() {
    if (!isFree) {
      delete [] data;
    }
    h = w = 0;
    data = NULL;
    isFree = 1;
  }

  int h, w;
  T* data;
  bool isFree;
};

typedef std::vector<float> vectorf;
typedef std::vector<int> vectori;
typedef Array<float> arrayf;
typedef Array<int> arrayi;

struct Box {
  int r, c, h, w;
  float s;
  Box() {}
  Box(int _r, int _c, int _h, int _w, float _s = 0.f): r(_r), c(_c), h(_h), w(_w), s(_s) {}
  bool operator < (const Box &rhs) const {
    return s < rhs.s;
  }
  float overlap(const Box &rhs) const {
    if (h <= 0 || w <= 0) return 0;
    if (h >= rhs.r + rhs.h || w >= rhs.c + rhs.w) return 0;
    int r0 = std::max(r, rhs.r), r1 = std::min(r + h, rhs.r + rhs.h);
    int c0 = std::max(c, rhs.c), c1 = std::min(c + w, rhs.c + rhs.w);
    float areai = w * h, areaj = rhs.w * rhs.h;
    float areaij = std::max(0, r1 - r0) * std::max(0, c1 - c0);
    return areaij / (areai + areaj - areaij);
  }
};

typedef std::vector<Box> Boxes;

class EdgeBoxes {
public:
  float _alpha, _beta, _eta, _gamma, _kappa;
  float _edgeMinMag, _edgeMergeThr, _clusterMinMag;
  float _minScore, _maxAspectRatio, _minBoxArea;
  int _maxBoxes;

  void initialize(float alpha, float beta, float eta, float minScore, int maxBoxes,
                  float edgeMinMag, float edgeMergeThr, float clusterMinMag,
                  float maxAspectRatio, float minBoxArea, float gamma, float kappa);
  Boxes generate(CellArray &E, CellArray &O);

private:
  // edge segment information (see clusterEdges)
  int h, w;                         // image dimensions
  int _segCnt;                      // total segment count
  arrayi _segIds;                   // segment ids (-1/0 means no segment)
  vectorf _segMag;                  // segment edge magnitude sums
  vectori _segR, _segC;             // segment lower-right pixel
  std::vector<vectorf> _segAff;          // segment affinities
  std::vector<vectori> _segAffIdx;       // segment neighbors

  // data structures for efficiency (see prepDataStructs)
  arrayf _segIImg, _magIImg;
  arrayi _hIdxImg, _vIdxImg;
  std::vector<vectori> _hIdxs, _vIdxs;
  vectorf _scaleNorm;
  float _scStep, _arStep, _rcStepRatio;

  // data structures for efficiency (see scoreBox)
  arrayf _sWts;
  arrayi _sDone, _sMap, _sIds;
  int _sId;

  void generate(Boxes &boxes, arrayf &E, arrayf &O, arrayf &V);

  // helper routines
  void clusterEdges(arrayf &E, arrayf &O, arrayf &V);
  void prepDataStructs(arrayf &E);
  void scoreAllBoxes(Boxes &boxes);
  void scoreBox(Box &box);
  void refineBox(Box &box);
  void drawBox(Box &box, arrayf &E, arrayf &V);
};

#endif
