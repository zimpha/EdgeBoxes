#ifndef _EDGE_BOXES_H
#define _EDGE_BOXES_H

#include "CellArray.h"
#include "global.h"
#include "box.h"
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>

#define PI 3.14159265f

template <class T>
class Array {
public:
  Array(): h(0), w(0), data(NULL), isFree(true) {}
  ~Array() {clear();}
  Array(int _h, int _w): h(_h), w(_w), data(new T[h * w]()), isFree(false) {}
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
  T& at(size_t c, size_t r) {
    return data[c * h + r];
  }
  const T& at(size_t c, size_t r) const {
    return data[c * h + r];
  }
  void clear() {
    if (!isFree) {
      delete [] data;
    }
    h = w = 0;
    data = NULL;
    isFree = true;
  }

  int h, w;
  T* data;
  bool isFree;
};

typedef std::vector<float> vectorf;
typedef std::vector<int> vectori;
typedef std::vector<std::pair<int, float> > vectorif;
typedef Array<float> arrayf;
typedef Array<int> arrayi;
typedef Array<bool> arrayb;

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
  std::vector<Point> _segP;         // segment lower-right pixel
  std::vector<vectorif> _segAff;    // segment neighbors and affinities

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

  void generate(Boxes &boxes, CellArray &E, CellArray &O, arrayf &V);

  // helper routines
  void clusterEdges(CellArray &E, CellArray &O, arrayf &V);
  void prepDataStructs(CellArray &E);
  void scoreAllBoxes(Boxes &boxes);
  void scoreBox(Box &box);
  void refineBox(Box &box);
  void drawBox(Box &box, CellArray &E, arrayf &V);
};

#endif
