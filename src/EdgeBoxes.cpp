#include "EdgeBoxes.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <queue>

template<typename T>
T clamp(T val, T lo, T hi) {
  return val < lo ? lo : (val > hi ? hi : val);
}

void boxesNms(Boxes &boxes, float thr, float eta, int maxBoxes);

void EdgeBoxes::initialize(float alpha, float beta, float eta, float minScore, int maxBoxes,
                float edgeMinMag, float edgeMergeThr, float clusterMinMag,
                float maxAspectRatio, float minBoxArea, float gamma, float kappa) {
  _alpha = alpha;
  _beta = beta;
  _eta = eta;
  _minScore = minScore;
  _maxBoxes = maxBoxes;
  _edgeMinMag = edgeMinMag;
  _edgeMergeThr = edgeMergeThr;
  _clusterMinMag = clusterMinMag;
  _maxAspectRatio = maxAspectRatio;
  _minBoxArea = minBoxArea;
  _gamma = gamma;
  _kappa = kappa;

  // initialize step sizes
  _scStep = sqrt(1 / _alpha);
  _arStep = (1 + _alpha) / (2 * _alpha);
  _rcStepRatio = (1 - _alpha) / (1 + _alpha);

  // create _scaleNorm
  _scaleNorm.resize(10000);
  for (int i = 0; i < 10000; ++i) {
    _scaleNorm[i] = pow(1.f / i, _kappa);
  }
}

Boxes EdgeBoxes::generate(CellArray &E, CellArray &O) {
  this->h = E.rows; this->w = E.cols;
  arrayf V;
  Boxes boxes;
  generate(boxes, E, O, V);
  return boxes;
}

void EdgeBoxes::generate(Boxes &boxes, CellArray &E, CellArray &O, arrayf &V) {
  clusterEdges(E, O, V);
  prepDataStructs(E);
  scoreAllBoxes(boxes);
}

struct ClusterNode {
    int c, r; float s;
    bool operator < (const ClusterNode &rhs) const {
      return s > rhs.s;
    }

};

void EdgeBoxes::clusterEdges(CellArray &E, CellArray &O, arrayf &V) {
  int i, j, c, r, cd, rd;

  // greedily merge connected edge pixels into clusters (create _segIds)
  _segIds.init(h, w); _segCnt = 1;
  for (c = 0; c < w; ++c) for (r = 0; r < h; ++r) {
    _segIds.at(c, r) = -(c == 0 || c == w - 1 || r == 0 || r == h - 1 ||
                       E.at(r, c) <= _edgeMinMag);
  }
  arrayi vis(h, w); int vis_cnt = 0;
  for (c = 1; c < w - 1; ++c) for (r = 1; r < h - 1; ++r) {
    if (_segIds.at(c, r)) continue;
    int c0 = c, r0 = r, cc, rr; vis_cnt++;
    std::priority_queue<ClusterNode> vs;
    for (float sumv = 0; sumv < _edgeMergeThr; ) {
      _segIds.at(c0, r0) = _segCnt;
      float o0 = O.at(r0, c0), o1, v;
      for (cd = -1; cd <= 1; ++cd) for (rd = -1; rd <= 1; ++rd) {
        if (_segIds.at(cc = c0 + cd, rr = r0 + rd) || vis.at(cc, rr) == vis_cnt) continue;
        o1 = O.at(rr, cc);
        v = fabs(o1 - o0) / PI;
        if (v > .5) v = 1 - v;
        vs.push((ClusterNode){cc, rr, v});
        vis.at(cc, rr) = vis_cnt;
      }
      if (vs.empty()) sumv += 1000;
      else {
        c0 = vs.top().c; r0 = vs.top().r;
        sumv += vs.top().s; vs.pop();
      }
    }
    ++_segCnt;
  }

  // merge or remove small segments
  _segMag.assign(_segCnt, 0);
  for (c = 1; c < w - 1; ++c) for (r = 1; r < h - 1; ++r) {
    if ((j = _segIds.at(c, r)) > 0) _segMag[j] += E.at(r, c);
  }
  for (c = 1; c < w - 1; ++c) for (r = 1; r < h - 1; ++r) {
    if ((j = _segIds.at(c, r)) > 0 && _segMag[j] <= _clusterMinMag) {
      _segIds.at(c, r) = 0;
    }
  }
  // TODO: optimize using invert search or queue
  for (i = 1; i > 0; ) {
    i = 0;
    for (c = 1; c < w - 1; ++c) for (r = 1; r < h - 1; ++r) {
      if (_segIds.at(c, r)) continue;
      float o0 = O.at(r, c), o1, v, minv = 1000;
      j = 0;
      for (cd = -1; cd <= 1; ++cd) for (rd = -1; rd <= 1; ++rd) {
        if (_segIds.at(c + cd, r + rd) <= 0) continue;
        o1 = O.at(r + rd, c + cd);
        v = fabs(o1 - o0) / PI;
        if (v > .5) v = 1 - v;
        if (v < minv) {
          minv = v;
          j = _segIds.at(c + cd, r + rd);
        }
      }
      _segIds.at(c, r) = j;
      if (j > 0) ++i;
    }
  }

  // compactify representation (remap _segIds)
  _segMag.assign(_segCnt, 0);
  vectori map(_segCnt, 0);
  _segCnt = 1;
  // TODO: update _segMag during above merge code
  for (c = 1; c < w - 1; ++c) for (r = 1; r < h - 1; ++r) {
    if ((j = _segIds.at(c, r)) > 0) _segMag[j] += E.at(r, c);
  }
  for (i = 0; i < _segMag.size(); ++i) {
    if (_segMag[i] > 0) map[i] = _segCnt++;
  }
  for (c = 1; c < w - 1; ++c) for (r = 1; r < h - 1; ++r) {
    if ((j = _segIds.at(c, r)) > 0) _segIds.at(c, r) = map[j];
  }

  // compute positional means and recompute _segMag
  _segMag.assign(_segCnt, 0);
  vectorf meanX(_segCnt, 0), meanY(_segCnt, 0);
  vectorf meanOx(_segCnt, 0), meanOy(_segCnt, 0), meanO(_segCnt, 0);
  for (c = 1; c < w - 1; ++c) for (r = 1; r < h - 1; ++r) {
    if ((j = _segIds.at(c, r)) <= 0) continue;
    float m = E.at(r, c), o = O.at(r, c) * 2;
    _segMag[j] += m;
    meanOx[j] += m * cos(o);
    meanOy[j] += m * sin(o);
    meanX[j] += m * c;
    meanY[j] += m * r;
  }
  for (i = 0; i < _segCnt; ++i) if (_segMag[i] > 0) {
    float m = _segMag[i];
    meanX[i] /= m;
    meanY[i] /= m;
    meanO[i] = atan2(meanOy[i] / m, meanOx[i] / m) * 0.5;
  }

  // compute segment affinities
  // TODO: store them together
  _segAff.resize(_segCnt);
  //_segAffIdx.resize(_segCnt);
  for (i = 0; i < _segCnt; ++i) _segAff[i].clear();
  // (i = 0; i < _segCnt; ++i) _segAffIdx[i].clear();
  const int rad = 2;
  for (c = rad; c < w - rad; ++c) for (r = rad; r < h - rad; ++r) {
    int s0 = _segIds.at(c, r); if (s0 <= 0) continue;
    for (cd = -rad; cd <= rad; ++cd) for (rd = -rad; rd <= rad; ++rd) {
      int s1 = _segIds.at(c + cd, r + rd); if (s1 <= s0) continue;
      bool found = false;
      // TODO: resize after all done
      for (i = 0; i < _segAff[s0].size(); ++i) {
        if (_segAff[s0][i].first == s1) {
          found = true;
          break;
        }
      }
      if (found) continue;
      float o = atan2(meanY[s0] - meanY[s1], meanX[s0] - meanX[s1]) + PI / 2;
      float a = fabs(cos(meanO[s0] - o) * cos(meanO[s1] - o));
      a = pow(a, _gamma);
      _segAff[s0].push_back(std::make_pair(s1, a));// _segAffIdx[s0].push_back(s1);
      _segAff[s1].push_back(std::make_pair(s0, a));// _segAffIdx[s1].push_back(s0);
    }
  }

  // compute _segC and _segR
  _segC.resize(_segCnt); _segR.resize(_segCnt);
  for (c = 1; c < w - 1; ++c) for (r = 1; r < h - 1; ++r) {
    if ((j = _segIds.at(c, r)) > 0) {
      _segC[j] = c;
      _segR[j] = r;
    }
  }

  // optionally create visualization (assume memory initialized is 3*w*h)
  if (!V.data) return;
  for (c = 0; c < w; ++c) for (r = 0; r < h; ++r) {
    i = _segIds.at(c, r);
    V.at(c + w * 0, r) = i <= 0 ? 1 : ((123 * i + 128) % 255) / 255.0f;
    V.at(c + w * 1, r) = i <= 0 ? 1 : ((7 * i + 3) % 255) / 255.0f;
    V.at(c + w * 2, r) = i <= 0 ? 1 : ((174 * i + 80) % 255) / 255.0f;
  }
}

void EdgeBoxes::prepDataStructs(CellArray &E) {
  int c, r, i;

  // create _segIImg
  arrayf E1(h, w);
  for (i = 0; i < _segCnt; ++i) if (_segMag[i] > 0) {
    E1.at(_segC[i], _segR[i]) = _segMag[i];
  }
  _segIImg.init(h + 1, w + 1);
  for (c = 1; c < w; ++c) for (r = 1; r < h; ++r) {
    _segIImg.at(c + 1, r + 1) = E1.at(c, r) + _segIImg.at(c, r + 1) +
      _segIImg.at(c + 1, r) - _segIImg.at(c, r);
  }

  // create _magIImg
  _magIImg.init(h + 1, w + 1);
  for (c = 1; c < w; ++c) for (r = 1; r < h; ++r) {
    float e = E.at(r, c) > _edgeMinMag ? E.at(r, c) : 0;
    _magIImg.at(c + 1, r + 1) = e + _magIImg.at(c, r + 1) +
      _magIImg.at(c + 1, r) - _magIImg.at(c, r);
  }

  // create remaining data structures
  _hIdxs.resize(h); _hIdxImg.init(h, w);
  for (r = 0; r < h; ++r) {
    int s = 0, s1;
    _hIdxs[r].clear(); _hIdxs[r].push_back(s);
    for (c = 0; c < w; ++c) {
      s1 = _segIds.at(c, r);
      if (s1 != s) {
        s = s1;
        _hIdxs[r].push_back(s);
      }
      _hIdxImg.at(c, r) = int(_hIdxs[r].size()) - 1;
    }
  }
  _vIdxs.resize(w); _vIdxImg.init(h, w);
  for (c = 0; c < w; ++c) {
    int s = 0, s1;
    _vIdxs[c].clear(); _vIdxs[c].push_back(s);
    for (r = 0; r < h; ++r) {
      int s1 = _segIds.at(c, r);
      if (s1 != s) {
        s = s1;
        _vIdxs[c].push_back(s);
      }
      _vIdxImg.at(c, r) = int(_vIdxs[c].size()) - 1;
    }
  }

  // initialize scoreBox() data structures
  int n = _segCnt + 1;
  _sWts.init(n, 1);
  _sDone.init(n, 1);
  _sMap.init(n, 1);
  _sIds.init(n, 1);
  for (i = 0; i < n; ++i) _sDone.at(0, i) = -1;
  _sId = 0;
}

void EdgeBoxes::scoreAllBoxes(Boxes &boxes) {
  // get list of all boxes roughly distributed in grid
  float minSize = sqrt(_minBoxArea);
  int arRad = int(log(_maxAspectRatio) / log(_arStep * _arStep));
  int scNum = int(ceil(log(std::max(w, h) / minSize) / log(_scStep)));
  boxes.resize(0);
  for (int s = 0; s < scNum; ++s) {
    int a, r, c, bh, bw, kr, kc, bId(-1);
    float sc = minSize * pow(_scStep, s);
    for (a = 0; a <= 2 * arRad; ++a) {
      float ar = pow(_arStep, a - arRad);
      bh = int(sc / ar);
      kr = std::max(2, int(bh * _rcStepRatio));
      bw = int(sc * ar);
      kc = std::max(2, int(bw * _rcStepRatio));
      for (c = 0; c < w - bw + kc; c += kc) for (r = 0; r < h - bh + kr; r += kr) {
        boxes.push_back(Box(r, c, bh, bw));
      }
    }
  }

  // score all boxes, refine top candidates, perform nms
  int i, k(0), m = boxes.size();
  for (i = 0; i < m; ++i) {
    //if (i % 1000 == 0) std::cerr << i << std::endl;
    scoreBox(boxes[i]);
    if (!boxes[i].s) continue;
    ++k;
    refineBox(boxes[i]);
  }
  std::sort(boxes.rbegin(), boxes.rend());
  boxes.resize(k);
  boxesNms(boxes, _beta, _eta, _maxBoxes);
}

void EdgeBoxes::scoreBox(Box &box) {
  int i, j, k, q;
  float *sWts = _sWts.data;
  int sId = _sId++; // meaning? -> just a global var working with sDone
  int *sDone = _sDone.data;
  int *sMap = _sMap.data;
  int *sIds = _sIds.data;
  // add edge count inside box
  int r1 = clamp(box.r + box.h,0,h-1);
  int r0 = box.r = clamp(box.r,0,h-1);
  int c1 = clamp(box.c + box.w,0,w-1);
  int c0 = box.c = clamp(box.c,0,w-1);
  int bh = box.h = r1 - box.r; bh >>= 1;
  int bw = box.w = c1 - box.c; bw >>= 1;
  float &score = box.s;
  score = _segIImg.at(c0, r0) + _segIImg.at(c1 + 1, r1 + 1)
        - _segIImg.at(c1 + 1, r0) - _segIImg.at(c0, r1 + 1);
  // subtract middle quarter of edges
  int r0m = r0 + bh / 2, r1m = r0m + bh;
  int c0m = c0 + bw / 2, c1m = c0m + bw;
  score -= _magIImg.at(c0m, r0m) + _magIImg.at(c1m + 1, r1m + 1)
         - _magIImg.at(c1m + 1, r0m) - _magIImg.at(c0m, r1m + 1);
  // short circuit computation if impossible to score highly
  float norm = _scaleNorm[bw + bh];
  if (norm * score < _minScore) {
    score = 0;
    return;
  }
  // find interesecting segments along four boundaries
  int cs, ce, rs, re, n = 0;
  cs = _hIdxImg.at(c0, r0); ce = _hIdxImg.at(c1, r0); // top
  for (i = cs; i <= ce; ++i) if ((j = _hIdxs[r0][i]) > 0 && sDone[j] != sId) {
    sIds[n] = j; sWts[n] = 1; sDone[j] = sId; sMap[j] = n++;
  }
  cs = _hIdxImg.at(c0, r1); ce = _hIdxImg.at(c1, r1); // bottom
  for (i = cs; i <= ce; ++i) if ((j = _hIdxs[r1][i]) > 0 && sDone[j] != sId) {
    sIds[n] = j; sWts[n] = 1; sDone[j] = sId; sMap[j] = n++;
  }
  rs = _vIdxImg.at(c0, r0); re = _vIdxImg.at(c0, r1); // left
  for (i = rs; i <= re; ++i) if ((j = _vIdxs[c0][i]) > 0 && sDone[j] != sId) {
    sIds[n] = j; sWts[n] = 1; sDone[j] = sId; sMap[j] = n++;
  }
  rs = _vIdxImg.at(c1, r0); re = _vIdxImg.at(c1, r1); // right
  for (i = rs; i <= re; ++i) if ((j = _vIdxs[c1][i]) > 0 && sDone[j] != sId) {
    sIds[n] = j; sWts[n] = 1; sDone[j] = sId; sMap[j] = n++;
  }
  //std::cerr << "start " << n << std::endl;
  // follow connected paths and set weights accordingly (w=1 means remove)
  // looks like just a bfs
  for (i = 0; i < n; ++i) {
    float w = sWts[i];
    j = sIds[i];
    for (k = 0; k < int(_segAff[j].size()); ++k) {
      q = _segAff[j][k].first;
      float wq = w * _segAff[j][k].second;
      if (wq < .05f) continue; // short circuit for efficiency
      if (sDone[q] == sId) {
        if (wq > sWts[sMap[q]]) {
          sWts[sMap[q]] = wq;
          i = std::min(i, sMap[q] - 1);
        }
      } else if (_segC[q] >= c0 && _segC[q] <= c1 && _segR[q] >= r0 && _segR[q] <= r1) {
        sIds[n] = q; sWts[n] = wq; sDone[q] = sId; sMap[q] = n++;
      }
    }
  }
  // finally remove segments connected to boundaries
  for (i = 0; i < n; ++i) {
    k = sIds[i];
    if (_segC[k] >= c0 && _segC[k] <= c1 && _segR[k] >= r0 && _segR[k] <= r1) {
      score -= sWts[i] * _segMag[k];
    }
  }
  score *= norm;
  if (score < _minScore) score = 0;
}

void EdgeBoxes::refineBox(Box &box) {
  int rStep = int(box.h * _rcStepRatio);
  int cStep = int(box.w * _rcStepRatio);
  Box B;
  while (true) {
    // prepare for iteration
    rStep >>= 1;
    cStep >>= 1;
    if (rStep <= 2 && cStep <= 2) break;
    rStep = std::max(1, rStep);
    cStep = std::max(1, cStep);
    // search over r start
    B = box; B.r -= rStep; B.h += rStep; scoreBox(B);
    if (B.s <= box.s) {
      B=box;
      B.r += rStep; B.h -= rStep; scoreBox(B);
    }
    if (B.s > box.s) box = B;
    // search over r end
    B = box; B.h += rStep; scoreBox(B);
    if (B.s <= box.s) {
      B=box;
      B.h -= rStep; scoreBox(B);
    }
    if (B.s > box.s) box = B;
    // search over c start
    B = box; B.c -= cStep; B.w += cStep; scoreBox(B);
    if (B.s <= box.s) {
      B = box;
      B.c += cStep; B.w -= cStep; scoreBox(B);
    }
    if (B.s > box.s) box = B;
    // search over c end
    B = box; B.w += cStep; scoreBox(B);
    if (B.s <= box.s) {
      B=box;
      B.w -= cStep; scoreBox(B);
    }
    if (B.s > box.s) box = B;
  }
}

void EdgeBoxes::drawBox(Box &box, CellArray &E, arrayf &V) {
  if (V.data == NULL) return;
  int i, c, r;
  float e, o;
  int sId = _sId;
  scoreBox(box);
  int r1= clamp(box.r+box.h,0,h-1);
  int r0= box.r = clamp(box.r,0,h-1);
  int c1= clamp(box.c+box.w,0,w-1);
  int c0= box.c = clamp(box.c,0,w-1);
  for( c=0; c<w; c++ ) for( r=0; r<h; r++ )
    V.at(c+w*0,r)=V.at(c+w*1,r)=V.at(c+w*2,r)=1;
  for( c=0; c<w; c++ ) for( r=0; r<h; r++ ) {
    i=_segIds.at(c,r); if(i<=0) continue; e = E.at(r,c);
    o = (_sDone.data[i]==sId) ? _sWts.data[_sMap.data[i]] :
      (_segC[i]>=c0 && _segC[i]<=c1 && _segR[i]>=r0 && _segR[i]<=r1 ) ? 0 : 1;
    V.at(c+w*0,r)=1-e+e*o; V.at(c+w*1,r)=1-e*o; V.at(c+w*2,r)=1-e;
  }
  // finally draw bounding box
  r=r0; for(c=c0; c<=c1; c++) V.at(c+w*0,r)=V.at(c+w*1,r)=V.at(c+w*2,r)=0;
  r=r1; for(c=c0; c<=c1; c++) V.at(c+w*0,r)=V.at(c+w*1,r)=V.at(c+w*2,r)=0;
  c=c0; for(r=r0; r<=r1; r++) V.at(c+w*0,r)=V.at(c+w*1,r)=V.at(c+w*2,r)=0;
  c=c1; for(r=r0; r<=r1; r++) V.at(c+w*0,r)=V.at(c+w*1,r)=V.at(c+w*2,r)=0;
}

void boxesNms(Boxes &boxes, float thr, float eta, int maxBoxes) {
  // TODO: remove this line, already sorted outside
  sort(boxes.rbegin(), boxes.rend());
  if (thr > .99) return;
  const int nBin = 10000;
  const float step = 1 / thr, lstep = log(step);

  std::vector<Boxes> kept(nBin + 1);
  int i = 0, j, k, n = (int)boxes.size(), m = 0, b, d = 1;
  for (; i < n && m < maxBoxes; ++i) {
    b = boxes[i].w * boxes[i].h;
    b = clamp(int(ceil(log(float(b)) / lstep)), d, nBin - d);
    bool keep = true;
    for (j = b - d; j <= b + d; ++j) {
      for (k = 0; k < kept[j].size(); ++k) if (keep) {
        keep = boxes[i].overlap(kept[j][k]) <= thr;
      }
    }
    if (keep) {
      kept[b].push_back(boxes[i]);
      ++m;
    }
    if (keep && eta < 1 && thr > .5) {
      thr *= eta;
      d = ceil(log(1 / thr) / lstep);
    }
  }
  boxes.resize(m); m = 0;
  for (j = 0; j < nBin; ++j) for (k = 0; k < kept[j].size(); ++k) {
    boxes[m++] = kept[j][k];
  }
  sort(boxes.rbegin(), boxes.rend());
}
