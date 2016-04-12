#include "box.h"
#include <cmath>
#include <cassert>
#include <algorithm>

std::vector<int> bb_area(Boxes &bb) {
  std::vector<int> res(bb.size());
  for (size_t i = 0; i < bb.size(); ++i) {
    res[i] = bb[i].h * bb[i].w;
  }
  return res;
}

Boxes bb_shift(Boxes &bb, int xdel, int ydel) {
  Boxes res(bb);
  for (size_t i = 0; i < bb.size(); ++i) {
    res[i].c -= xdel; res[i].r -= ydel;
  }
  return res;
}

std::vector<Point> bb_center(Boxes &bb) {
  std::vector<Point> res(bb.size());
  for (size_t i = 0; i < bb.size(); ++i) {
    res[i].r = bb[i].r + bb[i].h / 2;
    res[i].c = bb[i].c + bb[i].w / 2;
  }
  return res;
}

Boxes bb_intersect(Boxes &bb1, Boxes &bb2) {
  assert(bb1.size() == bb2.size());
  // TODO: check size
  Boxes res(bb1.size());
  for (size_t i = 0; i < bb1.size(); ++i) {
    res[i].r = std::max(bb1[i].r, bb2[i].r);
    res[i].c = std::max(bb1[i].c, bb2[i].c);
    res[i].h = std::min(bb1[i].r + bb1[i].h, bb2[i].r + bb2[i].h);
    res[i].w = std::min(bb1[i].c + bb1[i].w, bb2[i].c + bb2[i].w);
    res[i].h -= res[i].r; res[i].w -= res[i].c;
    if (res[i].h <= 0 || res[i].w <= 0) res[i].h = res[i].w = 0;
  }
  return res;
}

Boxes bb_union(Boxes &bb1, Boxes &bb2) {
  assert(bb1.size() == bb2.size());
  // TODO: check size
  Boxes res(bb1.size());
  for (size_t i = 0; i < bb1.size(); ++i) {
    res[i].r = std::min(bb1[i].r, bb2[i].r);
    res[i].c = std::min(bb1[i].c, bb2[i].c);
    res[i].h = std::max(bb1[i].r + bb1[i].h, bb2[i].r + bb2[i].h);
    res[i].w = std::max(bb1[i].c + bb1[i].w, bb2[i].c + bb2[i].w);
    res[i].h -= res[i].r; res[i].w -= res[i].c;
  }
  return res;
}

Boxes bb_resize(Boxes &bb, float hr, float wr, float ar) {
  Boxes res(bb);
  if (hr == 0 && wr == 0) {
    ar = sqrt(ar);
    for (size_t i = 0; i < res.size(); ++i) {
      float a = sqrt(res[i].h * res[i].w), d;
      d = a * ar - res[i].w; res[i].c -= d / 2; res[i].w += d;
      d = a / ar - res[i].h; res[i].r -= d / 2; res[i].h += d;
    }
  } else {
    for (size_t i = 0; i < res.size(); ++i) {
      if (hr != 0) {
        float d = (hr - 1) * res[i].h;
        res[i].r -= d / 2; res[i].h += d;
      }
      if (wr != 0) {
        float d = (wr - 1) * res[i].w;
        res[i].c -= d / 2; res[i].w += d;
      }
      if (hr == 0) {
        float d = res[i].w / ar - res[i].h;
        res[i].r -= d / 2; res[i].h += d;
      }
      if (wr == 0) {
        float d = res[i].h * ar - res[i].w;
        res[i].c -= d / 2; res[i].w += d;
      }
    }
  }
  return res;
}

Boxes bb_squarify(Boxes &bb, int flag, float ar) {
  if (flag == 4) return bb_resize(bb, 0, 0, ar);
  Boxes res(bb);
  for (size_t i = 0; i < bb.size(); ++i) {
    bool usew = (flag == 0 && bb[i].w > bb[i].h * ar) ||
      (flag == 1 && bb[i].w < bb[i].h * ar) || flag == 2;
    Boxes p(res.begin() + i, res.begin() + i + 1);
    if (usew) res[i] = bb_resize(p, 0, 1, ar)[0];
    else res[i] = bb_resize(p, 1, 0, ar)[0];
  }
  return res;
}
