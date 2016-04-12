#include "bbNms.h"
#include "wrappers.h"

bool cmpByScoreDec(const Box &a, const Box &b) {return a.s > b.s;}

Boxes nmsMax(Boxes &bb, float overlap, bool greedy, bool ovrDnm) {
  // for each i suppress all j st j>i and area-overlap>overlap
  sort(bb.begin(), bb.end(), cmpByScoreDec);
  std::vector<bool> keep(bb.size(), true);
  std::vector<Point> low(bb.size());
  std::vector<Point> upp(bb.size());
  std::vector<int> as(bb.size());
  for (size_t i = 0; i < bb.size(); ++i) {
    low[i] = (Point){bb[i].c, bb[i].r};
    upp[i] = (Point){bb[i].c + bb[i].w, bb[i].r + bb[i].h};
    as[i] = bb[i].w * bb[i].h;
  }
  for (size_t i = 0; i < bb.size(); ++i) {
    if (greedy && !keep[i]) continue;
    for (size_t j = i + 1; j < bb.size(); ++j) if (keep[j]) {
      int iw = std::min(upp[i].c, upp[j].c) - std::max(low[i].c, low[j].c);
      if (iw <= 0) continue;
      int ih = std::min(upp[i].r, upp[j].r) - std::max(low[i].r, low[j].r);
      if (ih <= 0) continue;
      int o = iw * ih, u;
      if (ovrDnm) u = as[i] + as[j] - o;
      else u = std::min(as[i], as[j]);
      if (o > u * overlap) keep[j] = false;
    }
  }
  Boxes res;
  for (size_t i = 0; i < bb.size(); ++i) {
    if (keep[i]) res.push_back(bb[i]);
  }
  return res;
}

Boxes nmsMs(Boxes &bb, float thr, float *radii) {
  // position = [x+w/2,y+h/2,log2(w),log2(h)], ws=weights-thr
  // TODO
  return bb;
}

Boxes nmsCover(Boxes &bb, float overlap, bool ovrDnm) {
  // TODO
  return bb;
}

Boxes bbNms(Boxes &bb, NmsParam &pNms) {
  // discard bbs below threshold and run nms1
  bool ovrDnm = pNms.ovrDnm == "union";
  Boxes res;
  for (size_t i = 0; i < bb.size(); ++i) {
    if (bb[i].s > pNms.thr) res.push_back(bb[i]);
  }
  if (!pNms.resize.empty()) {
    res = bb_resize(res, pNms.resize[0], pNms.resize[1], pNms.resize[2]);
  }
  if (pNms.separate) {
    // TODO
  } else {
    if (pNms.type == "max") res = nmsMax(res, pNms.overlap, 0, ovrDnm);
    else if (pNms.type == "maxg") res = nmsMax(res, pNms.overlap, 1, ovrDnm);
    else if (pNms.type == "ms") res = nmsMs(res, pNms.thr, pNms.radii);
    else if (pNms.type == "cover") res = nmsCover(res, pNms.overlap, ovrDnm);
    else wrError("unknown type: " + pNms.type);
  }
  return res;
}
