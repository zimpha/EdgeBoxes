#ifndef _BOX_H
#define _BOX_H

#include <vector>
#include <algorithm>

struct Point {
  int r, c;
};

struct Box {
  int c, r, w, h;
  float s;
  int bbType; // optional
  Box() {}
  Box(int _c, int _r, int _w, int _h, float _s = 0.f): c(_c), r(_r), w(_w), h(_h), s(_s) {}
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

/**
 * Compute area of bbs.
 *
 * @param  bb [nx4] original bbs
 * @return    [nx1] area of each bb
 */
std::vector<int> bb_area(Boxes &bb);

/**
 * Shift center of bbs.
 *
 * @param  bb   [nx4] original bbs
 * @param  xdel amount to shift x coord of each bb left
 * @param  ydel amount to shift y coord of each bb up
 * @return      [nx4] shifted bbs
 */
Boxes bb_shift(Boxes &bb, int xdel, int ydel);

/**
 * Compute area of bbs.
 *
 * @param  bb [nx4] original bbs
 * @return    [nx2] centers of bbs
 */
std::vector<Point> bb_center(Boxes &bb);

/**
 * Get bb at intersection of bb1 and bb2 (may be empty).
 *
 * @param  bb1 [nx4] first set of bbs
 * @param  bb2 [nx4] second set of bbs
 * @return     [nx4] intersection of bbs
 */
Boxes bb_intersect(Boxes &bb1, Boxes &bb2);

/**
 * Get bb that is union of bb1 and bb2 (smallest bb containing both).
 *
 * @param  bb1 [nx4] first set of bbs
 * @param  bb2 [nx4] second set of bbs
 * @return     [nx4] intersection of bbs
 */
Boxes bb_union(Boxes &bb1, Boxes &bb2);

/**
 * Resize the bbs (without moving their centers).
 *
 * If wr>0 or hr>0, the w/h of each bb is adjusted in the following order:\
 *  if(hr~=0), h=h*hr; end
 *  if(wr~=0), w=w*wr; end
 *  if(hr==0), h=w/ar; end
 *  if(wr==0), w=h*ar; end
 * Only one of hr/wr may be set to 0, and then only if ar>0. If, however,
 * hr=wr=0 and ar>0 then resizes bbs such that areas and centers are
 * preserved but aspect ratio becomes ar.
 *
 * @param  bb [nx4] original bbs
 * @param  hr ratio by which to multiply height (or 0)
 * @param  wr ratio by which to multiply width (or 0)
 * @param  ar [0] target aspect ratio (used only if hr=0 or wr=0)
 * @return    [nx4] the output resized bbs
 */
Boxes bb_resize(Boxes &bb, float hr, float wr, float ar);

/**
 * Fix bb aspect ratios (without moving the bb centers).
 * The w or h of each bb is adjusted so that w/h=ar.
 * The parameter flag controls whether w or h should change:
 *  flag==0: expand bb to given ar
 *  flag==1: shrink bb to given ar
 *  flag==2: use original w, alter h
 *  flag==3: use original h, alter w
 *  flag==4: preserve area, alter w and h
 * If ar==1 (the default), always converts bb to a square, hence the name.
 *
 * @param  bb   [nx4] original bbs
 * @param  flag controls whether w or h should change
 * @param  ar   [1] desired aspect ratio
 * @return      the output 'squarified' bbs
 */
Boxes bb_squarify(Boxes &bb, int flag, float ar = 1.0);

#endif
