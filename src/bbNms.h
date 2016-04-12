#ifndef _BB_NMS_H
#define _BB_NMS_H

#include "box.h"
#include <limits>
#include <vector>
#include <string>

struct NmsParam {
  std::string type;           // ['max'] 'max', 'maxg', 'ms', 'cover', or 'none'
  float thr;                  // [-inf] threshold below which to discard (0 for 'ms')
  int maxn;                   // [inf] if n>maxn split and run recursively (see above)
  float radii[4];             // [.15 .15 1 1] supression radii ('ms' only, see above)
  float overlap;              // [.5] area of overlap for bbs
  std::string ovrDnm;         // ['union'] area of overlap denominator ('union' or 'min')
  std::vector<float> resize;  // {} parameters for bbApply('resize')
  bool separate;              // [0] run nms separately on each bb type (bbType)

  NmsParam():type("max"), overlap(.5f), ovrDnm("union"), resize(), separate(false) {
    thr = std::numeric_limits<float>::min();
    maxn = std::numeric_limits<int>::max();
    radii[0] = radii[1] = .15f;
    radii[2] = radii[3] = 1.f;
  }
};

Boxes bbNms(Boxes &bb, NmsParam &pNms);

#endif
