#ifndef _CHNS_COMPUTE_H
#define _CHNS_COMPUTE_H

#include "global.h"
#include "CellArray.h"

#include <vector>
#include <string>

struct ColorParam {
  bool enabled;    // [1] if true enable color channels
  float smooth;   // [1] radius for image smoothing (using convTri)
  int colorSpace; // [CS_LUV] choices are: CS_GRAY, CS_RGB, CS_HSV, CS_ORIG
};

struct GradMagParam {
  bool enabled;     // [1] if true enable gradient magnitude channel
  int colorChannel; // [0] if>0 color channel to use for grad computation
  float normRad;    // [5] normalization radius for gradient
  float normConst;  // [.005] normalization constant for gradient
  bool full;        // [0] if true compute angles in [0,2*pi) else in [0,pi)
};

struct GradHistParam {
  bool enabled;     // [1] if true enable gradient histogram channels
  int binSize;      // [shrink] spatial bin size (defaults to shrink)
  int nOrients;     // [6] number of orientation channels
  bool softBin;     // [0] if true use "soft" bilinear spatial binning
  bool useHog;      // [0] if true perform 4-way hog normalization/clipping
  float clipHog;    // [.2] value at which to clip hog histogram bins
};

struct ChnsInput {
  int shrink;       // [4] integer downsampling amount for channels
  ColorParam pColor;
  GradMagParam pGradMag;
  GradHistParam pGradHist;
  // TODO: add custom channels
  bool complete;     // [] if true does not check/set default vals in pChns
  ChnsInput(): shrink(4), complete(true) {
    pColor = (ColorParam){true, 1.f, CS_LUV};
    pGradMag = (GradMagParam){true, 0, 5.f, .005f, false};
    pGradHist = (GradHistParam){true, shrink, 6, false, false, .2f};
  }
};

struct ChnsOutput {
  int nTypes;                   // [3] number of channel types
  std::vector<CellArray> data;  // [nTypes x 1] cell [h/shrink x w/shrink x nChns] channels
  struct Info {
    std::string name; // channel type name
    int nChannels;    // number of channels for given channel type
    int padwith;      // how channel should be padded (0,'replicate')
  };
  std::vector<Info> infos;
};

void chnsCompute(CellArray &I, ChnsInput &chnsInput, ChnsOutput &chnsOutput);

#endif
