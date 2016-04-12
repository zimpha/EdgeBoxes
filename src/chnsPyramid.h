#ifndef _CHNS_PYRAMID_H
#define _CHNS_PYRAMID_H

#include "chnsCompute.h"

#include <utility>

typedef std::pair<int, int> PII;

struct PyramidInput {
  ChnsInput chnsInput;        // parameters for creating channels (see chnsCompute.h)
  int nPerOct;                // [8] number of scales per octave
  int nOctUp;                 // [0] number of upsampled octaves to compute
  int nApprox;                // [-1] number of approx. scales (if -1 nApprox=nPerOct-1)
  std::vector<float> lambdas; // [] coefficients for power law scaling (see BMVC10)
  int pad[2];                 // [0 0] amount to pad channels (along T/B and L/R)
  float minDs[2];             // [16 16] minimum image size for channel computation
  float smooth;               // [1] radius for channel smoothing (using convTri)
  bool complete;              // [] if true does not check/set default vals in pPyramid

  PyramidInput():chnsInput(), nPerOct(8), nOctUp(0), nApprox(7), lambdas(), smooth(1) {
    pad[0] = pad[1] = 0;
    pad[0] = int(round(1.0 * pad[0] / chnsInput.shrink)) * chnsInput.shrink;
    pad[0] = int(round(1.0 * pad[1] / chnsInput.shrink)) * chnsInput.shrink;
    minDs[0] = minDs[1] = 16;
    minDs[0] = std::min(minDs[0], chnsInput.shrink * 4.f);
    minDs[1] = std::min(minDs[1], chnsInput.shrink * 4.f);
    complete = true;
  }
};

struct PyramidOutput {
  int nTypes;                             // number of channel types
  int nScales;                            // number of scales computed
  std::vector<CellArray> data;            // [nScales x 1] cell array of concated computed channels
  std::vector<ChnsOutput::Info> infos;    // [nTypes x 1] struct array (mirrored from chnsCompute)
  std::vector<float> lambdas;             // [nTypes x 1] scaling coefficients actually used
  float* scales;                          // [nScales x 1] relative scales (approximate)
  float* scaleshw;                        // [nScales x 2] exact scales for resampling h and w
};

void chnsPyramid(uint8_t *image, int h, int w, int d, PyramidInput &pyramidInput, PyramidOutput &pyramidOutput);

#endif
