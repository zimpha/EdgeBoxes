#ifndef _ACF_DETECTOR_H
#define _ACF_DETECTOR_H

#include "chnsPyramid.h"
#include "bbNms.h"
#include "wrappers.h"

class ACFDetector {
public:
  ACFDetector();
  void loadModel(const std::string &filepath);
  Boxes acfDetect(uint8_t *I, int h, int w, int d);

private:
  // model parameters
  PyramidInput pPyramid;// [{}] params for creating pyramid (see chnsPyramid.h)
  NmsParam pNms;        // [..] params for non-maximal suppression (see bbNms.h)
  CellArray filters;    // [] [wxwxnChnsxnFilter] filters or [wFilter,nFilter]
  uint32_t stride;           // [4] spatial stride between detection windows
  float cascThr;        // [-1] constant cascade threshold (affects speed/accuracy)
  float cascCal;        // [.005] cascade calibration (affects speed/accuracy)
  float modelDs[2];     // model height+width without padding (eg [100 41])
  float modelDsPad[2];  // model height+width with padding (eg [128 64])

  struct Forest {
    uint32_t *fids;     // [K x nWeak] feature ids for each node
    uint32_t *child;    // [K x nWeak] index of child for each node (1-indexed)
    uint32_t *depth;    // [K x nWeak] depth of each node
    float *thrs;        // [K x nWeak] threshold corresponding to each fid
    float *hs;          // [K x nWeak] log ratio (.5*log(p/(1-p)) at each node
    float *weights;     // [K x nWeak] total sample weight at each node
    uint32_t treeDepth; // depth of all leaf nodes (or 0 if leaf depth varies)
    uint32_t nTrees, nTreeNodes;
    Forest(): treeDepth(0) {
      fids = child = depth = NULL;
      thrs = hs = weights = NULL;
    }
    ~Forest() {
      clear();
    }
    void clear() {
      if (fids != NULL) delete[] fids; fids = NULL;
      if (child != NULL) delete[] child; child = NULL;
      if (depth != NULL) delete[] depth; depth = NULL;
      if (thrs != NULL) delete[] thrs; thrs = NULL;
      if (hs != NULL) delete[] hs; hs = NULL;
      if (weights != NULL) delete[] weights; weights = NULL;
    }
  };
  Forest clf;

  void getChild(float* chns, uint32_t *cids, uint32_t *fids, float *thrs, uint32_t offset, uint32_t &k0, uint32_t &k);
  Boxes detect(CellArray &chns_data, int shrink, int modelHt, int modelWd, int stride, float cascThr);
};

inline void ACFDetector::getChild(float* chns, uint32_t *cids, uint32_t *fids, float *thrs, uint32_t offset, uint32_t &k0, uint32_t &k) {
  float ftr = chns[cids[fids[k]]];
  k = (ftr < thrs[k]) ? 1 : 2;
  k0 = k += k0 * 2; k += offset;
}

#endif
