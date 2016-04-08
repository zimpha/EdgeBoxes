#ifndef _EDGE_DETECTOR_H
#define _EDGE_DETECTOR_H

#include "CellArray.h"
#include <string>
#include <opencv2/opencv.hpp>

class EdgeDetector {
public:
  EdgeDetector();
  ~EdgeDetector();

  /**
   * load structured forest model
   *
   * @param path path to model file
   */
  void loadModel(const std::string &path);

  /**
   * Compute features for structured edge detection.
   *
   * @param I       [h x w x 3] color input image
   * @param chnsReg [h x w x nChannel] regular output channels
   * @param chnsSim [h x w x nChannel] self-similarity output channels
   */
  void featureExtract(uint8_t *I, int h, int w, int d, CellArray &chnsReg, CellArray &chnsSim);

  /**
  * Detect edges in image.
  *
  * @param I    [h x w x 3] color input image
  * @param E    [h x w] edge probability map
  * @param O    [h x w] coarse edge normal orientation (0=left, pi/2=up)
  */
  void edgesDetect(uint8_t *I, int h, int w, int d, CellArray &E, CellArray &O);

private:
  // (1) model parameters
  int imWidth;    // width of image patches
  int gtWidth;    // width of ground truth patches

  // (2) tree parameters:
  int nTrees;     // number of trees in forest to train
  int nTreeNodes; // maximum number of tree nodes in each tree

  // (3) feature parameters
  int nOrients;   // number of orientations per gradient scale
  float grdSmooth;// radius for image gradient smoothing (using convTri)
  float chnSmooth;// radius for reg channel smoothing (using convTri)
  float simSmooth;// radius for sim channel smoothing (using convTri)
  float normRad;  // gradient normalization radius (see gradientMag)
  int shrink;     // amount to shrink channels
  int nCells;     // number of self similarity cells
  int rgbd;       // 0:RGB, 1:depth, 2:RBG+depth (for NYU data only)

  // (4) detection parameters (can be altered after training)
  int stride;     // stride at which to compute edges
  int multiscale; // if true run multiscale edge detector
  int sharpen;    // sharpening amount (can only decrease after training)
  int nTreesEval; // number of trees to evaluate per location
  int nThreads;   // number of threads for evaluation of trees
  bool nms;       // if true apply non-maximum suppression to edges

  // (5) other parameters:
  int nChns;      // number of channels of the feature
  int nChnFtrs;
  int nSimFtrs;
  int eBinsSize;  // number of elements in eBins
  int eBndsSize;  // number of elements in eBnds
  int nBnds;

  float* thrs;        // [nTreeNodes x nTrees]
  uint32_t *fids;     // [nTreeNodes x nTrees]
  uint32_t *child;    // [nTreeNodes x nTrees]
  uint8_t *segs;      // [gtWidth x gtWidth x nTreeNodes x nTrees]
  uint8_t *nSegs;     // [nTreeNodes x nTrees]
  uint16_t *eBins;
  uint32_t *eBnds;

  void clear();
  uint32_t* buildLookup(int *dims, int w);
  void buildLookupSs(uint32_t *&cids1, uint32_t *&cids2, int *dims, int w, int m);

  void edgeNms(CellArray &E, CellArray &O, int r, int s, float m, int nThreads);
  void detect(CellArray &I, CellArray &E, CellArray &O, CellArray &chnsReg, CellArray &chnsSim, int oRows, int oCols);
};

#endif
