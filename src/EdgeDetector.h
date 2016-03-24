#ifndef _EDGE_DETECTOR_H
#define _EDGE_DETECTOR_H

#include <string>
#include <opencv2/opencv.hpp>

struct DetectParam {
  // (1) model parameters:
  int imWidth;    // [32] width of image patches
  int gtWidth;    // [16] width of ground truth patches

  // (2) tree parameters:
  int nPositive;  // [5e5] number of positive patches per tree
  int nNegetive;  // [5e5] number of negative patches per tree
  int nImgs;      // [inf] maximum number of images to use for training
  int nTrees;     // [8] number of trees in forest to train
  float fracFtrs; // [1/4] fraction of features to use to train each tree
  int minCount;   // [1] minimum number of data points to allow split
  int minChild;   // [8] minimum number of data points allowed at child nodes
  int maxDepth;   // [64] maximum depth of tree
  std::string discretize; // ['pca'] options include 'pca' and 'kmeans'
  int nSamples;   // [256] number of samples for clustering structured labels
  int nClasses;   // [2] number of classes (clusters) for binary splits
  std::string split;      // ['gini'] options include 'gini', 'entropy' and 'twoing'

  // (3) feature parameters:
  int nOrients;   // [4] number of orientations per gradient scale
  float grdSmooth;// [0] radius for image gradient smoothing (using convTri)
  float chnSmooth;// [2] radius for reg channel smoothing (using convTri)
  float simSmooth;// [8] radius for sim channel smoothing (using convTri)
  float normRad;  // [4] gradient normalization radius (see gradientMag)
  int shrink;     // [2] amount to shrink channels
  int nCells;     // [5] number of self similarity cells
  int rgbd;       // [0] 0:RGB, 1:depth, 2:RBG+depth (for NYU data only)

  // (4) detection parameters (can be altered after training):
  int stride;     // [2] stride at which to compute edges
  bool multiscale;// [0] if true run multiscale edge detector
  int sharpen;    // [2] sharpening amount (can only decrease after training)
  int nTreesEval; // [4] number of trees to evaluate per location
  int nThreads;   // [4] number of threads for evaluation of trees
  bool nms;       // [0] if true apply non-maximum suppression to edges

  // (5) other parameters:
  int seed;       // [1] seed for random stream (for reproducibility)
  bool useParfor; // [0] if true train trees in parallel (memory intensive)
  std::string modelDir;   // ['models/'] target directory for storing models
  std::string modelFnm;   // ['model'] model filename
  std::string bsdsDir;    // ['BSR/BSDS500/data/'] location of BSDS dataset
  int nChns;      // [13] number of channels of the feature
};

class EdgeDetector {
public:
  EdgeDetector();
  ~EdgeDetector();

  /**
   * Compute features for structured edge detection.
   *
   * @param I       [h x w x 3] color input image
   * @param chnsReg [h x w x nChannel] regular output channels
   * @param chnsSim [h x w x nChannel] self-similarity output channels
   * @param opts    structured edge model options
   */
  void featureExtract(cv::Mat &I, cv::Mat &chnsReg, cv::Mat &chnsSim, DetectParam &opts);

  /**
  * Detect edges in image.
  *
  * @param I    [h x w x 3] color input image
  * @param E    [h x w] edge probability map
  * @param O    [h x w] coarse edge normal orientation (0=left, pi/2=up)
  * @param opts structured edge model options
  */
  void edgesDetect(cv::Mat &I, cv::Mat &E, cv::Mat &O, DetectParam &opts);
};

#endif
