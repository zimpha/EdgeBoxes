#ifndef _GRADIENT_UTIL_H
#define _GRADIENT_UTIL_H

#include "CellArray.h"

/**
 * Compute numerical gradients along x and y directions.
 *
 * @param I     [hxwxk] input k channel single image
 * @param Gx    [hxwxk] x-gradient (horizontal)
 * @param Gy    [hxwxk] y-gradient (vertical)
 */
void gradient(CellArray& I, CellArray& Gx, CellArray& Gy);

/**
 * Compute gradient magnitude and orientation at each image location.
 *
 * If input image has k>1 channels and channel=0, keeps gradient with
 * maximum magnitude (over all channels) at each location. Otherwise if
 * channel is between 1 and k computes gradient for the given channel.
 * If full==1 orientation is computed in [0,2*pi) else it is in [0,pi).
 *
 * If normRad>0, normalization is performed by first computing S, a smoothed
 * version of the gradient magnitude, then setting: M = M./(S + normConst).
 * S is computed by S = convTri( M, normRad ).
 *
 * @param I         [hxwxk] input k channel single image
 * @param M         [hxw] output gradient magnitude at each location
 * @param O         [hxw] output approximate gradient orientation modulo PI
 * @param channel   [0] if>0 color channel to use for gradient computation
 * @param normRad   [0] normalization radius (no normalization if 0)
 * @param normConst [.005] normalization constant
 * @param full      [0] if true compute angles in [0,2*pi) else in [0,pi)
 */
void gradientMag(CellArray& I, CellArray& M, CellArray& O, int channel = 0, float normRad = 0.f, float normConst = .005f, bool full = false);

/**
 * Compute oriented gradient histograms.
 *
 * For each binSize x binSize region in an image I, computes a histogram of
 * gradients, with each gradient quantized by its angle and weighed by its
 * magnitude. If I has dimensions [hxw], the size of the computed feature
 * vector H is floor([h/binSize w/binSize nOrients]).
 *
 * @param M        [hxw] input gradient magnitude at each location (see gradientMag.m)
 * @param O        [hxw] input gradient orientation in range defined by param flag
 * @param H        [w/binSize x h/binSize x nOrients] output gradient histograms
 * @param binSize  [8] spatial bin size
 * @param nOrients [9] number of orientation bins
 * @param softBin  [1] set soft binning (odd: spatial=soft, >=0: orient=soft)
 * @param useHog   [0] 1: compute HOG, 2: compute FHOG
 * @param clipHog  [.2] value at which to clip hog histogram bins
 * @param full     [false] if true expects angles in [0,2*pi) else in [0,pi)
 */
void gradientHist(CellArray& M, CellArray &O, CellArray& H, int binSize = 8, int nOrients = 9, int softBin = 1, int useHog = 0, float clipHog = 0.2f, bool full = false);

#endif
