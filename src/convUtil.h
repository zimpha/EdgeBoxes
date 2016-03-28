#ifndef _CONV_UTIL_H
#define _CONV_UTIL_H

#include "CellArray.h"
#include <string>

/**
 * Extremely fast 2D image convolution with a triangle filter.
 *
 * Convolves an image by a 2D triangle filter (the 1D triangle filter f is
 * [1:r r+1 r:-1:1]/(r+1)^2, the 2D version is simply conv2(f,f')). The
 * convolution can be performed in constant time per-pixel, independent of
 * the radius r. In fact the implementation is nearly optimal, with the
 * convolution taking only slightly more time than creating a copy of the
 * input array. Boundary effects are handled as if the image were padded
 * symmetrically prior to performing the convolution. An optional integer
 * downsampling parameter "s" can be specified, in which case the output is
 * downsampled by s (the implementation is efficient with downsampling
 * occurring simultaneously with smoothing, saving additional time).
 *
 * When used as a smoothing filter, the standard deviation (sigma) of a tri
 * filter with radius r can be computed using [sigma=sqrt(r*(r+2)/6)]. For
 * the first few values of r this translates to: r=1: sigma=1/sqrt(2), r=2:
 * sigma=sqrt(4/3), r=3: sqrt(5/2), r=4: sigma=2. Given sigma, the
 * equivalent value of r can be computed via [r=sqrt(6*sigma*sigma+1)-1].
 *
 * @param input  [hxwxk] input k channel single image
 * @param output [hxwxk] output smoothed image
 * @param r      integer filter radius (or any value between 0 and 1)
 *               filter standard deviation is: sigma=sqrt(r*(r+2)/6)
 * @param s      [1] integer downsampling amount after convolving
 */
void convTri(CellArray& input, CellArray& output, float r, int s = 1);
CellArray convTri(CellArray& input, float r, int s = 1);

/**
 * 2D image convolution with a filter
 *
 * @param type   filter type include: 'convBox', 'convTri', 'conv11', 'convTri1', 'convMax'
 * @param input  [hxwxk] input k channel single image
 * @param output [hxwxk] output smoothed image
 * @param r      integer filter radius (or any value between 0 and 1)
 *               filter standard deviation is: sigma=sqrt(r*(r+2)/6)
 * @param s      [1] integer downsampling amount after convolving
 */
void convConst(const std::string &type, CellArray& input, CellArray& output, float r, int s = 1);
CellArray convConst(const std::string &type, CellArray& input, float r, int s = 1);

#endif
