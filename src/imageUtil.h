#ifndef _IMAGE_UTIL_H
#define _IMAGE_UTIL_H

#include "global.h"
#include "CellArray.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

/**
 * Convert RGB image to other color spaces (highly optimized).
 *
 * If colorSpace=='gray' transforms I to grayscale. The output is within
 * numerical error of Matlab's rgb2gray, except ~10x faster. The output in
 * this case is hxwx1, and while the input must be hxwx3 for all other
 * cases, the input for this case can also be hxwx1 (normalization only).
 *
 * If colorSpace=='hsv' transforms I to the HSV color space. The output is
 * within numerical error of Matlab's rgb2hsv, except ~15x faster.
 *
 * If colorSpace=='rgb' or colorSpace='orig' only normalizes I to be in the
 * range [0,1]. In this case both the input and output may have an arbitrary
 * number of channels (that is I may be [hxwxd] for any d).
 *
 * If colorSpace=='luv' transforms I to the LUV color space. The LUV color
 * space is "perceptually uniform" (meaning that two colors equally distant
 * in the color space according to the Euclidean metric are equally distant
 * perceptually). The L,u,v channels correspond roughly to luminance,
 * green-red, blue-yellow. For more information see:
 *   http://en.wikipedia.org/wiki/CIELUV - using this color spaces
 *   http://en.wikipedia.org/wiki/CIELAB - more info about color spaces
 * The LUV channels are normalized to fall in ~[0,1]. Without normalization
 * the ranges are L~[0,100], u~[-88,182], and v~[-134,105] (and typically
 * u,v~[-100,100]). The applied transformation is L=L/270, u=(u+88)/270, and
 * v=(v+134)/270. This results in ranges L~[0,.37], u~[0,1], and v~[0,.89].
 * Perceptual uniformity is maintained since divisor is constant
 * (normalizing each color channel independently would break uniformity).
 *
 * To undo the normalization on an LUV image J use:
 * 	 J=J*270; J(:,:,2)=J(:,:,2)-88; J(:,:,3)=J(:,:,3)-134;
 *
 * @param input      [hxwx3] input rgb image (uint8 or single/double in [0,1])
 * @param output     [hxwx3] single or double output image (normalized to [0,1])
 * @param colorSpace ['luv'] other choices include: 'gray', 'hsv', 'rgb', 'orig'
 * @param useSingle  [true] determines output type (faster if useSingle)
 */
void rgbConvert(CellArray& input, CellArray& output, const int colorSpace = CS_LUV, bool useSingle = true);
CellArray rgbConvert(CellArray& input, const int colorSpace = CS_LUV, bool useSingle = true);

/**
 * Resizes an image.
 * @param input  [hxwxk] input k channel image
 * @param output output image; it has the size dsize (when it is non-zero) or
 *               the size computed from input.size(), fx, and fy;
 *               the type of output is the same as of input.
 * @param dsize  output image size; if it equals zero, it is computed as:
 *               	dsize = Size(round(fx * input.cols), round(fy * input.rows))
 *               Either dsize or both fx and fy must be non-zero.
 * @param fx     scale factor along the horizontal axis; when it equals 0, it is computed as
 *               	(double)dsize.width / input.cols
 * @param fy     scale factor along the vertical axis; when it equals 0, it is computed as
 *               	(double)dsize.height / input.rows;
 * @param method interpolation method:
 *               	+ nearest - a nearest-neighbor interpolation
 *               	+ bilinear - a bilinear interpolation (used by default)
 * @param norm   normalize the output image, defualt is 1
 */
void imResample(CellArray& input, CellArray& output, cv::Size dsize, double fx = 0, double fy = 0, const std::string& method = "bilinear", float norm = 1.0f);
CellArray imResample(CellArray& input, cv::Size dsize, double fx = 0, double fy = 0, const std::string& method = "bilinear", float norm = 1.0f);

/**
 * Pad an image along its four boundaries.
 *
 * Similar to Matlab's padarray, with the following differences:
 *  (1) limited to padding along height and width
 *  (2) input format allows for separate padding along each dimension
 *  (3) padding values may be negative, in which case performs *cropping*
 *  (4) optimized (speedup can be significant, esp. for small arrays)
 *
 * The amount of padding along each of the four boundaries (referred to as
 * T/B/L/R) is determined by the parameter "pad" as follows:
 *  if(numel(pad)==1): T=B=L=R=pad
 *  if(numel(pad)==2): T=B=pad(1), L=R=pad(2)
 *  if(numel(pad)==4): T=pad(1), B=pad(2), L=pad(3), R=pad(4)
 *
 * @param input  [hxwxk] input image (single, double or uint8 array)
 * @param output [T+h+B x L+w+R x k] padded image
 * @param pad    pad or crop amount: 1, 2, or 4 element vector (see above)
 * @param type   pad type, include: 'replicate', 'symmetric', 'circular'
 * @param val    pad value
 */
void imPad(CellArray& input, CellArray& output, std::vector<int> &pad, const std::string &type = "", double val = 0);
CellArray imPad(CellArray& input, std::vector<int> &pad, const std::string &type = "", double val = 0);

#endif
