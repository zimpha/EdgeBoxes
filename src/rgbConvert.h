#ifndef _RGBCONVERT_H
#define _RGBCONVERT_H

#include "rgbConvertMex.h"
#include "CellArray.h"

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
void rgbConvert(CellArray& input, CellArray& output, const int colorSpace = CS_LUV);
CellArray rgbConvert(CellArray& input, const int colorSpace = CS_LUV);

template<typename T>
float* rgbConvert(T* image, int h, int w, int d, int colorSpace) {
  int flag = colorSpace; if (flag == 4) flag = 1;
  bool norm = (d == 1 && flag == 0) || flag == 1;
  float *J;
  if (norm) {
    J = (float*)wrCalloc(h * w * d, sizeof(float));
    int len = h * w * d;
    if (typeid(T) == typeid(float)) {
      memcpy(J, image, sizeof(float) * len);
    } else {
      normalize(image, J, len, 1.0f / 255);
    }
  } else {
    if (typeid(T) == typeid(float)) {
      J = rgbConvert(image, h * w, d, colorSpace, 1.0f);
    } else {
      J = rgbConvert(image, h * w, d, colorSpace, 1.0f / 255);
    }
  }
  return J;
}

#endif
