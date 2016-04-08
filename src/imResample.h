#ifndef _IMRESAMPLE_H
#define _IMRESAMPLE_H

#include "imResampleMex.h"
#include "CellArray.h"
#include <string>
#include <opencv2/opencv.hpp>

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

#endif
