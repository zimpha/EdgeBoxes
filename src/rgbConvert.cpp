#include "rgbConvert.h"
#include <cstdlib>
#include <cmath>
#include <typeinfo>

template<class iT, class oT>
void rgbConvert(iT* I, oT* O, int n, int d, int flag, oT nrm) {
  int i, n1=d*(n<1000?n/10:100); oT thr = oT(1.001);
  if(flag>1 && nrm==1) for(i=0; i<n1; i++) if(I[i]>thr)
    wrError("For floats all values in I must be smaller than 1.");
  bool useSse = n%4==0 && typeid(oT)==typeid(float);
  if( flag==2 && useSse )
    for(i=0; i<d/3; i++) rgb2luv_sse(I+i*n*3,(float*)(O+i*n*3),n,(float)nrm);
  else if( (flag==0 && d==1) || flag==1 ) normalize(I,O,n*d,nrm);
  else if( flag==0 ) for(i=0; i<d/3; i++) rgb2gray(I+i*n*3,O+i*n*1,n,nrm);
  else if( flag==2 ) for(i=0; i<d/3; i++) rgb2luv(I+i*n*3,O+i*n*3,n,nrm);
  else if( flag==3 ) for(i=0; i<d/3; i++) rgb2hsv(I+i*n*3,O+i*n*3,n,nrm);
  else wrError("Unknown flag.");
}

void rgbConvert(CellArray& input, CellArray& output, const int colorSpace) {
  // TODO: check input matrix

  // get flag
  int flag = -1;
  if (colorSpace == CS_GRAY) flag = 0;
  else if (colorSpace == CS_RGB) flag = 1;
  else if (colorSpace == CS_LUV) flag = 2;
  else if (colorSpace == CS_HSV) flag = 3;
  else if (colorSpace == CS_ORIG) flag = 4;
  else wrError("unknown color space");

  int inputChannel = input.channels;
  int n = input.rows * input.cols;

  // deal some special convert
  if (flag == 4) flag = 1;
  bool norm = (inputChannel == 1 && flag == 0) || flag == 1;
  if (norm) {
    output = input;
    return;
  }

  // alloc output data
  int outputChannel = (flag == 0) ? (inputChannel == 1 ? 1 : inputChannel / 3) : inputChannel;

  float* I = input.data;
  output.release();
  output.rows = input.rows;
  output.cols = input.cols;
  output.channels = outputChannel;
  if (!((inputChannel == 1 && flag == 0) || flag == 1 || inputChannel % 3 == 0)) {
    wrError("Input image must have third dimension d==1 or (d/3)*3==d.");
  }
  output.data = rgbConvert(I, n, inputChannel, flag, 1.0f);
}

CellArray rgbConvert(CellArray& input, const int colorSpace) {
  CellArray output;
  rgbConvert(input, output, colorSpace);
  return output;
}
