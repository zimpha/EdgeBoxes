#include "convUtil.h"
#include "convConstMex.h"

void convTri(CellArray& input, CellArray& output, float r, int s) {
  if (r == 0 && s == 1) {
    output = input;
    return;
  }
  if (s < 1) wrError("Invalid sampling value s");
  if (r < 0) wrError("Invalid radius r");

  int h = input.rows, w = input.cols, d = input.channels;
  output.create(h / s, w / s, d);
  if (r > 0 && r <= 1 && s <= 2) {
    float rnew = 12 / r / (r + 2) - 2;
    convTri1(input.data, output.data, h, w, d, rnew, s);
  } else {
    convTri(input.data, output.data, h, w, d, round(r), s);
  }
}

CellArray convTri(CellArray& input, float r, int s) {
  CellArray output;
  convTri(input, output, r, s);
  return output;
}
