#include "imPad.h"
#include "wrappers.h"
#include "global.h"
#include <cstring>

void imPad(CellArray& input, CellArray& output, std::vector<int> &pad, const std::string &type, double val) {
  int ns[3] = {input.rows, input.cols, input.channels}, ms[3];

  // extract padding amounts
  int pt, pb, pl, pr;
  if (pad.size() == 1) pt = pb = pl = pr = pad[0];
  else if (pad.size() == 2) pt = pb = pad[0], pl = pr = pad[1];
  else if (pad.size() == 4) pt = pad[0], pb = pad[1], pl = pad[2], pr = pad[3];
  else wrError("Input pad must have 1, 2, or 4 values.");

  // figure out padding type (flag and val)
  int flag = 0;
  if (type == "") flag = 0;
  else if (type == "replicate") flag = 1;
  else if (type == "symmetric") flag = 2;
  else if (type == "circular") flag = 3;
  else wrError("Invalid pad value");
  if (ns[0] == 0 || ns[1] == 0) flag = 0;

  // create output array
  ms[0] = ns[0] + pt + pb; ms[1] = ns[1] + pl + pr; ms[2] = ns[2];
  if (ms[0] < 0 || ns[0] <= -pt || ns[0] <= -pb ) ms[0] = 0;
  if (ms[1] < 0 || ns[1] <= -pl || ns[1] <= -pr ) ms[1] = 0;
  output.create(ms[0], ms[1], ms[2]);

  // pad array
  float *A = input.data, *B = output.data;
  imPad(A, B, ns[0], ns[1], ns[2], pt, pb, pl, pr, flag, float(val));
}

CellArray imPad(CellArray& input, std::vector<int> &pad, const std::string &type, double val) {
  CellArray output;
  imPad(input, output, pad, type, val);
  return output;
}
