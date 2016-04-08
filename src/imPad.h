#ifndef _IMPAD_H
#define _IMPAD_H

#include "imPadMex.h"
#include "CellArray.h"
#include <string>

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

template<typename T>
T* imPad(T* I, int h, int w, int d, std::vector<int> &pad, const std::string &type = "", double val = 0) {
  int ns[3] = {h, w, d}, ms[3];
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
  T *J = (T*)wrCalloc(ms[0] * ms[1] * ms[2], sizeof(T));

  imPad(I, J, ns[0], ns[1], ns[2], pt, pb, pl, pr, flag, T(val));
  return J;
}

#endif
