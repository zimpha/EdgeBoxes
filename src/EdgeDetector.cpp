#include "global.h"
#include "EdgeDetector.h"
#include "convUtil.h"
#include "gradientUtil.h"
#include "imPad.h"
#include "imResample.h"
#include "rgbConvert.h"
#include "wrappers.h"
#include <string>

#ifdef USEOMP
#include <omp.h>
#endif

#define PI 3.14159265f

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// return I[x, y] via bilinear interpolation
inline float interp(float *I, int h, int w, float x, float y) {
  x = x < 0 ? 0 : (x > w - 1.001 ? w - 1.001 : x);
  y = y < 0 ? 0 : (y > h - 1.001 ? h - 1.001 : y);
  int x0 = int(x), y0 = int(y), x1 = x0 + 1, y1 = y0 + 1;
  float dx0 = x - x0, dy0 = y - y0, dx1 = 1 - dx0, dy1 = 1 - dy0;
  return I[x0 * h + y0] * dx1 * dy1 + I[x1 * h + y0] * dx0 * dy1 +
         I[x0 * h + y1] * dx1 * dy0 + I[x1 * h + y1] * dx0 * dy0;
}

EdgeDetector::EdgeDetector() {
}

EdgeDetector::~EdgeDetector() {
  clear();
}

void EdgeDetector::loadModel(const std::string &path) {
  FILE* fp = fopen(path.c_str(), "rb");
  if (fp == NULL) {
    wrError("unable to open model file");
  }
  fread(&imWidth, sizeof(int), 1, fp);
  fread(&gtWidth, sizeof(int), 1, fp);

  fread(&nTrees, sizeof(int), 1, fp);
  fread(&nTreeNodes, sizeof(int), 1, fp);

  fread(&nOrients, sizeof(int), 1, fp);
  fread(&grdSmooth, sizeof(float), 1, fp);
  fread(&chnSmooth, sizeof(float), 1, fp);
  fread(&simSmooth, sizeof(float), 1, fp);
  fread(&normRad, sizeof(float), 1, fp);
  fread(&shrink, sizeof(int), 1, fp);
  fread(&nCells, sizeof(int), 1, fp);
  fread(&rgbd, sizeof(int), 1, fp);

  fread(&stride, sizeof(int), 1, fp);
  fread(&multiscale, sizeof(int), 1, fp);
  fread(&sharpen, sizeof(int), 1, fp);
  fread(&nTreesEval, sizeof(int), 1, fp);
  fread(&nThreads, sizeof(int), 1, fp);
  fread(&nms, sizeof(int), 1, fp);

  fread(&nChns, sizeof(int), 1, fp);
  fread(&nChnFtrs, sizeof(int), 1, fp);
  fread(&nSimFtrs, sizeof(int), 1, fp);

  thrs = new float[nTrees * nTreeNodes];
  fids = new uint32_t[nTrees * nTreeNodes];
  child = new uint32_t[nTrees * nTreeNodes];
  segs = new uint8_t[gtWidth * gtWidth * nTrees * nTreeNodes];
  nSegs = new uint8_t[nTrees * nTreeNodes];
  fread(thrs, sizeof(float), nTrees * nTreeNodes, fp);
  fread(fids, sizeof(uint32_t), nTrees * nTreeNodes, fp);
  fread(child, sizeof(uint32_t), nTrees * nTreeNodes, fp);
  fread(segs, sizeof(uint8_t), gtWidth * gtWidth * nTrees * nTreeNodes, fp);
  fread(nSegs, sizeof(uint8_t), nTrees * nTreeNodes, fp);

  fread(&eBinsSize, sizeof(int), 1, fp);
  eBins = new uint16_t[eBinsSize];
  fread(eBins, sizeof(uint16_t), eBinsSize, fp);

  fread(&eBndsSize, sizeof(int), 1, fp);
  eBnds = new uint32_t[eBndsSize];
  fread(eBnds, sizeof(uint32_t), eBndsSize, fp);

  stride = std::max(stride, shrink);
  nTreesEval = std::min(nTreesEval, nTrees);
  nBnds = (eBndsSize - 1) / (nTrees * nTreeNodes);
  sharpen = std::min(sharpen, nBnds - 1);
  sharpen = 0;
  nTreesEval = 4;
}

void EdgeDetector::featureExtract(uint8_t* I, int h, int w, int d, CellArray &chnsReg, CellArray &chnsSim) {
  if (d != 3) {
    wrError("input image must have 3 channels");
  }
  CellArray luv, M, O, H, I1, I2;
  std::vector<CellArray> chns(nChns);
  luv.rows = h, luv.cols = w, luv.channels = d;
  luv.data = rgbConvert(I, h, w, d, (int)CS_LUV);
  imResample(luv, chns[0], cv::Size(0, 0), 1.0 / shrink, 1.0 / shrink);
  int k = 1;
  for (int i = 1, s = 1; i <= 2; ++i, s <<= 1) {
    if (s == shrink) I1 = chns[0];
    else imResample(luv, I1, cv::Size(0, 0), 1.0 / s, 1.0 / s);
    convTri(I1, I2, grdSmooth);
    gradientMag(I2, M, O, 0, normRad, .01);
    gradientHist(M, O, H, std::max(1, shrink / s), nOrients, 0);
    imResample(M, chns[k++], cv::Size(0, 0), (double)s / shrink, (double)s / shrink);
    imResample(H, chns[k++], cv::Size(0, 0), std::max(1.0, (double)s / shrink), std::max(1.0, (double)s / shrink));
  }
  CellArray tmp;
  mergeCellArray(chns, k, tmp);
  assert(tmp.channels == nChns);
  float chnSm = chnSmooth / shrink;
  float simSm = simSmooth / shrink;

  convTri(tmp, chnsReg, chnSm);
  convTri(tmp, chnsSim, simSm);
}

void EdgeDetector::detect(CellArray &I, CellArray &E, CellArray &O, CellArray &chnsReg, CellArray &chnsSim, int oRows, int oCols) {
  const int h = I.rows, w = I.cols, Z = I.channels;
  const int h1 = (int)ceil(double(h - imWidth) / stride);
  const int w1 = (int)ceil(double(w - imWidth) / stride);
  const int h2 = h1 * stride + gtWidth;
  const int w2 = w1 * stride + gtWidth;
  const int imgDims[3] = {h, w, Z};
  const int chnDims[3] = {h / shrink, w / shrink, nChns};
  const int indDims[3] = {h1, w1, nTreesEval};
  const int outDims[3] = {h2, w2, 1};

  // construct lookup tables
  uint32_t *iids, *eids, *cids, *cids1, *cids2;
  iids = buildLookup((int*)imgDims, gtWidth);
  eids = buildLookup((int*)outDims, gtWidth);
  cids = buildLookup((int*)chnDims, imWidth / shrink);
  buildLookupSs(cids1, cids2, (int*)chnDims, imWidth / shrink, nCells);

  E.create(outDims[0], outDims[1], outDims[2]);
  uint32_t *ind = new uint32_t[indDims[0] * indDims[1] * indDims[2]];
  float* chns = (float*)chnsReg.data;
  float* chnsSs = (float*)chnsSim.data;
  clock_t st = clock();
  #ifdef USEOMP
  nThreads = std::min(nThreads, omp_get_max_threads());
  #pragma omp parallel for num_threads(nThreads)
  #endif
  for (int c = 0; c < w1; ++c) for (int t = 0; t < nTreesEval; ++t) {
    for (int r0 = 0; r0 < 2; ++r0) for (int r = r0; r < h1; r+=2) {
      int o = (r * stride / shrink) + (c * stride / shrink) * h / shrink;
      // select tree to evaluate
      int t1 = ((r + c) % 2 * nTreesEval + t) % nTrees;
      uint32_t k = t1 * nTreeNodes;
      while (child[k]) {
        // compute feature (either channel or self-similarity feature)
        uint32_t f = fids[k];
        float ftr;
        if (f < nChnFtrs) ftr = chns[cids[f] + o];
        else ftr = chnsSs[cids1[f - nChnFtrs] + o] - chnsSs[cids2[f - nChnFtrs] + o];
        // compare ftr to threshold and move left or right accordingly
        if (ftr < thrs[k]) k = child[k] - 1;
        else k = child[k];
        k += t1 * nTreeNodes;
      }
      // store leaf index and update edge maps
      ind[r + c * h1 + t * h1 * w1] = k;
    }
  }
  clock_t ed = clock();
  std::cerr << "time for ind: " << (double)(ed - st) / CLOCKS_PER_SEC << std::endl;
  if (!sharpen) { // compute edge maps (avoiding collisions from parallel executions)
    st = clock();
    for (int c0 = 0; c0 < gtWidth / stride; ++c0) {
      #ifdef USEOMP
      #pragma omp parallel for num_threads(nThreads)
      #endif
      for (int c = c0; c < w1; c += gtWidth / stride) {
        for (int r = 0; r < h1; ++r) for (int t = 0; t < nTreesEval; ++t) {
          uint32_t k = ind[r + c * h1 + t * h1 * w1];
          float *E1 = (float*)E.data + (r * stride) + (c * stride) * h2;
          int b0 = eBnds[k * nBnds], b1 = eBnds[k * nBnds + 1];
          if (b0 == b1) continue;
          for (int b = b0; b < b1; ++b) E1[eids[eBins[b]]]++;
        }
      }
    }
    ed = clock();
    std::cerr << "time for edge maps: " << (double)(ed - st) / CLOCKS_PER_SEC << std::endl;
  } else { // computed sharpened edge maps, snapping to local color values
    const int g = gtWidth;
    uint16_t N[4096 * 4];
    for (int c = 0; c < g; ++c) for (int r = 0; r < g; ++r) {
      int i = c * g + r;
      uint16_t* N1 = N + i * 4;
      N1[0] = c > 0 ? i - g : i; N1[1] = c < g - 1 ? i + g : i;
      N1[2] = r > 0 ? i - 1 : i; N1[3] = r < g - 1 ? i + 1 : i;
    }
    #ifdef USEOMP
    #pragma omp parallel for num_threads(nThreads)
    #endif
    for (int c = 0; c < w1; ++c) for (int r = 0; r < h1; ++r) {
      for (int t = 0; t < nTreesEval; ++t) {
        // get current segment and copy into S
        uint32_t k = ind[r + c * h1 + t * h1 * w1];
        int m = nSegs[k];
        if (m == 1) continue;
        uint8_t S[4096];
        memcpy(S, segs + k * g * g, sizeof(uint8_t) * g * g);
        // compute color model for each segment using every other pixel
        int ci, ri, s, z;
        float ns[100], mus[1000];
        const float *I1 = (float*)I.data + (c * stride + (imWidth - g) / 2) * h + r * stride + (imWidth - g) / 2;
        for (s = 0; s < m; ++s) {
          memset(mus + s * Z, ns[s] = 0, sizeof(float) * Z);
        }
        for (ci = 0; ci < g; ci += 2) for (ri = 0; ri < g; ri += 2) {
          s = S[ci * g + ri]; ++ns[s];
          for (z = 0; z < Z; ++z) mus[s * Z + z] += I1[z * h * w + ci * h + ri];
        }
        for (s = 0; s < m; ++s) for (z = 0; z < Z; ++z) mus[s * Z + z] /= ns[s];
        // update segment S according to local color values
        int b0 = eBnds[k * nBnds], b1 = eBnds[k * nBnds + sharpen];
        for (int b = b0; b < b1; ++b) {
          float vs[10], d, e, eBest = 1e10f;
          int i, sBest = -1, ss[4];
          for (i = 0; i < 4; ++i) ss[i] = S[N[eBins[b] * 4 + i]];
          for (z = 0; z < Z; ++z) vs[z] = I1[iids[eBins[b]] + z * h * w];
          for (i = 0; i < 4; ++i) {
            s = ss[i]; e = 0;
            if (s == sBest) continue;
            for (z = 0; z < Z; ++z) {
              d = mus[s * Z + z] - vs[z];
              e += d * d;
            }
            if (e < eBest) {
              eBest = e;
              sBest = s;
            }
          }
          S[eBins[b]] = sBest;
        }
        // convert mask to edge maps (examining expanded set of pixels)
        float *E1 = (float*)E.data + c * stride * h2 + r * stride;
        b1 = eBnds[k * nBnds + sharpen + 1];
        for (int b = b0; b < b1; ++b) {
          int i = eBins[b];
          uint8_t s = S[i];
          uint16_t *N1 = N + i * 4;
          if (s != S[N1[0]] || s != S[N1[1]] || s != S[N1[2]] || s != S[N1[3]]) {
            E1[eids[i]]++;
          }
        }
      }
    }
  }

  // free memory
  delete [] iids; delete [] eids;
  delete [] cids; delete [] cids1; delete [] cids2;
  delete [] ind;

  st = clock();
  // normalize and finalize edge maps
  float t = 1.0f * stride * stride / (gtWidth * gtWidth) / nTreesEval;
  int r = gtWidth / 2;

  if (sharpen == 0) t *= 2;
  else if (sharpen == 1) t *= 1.8;
  else t *= 1.66;

  E.crop(r, oRows + r, r, oCols + r);
  E.multiply(t);
  CellArray E_tmp;
  convTri(E, E_tmp, 1);
  E.swap(E_tmp);

  // compute approximate orientation O from edges E
  convTri(E, E_tmp, 4);
  CellArray Ox, Oy, Oxx, Oxy, Oyy;
  gradient(E_tmp, Ox, Oy);
  gradient(Ox, Oxx, Oxy);
  gradient(Oy, Oxy, Oyy);
  O.create(E.rows, E.cols, E.channels);
  for (int j = 0; j < O.cols; ++j) {
    for (int i = 0; i < O.rows; ++i) {
      float val = atan(Oyy.at(i, j) * sgn(-Oxy.at(i, j)) / (Oxx.at(i, j) + 1e-5));
      if (val < -1e-8) val += PI;
      O.at(i, j) = val;
    }
  }
  ed = clock();
  std::cerr << "time for post process: " << (double)(ed - st) / CLOCKS_PER_SEC << std::endl;
}

void EdgeDetector::edgesDetect(uint8_t* image, int h, int w, int d, CellArray &E, CellArray &O) {
  // store original size of image
  int oRows = h, oCols = w;
  // pad image, making divisible by 4
  int r = imWidth / 2;
  std::vector<int> p = {r, r, r, r};
  p[1] += (4 - (h + r * 2) % 4) % 4;
  p[3] += (4 - (w + r * 2) % 4) % 4;
  image = imPad(image, h, w, d, p, "symmetric");
  h += p[0] + p[1];
  w += p[2] + p[3];

  // get feature
  CellArray chnsReg, chnsSim;
  featureExtract(image, h, w, d, chnsReg, chnsSim);
  CellArray I; I.rows = h, I.cols = w, I.channels = d;
  //I.data = rgbConvert(image, h, w, d, (int)CS_RGB);
  if (sharpen) {
    CellArray tmp;
    convTri(I, tmp, 1.0f);
    I.swap(tmp);
  }
  detect(I, E, O, chnsReg, chnsSim, oRows, oCols);
  // perform nms
  edgeNms(E, O, 2, 0, 1, nThreads);
}

uint32_t* EdgeDetector::buildLookup(int *dims, int w) {
  int c, r, z, n=w*w*dims[2];
  uint32_t *cids=new uint32_t[n]; n=0;
  for(z=0; z<dims[2]; z++) for(c=0; c<w; c++) for(r=0; r<w; r++)
    cids[n++] = z*dims[0]*dims[1] + c*dims[0] + r;
  return cids;
}

void EdgeDetector::buildLookupSs(uint32_t *&cids1, uint32_t *&cids2, int *dims, int w, int m) {
  int i, j, z, z1, c, r; int locs[1024];
  int m2=m*m, n=m2*(m2-1)/2*dims[2], s=int(w/m/2.0+.5);
  cids1 = new uint32_t[n]; cids2 = new uint32_t[n]; n=0;
  for(i=0; i<m; i++) locs[i]=uint32_t((i+1)*(w+2*s-1)/(m+1.0)-s+.5);
  for(z=0; z<dims[2]; z++) for(i=0; i<m2; i++) for(j=i+1; j<m2; j++) {
    z1=z*dims[0]*dims[1]; n++;
    r=i%m; c=(i-r)/m; cids1[n-1]= z1 + locs[c]*dims[0] + locs[r];
    r=j%m; c=(j-r)/m; cids2[n-1]= z1 + locs[c]*dims[0] + locs[r];
  }
}

void EdgeDetector::edgeNms(CellArray &E, CellArray &O, int r, int s, float m, int nThreads) {
  CellArray E0(E);
  int h = E.rows, w = E.cols;

  // supress edges where edge is stronger in orthogonal direction
  #ifdef USEOMP
  nThreads = nThreads < omp_get_max_threads() ? nThreads : omp_get_max_threads();
  #pragma omp parallel for num_threads(nThreads)
  #endif
  for (int x = 0; x < w; ++x) for (int y = 0; y < h; ++y) {
    float e = E.at(y,x); if (!e) continue; e *= m;
    float coso = cos(O.at(y,x)), sino = sin(O.at(y,x));
    for (int d = -r; d <= r; ++d) if (d) {
      float e0 = interp(E0.data, h, w, x + d * coso, y + d * sino);
      if (e < e0) {
        E.at(y,x) = 0;
        break;
      }
    }
  }
  // supress noisy edge estimates near boundaries
  s = std::min(s, std::min(w / 2, h / 2));
  for (int x = 0; x < s; ++x) for (int y = 0; y < h; ++y) {
    E.at(y, x) *= x / float(s);
    E.at(y, w - 1 - x) *= x / float(s);
  }
  for (int x = 0; x < w; ++x) for (int y = 0; y < s; ++y) {
    E.at(y, x) *= y / float(s);
    E.at(h - 1 - y, x) *= y / float(s);
  }
}

void EdgeDetector::clear() {
  delete[] thrs; thrs = NULL;
  delete[] fids; fids = NULL;
  delete[] child; child = NULL;
  delete[] segs; segs = NULL;
  delete[] nSegs; nSegs = NULL;
  delete[] eBins; eBins = NULL;
  delete[] eBnds; eBnds = NULL;
}
