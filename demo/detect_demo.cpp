#include "gradientUtil.h"
#include "imageUtil.h"
#include "wrappers.h"
#include "CellArray.h"
#include "EdgeDetector.h"
#include "EdgeBoxes.h"

int main() {
  try {
    CellArray I, E, O;
    I.create(348, 600, 3, UINT8_CLASS);
    FILE* fp = fopen("/home/zimpha/EdgeBoxes/image/image.bin", "rb");
    fread(I.data, 1, 348 * 600 * 3, fp);

    EdgeDetector detector;
    EdgeBoxes edgeBoxes;
    detector.loadModel("./model/model.bin");
    edgeBoxes.initialize(.65, .75, 1, .01, 1e4, .1, .5, .5, 3, 1000, 2, 1.5);
    clock_t st = clock();
    detector.edgesDetect(I, E, O);
    clock_t ed = clock();
    std::cerr << double(ed - st) / CLOCKS_PER_SEC << std::endl;
    int scnt = 0;
    for (int i = 0; i < E.rows; ++i) {
      for (int j = 0; j < E.cols; ++j) {
        scnt += E.at<float>(i,j) > .1;
      }
    }
    std::cerr << scnt << std::endl;
    st = clock();
    Boxes boxes = edgeBoxes.generate(E, O);
    ed = clock();
    std::cerr << double(ed - st) / CLOCKS_PER_SEC << std::endl;
  } catch (const std::string &e) {
    std::cerr << e << std::endl;
  }
  return 0;
}
