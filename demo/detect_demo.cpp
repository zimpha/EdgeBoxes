#include "gradientUtil.h"
#include "imageUtil.h"
#include "wrappers.h"
#include "CellArray.h"
#include "EdgeDetector.h"
#include "EdgeBoxes.h"

int main() {
  cv::Mat bgr = cv::imread("./image/pepper.png", 1);
  cv::Mat img;
  cv::cvtColor(bgr, img, CV_BGR2RGB);

  try {
    CellArray I(img), E, O;
    EdgeDetector detector;
    EdgeBoxes edgeBoxes;
    detector.loadModel("./model/model.bin");
    edgeBoxes.initialize(.65, .75, 1, .01, 1e4, .1, .5, .5, 3, 1000, 2, 1.5);
    clock_t st = clock();
    detector.edgesDetect(I, E, O);
    clock_t ed = clock();
    std::cerr << double(ed - st) / CLOCKS_PER_SEC << std::endl;
    int cnt = 0;
    for (int i = 0; i < E.rows; ++i) {
      for (int j = 0; j < E.cols; ++j) {
        cnt += E.at<float>(i, j) > 0.1;
      }
    }
    std::cerr << cnt << std::endl;
    Boxes boxes = edgeBoxes.generate(E, O);
  } catch (const std::string &e) {
    std::cerr << e << std::endl;
  }
  return 0;
}
