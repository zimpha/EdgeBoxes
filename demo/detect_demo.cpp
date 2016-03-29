#include "gradientUtil.h"
#include "imageUtil.h"
#include "wrappers.h"
#include "CellArray.h"
#include "EdgeDetector.h"

int main() {
  cv::Mat bgr = cv::imread("./image/pepper.png", 1);
  cv::Mat img;
  cv::cvtColor(bgr, img, CV_BGR2RGB);

  try {
    CellArray I(img), E, O;
    EdgeDetector detector;
    detector.loadModel("./model/model.bin");
    detector.edgesDetect(I, E, O);
  } catch (const std::string &e) {
    std::cerr << e << std::endl;
  }
  return 0;
}
