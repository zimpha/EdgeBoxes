#include "gradientUtil.h"
#include "imageUtil.h"
#include "wrappers.h"
#include "CellArray.h"
#include "EdgeDetector.h"
#include "EdgeBoxes.h"

#include <opencv2/opencv.hpp>

int main() {
  try {
    CellArray I, E, O;
    EdgeDetector detector;
    EdgeBoxes edgeBoxes;
    detector.loadModel("./model/model.bin");
    edgeBoxes.initialize(.65, .75, 1, .01, 1e4, .1, .5, .5, 3, 1000, 2, 1.5);

    std::vector<cv::String> filenames;
    cv::String folder = "/home/zimpha/EdgeBoxes/image/BSR/BSDS500/data/images/test";
    cv::glob(folder, filenames);
    clock_t st = clock();
    for (size_t i = 0; i < filenames.size(); ++i) {
      cv::Mat src = cv::imread(filenames[i]), dst;
      //std::cerr << filenames[i] << std::endl;
      if (filenames[i] != "/home/zimpha/EdgeBoxes/image/BSR/BSDS500/data/images/test/2018.jpg") continue;
      cv::cvtColor(src, dst, CV_BGR2RGB);
      dst.convertTo(src, CV_32FC3, 1.0 / 255);
      I.fromCvMat(src);
      detector.edgesDetect(I, E, O);
      /*std::cerr << "edge box start:" << std::endl;
      Boxes boxes = edgeBoxes.generate(E, O);
      std::cerr << "edge box finish:" << std::endl;*/
    }
    clock_t ed = clock();
    double total = double(ed - st) / CLOCKS_PER_SEC;
    std::cerr << "total: " << total << std::endl;
    std::cerr << "average: " << total / filenames.size() << std::endl;
  } catch (const std::string &e) {
    std::cerr << e << std::endl;
  }
  return 0;
}
