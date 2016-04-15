#include "gradientUtil.h"
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
    std::string ff = "/home/zimpha/EdgeBoxes/image/test.jpg";
    for (size_t i = 0; i < filenames.size(); ++i) {
      cv::Mat src = cv::imread(filenames[i]), dst;
      std::cerr << filenames[i] << std::endl;
      cv::cvtColor(src, dst, CV_BGR2RGB);
      int h = dst.rows, w = dst.cols, d = 3;
      uint8_t* I = (uint8_t*)wrCalloc(h * w * d, sizeof(uint8_t));
      for (int k = 0; k < d; ++k) {
        for (int c = 0; c < w; ++c) {
          for (int r = 0; r < h; ++r) {
            I[k * w * h + c * h + r] = ((uint8_t*)dst.data)[r * w * d + c * d + k];
          }
        }
      }
      clock_t a = clock();
      detector.edgesDetect(I, h, w, d, E, O);
      clock_t b = clock();
      Boxes boxes = edgeBoxes.generate(E, O);
      clock_t c = clock();
      std::cerr << "edge detect: " << double(b - a) / CLOCKS_PER_SEC << std::endl;
      std::cerr << "edge boxes: " << double(c - b) / CLOCKS_PER_SEC << std::endl;
      std::cerr << "number of boxes: " << boxes.size() << std::endl;
      wrFree(I);
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
