#include "gradientUtil.h"
#include "wrappers.h"
#include "CellArray.h"
//#include "EdgeDetector.h"
//#include "EdgeBoxes.h"
#include "ACFDetector.h"

#include <opencv2/opencv.hpp>

int main() {
  try {
    CellArray I, E, O;
    //EdgeDetector detector;
    //EdgeBoxes edgeBoxes;
    //detector.loadModel("./model/model.bin");
    //edgeBoxes.initialize(.65, .75, 1, .01, 1e4, .1, .5, .5, 3, 1000, 2, 1.5);
    ACFDetector acfDetector;
    acfDetector.loadModel("./model/acfmodel.bin");

    std::vector<cv::String> filenames;
    cv::String folder = "/home/zimpha/EdgeBoxes/image/INRIAPerson/Test/pos";
    cv::glob(folder, filenames);
    long extra = 0;
    clock_t st = clock();
    std::string ff = "/home/zimpha/EdgeBoxes/image/test.jpg";
    for (size_t i = 0; i < filenames.size(); ++i) {
      clock_t e_st = clock();
      cv::Mat src = cv::imread(filenames[i]), dst;
      std::cerr << filenames[i] << std::endl;
      cv::cvtColor(src, dst, CV_BGR2RGB);
      cv::resize(dst, src, cv::Size(640, 480));
      int h = src.rows, w = src.cols, d = 3;
      uint8_t* I = (uint8_t*)wrCalloc(h * w * d, sizeof(uint8_t));
      for (int k = 0; k < d; ++k) {
        for (int c = 0; c < w; ++c) {
          for (int r = 0; r < h; ++r) {
            I[k * w * h + c * h + r] = ((uint8_t*)src.data)[r * w * d + c * d + k];
          }
        }
      }
      extra += clock() - e_st;
      Boxes res = acfDetector.acfDetect(I, h, w, d);
      printf("%d\n", (int)res.size());
    //  for (size_t i = 0; i < res.size(); ++i) {
    //    printf("%d %d %d %d %.4f\n", res[i].c + 1, res[i].r + 1, res[i].w, res[i].h, res[i].s);
    //  }
      /*PyramidInput pyramidInput;
      PyramidOutput pyramidOutput;
      chnsPyramid(I, h, w, d, pyramidInput, pyramidOutput);*/
      /*clock_t a = clock();
      detector.edgesDetect(I, h, w, d, E, O);
      clock_t b = clock();
      Boxes boxes = edgeBoxes.generate(E, O);
      clock_t c = clock();
      std::cerr << "edge detect: " << double(b - a) / CLOCKS_PER_SEC << std::endl;
      std::cerr << "edge boxes: " << double(c - b) / CLOCKS_PER_SEC << std::endl;
      std::cerr << "number of boxes: " << boxes.size() << std::endl;*/
      wrFree(I);
    }
    clock_t ed = clock();
    double total = double(ed - st) / CLOCKS_PER_SEC;
    std::cerr << "total: " << total << std::endl;
    std::cerr << "fps: " << filenames.size() / total << std::endl;
  } catch (const std::string &e) {
    std::cerr << e << std::endl;
  }
  return 0;
}
