#include "gradientUtil.h"
#include "imageUtil.h"
#include "wrappers.h"
#include "CellArray.h"

int main() {
  cv::Mat bgr = cv::imread("./image/pepper.png", 1);
  cv::Mat img;
  cv::cvtColor(bgr, img, CV_BGR2RGB);
  if (img.empty()) {
    wrError("Could not open or find the image");
    return -1;
  }
  try {
    cv::Mat img_flt;
    img.convertTo(img_flt, CV_32F);
    img_flt /= 255.0;
    CellArray A(img_flt), B, M, O, H;
    gradientMag(A, M, O);
    gradientHist(M, O, H, 2, 6, 1);
    for (int i = 0; i < 10; ++i) {
      for (int j = 0; j < 10; ++j) {
        std::cout << H.at<float>(i+99,j+99) << " ";
      }
      std::cout << std::endl;
    }
  } catch (const std::string& e) {
    std::cout << e << std::endl;
  }
  return 0;
}
