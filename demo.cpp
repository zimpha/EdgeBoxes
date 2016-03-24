#include "gradientUtil.h"
#include "wrappers.h"

int main() {
  cv::Mat img = cv::imread("image/pepper.png", 1);
  if (img.empty()) {
    wrError("Could not open or find the image");
    return -1;
  }
  try {
    cv::Mat gray;
    cv::cvtColor(img, gray, CV_RGB2GRAY);
    cv::Mat img_flt;
    gray.convertTo(img_flt, CV_32F);
    img_flt /= 255.0;
    cv::Mat Gx, Gy;
    gradient(img_flt, Gx, Gy);
    cv::Mat grad_x, grad_y;
    cv::Scharr(gray, grad_x, CV_32FC1, 1, 0);
    cv::Scharr(gray, grad_y, CV_32FC1, 0, 1);
  } catch (const std::string& e) {
    std::cout << e << std::endl;
  }
  return 0;
}
