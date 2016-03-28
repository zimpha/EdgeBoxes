#include "gradientUtil.h"
#include "imageUtil.h"
#include "wrappers.h"
#include "CellArray.h"

int main() {
  cv::Mat img = cv::imread("image/pepper.png", 1);
  if (img.empty()) {
    wrError("Could not open or find the image");
    return -1;
  }
  try {
    CellArray A(img), B, Gx, Gy;
    rgbConvert(A, B, "gray");
    gradient(B, Gx, Gy);
  } catch (const std::string& e) {
    std::cout << e << std::endl;
  }
  return 0;
}
