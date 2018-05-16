/// \file
/// Maintainer: Felice Serena
///
///

#include "disparity_median.h"
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace MouseTrack {

FrameWindow DisparityMedian::operator()(const FrameWindow &window) const {
  FrameWindow result = window;
  for (size_t i = 0; i < window.frames().size(); ++i) {
    Frame &f = result.frames()[i];
    auto &disp = f.normalizedDisparityMap.zMap();
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> charMat =
        (255 * disp).cast<unsigned char>();
    cv::Mat img;
    cv::eigen2cv(charMat, img);
    cv::medianBlur(img, img, 2 * diameter() + 1);
    cv::cv2eigen(img, charMat);
    f.normalizedDisparityMap.zMap() = charMat.cast<double>() / 255.0;
  }
  return result;
}

int DisparityMedian::diameter() const { return _diameter; }
void DisparityMedian::diameter(int _new) { _diameter = _new; }

} // namespace MouseTrack
