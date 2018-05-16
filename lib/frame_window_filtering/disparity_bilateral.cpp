/// \file
/// Maintainer: Felice Serena
///
///

#include "disparity_bilateral.h"
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace MouseTrack {

FrameWindow DisparityBilateral::operator()(const FrameWindow &window) const {
  FrameWindow result = window;
  for (size_t i = 0; i < window.frames().size(); ++i) {
    Frame &f = result.frames()[i];
    auto &disp = f.normalizedDisparityMap.zMap();
    Eigen::MatrixXf floatMat = disp.cast<float>();
    cv::Mat raw, smoothed;
    cv::eigen2cv(floatMat, raw);
    cv::bilateralFilter(raw, smoothed, diameter(), sigmaColor(), sigmaSpace());
    cv::cv2eigen(smoothed, f.normalizedDisparityMap.zMap());
  }
  return result;
}

int DisparityBilateral::diameter() const { return _diameter; }
void DisparityBilateral::diameter(int _new) { _diameter = _new; }

double DisparityBilateral::sigmaColor() const { return _sigmaColor; }
void DisparityBilateral::sigmaColor(double _new) { _sigmaColor = _new; }

double DisparityBilateral::sigmaSpace() const { return _sigmaSpace; }
void DisparityBilateral::sigmaSpace(double _new) { _sigmaSpace = _new; }

} // namespace MouseTrack
