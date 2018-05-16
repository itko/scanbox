/// \file
/// Maintainer: Felice Serena
///
///

#include "disparity_gaussian_blur.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace MouseTrack {

FrameWindow DisparityGaussianBlur::operator()(const FrameWindow &window) const {
  FrameWindow result = window;
  for (size_t i = 0; i < window.frames().size(); ++i) {
    Frame &f = result.frames()[i];
    auto &disp = f.normalizedDisparityMap.zMap();
    cv::Mat img;
    cv::eigen2cv(disp, img);
    cv::GaussianBlur(img, img, cv::Size(2 * kx() + 1, 2 * ky() + 1), sigmax(),
                     sigmay());
    cv::cv2eigen(img, f.normalizedDisparityMap.zMap());
  }
  return result;
}

int DisparityGaussianBlur::kx() const { return _kx; }

void DisparityGaussianBlur::kx(int _new) { _kx = _new; }

int DisparityGaussianBlur::ky() const { return _ky; }

void DisparityGaussianBlur::ky(int _new) { _ky = _new; }

double DisparityGaussianBlur::sigmax() const { return _sigmax; }

void DisparityGaussianBlur::sigmax(double _new) { _sigmax = _new; }

double DisparityGaussianBlur::sigmay() const { return _sigmay; }

void DisparityGaussianBlur::sigmay(double _new) { _sigmay = _new; }

} // namespace MouseTrack
