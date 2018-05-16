/// \file
/// Maintainer: Felice Serena
///
///

#include "disparity_morphology.h"
#include <boost/log/trivial.hpp>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace MouseTrack {

FrameWindow DisparityMorphology::operator()(const FrameWindow &window) const {
  FrameWindow result = window;
  int op = opencvOperation();
  cv::Mat kernel = getStructuringElement(
      opencvKernelShape(), cv::Size(2 * diameter() + 1, 2 * diameter() + 1),
      cv::Point(diameter(), diameter()));
  for (size_t i = 0; i < window.frames().size(); ++i) {
    Frame &f = result.frames()[i];
    auto &disp = f.normalizedDisparityMap.zMap();
    cv::Mat raw, processed;
    cv::eigen2cv(disp, raw);
    cv::morphologyEx(raw, processed, op, kernel);
    cv::cv2eigen(processed, f.normalizedDisparityMap.zMap());
  }
  return result;
}

int DisparityMorphology::diameter() const { return _diameter; }
void DisparityMorphology::diameter(int _new) { _diameter = _new; }

DisparityMorphology::Morph DisparityMorphology::operation() const {
  return _operation;
}
void DisparityMorphology::operation(Morph _new) { _operation = _new; }

DisparityMorphology::KernelShape DisparityMorphology::kernelShape() const {
  return _kernelShape;
}
void DisparityMorphology::kernelShape(KernelShape _new) { _kernelShape = _new; }

int DisparityMorphology::opencvOperation() const {
  switch (operation()) {
  case Morph::open:
    return cv::MORPH_OPEN;
  case Morph::close:
    return cv::MORPH_CLOSE;
  default:
    BOOST_LOG_TRIVIAL(warning)
        << "Unexpected value for morphology operation encountered "
        << operation() << ". Please add to switch statement.";
    throw "Unexpected morphology operation encountered, please add to switch "
          "statement.";
  }
}

int DisparityMorphology::opencvKernelShape() const {
  switch (kernelShape()) {
  case KernelShape::rect:
    return cv::MORPH_RECT;
  case KernelShape::ellipse:
    return cv::MORPH_ELLIPSE;
  case KernelShape::cross:
    return cv::MORPH_CROSS;
  default:
    BOOST_LOG_TRIVIAL(warning)
        << "Unexpected value for morphology kernel shape encountered "
        << operation() << ". Please add to switch statement.";
    throw "Unexpected morphology kernel shape encountered, please add to "
          "switch statement.";
  }
}

} // namespace MouseTrack
