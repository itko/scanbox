/// \file
/// Maintainer: Luzian Hug
///

#include "background_subtraction.h"

#include "Eigen/Core"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/log/trivial.hpp>

namespace MouseTrack {
BackgroundSubtraction::BackgroundSubtraction() {
  // empty
}

FrameWindow BackgroundSubtraction::operator()(const FrameWindow &window) const {
  // Initializing return object
  FrameWindow output = window;
  // Cycle through all streams
  for (size_t i = 0; i < _cage_frame.frames().size(); i++) {

    // Copy the stuff we need from the FrameWindow objects
    const PictureD &cage_image = _cage_frame.frames()[i].referencePicture;
    const PictureD &mouse_image = window.frames()[i].referencePicture;

    // Dimensions must match
    if (!(cage_image.rows() == mouse_image.rows() &&
          cage_image.cols() == mouse_image.cols())) {
      // If it fails, return the input (do nothing)
      BOOST_LOG_TRIVIAL(info)
          << "Frame dimensions (" << mouse_image.rows() << "x"
          << mouse_image.cols()
          << ") do not match empty cage frame dimensions (" << cage_image.rows()
          << "x" << cage_image.cols()
          << "). Background subtraction cannot be performed.";
      return window;
    }

    // Perform the subtraction
    PictureD sub = (mouse_image - cage_image).array().abs();

    // Convert to opencv format and from [0,1] to [0,255] format
    cv::Mat subcv;
    sub = Eigen::floor(sub.array() * 255);
    cv::eigen2cv(sub, subcv);
    subcv.convertTo(subcv, CV_8UC1);

    // Apply Otsu's Method for thresholding
    cv::Mat maskcv;
    double thresh_otsu = cv::threshold(subcv, maskcv, 0, 255,
                                       cv::THRESH_BINARY + cv::THRESH_OTSU);
    maskcv = subcv > (thresh_otsu * _otsu_factor);

    // Convert back to Eigen
    Eigen::MatrixXd mask;
    maskcv = maskcv / 255;
    cv::cv2eigen(maskcv, mask);
    // A very low threshold means there's no significant bright
    // spots, i.e. no mouse => set mask to zeros
    if (thresh_otsu < 0.01 * 255) {
      mask.setZero();
    }

    // Build Frame object...
    output.frames()[i].normalizedDisparityMap =
        mask.array() * output.frames()[i].normalizedDisparityMap.array();
  }

  return output;
}

const FrameWindow &BackgroundSubtraction::cage_frame() const {
  return _cage_frame;
}

void BackgroundSubtraction::cage_frame(FrameWindow &cage_frame) {

  _cage_frame = cage_frame;
}

double BackgroundSubtraction::otsu_factor() const { return _otsu_factor; }
void BackgroundSubtraction::otsu_factor(double otsu_factor) {
  _otsu_factor = otsu_factor;
}

} // namespace MouseTrack
