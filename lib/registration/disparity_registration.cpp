/// \file
/// Maintainer: Felice Serena
///

#include "disparity_registration.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/log/trivial.hpp>

namespace MouseTrack {
PointCloud DisparityRegistration::operator()(const FrameWindow &window) const {
  const auto &frames = window.frames();

  // how many points will there be at most?
  int expected_points = 0;
  for (const auto &f : frames) {
    expected_points += f.normalizedDisparityMap.size();
  }

  // absolute transformation matrix relative to first camera
  auto Ts = absoluteTransformations(window);

  std::vector<Inverse> inverses(frames.size());
  for (size_t i = 0; i < frames.size(); i += 1) {
    Eigen::Matrix4d mat = frames[i].rotationCorrection * Ts[i];
    inverses[i] = prepareInverseTransformation(mat);
  }

  // Allocate point cloud enough large to capture all points
  PointCloud cloud;
  cloud.resize(expected_points);
  int next_insert = 0;
  const int border = frameBorder();
  const double xshift = correctingXShift();
  const double yshift = correctingYShift();
  const double minDisp = minDisparity();
  // go through each frame, converting the disparity values to 3d points
  // relative to first camera
  for (size_t i = 0; i < frames.size(); i += 1) {
    const auto &f = frames[i];
    const auto &disp = f.normalizedDisparityMap;
    // convert each pixel
    for (int y = border - 1; y < disp.rows() - border; y += 1) {
      for (int x = border - 1; x < disp.cols() - border; x += 1) {
        // disparity is returned between [0,1],
        // but originally stored as [0,255]
        double disparity = 255 * disp(y, x);
        if (disparity < minDisp) {
          // just skip those points
          continue;
        }
        const double invDisparity = 1.0 / disparity;
        auto p = cloud[next_insert];
        p.x((x + xshift - f.ccx) * f.baseline * invDisparity);
        p.y((y + yshift - f.ccy) * f.baseline * invDisparity);
        p.z(f.focallength * f.baseline * invDisparity);

        Eigen::Vector4d tmp = applyInverseTransformation(
            inverses[i], Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0));
        p.x(tmp[0]);
        p.y(tmp[1]);
        p.z(tmp[2]);
        p.intensity(f.referencePicture(y, x));
        PointCloud::LabelVec labels(f.labels.size());
        for (int l = 0; l < f.labels.size(); ++l) {
          labels[l] = f.labels[l](y, x);
        }
        p.labels(std::move(labels));
        next_insert += 1;
      }
    }
  }
  cloud.resize(next_insert); // shrink to actual number of points

  auto min = cloud.posMin();
  auto max = cloud.posMax();

  BOOST_LOG_TRIVIAL(debug) << "Found point cloud with " << cloud.size()
                           << " points, xyz-min: [" << min[0] << ", " << min[1]
                           << ", " << min[2] << "], xyz-max: [" << max[0]
                           << ", " << max[1] << ", " << max[2] << "]"
                           << std::flush;
  return cloud;
}

std::vector<Eigen::Matrix4d> DisparityRegistration::absoluteTransformations(
    const FrameWindow &window) const {
  const auto &frames = window.frames();
  std::vector<Eigen::Matrix4d> Ts(frames.size());
  Ts[0] = Eigen::Matrix4d::Identity();
  for (size_t i = 1; i < frames.size(); i += 1) {
    Ts[i] =
        frames[i].camChainPicture * frames[i - 1].camChainDisparity * Ts[i - 1];
  }
  return Ts;
}

DisparityRegistration::Inverse
DisparityRegistration::prepareInverseTransformation(
    const Eigen::Matrix4d &mat) const {
  // For the moment, we just return the inverse.
  // We could also use a decomposition object from Eigen
  // to increase robustness.
  // Or we could use domain knowledge about the
  // transformation to speed up computations.
  return mat.inverse();
}

Eigen::Vector4d DisparityRegistration::applyInverseTransformation(
    const DisparityRegistration::Inverse &inv, const Eigen::Vector4d &p) const {
  // for the moment just a simple multiplication
  return inv * p;
}

double &DisparityRegistration::minDisparity() { return _min_disparity; }

/// Lowest disparity value we accept (we remove points at infinity)
const double &DisparityRegistration::minDisparity() const {
  return _min_disparity;
}

/// Set X shift to correct disparity map position
int &DisparityRegistration::correctingXShift() { return _xshift; }

/// Read X shift to correct disparity map position
const int &DisparityRegistration::correctingXShift() const { return _xshift; }

/// Set Y shift to correct disparity map position
int &DisparityRegistration::correctingYShift() { return _yshift; }

/// Read Y shift to correct disparity map position
const int &DisparityRegistration::correctingYShift() const { return _yshift; }

/// Ignores boder of n pixels around disparity map
int &DisparityRegistration::frameBoder() { return _frame_border; }

/// Ignores boder of n pixels around disparity map
const int &DisparityRegistration::frameBorder() const { return _frame_border; }

} // namespace MouseTrack
