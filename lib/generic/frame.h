/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "generic/disparity_map.h"
#include "generic/types.h"
#include <Eigen/Core>

namespace MouseTrack {

/// Holds all information related to a single frame of a stream
struct Frame {
  DisparityMap normalizedDisparityMap;
  DisparityMap rawDisparityMap;
  PictureD referencePicture;
  std::vector<PictureD> labels;
  Precision focallength;
  Precision baseline;
  Precision ccx;
  Precision ccy;
  Eigen::Matrix4d rotationCorrection;
  Eigen::Matrix4d camChainPicture;
  Eigen::Matrix4d camChainDisparity;
};

} // namespace MouseTrack
