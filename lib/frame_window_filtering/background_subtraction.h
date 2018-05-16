/// \file
/// Maintainer: Luzian Hug
///

#pragma once

#include "frame_window_filtering.h"
#include <string>

namespace MouseTrack {

/// This subtracts pictures of a given frame (cage_frame) from the target frame
/// (cage with mouse).
///
class BackgroundSubtraction : public FrameWindowFiltering {
public:
  BackgroundSubtraction();
  virtual FrameWindow operator()(const FrameWindow &window) const;

  const FrameWindow &cage_frame() const;
  void cage_frame(FrameWindow &cage_frame);

  double threshold() const;
  void threshold(double threshold);

private:
  FrameWindow _cage_frame = FrameWindow();
  double _threshold = 0.01;
};

} // namespace MouseTrack
