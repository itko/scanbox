/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "frame_window_filtering.h"

namespace MouseTrack {

class DisparityMedian : public FrameWindowFiltering {
public:
  virtual FrameWindow operator()(const FrameWindow &window) const;

  int diameter() const;
  void diameter(int _new);

private:
  int _diameter;
};

} // namespace MouseTrack
