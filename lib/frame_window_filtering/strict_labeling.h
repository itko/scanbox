/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "frame_window_filtering.h"

#include <memory>
#include <set>

namespace MouseTrack {

/// Assumes existing labels
/// For each pixel, it sets one label to 1 and all others to 0
class StrictLabeling : public FrameWindowFiltering {
public:
  virtual FrameWindow operator()(const FrameWindow &window) const;

private:
  /// labels that should not be considered for classification, they won't get a
  /// cluster
  std::set<size_t> labelsToIgnore;
};

} // namespace MouseTrack
