/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "generic/types.h"

namespace MouseTrack {

class FrameIterable {
public:
  ~FrameIterable() = default;

  /// Is another frame available to process?
  ///
  /// `true` if there exists a frame to process.
  ///
  /// `false` if no more frames are available.
  virtual bool hasNextFrame() const = 0;

  /// Return the next frame number to process (f_i and f_{i+1} could be
  /// non-sequential: f_{i+1} - f_i > 1) If nextFrame() == endFrame(), there are
  /// no more frames available.
  virtual FrameNumber nextFrame() = 0;
};

} // namespace MouseTrack
