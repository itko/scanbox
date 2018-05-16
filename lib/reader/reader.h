/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "frame_iterable.h"
#include "generic/disparity_map.h"
#include "generic/frame_window.h"

namespace MouseTrack {

class Reader : public FrameIterable {
public:
  /// true if the reader is ready to provide output
  virtual bool valid() const = 0;

  /// Creates the Frame Window for frame index f, might read from disk/network
  /// This method is allowed to change the result of `endFrame`, if new frames
  /// get available. It might also block and wait for the next frame from the
  /// network. This is implementation dependent.
  virtual FrameWindow operator()(FrameNumber f) = 0;
};

} // namespace MouseTrack
