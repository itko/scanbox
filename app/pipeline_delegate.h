/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "generic/types.h"
#include "reader/frame_iterable.h"

namespace MouseTrack {

/// Implement this interface if you want to control the pipeline
/// at run time in an interactive manner.
class PipelineDelegate : public FrameIterable {
public:
  virtual ~PipelineDelegate() = default;
};
} // namespace MouseTrack
