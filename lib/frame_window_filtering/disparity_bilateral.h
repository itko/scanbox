/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "frame_window_filtering.h"
namespace MouseTrack {

class DisparityBilateral : public FrameWindowFiltering {
public:
  virtual FrameWindow operator()(const FrameWindow &window) const;

  int diameter() const;
  void diameter(int _new);

  double sigmaColor() const;
  void sigmaColor(double _new);

  double sigmaSpace() const;
  void sigmaSpace(double _new);

private:
  int _diameter;
  double _sigmaColor;
  double _sigmaSpace;
};

} // namespace MouseTrack
