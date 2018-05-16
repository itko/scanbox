/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "frame_window_filtering.h"
namespace MouseTrack {

class DisparityGaussianBlur : public FrameWindowFiltering {
public:
  virtual FrameWindow operator()(const FrameWindow &window) const;

  /// patch diameter in x direction, kx = 0 results in a patch width of 1
  int kx() const;
  void kx(int _new);

  /// patch diameter in y direction, ky = 0 results in a patch height of 1
  int ky() const;
  void ky(int _new);

  /// gaussian standard deviation in x direction
  double sigmax() const;
  void sigmax(double _new);

  /// gaussian standard deviation in y direction
  double sigmay() const;
  void sigmay(double _new);

private:
  int _kx;
  int _ky;
  double _sigmax;
  double _sigmay;
};

} // namespace MouseTrack
