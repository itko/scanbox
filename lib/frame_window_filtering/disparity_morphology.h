/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "frame_window_filtering.h"
namespace MouseTrack {

class DisparityMorphology : public FrameWindowFiltering {
public:
  virtual FrameWindow operator()(const FrameWindow &window) const;

  enum Morph { open, close };

  /// Shape of the kernel
  ///
  /// Rect: all entries equal one: E_ij = 1
  ///
  /// Ellipse: a filled ellipse inscribed in the rectangle [0, 2*diameter() +
  /// 1]^2
  ///
  /// Cross: all zeros except for one row and one column which are equal 1,
  /// crossing right in the center element
  enum KernelShape { rect, ellipse, cross };

  int diameter() const;
  void diameter(int _new);

  Morph operation() const;
  void operation(Morph _new);

  KernelShape kernelShape() const;
  void kernelShape(KernelShape _new);

private:
  int _diameter;
  Morph _operation;
  KernelShape _kernelShape;

  int opencvOperation() const;
  int opencvKernelShape() const;
};

} // namespace MouseTrack
