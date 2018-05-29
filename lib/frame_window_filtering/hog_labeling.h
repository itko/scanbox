/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "frame_window_filtering.h"

#include "classifier/classifier.h"

#include <memory>

namespace MouseTrack {

class HogLabeling : public FrameWindowFiltering {
public:
  typedef Classifier::Mat Mat;
  typedef Classifier::Vec Vec;
  void train(const Mat &X_train, const Vec &y_train);
  virtual FrameWindow operator()(const FrameWindow &window) const;

private:
  std::unique_ptr<Classifier> _classifier;
  // highest label + 1: we need to know, how many labels there are
  int _numLabels;

  /// width of the sliding window
  int _windowWidth = 64;

  /// height of the sliding window
  int _windowHeight = 64;

  /// Distance between two neighboring sliding windows
  int _stepSize = 64 / 8;

  /// number of histogram bins for HOG??
  int _nbins = 9;

  /// ?? from opencv HOG
  int _blockWidth = 16;
  int _blockHeight = 16;
  int _blockStrideWidth = 8;
  int _blockStrideHeight = 8;
  int _cellWidth = 8;
  int _cellHeight = 8;

  /// make sure the highest value is 1 and the lowest value is 0
  bool _normalizeRange = true;

  /// normalize accross labels such that all labels for one pixel sum to 1
  bool _normalizeAccross = false;
};

} // namespace MouseTrack
