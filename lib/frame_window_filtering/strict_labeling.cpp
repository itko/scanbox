/// \file
/// Maintainer: Felice Serena
///
///

#include "strict_labeling.h"

namespace MouseTrack {

FrameWindow StrictLabeling::operator()(const FrameWindow &window) const {
  FrameWindow w = window;

  for (size_t f = 0; f < w.frames().size(); ++f) {
    Frame &frame = w.frames()[f];
    if (frame.labels.empty()) {
      continue;
    }
    int rows = frame.labels[0].rows();
    int cols = frame.labels[0].cols();

    // decide which label to take for each pixel
    // and set all maps to 0 or 1
    for (int y = 0; y < rows; ++y) {
      for (int x = 0; x < cols; ++x) {
        double v = 0;
        int maxL = 0;
        for (size_t l = 0; l < frame.labels.size(); ++l) {
          if (labelsToIgnore.find(l) != labelsToIgnore.end()) {
            continue;
          }
          auto c = frame.labels[l](y, x);
          if (v < c) {
            v = c;
            maxL = l;
          }
        }
        for (size_t l = 0; l < frame.labels.size(); ++l) {
          if (maxL == l) {
            frame.labels[l](y, x) = 1.0;
          } else {
            frame.labels[l](y, x) = 0.0;
          }
        }
      }
    }
  }
  return w;
} // namespace MouseTrack

} // namespace MouseTrack
