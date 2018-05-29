/// \file
/// Maintainer: Luzian Hug
///
///

#include "generic/random_sample.h"
#include <stdlib.h>
#include <vector>

namespace MouseTrack {
/// Returns a random sample of size "size" of points from a point cloud
PointCloud random_sample(const PointCloud &cloud, const int size) {
  srand(43);

  // Create new point cloud object
  PointCloud output;
  output.resize(size);

  // Flags corresponding to which points have already been used
  std::vector<bool> already_used(cloud.size());
  std::fill(already_used.begin(), already_used.end(), false);

  int current_size = 0;
  while (current_size < size) {
    // Take a random point from cloud and append it to output
    int index = rand() % cloud.size();
    if (!already_used[index]) {
      output[current_size].x(cloud[index].x());
      output[current_size].y(cloud[index].y());
      output[current_size].z(cloud[index].z());
      output[current_size].r(cloud[index].r());
      output[current_size].g(cloud[index].g());
      output[current_size].b(cloud[index].b());
      already_used[index] = true;
      current_size++;
    }
  }

  return output;
}

} // namespace MouseTrack
