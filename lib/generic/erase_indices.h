/// \file
/// Maintainer: Felice Serena
///
///

#include "disparity_map.h"

namespace MouseTrack {

/// Remove a list of indices from a vector (in-place)
///
/// Assumes `indices` to be sorted in ascending order
template <typename T, typename Index>
void erase_indices(std::vector<T> &target, const std::vector<Index> &indices) {
  if (indices.empty() || target.empty()) {
    return;
  }
  auto insert_at = target.begin() + indices[0];
  for (size_t i = 0; i < indices.size() - 1; ++i) {
    // move elements between indices[j] and indices[j+1] to insert_at
    insert_at = std::copy(target.begin() + indices[i] + 1,
                          target.begin() + indices[i + 1], insert_at);
  }
  // copy elements after indices.last() to insert_at
  insert_at =
      std::copy(target.begin() + indices.back() + 1, target.end(), insert_at);
  target.resize(target.size() - indices.size());
}

} // namespace MouseTrack
