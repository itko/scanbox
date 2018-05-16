/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "generic/types.h"
#include <Eigen/Core>

namespace MouseTrack {

/// A spatial oracle takes a list of points via the `compute()` method,
/// proecesses and stores it internally, and allows the client
/// to perform spatial queries.
///
/// `PointList`: data type to communicate a batch of points.
///
/// `Point`: representation of points
///
/// `Precision`: desired numerical precision (double, float)
template <typename _PointList, typename _Point, typename _Precision>
class SpatialOracle {
public:
  /// Make available to client
  typedef _PointList PointList;
  /// Make available to client
  typedef _Point Point;
  /// Make available to client
  typedef _Precision Precision;

  /// Set points you want to perform queries on.
  /// The spatial oracle is not responsible for the lifetime of `points`,
  /// you have to take care of that. `points` needs to exist as long as
  /// the spatial oracle has to perform queries on it.
  /// convention: a single point is a colum vector, so `points` is a dxn
  /// matrix for d dimensions and n vectors
  virtual void compute(const PointList &points) = 0;

  /// Find `k` nearest points to `p`
  virtual std::vector<PointIndex> find_closest(const Point &p,
                                               unsigned int k = 1) const = 0;

  /// Give an index-list of all points within distance `r` around `p`
  /// The indexes are returned in random order.
  virtual std::vector<PointIndex> find_in_range(const Point &p,
                                                Precision r) const = 0;
};

} // namespace MouseTrack
