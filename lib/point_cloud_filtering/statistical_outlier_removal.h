/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "point_cloud_filtering.h"

namespace MouseTrack {
/// Based on: Towards 3D point cloud based object maps for household
/// environments, Bogdan Rusu, section 4.1
///
/// Idea: We characterize outliers based on their local neighborhood. For this
/// we calculate the mean and standard deviation of nearest neighbor distances.
///
/// Then we trim points outside mu +- alpha * sigma
///
/// k: number of nearest neighbors to take into account
///
/// mu: mean of k nearest neighbors
///
/// sigma: standard deviation of nearest neighbors
///
/// alpha: decides how much variance we want to allow
class StatisticalOutlierRemoval : public PointCloudFiltering {
public:
  StatisticalOutlierRemoval() = default;
  StatisticalOutlierRemoval(double alpha, int k);
  virtual ~StatisticalOutlierRemoval() = default;

  virtual PointCloud operator()(const PointCloud &inCloud) const;

  void k(int _new);
  int k() const;

  void alpha(double _new);
  double alpha() const;

private:
  int _k = 30;
  double _alpha = 1.0;
};

} // namespace MouseTrack
