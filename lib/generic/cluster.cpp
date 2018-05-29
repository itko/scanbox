/// \file
/// Maintainer: Felice Serena
///
///

#include "cluster.h"

namespace MouseTrack {

Cluster::Cluster() : _points(std::vector<PointIndex>()) {
  // empty
}

Cluster::Cluster(const std::vector<PointIndex> &points) : _points(points) {
  // empty
}

Cluster::Cluster(std::vector<PointIndex> &&points)
    : _points(std::move(points)) {
  // empty
}

std::vector<PointIndex> &Cluster::points() { return _points; }
const std::vector<PointIndex> &Cluster::points() const { return _points; }

Eigen::VectorXd Cluster::center_of_gravity(const PointCloud &cloud) const {
  Eigen::VectorXd cog;
  cog.setZero(cloud.charDim());
  for (size_t i = 0; i < points().size(); i++) {
    cog += cloud[points()[i]].characteristic();
  }
  if (points().size() > 0) {
    cog /= points().size();
  }
  return cog;
}

} // namespace MouseTrack
