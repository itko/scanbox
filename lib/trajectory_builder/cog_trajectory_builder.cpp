/// \file
/// Maintainer: Luzian Hug
///
///

#include "cog_trajectory_builder.h"

namespace MouseTrack {

/// Creates a new control point for a trajectory from a cluster and its
/// descriptor
Eigen::Vector3d CogTrajectoryBuilder::
operator()(const std::shared_ptr<const ClusterDescriptor>,
           const Cluster &cluster, const PointCloud &cloud) const {
  Eigen::VectorXd cog4 = cluster.center_of_gravity(cloud);
  return Eigen::Vector3d(cog4[0], cog4[1], cog4[2]);
}

} // namespace MouseTrack
