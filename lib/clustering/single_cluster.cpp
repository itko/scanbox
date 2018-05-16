/// \file
/// Maintainer: Luzian Hug
/// Created: 23.04.2018
///
///

#include "single_cluster.h"
#include <boost/log/trivial.hpp>
#include <numeric>
#include "generic/cluster.h"


namespace MouseTrack {

SingleCluster::SingleCluster() {
  // empty
}

std::vector<Cluster> SingleCluster::operator()(const PointCloud &cloud) const {
  BOOST_LOG_TRIVIAL(trace) << "SingleCluster started";
  //Create vector with single cluster containing all the points
  std::vector<Cluster> cluster_vec(1);
  cluster_vec[0].points().resize(cloud.size());
  std::iota(cluster_vec[0].points().begin(), cluster_vec[0].points().end(), 0);
  BOOST_LOG_TRIVIAL(trace) << "SingleCluster finished";
  return cluster_vec;
}


} // namespace MouseTrack
