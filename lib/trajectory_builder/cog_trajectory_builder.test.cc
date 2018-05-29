/// \file
/// Maintainer: Luzian Hug
///
///

#include "cog_trajectory_builder.h"
#include "generic/center.h"
#include "generic/types.h"

#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>

namespace utf = boost::unit_test;

BOOST_AUTO_TEST_CASE(two_points_cog) {
  MouseTrack::PointCloud pc;
  pc.resize(2);

  pc[0].x(0.0);
  pc[0].y(-1.0);
  pc[0].z(3.0);
  pc[0].intensity(0);

  pc[1].x(4.0);
  pc[1].y(3.0);
  pc[1].z(3.0);
  pc[1].intensity(0);

  std::vector<size_t> indices;
  indices.push_back(0);
  indices.push_back(1);
  MouseTrack::Cluster cluster(indices);

  std::shared_ptr<MouseTrack::ClusterDescriptor> cd(
      new MouseTrack::Center(2, 1, 3));

  MouseTrack::CogTrajectoryBuilder tb;

  Eigen::Vector3d cog = tb(cd, cluster, pc);

  BOOST_CHECK_EQUAL(cog, Eigen::Vector3d(2, 1, 3));
}
