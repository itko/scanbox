/// \file
/// Maintainer: Felice Serena
///
///

#include "point_cloud.h"

#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>

namespace utf = boost::unit_test;

BOOST_AUTO_TEST_CASE(point_cloud_read_write) {
  MouseTrack::PointCloud pc;
  pc.resize(1);
  pc[0].x(1.0);
  pc[0].y(2.0);
  pc[0].z(3.0);
  pc[0].intensity(4.0);

  const auto point = pc[0];

  BOOST_CHECK_CLOSE(1.0, point.x(), .00001);
  BOOST_CHECK_CLOSE(2.0, point.y(), .00001);
  BOOST_CHECK_CLOSE(3.0, point.z(), .00001);
  BOOST_CHECK_CLOSE(4.0, point.intensity(), .00001);
}
