/// \file
/// Maintainer: Luzian Hug
///
///

#include "generic/random_sample.h"

#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>

namespace utf = boost::unit_test;

BOOST_AUTO_TEST_CASE(random_sample) {
  MouseTrack::PointCloud pc;
  pc.resize(2, 0);
  pc[0].x(1.0);
  pc[0].y(2.0);
  pc[0].z(3.0);
  pc[0].intensity(4.0);

  pc[1].x(1.0);
  pc[1].y(2.0);
  pc[1].z(3.0);
  pc[1].intensity(4.0);

  MouseTrack::PointCloud sampled = MouseTrack::random_sample(pc, 1);
  BOOST_CHECK_EQUAL(sampled.size(), 1);

  const auto point = sampled[0];

  BOOST_CHECK_CLOSE(1.0, point.x(), .00001);
  BOOST_CHECK_CLOSE(2.0, point.y(), .00001);
  BOOST_CHECK_CLOSE(3.0, point.z(), .00001);
  BOOST_CHECK_CLOSE(4.0, point.intensity(), .00001);
}
