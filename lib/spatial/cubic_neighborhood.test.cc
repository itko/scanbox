/// \file
/// Maintainer: Felice Serena
///
///

#include "cubic_neighborhood.h"

#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>

namespace utf = boost::unit_test;

using namespace Eigen;
using namespace MouseTrack::SpatialImpl;

BOOST_AUTO_TEST_CASE(cubic_neighborhood_dists) {
  CubicNeighborhood<3> neigh(2); // create first three layers

  BOOST_CHECK_EQUAL(neigh[0].size(), 1);
  BOOST_CHECK_EQUAL(neigh[0][0], Vector3i(0, 0, 0));

  BOOST_CHECK_CLOSE(neigh[0].min(), 0.0, 0.00001);
  BOOST_CHECK_CLOSE(neigh[0].max(), std::sqrt(3 * 0.5 * 0.5), 0.00001);

  BOOST_CHECK_EQUAL(neigh[1].size(), 27 - 1);

  BOOST_CHECK_CLOSE(neigh[1].min(), 0.5, 0.00001);
  BOOST_CHECK_CLOSE(neigh[1].max(), std::sqrt(3 * 1.5 * 1.5), 0.00001);

  BOOST_CHECK_EQUAL(neigh[2].size(), 125 - 27);

  BOOST_CHECK_CLOSE(neigh[2].min(), 1.5, 0.00001);
  BOOST_CHECK_CLOSE(neigh[2].max(), std::sqrt(3 * 2.5 * 2.5), 0.00001);
}

BOOST_AUTO_TEST_CASE(cubic_neighborhood_constructor_tests) {
  CubicNeighborhood<-1> neighDefault();

  CubicNeighborhood<-1> neigh00(0, 1);
  CubicNeighborhood<-1> neigh01(1, 1);
  CubicNeighborhood<-1> neigh02(2, 2);
  CubicNeighborhood<-1> neigh03(3, 3);
  CubicNeighborhood<-1> neigh04(4, 4);

  CubicNeighborhood<1> neigh1(0);
  CubicNeighborhood<1> neigh2(1);
  CubicNeighborhood<1> neigh3(2);
  CubicNeighborhood<1> neigh4(3);
  CubicNeighborhood<1> neigh5(4);

  CubicNeighborhood<2> neigh6(0);
  CubicNeighborhood<2> neigh7(1);
  CubicNeighborhood<2> neigh8(2);
  CubicNeighborhood<2> neigh9(3);
  CubicNeighborhood<2> neigh0(4);

  CubicNeighborhood<3> neigh11(0);
  CubicNeighborhood<3> neigh12(1);
  CubicNeighborhood<3> neigh13(2);
  CubicNeighborhood<3> neigh14(3);
  CubicNeighborhood<3> neigh15(4);

  CubicNeighborhood<4> neigh16(0);
  CubicNeighborhood<4> neigh17(1);
  CubicNeighborhood<4> neigh18(2);
  CubicNeighborhood<4> neigh19(3);
  CubicNeighborhood<4> neigh20(4);
}
