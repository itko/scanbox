/// \file
/// Maintainer: Felice Serena
///
///

#include "uniform_grid.h"
#include <Eigen/Core>
#include <set>

#include <boost/test/unit_test.hpp>

namespace utf = boost::unit_test;

typedef std::vector<int> CellCoordinate;

using namespace MouseTrack;
using namespace Eigen;

BOOST_AUTO_TEST_CASE( uniform_grid_4d ) {
    constexpr int Dim = 4;
    typedef UniformGrid4d UG;
    UG::PointList all(Dim, 5);
    all.col(0) = Vector4d(0.0,0.0,0.0,0.0);
    all.col(1) = Vector4d(2.0,2.0,2.0,2.0);
    all.col(2) = Vector4d(10.0,0.0,0.0,0.0);
    all.col(3) = Vector4d(0.0,-1.0,-1.0,-1.0);
    all.col(4) = Vector4d(10.0,1.0,1.0,1.0);

    std::set<PointIndex> expected;
    expected.insert(2);
    expected.insert(4);
    std::unique_ptr<SpatialOracle<UG::PointList, UG::Point, double>> oracle(new UG(2, 10.0/20));
    oracle->compute(all);

    auto result = oracle->find_in_range(Vector4d(9.5,0,0,0), 2);

    std::set<PointIndex> received(result.begin(), result.end());

    BOOST_CHECK_EQUAL(expected.size(), received.size());
    BOOST_CHECK_MESSAGE(expected == received, "Expected and received set do not contain same elements." );
}


BOOST_AUTO_TEST_CASE( uniform_grid_4d_two_close_points ) {
    constexpr int Dim = 4;
    typedef UniformGrid4d UG;
    UG::PointList all(Dim, 2);
    all.col(0) = Vector4d(0.0,0.0,0.0,0.0);
    all.col(1) = Vector4d(0.1,0.1,0.1,0);

    std::unique_ptr<SpatialOracle<UG::PointList, UG::Point, double>> oracle(new UG(1, 0.5));
    oracle->compute(all);

    std::set<PointIndex> expected;

    // first test
    expected.insert(0);
    expected.insert(1);
    auto result = oracle->find_in_range(Vector4d(0.0,0,0,0), 1);
    std::set<PointIndex> received(result.begin(), result.end());

    BOOST_CHECK_EQUAL(expected.size(), received.size());
    BOOST_CHECK_MESSAGE(expected == received, "Expected and received set do not contain same elements." );
}
