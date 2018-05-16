/// \file
/// Maintainer: Felice Serena
///
///

#include "statistical_outlier_detection.h"
#include "uniform_grid.h"
#include <set>

#include <boost/test/unit_test.hpp>

namespace utf = boost::unit_test;

using namespace Eigen;
using namespace MouseTrack;

BOOST_AUTO_TEST_CASE(outlier_2d) {
  typedef UniformGrid2d UG;
  UG::PointList all(2, 6);
  all.col(0) = Vector2d(0.0, 0.0);
  all.col(1) = Vector2d(2.0, 2.0);
  all.col(2) = Vector2d(0.1, 0.0);
  all.col(3) = Vector2d(-0.1, 0.0);
  all.col(4) = Vector2d(0.0, 0.1);
  all.col(5) = Vector2d(0.0, -0.1);

  std::multiset<PointIndex> expected;
  expected.insert(1);

  UG oracle(10.0, 0.05);
  oracle.compute(all);

  std::vector<PointIndex> result =
      statisticalOutlierDetection<UG::PointList, UG::Point, UG::Precision>(
          all, &oracle, 2.0, 10);

  std::multiset<PointIndex> received(result.begin(), result.end());

  BOOST_CHECK_MESSAGE(
      expected.size() == received.size(),
      "Expected and received sets have different cardinalities.");
  BOOST_CHECK_MESSAGE(
      expected == received,
      "Expected and received set do not contain same elements.");
}
