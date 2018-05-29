/// \file
/// Maintainer: Luzian Hug
///
///

#include "mean_shift.h"

#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <random>
#include <set>
#include <stdlib.h>

namespace utf = boost::unit_test;

BOOST_AUTO_TEST_CASE(mean_shift_read_write) {
  MouseTrack::PointCloud pc;
  MouseTrack::MeanShift ms = MouseTrack::MeanShift(1);
  ms(pc);
}

BOOST_AUTO_TEST_CASE(mean_shift_single_point) {
  MouseTrack::MeanShift ms = MouseTrack::MeanShift(1);

  MouseTrack::PointCloud pc;
  pc.resize(1);
  pc[0].x(0);
  pc[0].y(3);
  pc[0].z(5);
  pc[0].intensity(1);

  std::vector<MouseTrack::Cluster> clusters = ms(pc);
  BOOST_CHECK_EQUAL(clusters.size(), 1);
}

BOOST_AUTO_TEST_CASE(two_faraway_points) {
  MouseTrack::MeanShift ms = MouseTrack::MeanShift(1);
  MouseTrack::PointCloud pc;
  pc.resize(2);
  pc[0].x(0);
  pc[0].y(0);
  pc[0].z(0);
  pc[0].intensity(0);
  pc[1].x(100);
  pc[1].y(100);
  pc[1].z(100);
  pc[1].intensity(1);

  std::vector<MouseTrack::Cluster> clusters = ms(pc);
  BOOST_CHECK_EQUAL(clusters.size(), 2);
}

BOOST_AUTO_TEST_CASE(two_closetogether_points) {
  MouseTrack::MeanShift ms = MouseTrack::MeanShift(1);
  MouseTrack::PointCloud pc;
  pc.resize(2);
  pc[0].x(0);
  pc[0].y(0);
  pc[0].z(0);
  pc[0].intensity(0);
  pc[1].x(0.1);
  pc[1].y(0.1);
  pc[1].z(0.1);
  pc[1].intensity(0);

  std::vector<MouseTrack::Cluster> clusters = ms(pc);
  BOOST_CHECK_EQUAL(clusters.size(), 1);
}

BOOST_AUTO_TEST_CASE(three_gaussian_clusters) {
  std::default_random_engine gen;

  std::normal_distribution<double> gauss0(0.0, 1.0);
  std::normal_distribution<double> gauss10(100.0, 1.0);

  MouseTrack::PointCloud pc;
  pc.resize(300);

  MouseTrack::MeanShift ms = MouseTrack::MeanShift(2.0);

  for (int i = 0; i < 300; i += 3) {
    pc[i].x(gauss10(gen));
    pc[i].y(gauss0(gen));
    pc[i].z(gauss0(gen));
    pc[i].intensity(gauss0(gen));

    pc[i + 1].x(gauss0(gen));
    pc[i + 1].y(gauss10(gen));
    pc[i + 1].z(gauss0(gen));
    pc[i + 1].intensity(gauss0(gen));

    pc[i + 2].x(gauss0(gen));
    pc[i + 2].y(gauss0(gen));
    pc[i + 2].z(gauss10(gen));
    pc[i + 2].intensity(gauss0(gen));
  }

  std::vector<MouseTrack::Cluster> clusters = ms(pc);

  BOOST_CHECK_EQUAL(clusters.size(), 3);
  BOOST_CHECK_EQUAL(clusters[0].points().size(), 100);
  BOOST_CHECK_EQUAL(clusters[1].points().size(), 100);
  BOOST_CHECK_EQUAL(clusters[2].points().size(), 100);
}

BOOST_AUTO_TEST_CASE(expected_clustering) {
  MouseTrack::PointCloud pc;
  pc.resize(10);
  for (int i = 0; i < 10; i += 1) {
    pc[i].y(0);
    pc[i].z(0);
    pc[i].intensity(0);
  }
  pc[0].x(1.0);
  pc[1].x(2.0);
  pc[2].x(2.0);
  pc[3].x(3.0);
  pc[4].x(10.0);
  pc[5].x(11.0);
  pc[6].x(12.0);
  pc[7].x(100.0);
  pc[8].x(101.0);
  pc[9].x(102.0);

  std::vector<std::multiset<size_t>> expected{3};
  // first cluster
  expected[0].insert(0);
  expected[0].insert(1);
  expected[0].insert(2);
  expected[0].insert(3);
  // second cluster
  expected[1].insert(4);
  expected[1].insert(5);
  expected[1].insert(6);
  // third cluster
  expected[2].insert(7);
  expected[2].insert(8);
  expected[2].insert(9);

  MouseTrack::MeanShift ms = MouseTrack::MeanShift(1.0);

  std::vector<MouseTrack::Cluster> clusters = ms(pc);

  BOOST_CHECK_EQUAL(clusters.size(), 3);

  std::vector<std::multiset<size_t>> received{3};
  for (int i = 0; i < received.size(); ++i) {
    received[i].insert(clusters[i].points().begin(),
                       clusters[i].points().end());
  }

  BOOST_CHECK(expected[0] == received[0]);
  BOOST_CHECK(expected[1] == received[1]);
  BOOST_CHECK(expected[2] == received[2]);
}
