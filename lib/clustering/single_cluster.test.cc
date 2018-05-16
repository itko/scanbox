/// \file
/// Maintainer: Luzian Hug
/// Created: 23.04.2018
///

#include "single_cluster.h"

#include <boost/test/unit_test.hpp>
namespace utf = boost::unit_test;


BOOST_AUTO_TEST_CASE( single_cluster ) {
    MouseTrack::PointCloud pc;
    pc.resize(10);
    for(int i = 0; i < 10; i += 1){
        pc[i].y() = 0;
        pc[i].z() = 0;
        pc[i].intensity(0);
    }
    pc[0].x() = 1.0;
    pc[1].x() = 2.0;
    pc[2].x() = 2.0;
    pc[3].x() = 3.0;
    pc[4].x() = 10.0;
    pc[5].x() = 11.0;
    pc[6].x() = 12.0;
    pc[7].x() = 100.0;
    pc[8].x() = 101.0;
    pc[9].x() = 102.0;

    MouseTrack::SingleCluster sc = MouseTrack::SingleCluster();

    std::vector<MouseTrack::Cluster> clusters = sc(pc);

    BOOST_CHECK_EQUAL(clusters.size(), 1);
    BOOST_CHECK_EQUAL(clusters[0].points().size(), pc.size());
}
