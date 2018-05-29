/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///

#include "kmeans.h"
#include "spatial/uniform_grid.h"
#include <Eigen/Dense>
#include <boost/log/trivial.hpp>
#include <iostream>

namespace MouseTrack {

KMeans::KMeans(int K) : _k(K) {
  // empty
}

std::vector<Cluster> KMeans::operator()(const PointCloud &cloud) const {
  BOOST_LOG_TRIVIAL(trace) << "KMeans algorithm started";
  // Convert point cloud to Eigen vectors

  if (cloud.size() == 0) {
    return std::vector<Cluster>();
  }

  const int dims = cloud.charDim();
  PointList means(dims, K());
  PointList prevMeans;

  PointList points;
  points.resize(dims, cloud.size());
  for (PointIndex i = 0; i < cloud.size(); i += 1) {
    auto v = cloud[i].characteristic();
    points.col(i) = v;
  }

  // TODO: get kmeans++ initialization
  means = points.block(0, 0, dims, K());

  Eigen::VectorXd min = points.rowwise().minCoeff();
  Eigen::VectorXd max = points.rowwise().maxCoeff();
  Eigen::VectorXd bb_size = max - min;

  BOOST_LOG_TRIVIAL(debug) << "KMeans: bb: " << bb_size;

  std::lock_guard<std::mutex> lock(_oracleMutex);
  if (_cachedOracle.get() == nullptr ||
      !(bb_size.array() <= _cachedBoundingBox.array()).all()) {
    bb_size *= 1.5;
    OFactory::Query q;
    q.dimensions = bb_size.size();
    q.bb_size = &bb_size;
    _cachedOracle = oracleFactory().forQuery(q);
    _cachedBoundingBox = bb_size;
  }
  Oracle &oracle = *_cachedOracle;

  std::vector<Cluster> clusters(K()), prevClusters;

  BOOST_LOG_TRIVIAL(debug) << "KMeans: Starting iterations.";
  do {
    // assign clusters
    BOOST_LOG_TRIVIAL(trace) << "KMeans: Assigning clusters";
    oracle.compute(means);
    prevClusters = std::move(clusters);
    clusters = std::vector<Cluster>(K());

    for (int i = 0; i < points.cols(); ++i) {
      std::vector<size_t> cs = oracle.find_closest(points.col(i), 1);
      if (cs.empty()) {
        oracle.find_closest(points.col(i), 1);
        BOOST_LOG_TRIVIAL(error)
            << "Couldn't find nearest mean for point " << i << " ("
            << points.col(i) << "), assigning to 1";
        cs.push_back(1);
      }
      clusters[cs[0]].points().push_back(i);
    }

    // calculate new cluster centers
    BOOST_LOG_TRIVIAL(trace) << "KMeans: Assigning centroids";
    prevMeans = means;
    for (PointIndex c = 0; c < clusters.size(); ++c) {
      PointList assigned(dims, clusters[c].points().size());
      for (PointIndex i = 0; i < clusters[c].points().size(); ++i) {
        int pi = clusters[c].points()[i];
        assigned.col(i) = points.col(pi);
      }
      Eigen::VectorXd mean =
          assigned.array().rowwise().sum() / (assigned.cols() + .000001);
      means.col(c) = mean;
    }
  } while (!meansConverged(means, prevMeans) &&
           !assignmentConverged(clusters, prevClusters, cloud.size()));

  std::stringstream ss;
  ss << clusters[0].points().size();
  for (size_t i = 1; i < clusters.size(); ++i) {
    ss << ", " << clusters[i].points().size();
  }

  BOOST_LOG_TRIVIAL(trace) << "K-Means cluster sizes: " << ss.str();

  return clusters;
}

void KMeans::K(int k) { _k = k; }
int KMeans::K() const { return _k; }

void KMeans::centroidThreshold(double threshold) {
  _centroidThreshold = threshold;
}

double KMeans::centroidThreshold() const { return _centroidThreshold; }

void KMeans::assignmentThreshold(double threshold) {
  _assignmentThreshold = threshold;
}

double KMeans::assignmentThreshold() const { return _assignmentThreshold; }

KMeans::OFactory &KMeans::oracleFactory() { return _oracleFactory; }

const KMeans::OFactory &KMeans::oracleFactory() const { return _oracleFactory; }

bool KMeans::meansConverged(const PointList &newMeans,
                            const PointList &lastMeans) const {
  auto centroidChange = (lastMeans - newMeans).colwise().norm();
  Precision change = centroidChange.maxCoeff();
  BOOST_LOG_TRIVIAL(trace) << "worst means change: " << change;
  return change <= centroidThreshold();
}
bool KMeans::assignmentConverged(const std::vector<Cluster> &newClusters,
                                 const std::vector<Cluster> &lastClusters,
                                 int totalPoints) const {
  int switched = 0;
  for (int i = 0; i < K(); ++i) {
    // rough estimate how many points changed assignment
    int delta =
        newClusters[i].points().size() - lastClusters[i].points().size();
    delta = std::abs(delta);
    switched += delta;
  }
  // double-counting??
  // switched /= 2;
  Precision switchedPercentage = switched / (Precision)totalPoints;
  BOOST_LOG_TRIVIAL(trace) << "switched change: " << switched << "/"
                           << totalPoints << " (" << (switchedPercentage * 100)
                           << "%)";
  return switchedPercentage <= assignmentThreshold();
}

} // namespace MouseTrack
