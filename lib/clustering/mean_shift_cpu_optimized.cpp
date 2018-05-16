/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///

#include "mean_shift_cpu_optimized.h"
#include "generic/erase_indices.h"
#include <Eigen/Dense>
#include <boost/log/trivial.hpp>
#include <iostream>

namespace MouseTrack {

MeanShiftCpuOptimized::MeanShiftCpuOptimized(double window_size)
    : MeanShift(window_size) {
  // empty
}

std::vector<Eigen::VectorXd>
MeanShiftCpuOptimized::convergePoints(const Oracle::PointList &points) const {

  const int dimensions = points.rows();

  std::vector<Eigen::VectorXd> currCenters;

  for (int i = 0; i < points.cols(); i += 1) {
    auto v = points.col(i);
    currCenters.push_back(v);
  }

  std::lock_guard<std::mutex> lock(_convergeOracleMutex);
  if (_cachedConvergeOracle.get() == nullptr) {
    OFactory::Query q;
    q.maxR = 2 * getWindowSize();
    q.dimensions = dimensions;
    _cachedConvergeOracle = oracleFactory().forQuery(q);
  }
  Oracle &oracle = *_cachedConvergeOracle;

  oracle.compute(points);

  // For each point...
  for (size_t i = 0; i < currCenters.size(); i++) {
    int iterations = 0; // for logging and abort condition

    Eigen::VectorXd prevCenter;
    // ... iterate until convergence
    do {
      iterations++;
      // perform one iteration of mean shift
      prevCenter = currCenters[i];
      std::vector<PointIndex> locals =
          oracle.find_in_range(currCenters[i], 2 * getWindowSize());
      if (locals.empty()) {
        BOOST_LOG_TRIVIAL(warning)
            << "No points in neighborhood, falling back to brute force.";
        currCenters[i] = iterate_mode(currCenters[i], points);
        break;
      } else {
        PointList localPoints(points.rows(), locals.size());
        for (size_t i = 0; i < locals.size(); ++i) {
          const auto li = locals[i];
          localPoints.col(i) = points.col(li);
        }

        currCenters[i] = iterate_mode(currCenters[i], localPoints);
      }

      if (iterations > getMaxIterations()) {
        BOOST_LOG_TRIVIAL(warning)
            << "Max number of iterations for point " << i
            << " reached - continuing without convergence for this point";
        break;
      }
    } while ((prevCenter - currCenters[i]).norm() > getConvergenceThreshold());
    if (i % 1024 == 0) {
      BOOST_LOG_TRIVIAL(trace)
          << "converged i " << i << " after " << iterations << " iterations";
    }
  }
  return currCenters;
}

std::vector<Cluster> MeanShiftCpuOptimized::mergePoints(
    std::vector<Eigen::VectorXd> &currCenters) const {
  std::vector<Cluster> clusters;
  std::vector<int> remainingPoints(currCenters.size());
  std::iota(remainingPoints.begin(), remainingPoints.end(), 0);
  const int dimensions = currCenters[0].size();
  const size_t nPoints = currCenters.size();

  std::lock_guard<std::mutex> lock(_mergeOracleMutex);
  if (_cachedMergeOracle.get() == nullptr) {
    OFactory::Query q;
    q.maxR = getMergeThreshold();
    q.dimensions = dimensions;
    _cachedMergeOracle = oracleFactory().forQuery(q);
  }
  Oracle &mergeOracle = *_cachedMergeOracle;

  // For every mode..
  int merged = 0;
  int mergedTotal = 0;
  // as long as there are un-classified points:
  // pick first point and ask spatial oracle for it's neighbors within the merge
  // threshold and create a new cluster from it.
  //
  // Removed the clustered points from `remainingPoints` and repeat
  while (!remainingPoints.empty()) {
    Oracle::PointList points(dimensions, remainingPoints.size());

    for (size_t j = 0; j < remainingPoints.size(); j++) {
      points.col(j) = currCenters[remainingPoints[j]];
    }

    mergeOracle.compute(points);
    auto neighbors = mergeOracle.find_in_range(currCenters[remainingPoints[0]],
                                               getMergeThreshold());
    // erase_indices assumes a sorted index vector
    std::sort(neighbors.begin(), neighbors.end());

    std::vector<size_t> cluster(neighbors.size());
    for (const auto ni : neighbors) {
      cluster.push_back(remainingPoints[ni]);
    }
    // check cluster
    {
      auto max = *std::max_element(cluster.begin(), cluster.end());
      assert(max < nPoints);
    }
    merged += cluster.size();
    mergedTotal += cluster.size();
    clusters.push_back(std::move(cluster));

    if (merged > 1024) {
      BOOST_LOG_TRIVIAL(trace)
          << mergedTotal << " points merged into " << clusters.size()
          << " clusters, remaining: " << remainingPoints.size();
      merged = 0;
    }

    erase_indices(remainingPoints, neighbors);
  }

  return clusters;
}

Eigen::VectorXd
MeanShiftCpuOptimized::iterate_mode(const Eigen::VectorXd &mode,
                                    const PointList &fixedPoints) const {
  auto w = (fixedPoints.colwise() - mode).colwise().squaredNorm();
  Eigen::RowVectorXd weights = (-w.array() / (2 * getWindowSize())).exp();
  assert(weights.rows() == 1);
  assert(weights.cols() == fixedPoints.cols());
  // COG Normalization Factor
  double normfact = weights.sum();
  Eigen::VectorXd cog =
      (fixedPoints.array().rowwise() * weights.array()).rowwise().sum();
  assert(cog.size() == mode.size());
  cog = cog * (1.0 / normfact);
  return cog;
}

} // namespace MouseTrack
