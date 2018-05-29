/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///

#include "mean_shift.h"
#include "spatial/flann.h"
#include "spatial/uniform_grid.h"
#include <Eigen/Dense>
#include <boost/log/trivial.hpp>
#include <iostream>

namespace MouseTrack {

MeanShift::MeanShift(double window_size) : _window_size(window_size) {
  // empty
}

std::vector<Cluster> MeanShift::operator()(const PointCloud &cloud) const {
  BOOST_LOG_TRIVIAL(trace) << "MeanShift algorithm started";
  // Convert point cloud to Eigen vectors

  if (cloud.size() == 0) {
    return std::vector<Cluster>();
  }

  const int dimensions = cloud.charDim();

  Oracle::PointList points;
  points.resize(dimensions, cloud.size());
  for (PointIndex i = 0; i < cloud.size(); i += 1) {
    auto v = cloud[i].characteristic();
    points.col(i) = v;
  }

  bool normalize = false;
  if (normalize) {
    Eigen::VectorXd min = points.rowwise().minCoeff();
    Eigen::VectorXd max = points.rowwise().maxCoeff();
    Eigen::VectorXd bb_size = max - min;
    points = (points.array().colwise()) / bb_size.array();
  }

  std::vector<Eigen::VectorXd> currCenters = convergePoints(points);

  BOOST_LOG_TRIVIAL(debug) << "Merging points to clusters";
  // Merge step

  std::vector<Cluster> clusters = mergePoints(currCenters);

  BOOST_LOG_TRIVIAL(trace) << "MeanShift converged! #Clusters: "
                           << clusters.size();
  return clusters;
}

std::vector<Eigen::VectorXd>
MeanShift::convergePoints(const Oracle::PointList &points) const {

  const int dimensions = points.rows();

  std::vector<Eigen::VectorXd> currCenters, pointsVec;

  for (int i = 0; i < points.cols(); i += 1) {
    auto v = points.col(i);
    currCenters.push_back(v);
    pointsVec.push_back(v);
  }

  std::unique_ptr<Oracle> oraclePtr;
  {
    OFactory::Query q;
    q.maxR = 2 * _window_size;
    q.dimensions = dimensions;
    oraclePtr = oracleFactory().forQuery(q);
  }
  Oracle &oracle = *oraclePtr;

  // Initialize some stuff used in the MeanShift loop
  Eigen::VectorXd prevCenter;

  oracle.compute(points);

  // For each point...
  for (PointIndex i = 0; i < currCenters.size(); i++) {
    int iterations = 0; // for logging and abort condition

    // ... iterate until convergence
    do {
      iterations++;
      // perform one iteration of mean shift
      prevCenter = currCenters[i];
      std::vector<PointIndex> locals =
          oracle.find_in_range(currCenters[i], 2 * _window_size);
      if (locals.empty()) {
        BOOST_LOG_TRIVIAL(warning)
            << "No points in neighborhood, falling back to brute force.";
        currCenters[i] = iterate_mode(currCenters[i], pointsVec);
        break;
      } else {
        std::vector<Eigen::VectorXd> localPoints;
        for (int li : locals) {
          localPoints.push_back(points.col(li));
        }

        currCenters[i] = iterate_mode(currCenters[i], localPoints);
      }

      if (iterations > _max_iterations) {
        BOOST_LOG_TRIVIAL(warning)
            << "Max number of " << _max_iterations << " iterations for point "
            << i << " reached - continuing without convergence for this point";
        break;
      }
    } while ((prevCenter - currCenters[i]).norm() > _convergence_threshold);
    if (i % 1024 == 0) {
      BOOST_LOG_TRIVIAL(trace)
          << "converged i " << i << " after " << iterations << " iterations";
    }
  }
  return currCenters;
}

std::vector<Cluster>
MeanShift::mergePoints(std::vector<Eigen::VectorXd> &currCenters) const {
  // Initialize Clusters. Initially, every point has its own cluster.
  const size_t nPoints = currCenters.size();
  std::vector<Cluster> clusters(nPoints);
  for (PointIndex i = 0; i < nPoints; i++) {
    clusters[i].points().push_back(i);
  }
  // For every mode..
  int merged = 0;
  for (size_t i = 0; i < currCenters.size(); i++) {
    if (merged % 1024 == 0) {
      BOOST_LOG_TRIVIAL(trace) << merged << " points merged, i: " << i
                               << ", remaining: " << currCenters.size() - i;
    }
    // Check modes we haven't already executed the (i)-loop for
    for (size_t j = i + 1; j < currCenters.size(); j++) {
      Eigen::VectorXd diff = currCenters[i] - currCenters[j];
      if (diff.norm() < _merge_threshold) {
        // Merge clusters. Erase one of the modes corresponding to the clusters
        // and append points belonging to j to cluser of i
        clusters[i].points().push_back(clusters[j].points()[0]);
        clusters.erase(clusters.begin() + j);
        currCenters.erase(currCenters.begin() + j);
        // We need to decrement i here because there might be another cluster
        // that wants to merge with i.
        i--;
        ++merged;
        break;
      }
    }
  }
  return clusters;
}

double MeanShift::gaussian_weight(const Eigen::VectorXd point,
                                  const Eigen::VectorXd mean) const {

  // assert if dimensions match
  assert(mean.size() == point.size());

  // Euclidean distance squared btw. mean of kernel and point of interest
  const double d = (mean - point).squaredNorm();

  // Apply gaussian distribution
  return exp(-d / (2 * _window_size));
}

Eigen::VectorXd
MeanShift::iterate_mode(const Eigen::VectorXd mode,
                        const std::vector<Eigen::VectorXd> &fixedPoints) const {
  // COG Normalization Factor
  double normfact = 0;
  // Rest of COG
  Eigen::VectorXd cog = Eigen::VectorXd::Zero(mode.size());
  for (size_t i = 0; i < fixedPoints.size(); i++) {
    double temp = gaussian_weight(fixedPoints[i], mode);
    normfact += temp;
    cog += temp * fixedPoints[i];
  }
  cog = cog * (1.0 / normfact);
  return cog;
}

void MeanShift::setMaxIterations(int max_iterations) {
  _max_iterations = max_iterations;
}

int MeanShift::getMaxIterations() const { return _max_iterations; }

void MeanShift::setMergeThreshold(double merge_threshold) {
  _merge_threshold = merge_threshold;
}
double MeanShift::getMergeThreshold() const { return _merge_threshold; }

void MeanShift::setConvergenceThreshold(double convergence_threshold) {
  _convergence_threshold = convergence_threshold;
}
double MeanShift::getConvergenceThreshold() const {
  return _convergence_threshold;
}

void MeanShift::setWindowSize(double window_size) {
  _window_size = window_size;
}
double MeanShift::getWindowSize() const { return _window_size; }

MeanShift::OFactory &MeanShift::oracleFactory() { return _oracleFactory; }

const MeanShift::OFactory &MeanShift::oracleFactory() const {
  return _oracleFactory;
}

} // namespace MouseTrack
