/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///

#pragma once

#include "clustering.h"
#include "generic/cluster.h"
#include "spatial/oracle_factory.h"
#include <Eigen/Core>
#include <vector>

/// ***The Mean Shift Algorithm***
/// Goal: assign each given point to a cluster based on the nearest local
/// maximum of the point density.
///
/// Algorithm:
/// 1. At each point location, a cluster is initialized.
/// 2. For each cluster i:
///    i) Apply a gaussian kernel centered at the location of i to all points.
///    ii) The new location of i is the weighted center of gravity of the
///    points. Weights are given by the gaussian kernel. iii) Repeat i and ii
///    until the location of i converges.
/// 3. All clusters that are sufficiently close to each other are merged into
/// one cluster.
///

namespace MouseTrack {
class MeanShift : public Clustering {
public:
  typedef OracleFactory<double> OFactory;
  typedef OFactory::Oracle Oracle;

  /// window_size is the sigma for the gaussian kernel
  MeanShift(double window_size);

  /// Performs MeanShift algorithm
  virtual std::vector<Cluster> operator()(const PointCloud &cloud) const;

  void setMaxIterations(int max_iterations);
  int getMaxIterations() const;

  void setMergeThreshold(double merge_threshold);
  double getMergeThreshold() const;

  void setConvergenceThreshold(double convergence_threshold);
  double getConvergenceThreshold() const;

  void setWindowSize(double window_size);
  double getWindowSize() const;

  /// modify factory settings
  OFactory &oracleFactory();

  /// query factory
  const OFactory &oracleFactory() const;

protected:
  /// Converge points according to mean shift procedure
  virtual std::vector<Eigen::VectorXd>
  convergePoints(const Oracle::PointList &points) const;

  /// Merge the converged points into clusters
  virtual std::vector<Cluster>
  mergePoints(std::vector<Eigen::VectorXd> &points) const;

  /// Returns a weight in [0,1] for point by applying a gaussian kernel with
  /// variance window_size and mean mean
  double gaussian_weight(const Eigen::VectorXd point,
                         const Eigen::VectorXd mean) const;

private:
  /// when two peaks are closer than this, they are merged. Must be larger than
  /// _convergence_threshold for convergence.
  double _merge_threshold = 0.001;

  /// Convergence is decided if modes haven't moved more than this on average.
  /// Must be smaller than _merge_threshold for convergence.
  double _convergence_threshold = 0.0001;

  /// If algorithm hasn't converged after this amount of iterations, we abort.
  int _max_iterations = 100;

  /// window size parameter for mean shift algorithm
  double _window_size;

  /// Performs one iteration of the mean shift algorithm for a single mode
  Eigen::VectorXd iterate_mode(const Eigen::VectorXd mode,
                               const std::vector<Eigen::VectorXd> &state) const;

  OFactory _oracleFactory;
};

} // namespace MouseTrack
