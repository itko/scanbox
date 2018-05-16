/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "clustering.h"
#include "generic/cluster.h"
#include "spatial/oracle_factory.h"
#include <Eigen/Core>
#include <mutex>
#include <vector>

namespace MouseTrack {
class KMeans : public Clustering {
public:
  typedef OracleFactory<double> OFactory;
  typedef OFactory::Oracle Oracle;
  typedef Oracle::PointList PointList;
  typedef Oracle::Point Point;

  KMeans(int k);

  virtual std::vector<Cluster> operator()(const PointCloud &cloud) const;

  void K(int k);
  int K() const;

  /// Set worst centroid movement below which convergence is assumed.
  void centroidThreshold(double threshold);
  double centroidThreshold() const;

  /// Set the percentage of points that change the their cluster in one
  /// iteration below which convergence is assumed.
  void assignmentThreshold(double threshold);
  double assignmentThreshold() const;

  /// modify factory settings
  OFactory &oracleFactory();

  /// query the factory
  const OFactory &oracleFactory() const;

private:
  int _k;

  /// If the largest movement of all centroids is below or equal this threshold,
  /// convergence is assumed.
  double _centroidThreshold = 0.03;

  /// If the precentage (number of points/total points) of points that changed
  /// their cluster is below this threshold, convergence is assumed.
  double _assignmentThreshold = 0.02;

  OFactory _oracleFactory;
  mutable std::mutex _oracleMutex;
  mutable OFactory::Point _cachedBoundingBox;
  mutable std::unique_ptr<Oracle> _cachedOracle;

  bool meansConverged(const PointList &newMeans,
                      const PointList &lastMeans) const;
  bool assignmentConverged(const std::vector<Cluster> &newClusters,
                           const std::vector<Cluster> &lastClusters,
                           int totalPoints) const;
};

} // namespace MouseTrack
