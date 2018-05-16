/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "spatial_oracle.h"
#include <Eigen/Core>
#include <boost/log/trivial.hpp>
#include <flann/flann.hpp>

namespace MouseTrack {

/// Class to wrap Flann.
template <typename _Precision, int _Dim>
class Flann
    : public SpatialOracle<Eigen::Matrix<_Precision, _Dim, Eigen::Dynamic,
                                         Eigen::RowMajor + Eigen::AutoAlign>,
                           Eigen::Matrix<_Precision, _Dim, 1>, _Precision> {
public:
  // columns are points
  typedef Eigen::Matrix<_Precision, _Dim, Eigen::Dynamic,
                        Eigen::RowMajor + Eigen::AutoAlign>
      PointList;
  typedef Eigen::Matrix<_Precision, _Dim, 1> Point;
  typedef _Precision Precision;

private:
  // rows are points
  typedef Eigen::Matrix<_Precision, Eigen::Dynamic, _Dim,
                        Eigen::RowMajor + Eigen::AutoAlign>
      PointListT;
  // make sure to store one point per row for flann
  PointListT _points;
  std::unique_ptr<flann::Index<flann::L2<Precision>>> _index;
  flann::Matrix<Precision> dataset;

public:
  virtual void compute(const PointList &srcData) {
    // transpose data: we get points as columns,
    // but points must be rows for flann
    _points = srcData.transpose();
    // flann matrices expects row major data,
    // we are responsible for the lifetime
    dataset = flann::Matrix<Precision>(_points.data(), _points.rows(),
                                       _points.cols());
    // choose exact implementation: this is a list of tried values
    flann::IndexParams params = flann::KDTreeIndexParams(); // inacurate
    params = flann::LinearIndexParams();                    // slow
    params = flann::KDTreeSingleIndexParams(); // for low dimensional data
    params = flann::AutotunedIndexParams();    // auto tuning: target precision,
                                            // build weight,... cool but crashes
    params = flann::KMeansIndexParams(); // fast

    _index =
        std::make_unique<flann::Index<flann::L2<Precision>>>(dataset, params);
    _index->buildIndex();
  }

  virtual std::vector<PointIndex> find_closest(const Point &p,
                                               unsigned int k) const {
    // d: dimensions
    // k: desired nearest neighbors

    std::vector<std::vector<PointIndex>> indices;
    std::vector<std::vector<Precision>> dists;
    // query
    // query: 1xd point
    // strip off const just for a moment
    const flann::Matrix<Precision> query((Precision *)p.data(), 1, p.size());

    // querying
    _index->knnSearch(query, indices, dists, k, flann::SearchParams(128));

    return std::move(indices[0]);
  }

  virtual std::vector<PointIndex> find_in_range(const Point &p,
                                                const Precision r) const {
    // d: dimensions
    std::vector<std::vector<PointIndex>> indices;
    std::vector<std::vector<Precision>> dists;

    // query
    // query: 1xd point
    // strip off const just for a moment
    const flann::Matrix<Precision> query((Precision *)p.data(), 1, p.size());

    // querying
    _index->radiusSearch(query, indices, dists, r, flann::SearchParams(128));

    return std::move(indices[0]);
  }
};

typedef Flann<double, -1> FlannXd;
typedef Flann<double, 1> Flann1d;
typedef Flann<double, 2> Flann2d;
typedef Flann<double, 3> Flann3d;
typedef Flann<double, 4> Flann4d;
typedef Flann<double, 5> Flann5d;

typedef Flann<float, -1> FlannXf;
typedef Flann<float, 1> Flann1f;
typedef Flann<float, 2> Flann2f;
typedef Flann<float, 3> Flann3f;
typedef Flann<float, 4> Flann4f;
typedef Flann<float, 5> Flann5f;

} // namespace MouseTrack
