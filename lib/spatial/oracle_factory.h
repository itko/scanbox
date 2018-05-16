/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "spatial_oracle.h"

#include "brute_force.h"
#include "flann.h"
#include "uniform_grid.h"

#include <boost/log/trivial.hpp>
#include <memory>

namespace MouseTrack {

/// Chooses and creates an implementation for a SpatialOracle based on given
/// parameters.
///
/// At the moment, it only chooses an implementation according to the value
/// set to `desiredOracle()`.
///
/// You have two options to request an oracle:
///
/// 1. call `forGeneral()`: The factory will do it's best to return you an
/// Oracle.
///
/// 2. call `forQuery()`: You need to create a `Query` and fill it with as much
/// knowledge about your data and the queries you want to perform as you can.
/// The Factory will choose an oracle based on the available data.
/// If you don't know a certain metric, leave it to the default value.
template <typename Precision, int Dim = -1> class OracleFactory {
public:
  enum Oracles { BRUTE_FORCE, FLANN, UNIFORM_GRID };

  Oracles desiredOracle() const { return _desiredOracle; }

  /// Which oracle should be returned when in doubt?
  void desiredOracle(Oracles oracle) { _desiredOracle = oracle; }

  typedef Eigen::Matrix<Precision, Dim, Eigen::Dynamic,
                        Eigen::RowMajor + Eigen::AutoAlign>
      PointList;
  typedef Eigen::Matrix<Precision, Dim, 1> Point;

  typedef SpatialOracle<PointList, Point, Precision> Oracle;

  struct Query {
    /// maximum range query you intend to perform
    Precision maxR = -1;
    /// Dimensions of bounding box (bb_max - bb_min)
    Point *bb_size = nullptr;
    /// Characteristic data
    PointList *example_data = nullptr;
    /// Dimensionality of data points
    int dimensions = -1;
  };

  /// Fill a Query with the parameters you know.
  /// The factory will try to figure out the best oracle
  /// for your prior knowledge.
  ///
  /// The query and it's referenced values only need to exist during the call of
  /// the method.
  std::unique_ptr<Oracle> forQuery(const Query query) const {
    switch (desiredOracle()) {
    case Oracles::BRUTE_FORCE:
      return getBruteForce();
    case Oracles::FLANN:
      return getFlann();
    case Oracles::UNIFORM_GRID:
      Precision maxR, cellSize;
      if (query.maxR > 0) {
        maxR = query.maxR;
      } else if (query.bb_size != nullptr) {
        maxR = query.bb_size->norm() * 2;
      } else if (query.example_data != nullptr) {
        auto max = query.example_data->array().colwise().maxCoeff();
        auto min = query.example_data->array().colwise().minCoeff();
        Point size = max - min;
        maxR = size.norm() * 2.0;
      } else {
        BOOST_LOG_TRIVIAL(info) << "Query contains not enought data to create "
                                   "a UniformGrid, falling back to BruteForce";
        return getBruteForce();
      }
      // maxR is known at this point, refine cellSize if possible
      cellSize = maxR / 2;
      if (query.bb_size != nullptr) {
        Precision candidate = query.bb_size->minCoeff() / 10;
        cellSize = std::min(candidate, cellSize);
      } else if (query.example_data != nullptr) {
        auto max = query.example_data->array().colwise().maxCoeff();
        auto min = query.example_data->array().colwise().minCoeff();
        auto size = max - min;
        Precision candidate = size.minCoeff() / 10;
        cellSize = std::min(candidate, cellSize);
      }
      int dimensions = -1;
      if (query.dimensions != -1) {
        dimensions = query.dimensions;
      } else if (query.bb_size != nullptr) {
        dimensions = query.bb_size->size();
      } else if (query.example_data != nullptr) {
        dimensions = query.example_data->rows();
      }
      if (Dim == -1 && dimensions == -1) {
        throw "Dynamic sized spatial oracle needs to know the data "
              "dimensionality for UniformGrid.";
      }

      return getUniformGrid(maxR, cellSize, dimensions);
    }

    return notFoundFallback();
  }

  /// No prior knowledge given, just choose something
  std::unique_ptr<Oracle> forGeneral() const {
    switch (desiredOracle()) {
    case Oracles::BRUTE_FORCE:
      return getBruteForce();
    case Oracles::FLANN:
      return std::make_unique<Flann<Precision, Dim>>();
    case Oracles::UNIFORM_GRID:
      BOOST_LOG_TRIVIAL(info) << "UniformGrid not supported for general "
                                 "request, falling back to BruteForce.";
      return getBruteForce();
    }
    return notFoundFallback();
  }

private:
  /// At the moment we strictly base our choice on this value.
  ///
  /// BruteForce always works but is slow.
  Oracles _desiredOracle = BRUTE_FORCE;

  std::unique_ptr<Oracle> notFoundFallback() const {
    BOOST_LOG_TRIVIAL(warning)
        << "Unknown desiredOracle encountered, falling back to BruteForce.";
    return getBruteForce();
  }

  std::unique_ptr<Oracle> getUniformGrid(Precision maxR, Precision cellSize,
                                         int dimensions) const {
    BOOST_LOG_TRIVIAL(debug)
        << "creating UniformGrid oracle with maxR=" << maxR
        << ", cellSize=" << cellSize << ", dimensions: " << dimensions;
    return std::make_unique<UniformGrid<Precision, Dim>>(maxR, cellSize,
                                                         dimensions);
  }
  std::unique_ptr<Oracle> getFlann() const {
    BOOST_LOG_TRIVIAL(debug) << "creating Flann oracle";
    return std::make_unique<Flann<Precision, Dim>>();
  }
  std::unique_ptr<Oracle> getBruteForce() const {
    BOOST_LOG_TRIVIAL(debug) << "creating Brute force oracle";
    return std::make_unique<BruteForce<Precision, Dim>>();
  }
};

typedef OracleFactory<double, -1> OracleFactoryXd;
typedef OracleFactory<double, 1> OracleFactory1d;
typedef OracleFactory<double, 2> OracleFactory2d;
typedef OracleFactory<double, 3> OracleFactory3d;
typedef OracleFactory<double, 4> OracleFactory4d;
typedef OracleFactory<double, 5> OracleFactory5d;

typedef OracleFactory<float, -1> OracleFactoryXf;
typedef OracleFactory<float, 1> OracleFactory1f;
typedef OracleFactory<float, 2> OracleFactory2f;
typedef OracleFactory<float, 3> OracleFactory3f;
typedef OracleFactory<float, 4> OracleFactory4f;
typedef OracleFactory<float, 5> OracleFactory5f;

} // namespace MouseTrack
