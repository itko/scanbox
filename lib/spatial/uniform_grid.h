/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "cubic_neighborhood.h"
#include "spatial_oracle.h"
#include <Eigen/Core>
#include <boost/log/trivial.hpp>
#include <queue>

namespace std {

template <typename Scalar, int Rows, int Cols>
class hash<Eigen::Matrix<Scalar, Rows, Cols>> {
public:
  size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols> &mat) const {
    // based on: http://de.cppreference.com/w/cpp/utility/hash/operator()
    size_t result = 2166136261;
    for (int i = 0; i < mat.rows(); ++i) {
      for (int j = 0; j < mat.cols(); ++j) {
        result = j * 86845 ^ i * 123421 ^ std::hash<Scalar>()(mat(i, j)) ^
                 (result * 16777619);
      }
    }
    return result;
  }
};

} // namespace std

namespace MouseTrack {

using namespace SpatialImpl;

/// Creates a grid and stores the indices of the corresponding points in each
/// cell. Each grid cell is cube shaped, but does not have unit width.
///
///
/// Note: this is a decent method for low dimensions,
/// But the number of cells grows exponentially in the number of dimensions, so
/// take care with higher dimensions.
template <typename _Precision, int _Dim>
class UniformGrid
    : public SpatialOracle<Eigen::Matrix<_Precision, _Dim, Eigen::Dynamic,
                                         Eigen::RowMajor + Eigen::AutoAlign>,
                           Eigen::Matrix<_Precision, _Dim, 1>, _Precision> {
public:
  typedef Eigen::Matrix<_Precision, _Dim, Eigen::Dynamic,
                        Eigen::RowMajor + Eigen::AutoAlign>
      PointList;
  typedef Eigen::Matrix<_Precision, _Dim, 1> Point;
  typedef _Precision Precision;

private:
  /// A word about coordiante systems, there are three:
  ///
  /// - Point uses a floating point coordinate system.
  ///
  /// - Bounding box grid: a grid from [0, resolution), it has
  ///   discrete indexes and is aligned with the `bb_` coordinates.
  ///
  /// - Neighbor grid: relative indices in [-maxLayer, maxLayer] to
  ///   walk the neighborhood in the bounding box grid.

  /// type alias
  typedef Eigen::Matrix<int, _Dim, 1> GridCellIndex;
  typedef CellCoordinate<_Dim> Cell;
  typedef CubicNeighborhood<_Dim> _Neighborhood;

  /// Points over which we perform queries
  const PointList *points = nullptr;

  /// Minimum point of bounding box
  Point bb_min;

  /// Maximum point of bounding box
  Point bb_max;

  /// size of bounding box
  Point bb_size;

  /// Width of a cell
  Precision cellWidth;

  _Neighborhood neighborhood;

  GridCellIndex resolution;

  /// largest dimension of grid
  int maxDiameter;

  std::unordered_map<Cell, std::vector<PointIndex>> grid;

  Cell indexOfPosition(const Point &p) const {
    auto normalized = ((p - bb_min) / cellWidth).array().floor();
    Cell index = normalized.template cast<int>();
    return index;
  }

  bool in_bb(const Cell &cell) const {
    for (int d = 0; d < cell.rows(); d += 1) {
      if (cell[d] < 0 || resolution[d] <= cell[d]) {
        return false;
      }
    }
    return true;
  }
  /// recache data
  void _compute() {
    assert(points != nullptr);
    if (points->size() == 0) {
      return;
    }
    bb_min = points->rowwise().minCoeff();
    bb_max = points->rowwise().maxCoeff();
    bb_size = bb_max - bb_min;

    // add some buffer for rounding errors
    bb_min -= bb_size * .1;
    bb_max += bb_size * .1;

    bb_size = bb_max - bb_min;
    if (_Dim == -1) {
      resolution.resize(bb_size.rows());
    }

    for (int d = 0; d < bb_size.rows(); d += 1) {
      resolution[d] = std::max(1.0, std::ceil(bb_size[d] / cellWidth));
    }

    maxDiameter = resolution.array().maxCoeff();

    // create grid
    grid.erase(grid.begin(), grid.end());

    // fill grid with indices
    for (int i = 0; i < points->cols(); i += 1) {
      auto j = indexOfPosition(points->col(i));
      auto &vec = grid[j];
      vec.push_back(i);
    }
  }

public:
  /// Create a grid with a cell size of `cellWidth` and support
  /// for range queries of up to `maxR`.
  UniformGrid(Precision maxR, Precision cellWidth, int dims = -1)
      : cellWidth(cellWidth) {
    assert(cellWidth > 0);
    neighborhood = _Neighborhood(std::ceil(maxR / cellWidth) + 1, dims);
  }

  virtual void compute(const PointList &src) {
    points = &src;
    _compute();
  }

  virtual std::vector<PointIndex> find_closest(const Point &p,
                                               unsigned int k) const {
    assert(points != nullptr);
    assert(k >= 1);
    // idea: we define hollow cubes around the cell containing p, called layers
    // we search the layers from the inside to the outside
    // if we have enough closest candidates, we stop as soon as our active
    // layer's closest point is above our furthest candidate
    auto zeroCell = indexOfPosition(p);
    typedef std::pair<Precision, PointIndex> P;
    std::priority_queue<P> neighbors;
    neighbors.push(P(std::numeric_limits<Precision>::max(), (PointIndex)-1));
    // We only loop up to the largest bounding box dimension (assumes cube
    // shells)
    for (int l = 0; l < neighborhood.size(); l += 1) {
      const auto &layer = neighborhood[l];
      double layerDist = layer.min() * cellWidth;
      if (neighbors.top().first < layerDist * layerDist) {
        // the closest point in the layer is further away than our worst
        // candidate we can stop
        break;
      }
      for (int i = 0; i < layer.size(); ++i) {
        Cell cell = zeroCell + layer[i];
        // skip, if outside of bounding box
        if (!in_bb(cell)) {
          continue;
        }
        // we are inside the bounding box and have to check against all verts in
        // the cell
        const auto candidates = grid.find(cell);
        if (candidates == grid.end()) {
          // no points stored in this cell
          continue;
        }
        for (PointIndex cIndex : candidates->second) {
          double dist = (points->col(cIndex) - p).squaredNorm();
          if (dist < neighbors.top().first) {
            neighbors.push(P(dist, cIndex));
            if (neighbors.size() > k) {
              neighbors.pop();
            }
          }
        }
      }
    }
    std::vector<PointIndex> result;
    while (neighbors.size() > 0) {
      auto i = neighbors.top().second;
      if (i != (PointIndex)-1) {
        result.push_back(i);
      }
      neighbors.pop();
    }
    return result;
  }

  virtual std::vector<PointIndex> find_in_range(const Point &p,
                                                const Precision r) const {
    assert(points != nullptr);
    std::vector<PointIndex> range;
    auto zeroCell = indexOfPosition(p);
    const double r2 = r * r;
    for (int l = 0; l <= neighborhood.size(); l += 1) {
      const auto &layer = neighborhood[l];
      if (r < layer.min() * cellWidth) {
        break;
      }
      for (int i = 0; i < layer.size(); i += 1) {
        Cell cell = zeroCell + layer[i];
        // skip, if outside of bounding box
        if (!in_bb(cell)) {
          continue;
        }
        // we are inside the bounding box and have to check against all verts in
        // the cell
        const auto candidates = grid.find(cell);
        if (candidates == grid.end()) {
          // no points stored in this cell
          continue;
        }
        for (PointIndex c : candidates->second) {
          auto &v = points->col(c);
          double dist = (v - p).squaredNorm();
          if (dist <= r2) {
            range.push_back(c);
          }
        }
      }
    }
    return range;
  }
};

typedef UniformGrid<double, -1> UniformGridXd;
typedef UniformGrid<double, 1> UniformGrid1d;
typedef UniformGrid<double, 2> UniformGrid2d;
typedef UniformGrid<double, 3> UniformGrid3d;
typedef UniformGrid<double, 4> UniformGrid4d;
typedef UniformGrid<double, 5> UniformGrid5d;

typedef UniformGrid<float, -1> UniformGridXf;
typedef UniformGrid<float, 1> UniformGrid1f;
typedef UniformGrid<float, 2> UniformGrid2f;
typedef UniformGrid<float, 3> UniformGrid3f;
typedef UniformGrid<float, 4> UniformGrid4f;
typedef UniformGrid<float, 5> UniformGrid5f;

} // namespace MouseTrack
