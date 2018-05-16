/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "cube_iterator.h"

#include <Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace MouseTrack {

namespace SpatialImpl {

// typedefs
template <int _Dim> using CellCoordinate = typename Eigen::Matrix<int, _Dim, 1>;

// declarations
template <int _Dim> class CubicNeighborhoodLayer;

/// Ideally, a neighborhood would correspond to all cells with distance [r,R] to
/// the reference cell (for a [r,R] as tight as possible) (they would build one
/// cube thick shells of a sphere) As a first implementation we'll use the
/// shells of a cube (from inside to outside)
template <int _Dim> class CubicNeighborhood {
  std::vector<CubicNeighborhoodLayer<_Dim>> _layers;
  void _createLayersUpTo(const int maxLayer, const int dims) {
    assert(maxLayer >= 0);
    CubeIterator<_Dim, CellCoordinate<_Dim>> iter(2 * maxLayer + 1, dims);
    CubeIterator<_Dim, CellCoordinate<_Dim>> end;
    /// move cube from [0,2*maxLayer] to [-maxLayer, maxLayer]
    CellCoordinate<_Dim> toCenter;
    if (_Dim == -1) {
      toCenter.resize(dims);
    }
    toCenter.setConstant(dims, -maxLayer);
    std::vector<std::vector<CellCoordinate<_Dim>>> layerCells(maxLayer + 1);
    for (; iter != end; ++iter) {
      auto centered = *iter + toCenter;
      int highestDimension = centered.array().abs().maxCoeff();
      assert(highestDimension <= maxLayer);
      assert(0 <= highestDimension);
      layerCells[highestDimension].push_back(centered);
    }

    _layers.resize(maxLayer + 1);
    for (int i = 0; i < maxLayer + 1; i += 1) {
      _layers[i] = CubicNeighborhoodLayer<_Dim>(std::move(layerCells[i]), dims);
    }
  }

public:
  /// maxLayer: index of outtest layer you intend to access
  /// 0th layer: only the central cube is generated
  /// 1st layer: coordinates for which the largest component magnitude is equal
  /// 1
  ///
  /// `dims`: number of dimensions, ignored if defined by template
  CubicNeighborhood(int maxLayer, int dims = -1) {
    int d = _Dim == -1 ? dims : _Dim;
    if (_Dim == -1 && dims <= 0) {
      throw "Dynamic sized CubicNeighborhood needs positive number of "
            "dimensions.";
    }
    _createLayersUpTo(maxLayer, d);
  }
  /// creates empty neighborhood
  CubicNeighborhood() {
    // empty
  }
  int size() const { return _layers.size(); }
  const CubicNeighborhoodLayer<_Dim> &operator[](int l) const {
    return _layers[l];
  }
};

template <int _Dim> class CubicNeighborhoodLayer {
  double minDist;
  double maxDist;
  /// holds a list of grid cells corresponding to the desired
  /// layer `l`
  std::vector<CellCoordinate<_Dim>> coordinates;

public:
  CubicNeighborhoodLayer() {
    // empty
  }
  /// Construct cells of layer `l`
  CubicNeighborhoodLayer(std::vector<CellCoordinate<_Dim>> &&cells,
                         int dims = -1) {
    int dim = _Dim == -1 ? dims : _Dim;
    if (_Dim == -1 && dims <= 0) {
      throw "Dynamic sized neighborhood layer needs positive number of "
            "dimension.";
    }
    coordinates = std::move(cells);
    int layer = coordinates[0].array().abs().maxCoeff();
    minDist = std::max(0.0, .5 + layer - 1);
    maxDist = std::sqrt(dim * std::pow(.5 + layer, 2));
  }
  const CellCoordinate<_Dim> operator[](int i) const { return coordinates[i]; }
  int size() const { return coordinates.size(); }
  /// minimal point distance from center cell measured in cell counts
  double min() const { return minDist; }
  /// maximal point distance from center cell measured in cell counts
  double max() const { return maxDist; }
};

} // namespace SpatialImpl

} // namespace MouseTrack
