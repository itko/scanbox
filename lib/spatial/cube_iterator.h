/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include <cassert>

namespace MouseTrack {

namespace SpatialImpl {

/// Generates grid points with indices `[0, width)^_Dim`
///
/// CellCoordiante needs to prived a `operator[size_t]` accessor
/// and `resize(size_t)`.
template <int _Dim, typename CellCoordinate> class CubeIterator {
  /// active cube cell to which we're pointing
  CellCoordinate _it;

  /// length of one 2d-edge
  int _width;

  /// Marks special case of end iterator
  bool _end = false;

  int dimensions() const {
    if (_Dim == -1) {
      return _it.size();
    }
    return _Dim;
  }

  void setZero(CellCoordinate &coord) const {
    for (int d = 0; d < dimensions(); ++d) {
      coord[d] = 0;
    }
  }

public:
  /// Default constructor creates an end-iterator, don't dereference it
  CubeIterator() : _end(true) {
    // empty
  }

  /// Create an iterator which points to 0.
  /// Ignores `dims` if the number of dimensions is fixed by template
  CubeIterator(int width, int dims = -1) : _width(width) {
    if (_Dim > 0) {
      _it.resize(_Dim);
    } else if (_Dim == -1 && width > 0) {
      assert(dims != -1);
      _it.resize(dims);
    }
    setZero(_it);
  }

  /// create an iterator which points to `start`
  CubeIterator(int width, CellCoordinate start) : _it(start), _width(width) {
    // empty
  }

  const CellCoordinate &operator*() const { return _it; }

  const CellCoordinate *operator->() const { return &_it; }

  CubeIterator &operator++() {
    _increment();
    return *this;
  }

  /// Behavior for iterators of different box sizes is undefined
  bool operator==(const CubeIterator &other) const {
    if (other._end || this->_end) {
      return other._end == this->_end;
    }
    return other._it == _it;
  }

  bool operator!=(const CubeIterator &other) const {
    if (other._end || this->_end) {
      return other._end != this->_end;
    }
    return other._it != _it;
  }

private:
  void _increment() {
    for (int d = 0; d < dimensions(); d += 1) {
      _it[d] += 1;
      if (_it[d] < _width) {
        return;
      }
      _it[d] = 0;
    }
    // overflow: carry passed on to d+1
    _end = true;
  }
};

} // namespace SpatialImpl

} // namespace MouseTrack
