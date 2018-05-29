/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "types.h"

#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace MouseTrack {

/// A point cloud holds a list of 3D points. Each point can have additional
/// attributes like color intensity.
class PointCloud {
  // Design note: In general, one has to decide betwen two basic concepts:
  // - Structure of Arrays
  // - Array of Structures
  // I would like to go for SoA, since it has the potential to be very efficient
  // (vector operations) But since it is a pain to work with it, I'd like to
  // implement a slim abstraction level (Point) providing accessors and the
  // concept of single points (we basically emulate AoS). Never done this
  // before, should be fun.
private:
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int Z = 2;
  static constexpr int POS_DIM = 3;
  static constexpr int R = 0;
  static constexpr int G = 0;
  static constexpr int B = 0;
  static constexpr int COL_DIM = 3;

public:
  /// type of a label entry:
  ///
  /// 0 means "not this label"
  ///
  /// 1 means "very much this label"
  ///
  /// Values inbetween show insecurity
  typedef double Label;
  typedef Eigen::Matrix<Label, -1, 1> LabelVec;
  typedef Eigen::Block<Eigen::MatrixXd, -1, 1, true> LabelVecOut;
  typedef Eigen::Block<const Eigen::MatrixXd, -1, 1, true> LabelVecConstOut;
  typedef Eigen::Matrix<Coordinate, POS_DIM, 1> PosVec;
  typedef Eigen::Block<const Eigen::Matrix<Coordinate, POS_DIM, -1>, POS_DIM, 1,
                       true>
      PosVecConstOut;
  class Point;
  class ConstantPoint;

  /// Exactly the same as `Point` but it only provides read-only access to the
  /// data.
  class ConstantPoint {
    /// `Point` and `ConstantPoint` are a little bit hacky:
    ///
    /// - `ConstantPoint` provides all read acessors
    ///
    /// - `Point` provides all write accessors and also inherits from
    /// `ConstantPoint` to reuse the read accessors
    ///
    /// We can't inverse the inheritance order, as a `ConstantPoint` with write
    /// accessors wouldn't really make sense.
    ///
    /// Why is it hacky?
    ///
    /// Let's return to the conceptual level and forget the inheritance for a
    /// moment:
    ///
    /// Both classes need a reference to the underlying `PointCloud`.
    ///
    /// If point cloud wants to return a `ConstantPoint` to a client,
    /// `ConstantPoint` needs to promise, not to change point cloud.
    ///
    /// But `Point` should be allowed to modify the underlying cloud.
    ///
    /// There are ways, to make the type system happy:
    ///
    /// - Create two abstract Read/Write classes with protected
    /// cloud()=0/constCloud()=0 methods and let the point classes inherit from
    /// them. Then Point no longer needs to inherit from ConstantPoint, can
    /// store its own reference and everything is fine. The problem: I try to
    /// make this class as efficient as possible, but having virtual methods
    /// doesn't seem as slim as possible.
    ///
    /// - Introduce duplicated code for read-operations: Both classes are
    /// completely independent but we get penalized by having a larger code
    /// size and a higher chance of bugs since we need to maintain redundant
    /// code.
    ///
    /// - Probably better/cleaner solutions then the one implemented, please
    /// tell me in that case.
    ///
    /// The most efficient way seems to let `Point` inherit from
    /// `ConstantPoint` and make sure that `ConstantPoint` never breaks its
    /// promise on its own.
    ///
    /// Single exception: It is allowed to do one thing:
    /// hand on a modifyable reference to its child as it is the
    /// only purpose of `Point` to actually modify things.
    ///
    /// This definitely breaks the one or other OO-principle, but I haven't
    /// found a cleaner way to get rid of all unnecessary overhead.
    friend class PointCloud;

  private:
    ConstantPoint(const PointCloud &cloud, size_t i);

    const PointCloud &_cloud;
    size_t _index;

  protected:
    PointCloud &cloud() const;
    const PointCloud &constCloud() const;
    size_t index() const;

  public:
    /// Read access to x coordinate.
    const Coordinate &x() const;

    /// Read access to y coordinate.
    const Coordinate &y() const;

    /// Read access to z coordinate.
    const Coordinate &z() const;

    /// Read access to color r.
    const ColorChannel &r() const;

    /// Read access to color g.
    const ColorChannel &g() const;

    /// Read access to color b.
    const ColorChannel &b() const;

    /// Read access to color intensity.
    ColorChannel intensity() const;

    /// Read access on labels
    LabelVecConstOut labels() const;

    /// Convert to dx1 Eigen Vector holding all characteristic values
    Eigen::VectorXd characteristic() const;

    /// Position as eigen column vector
    PosVecConstOut pos() const;
  };

  /// The Point class is a collection of accessors allowing to manipulate the
  /// values inside the PointCloud in an intuitive way.
  class Point : public ConstantPoint {

    // Design note: I tend to keep the hierarchy flat,
    // this way we can implement it by storing a reference to the PointCloud
    // and an index, and nothing more.
    friend class PointCloud;

    /// Create a Point instance that manipulates the i-th point of cloud
    Point(PointCloud &cloud, size_t i);

  public:
    // clang-format off
    using ConstantPoint::x;
    using ConstantPoint::y;
    using ConstantPoint::z;
    using ConstantPoint::r;
    using ConstantPoint::g;
    using ConstantPoint::b;
    using ConstantPoint::intensity;
    using ConstantPoint::labels;
    // clang-format on

    /// Write access to x coordinate.
    void x(const Coordinate &_new);

    /// Write access to y coordinate.
    void y(const Coordinate &_new);

    /// Write access to z coordinate.
    void z(const Coordinate &_new);

    /// Write access to color r.
    void r(const ColorChannel &_new);

    /// Write access to color g.
    void g(const ColorChannel &_new);

    /// Write access to color b.
    void b(const ColorChannel &_new);

    /// Write access: set `r = g = b = _new`
    void intensity(const ColorChannel &_new);

    /// Copies labels
    ///
    /// 0: This point does not hold this label (non existing entries are to
    /// zero)
    ///
    /// 1: This point very much holds this label
    void labels(const LabelVec &newLabels);

    /// Moves labels
    void labels(LabelVec &&newLabels);

    /// assignment
    void operator=(const Point &other);

    /// assignment
    void operator=(const ConstantPoint &other);
  };

  PointCloud();

  /// Make space to accomodate n points and `labelsCount` labels for each point.
  void resize(size_t n, size_t labelsCount = 0);

  /// Returns number of points stored.
  size_t size() const;

  /// Read-write access to i-th point.
  Point operator[](size_t i);

  /// Read-only access to i-th point.
  const ConstantPoint operator[](size_t i) const;

  /// How many labels are there?
  int labelsDim() const;

  /// How many characteristic dimensions are there?
  int charDim() const;

  /// min corner of bounding box (all characteristic dimensions)
  Eigen::VectorXd charMin() const;

  /// max corner of bounding box (all characteristic dimensions)
  Eigen::VectorXd charMax() const;

  /// min corner of bounding box (only 3d position of points)
  PosVec posMin() const;

  /// max corner of bounding box (only 3d position of points)
  PosVec posMax() const;

  /// Center of gravity/centroid of point cloud
  PosVec posCog() const;

private:
  Eigen::Matrix<Coordinate, POS_DIM, -1> _pos;
  Eigen::Matrix<ColorChannel, COL_DIM, -1> _col;
  Eigen::Matrix<Label, -1, -1> _labels;
};

} // namespace MouseTrack
