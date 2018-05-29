/// \file
/// Maintainer: Felice Serena
///
///

#include "point_cloud.h"

namespace MouseTrack {

// PointCloud implementation

PointCloud::PointCloud() {
  // empty
}

void PointCloud::resize(size_t n, size_t labelsCount) {
  _pos.conservativeResize(POS_DIM, n);
  _col.conservativeResize(COL_DIM, n);
  _labels.conservativeResize(labelsCount, n);
}

size_t PointCloud::size() const { return _pos.cols(); }

PointCloud::Point PointCloud::operator[](size_t i) {
  return PointCloud::Point(*this, i);
}

const PointCloud::ConstantPoint PointCloud::operator[](size_t i) const {
  return PointCloud::ConstantPoint(*this, i);
}

int PointCloud::labelsDim() const { return _labels.rows(); }

int PointCloud::charDim() const {
  // position: 3
  // intensity: 1
  // labels: labelsDim()
  return POS_DIM + 1 + labelsDim();
}

Eigen::Vector3d PointCloud::posMin() const {
  Eigen::Vector3d min;
  min.setConstant(POS_DIM, std::numeric_limits<double>::max());

  for (PointIndex i = 0; i < size(); i += 1) {
    const auto &p = (*this)[i];
    min = min.array().min(p.pos().array());
  }
  return min;
}

Eigen::Vector3d PointCloud::posMax() const {
  Eigen::Vector3d max;
  max.setConstant(POS_DIM, std::numeric_limits<double>::min());

  for (PointIndex i = 0; i < size(); i += 1) {
    const auto &p = (*this)[i];
    max = max.array().max(p.pos().array());
  }
  return max;
}

PointCloud::PosVec PointCloud::posCog() const {
  PosVec posCog = _pos.rowwise().sum();
  if (size() == 0) {
    return posCog;
  }
  posCog /= size();
  return posCog;
}

Eigen::VectorXd PointCloud::charMin() const {
  Eigen::VectorXd min;
  min.setConstant(charDim(), std::numeric_limits<double>::max());

  for (PointIndex i = 0; i < size(); i += 1) {
    const auto &p = (*this)[i];
    min = min.array().min(p.characteristic().array());
  }
  return min;
}

Eigen::VectorXd PointCloud::charMax() const {
  Eigen::VectorXd max;
  max.setConstant(charDim(), std::numeric_limits<double>::min());

  for (PointIndex i = 0; i < size(); i += 1) {
    const auto &p = (*this)[i];
    max = max.array().max(p.pos().array());
  }
  return max;
}

// Point implementation

PointCloud::Point::Point(PointCloud &cloud, size_t i)
    : ConstantPoint(cloud, i) {
  // empty
}

void PointCloud::Point::x(const Coordinate &_new) {
  cloud()._pos(X, index()) = _new;
}

void PointCloud::Point::y(const Coordinate &_new) {
  cloud()._pos(Y, index()) = _new;
}

void PointCloud::Point::z(const Coordinate &_new) {
  cloud()._pos(Z, index()) = _new;
}

void PointCloud::Point::r(const ColorChannel &_new) {
  cloud()._col(R, index()) = _new;
}

void PointCloud::Point::g(const ColorChannel &_new) {
  cloud()._col(G, index()) = _new;
}

void PointCloud::Point::b(const ColorChannel &_new) {
  cloud()._col(B, index()) = _new;
}

void PointCloud::Point::intensity(const ColorChannel &_new) {
  cloud()._col(R, index()) = _new;
  cloud()._col(G, index()) = _new;
  cloud()._col(B, index()) = _new;
}

void PointCloud::Point::labels(const PointCloud::LabelVec &newLabels) {
  cloud()._labels.col(index()) = newLabels;
}

void PointCloud::Point::labels(PointCloud::LabelVec &&newLabels) {
  cloud()._labels.col(index()) = std::move(newLabels);
}

void PointCloud::Point::operator=(const Point &o) {
  x(o.x());
  y(o.y());
  z(o.z());
  r(o.r());
  g(o.g());
  b(o.b());
  labels(o.labels());
}

void PointCloud::Point::operator=(const ConstantPoint &o) {
  x(o.x());
  y(o.y());
  z(o.z());
  r(o.r());
  g(o.g());
  b(o.b());
  labels(o.labels());
}

// ConstantPoint implementation

PointCloud::ConstantPoint::ConstantPoint(const PointCloud &cloud, size_t i)
    : _cloud(cloud), _index(i) {
  // empty
}

// a little hack to provide write-access for `Point`
PointCloud &PointCloud::ConstantPoint::cloud() const {
  return const_cast<PointCloud &>(_cloud);
}

const PointCloud &PointCloud::ConstantPoint::constCloud() const {
  return _cloud;
}
size_t PointCloud::ConstantPoint::index() const { return _index; }

const Coordinate &PointCloud::ConstantPoint::x() const {
  return constCloud()._pos(X, index());
}

const Coordinate &PointCloud::ConstantPoint::y() const {
  return constCloud()._pos(Y, index());
}

const Coordinate &PointCloud::ConstantPoint::z() const {
  return constCloud()._pos(Z, index());
}

const ColorChannel &PointCloud::ConstantPoint::r() const {
  return constCloud()._col(R, index());
}

const ColorChannel &PointCloud::ConstantPoint::g() const {
  return constCloud()._col(G, index());
}

const ColorChannel &PointCloud::ConstantPoint::b() const {
  return constCloud()._col(B, index());
}

ColorChannel PointCloud::ConstantPoint::intensity() const {
  return (constCloud()._col(R, index()) + cloud()._col(G, index()) +
          cloud()._col(B, index())) /
         3.0;
}

PointCloud::LabelVecConstOut PointCloud::ConstantPoint::labels() const {
  return constCloud()._labels.col(_index);
}

Eigen::VectorXd PointCloud::ConstantPoint::characteristic() const {
  Eigen::VectorXd result(constCloud().charDim());
  result[0] = x();
  result[1] = y();
  result[2] = z();
  result[3] = intensity();
  result.block(4, 0, constCloud().labelsDim(), 1) = labels();
  return result;
}

PointCloud::PosVecConstOut PointCloud::ConstantPoint::pos() const {
  return constCloud()._pos.col(index());
}

} // namespace MouseTrack
