/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "classifier.h"
#include "spatial/oracle_factory.h"

#include <memory>

namespace MouseTrack {

class KnnClassifier : public Classifier {
public:
  typedef SpatialOracle<Mat, Eigen::VectorXd, double> Oracle;
  typedef OracleFactory<Precision, -1> OFactory;

  KnnClassifier();

  OFactory &oracleFactory();
  const OFactory &oracleFactory() const;

  virtual void fit(const Mat &X_train, const Vec &y_train);

  virtual Vec predict(const Mat &X_test) const;

  virtual Mat predictProbabilities(const Mat &X_test) const;

  int k() const;
  void k(int newK);

private:
  OFactory _oracleFactory;
  std::unique_ptr<Oracle> _oracle;
  Mat _X_train;
  Vec _y_train;

  // number of nearest neighbors to consider
  int _k = 5;
  int _highestLabel;
};

} // namespace MouseTrack
