/// \file
/// Maintainer: Felice Serena
///

#include "subsample.h"
#include "generic/random_sample.h"
#include <boost/log/trivial.hpp>

namespace MouseTrack {

SubSample::SubSample(int desiredMaxPoints)
    : _desiredMaxPoints(desiredMaxPoints) {
  // empty
}

void SubSample::desiredMaxPoints(int _new) { _desiredMaxPoints = _new; }

int SubSample::desiredMaxPoints() const { return _desiredMaxPoints; }

PointCloud SubSample::operator()(const PointCloud &inCloud) const {
  PointCloud cloud = random_sample(inCloud, _desiredMaxPoints);
  auto min = cloud.posMin();
  auto max = cloud.posMax();

  BOOST_LOG_TRIVIAL(debug) << "Subsampled point cloud to " << cloud.size()
                           << " points, xyz-min: [" << min[0] << ", " << min[1]
                           << ", " << min[2] << "], xyz-max: [" << max[0]
                           << ", " << max[1] << ", " << max[2]
                           << "] with desired points " << _desiredMaxPoints;
  return cloud;
}

} // namespace MouseTrack
