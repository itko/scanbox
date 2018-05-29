/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "clustering.h"

namespace MouseTrack {

/// Assigns a point to the most likely cluster as defined by the point's
/// `labels()`
///
/// A point will be assigned to a "rubbish" cluster, if no label is above
/// `_rejectionThreshold`.
class LabelClustering : public Clustering {
public:
  std::vector<Cluster> operator()(const PointCloud &cloud) const;

private:
  Precision _rejectionThreshold = 0.2;
};

} // namespace MouseTrack
