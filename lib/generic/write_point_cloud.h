/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "generic/point_cloud.h"

namespace MouseTrack {

void write_point_cloud(const std::string &path, const PointCloud &cloud);

void write_point_cloud_metrics(const std::string &path,
                               const PointCloud &cloud);
} // namespace MouseTrack
