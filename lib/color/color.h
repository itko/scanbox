/** \file
 * color.h
 *
 *  Created on: Apr 15, 2018
 *      Author: itto
 */

#include "generic/point_cloud.h"
#include <vector>

#pragma once

namespace MouseTrack {

std::vector<double> GenerateRandomColor();

std::vector<double> ColorizePointCloud(PointCloud &pc);

} // namespace MouseTrack
