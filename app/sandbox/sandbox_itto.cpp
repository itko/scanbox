/** \file
 * sandbox_itto.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: itto
 */

#include "color/color.h"
#include "generic/point_cloud.h"

using namespace MouseTrack;
int main(int argc, char *argv[]) {
  auto pc = PointCloud();
  pc.resize(20);
  ColorizePointCloud(pc);
  return 0;
}
