/** \file
 * color.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: itto
 */

#include "color.h"
#include <opencv2/contrib/contrib.hpp>
#include <time.h>

using namespace cv;

namespace MouseTrack {

std::vector<double> GenerateRandomColor() {
  // Random variable
  // TODO add parameter for number of clusters that will determine how many
  // colors to output
  srand(time(NULL));
  int i = rand() % 64;
  // Now multiply to get the intensity
  int intensity = i * 4;
  // Create dummy image with the intensity and dummy 0 and full intensity
  int data[3] = {intensity, 0, 255};
  Mat dummy(1, 3, CV_8UC1, data);
  // Dummy output image
  Mat coloured;
  // Apply colour map on dummy
  applyColorMap(dummy, coloured, COLORMAP_JET);
  // Get the colour from the new image
  auto red = coloured.at<Vec3b>(0, 0)[2];
  auto green = coloured.at<Vec3b>(0, 0)[1];
  auto blue = coloured.at<Vec3b>(0, 0)[0];

  return std::vector<double>{red, green, blue};
}

std::vector<double> ColorizePointCloud(PointCloud &pc) {
  // Get random color
  auto colour = GenerateRandomColor();
  for (size_t i = 0; i < pc.size(); i++) {
    // Set colours
    pc[i].r(colour[0]);
    pc[i].b(colour[1]);
    pc[i].g(colour[2]);
  }
  // Return the colour
  return colour;
}

} // namespace MouseTrack
