/** \file
 * color.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: itto
 */

#include "color.h"
#include <boost/log/trivial.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <time.h>

using namespace cv;

namespace MouseTrack {

/// Will return a list of N rgb values that are as different from each other as
/// possible
std::vector<std::vector<double>> GenerateNColors(int n) {
  int nmod = std::min(256, n);
  std::vector<unsigned char> intensity;
  if (n == 1) {
    intensity.push_back(0);
  } else {
    // Grayscale intensity should be equally spread out
    for (unsigned char i = 0; i < (nmod - 1); i++) {
      intensity.push_back(256. / (double)(nmod - 1) * i);
    }
    // Last value is 255
    intensity.push_back(255);
  }
  Mat dummy(1, intensity.size(), CV_8UC1, intensity.data());
  // Dummy output image
  Mat coloured;
  applyColorMap(dummy, coloured, COLORMAP_JET);
  std::vector<std::vector<double>> colours;
  for (size_t i = 0; i < intensity.size(); ++i) {
    colours.push_back({coloured.at<Vec3b>(0, i)[2], coloured.at<Vec3b>(0, i)[1],
                       coloured.at<Vec3b>(0, i)[0]});
  }
  for (int i = 256; i < n; ++i) {
    colours.push_back(colours[i % 256]);
  }
  return colours;
}

/// Will return a random colour with the format [r, g, b]
/// Note: currently limited to 64 different colours
std::vector<double> GenerateRandomColor() {
  // Random variable
  int i = rand() % 64;
  // Now multiply to get the intensity
  int intensity = i * 4;
  // Create dummy image with the intensity and 0 and full intensity
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

} // namespace MouseTrack
