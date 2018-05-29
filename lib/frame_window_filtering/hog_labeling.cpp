/// \file
/// Maintainer: Felice Serena
///
///

#include "hog_labeling.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <boost/log/trivial.hpp>

#include <classifier/knn.h>

namespace MouseTrack {

/// Returns a n x 2 matrix of (x,y) pairs of valid corner coordinates for
/// sliding windows of the desired size such that they are within the given
/// image dimensions.
///
std::vector<cv::Point> slidingWindows(int imgWidth, int imgHeight, int stepSize,
                                      int windowWidth, int windowHeight) {
  std::vector<cv::Point> result;
  for (int x = 0; x < imgWidth - windowWidth; x += stepSize) {
    for (int y = 0; y < imgHeight - windowHeight; y += stepSize) {
      result.push_back(cv::Point(x, y));
    }
  }
  return result;
} // namespace MouseTrack

void HogLabeling::train(const Mat &X_train, const Vec &y_train) {
  auto ptr = std::make_unique<KnnClassifier>();
  ptr->fit(X_train, y_train);
  ptr->k(11);
  _classifier = std::move(ptr);
  _numLabels = y_train.maxCoeff() + 1;
}

FrameWindow HogLabeling::operator()(const FrameWindow &window) const {
  if (window.frames().empty()) {
    return window;
  }
  if (_classifier.get() == nullptr) {
    BOOST_LOG_TRIVIAL(warning)
        << "HOG classifier not trained, no labeling performed.";
    return window;
  }
  FrameWindow result = window;
  // create matrices for labels
  for (Frame &f : result.frames()) {
    f.labels.resize(_numLabels);
    for (auto &l : f.labels) {
      l.setZero(f.referencePicture.rows(), f.referencePicture.cols());
    }
  }

  // hog settings
  cv::Size windowSize(_windowWidth, _windowHeight);
  cv::Size blockSize(_blockWidth, _blockHeight);
  cv::Size blockStride(_blockStrideWidth, _blockStrideHeight);
  cv::Size cellSize(_cellWidth, _cellHeight);

  // create hog descriptor: it is able to tansform images patches to feature
  // vectors
  cv::HOGDescriptor hog(windowSize, blockSize, blockStride, cellSize, _nbins);

  // compute settings
  cv::Size windowStride;
  cv::Size padding;

  // build locations for sliding window
  const auto &first = result.frames()[0];
  std::vector<cv::Point> locations = slidingWindows(
      first.referencePicture.cols(), first.referencePicture.rows(), _stepSize,
      _windowWidth, _windowHeight);

  BOOST_LOG_TRIVIAL(trace) << "Created " << locations.size()
                           << " sliding window locations to check.";

  for (size_t f = 0; f < result.frames().size(); ++f) {
    BOOST_LOG_TRIVIAL(trace) << "Checking frame " << f;
    Frame &frame = result.frames()[f];
    cv::Mat img;
    PictureI eig = (frame.referencePicture * 255.0).cast<PictureI::Scalar>();
    cv::eigen2cv(eig, img);

    // holds `locations.size()` descriptors of size hog.getDescriptorSize()
    std::vector<float> descriptors;
    hog.compute(img, descriptors, windowStride, padding, locations);
    BOOST_LOG_TRIVIAL(trace)
        << "Found " << (descriptors.size() / hog.getDescriptorSize())
        << " HOG descriptors of size " << hog.getDescriptorSize() << " at "
        << locations.size() << " locations in frame " << f;
    Eigen::Map<Eigen::MatrixXf> map(descriptors.data(), hog.getDescriptorSize(),
                                    locations.size());
    Classifier::Mat tmp = map.cast<double>();
    BOOST_LOG_TRIVIAL(trace) << "Classifying...";
    auto labels = _classifier->predictProbabilities(tmp);
    BOOST_LOG_TRIVIAL(trace) << "Assigning...";
    // apply labels of windows to frame.labels
    for (size_t w = 0; w < locations.size(); ++w) {
      // iterate over pixels within window w
      for (int x = locations[w].x; x < locations[w].x + _windowWidth; ++x) {
        for (int y = locations[w].y; y < locations[w].y + _windowHeight; ++y) {
          // distribute labels
          for (int l = 0; l < _numLabels; ++l) {
            frame.labels[l](y, x) += labels(l, w);
          }
        }
      }
    }
    // normalize label weights
    if (_normalizeRange) {
      for (auto &l : frame.labels) {
        auto min = l.minCoeff();
        l = l.array() - min;
        auto max = l.maxCoeff();
        l = l.array() / max;
      }
    }

    // normalize accross labels
    if (_normalizeAccross) {
      PictureD sum;
      sum.setZero(frame.referencePicture.rows(), frame.referencePicture.cols());
      for (int l = 0; l < _numLabels; ++l) {
        sum += frame.labels[l];
      }
      // avoid division by zero
      sum = sum.array() + 0.0000001;
      sum = sum.array().inverse();
      for (int l = 0; l < _numLabels; ++l) {
        frame.labels[l] = frame.labels[l].array() * sum.array();
      }
    }
    BOOST_LOG_TRIVIAL(trace) << "Frame " << f << " finished.";
  }
  return result;
}

} // namespace MouseTrack
