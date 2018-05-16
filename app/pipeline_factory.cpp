/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_factory.h"
#include "clustering/kmeans.h"
#include "clustering/mean_shift.h"
#include "clustering/mean_shift_cpu_optimized.h"
#include "clustering/single_cluster.h"
#include "descripting/cog.h"
#include "frame_window_filtering/background_subtraction.h"
#include "frame_window_filtering/disparity_bilateral.h"
#include "frame_window_filtering/disparity_gaussian_blur.h"
#include "frame_window_filtering/disparity_median.h"
#include "frame_window_filtering/disparity_morphology.h"
#include "matching/nearest_neighbour.h"
#include "matlab_reader.h"
#include "matlab_reader_concurrent.h"
#include "point_cloud_filtering/statistical_outlier_removal.h"
#include "point_cloud_filtering/subsample.h"
#include "registration/disparity_registration.h"
#include "registration/disparity_registration_cpu_optimized.h"
#include "trajectory_builder/cog_trajectory_builder.h"
#include <boost/log/trivial.hpp>

namespace MouseTrack {

Pipeline
PipelineFactory::fromCliOptions(const op::variables_map &options) const {
  if (options.count("src-dir") == 0) {
    BOOST_LOG_TRIVIAL(info) << "No command line source path given.";
  }
  std::unique_ptr<Reader> reader = getReader(options);

  std::vector<std::unique_ptr<FrameWindowFiltering>> windowFiltering =
      getWindowFilters(options);

  std::unique_ptr<Registration> registration{getRegistration(options)};

  std::vector<std::unique_ptr<PointCloudFiltering>> cloudFiltering{
      getCloudFilters(options)};

  std::unique_ptr<Clustering> clustering{getClustering(options)};

  std::unique_ptr<Descripting> descripting{getDescripting(options)};

  std::unique_ptr<Matching> matching{getMatching(options)};

  std::unique_ptr<TrajectoryBuilder> trajectoryBuilder{
      getTrajectoryBuilder(options)};
  // clang-format off
  return Pipeline(std::move(reader),
                  std::move(windowFiltering),
                  std::move(registration),
                  std::move(cloudFiltering),
                  std::move(clustering),
                  std::move(descripting),
                  std::move(matching),
                  std::move(trajectoryBuilder));
  // clang-format on
}

std::unique_ptr<Reader>
PipelineFactory::getReader(const op::variables_map &options) const {
  std::string target = options["pipeline-reader"].as<std::string>();
  if (target == "matlab" || target == "matlab-concurrent") {
    std::unique_ptr<MatlabReader> reader;
    if (target == "matlab") {
      reader = std::unique_ptr<MatlabReader>(
          new MatlabReader(options["src-dir"].as<std::string>()));
    } else {
      reader = std::unique_ptr<MatlabReader>(
          new MatlabReaderConcurrent(options["src-dir"].as<std::string>()));
    }
    if (options.count("first-frame")) {
      reader->setBeginFrame(
          std::max(reader->beginFrame(), options["first-frame"].as<int>()));
    }
    if (options.count("last-frame")) {
      reader->setEndFrame(
          std::min(reader->endFrame(), options["last-frame"].as<int>() + 1));
    }
    if (options.count("first-stream")) {
      reader->setBeginStream(
          std::max(reader->beginStream(), options["first-stream"].as<int>()));
    }
    if (options.count("last-stream")) {
      reader->setEndStream(
          std::min(reader->endStream(), options["last-stream"].as<int>() + 1));
    }
    return reader;
  }

  return nullptr;
}

std::vector<std::unique_ptr<FrameWindowFiltering>>
PipelineFactory::getWindowFilters(const op::variables_map &options) const {
  std::vector<std::unique_ptr<FrameWindowFiltering>> filters;
  if (options.count("pipeline-frame-window-filtering") == 0) {
    return filters;
  }
  std::vector<std::string> targets =
      options["pipeline-frame-window-filtering"].as<std::vector<std::string>>();
  for (const auto &target : targets) {
    auto ptr = getWindowFiltering(target, options);
    if (ptr.get() == nullptr) {
      continue;
    }
    filters.push_back(std::move(ptr));
  }
  return filters;
}

DisparityMorphology::KernelShape morphShapeToEnum(const std::string &shapeStr) {
  DisparityMorphology::KernelShape shape;
  if (shapeStr == "rect") {
    shape = DisparityMorphology::KernelShape::rect;
  } else if (shapeStr == "ellipse") {
    shape = DisparityMorphology::KernelShape::ellipse;
  } else if (shapeStr == "cross") {
    shape = DisparityMorphology::KernelShape::cross;
  } else {
    BOOST_LOG_TRIVIAL(warning)
        << "Unknown disparity-morphology-shape: " << shapeStr
        << ". Falling back to rect.";
    shape = DisparityMorphology::KernelShape::rect;
  }
  return shape;
}

std::unique_ptr<FrameWindowFiltering>
PipelineFactory::getWindowFiltering(const std::string &target,
                                    const op::variables_map &options) const {
  if (target == "disparity-gauss") {
    auto ptr =
        std::unique_ptr<DisparityGaussianBlur>(new DisparityGaussianBlur());
    int k = options["disparity-gauss-k"].as<int>();
    double sigma;
    if (options.count("disparity-gauss-sigma")) {
      sigma = options["disparity-gauss-sigma"].as<double>();
    } else {
      // By default, we scale the standard deviation proportional to the window
      // size.
      // Why exactly these numbers? Not sure, they come from OpenCV's
      // getGaussianKernel():
      // https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html?highlight=gaussianblur#Mat%20getGaussianKernel(int%20ksize,%20double%20sigma,%20int%20ktype)
      sigma = 0.3 * k + 0.8;
    }
    ptr->kx(k);
    ptr->ky(k);
    ptr->sigmax(sigma);
    ptr->sigmay(sigma);
    return ptr;
  }
  if (target == "disparity-median") {
    auto ptr = std::unique_ptr<DisparityMedian>(new DisparityMedian());
    int diameter = options["disparity-median-diameter"].as<int>();
    ptr->diameter(diameter);
    return ptr;
  }
  if (target == "disparity-bilateral") {
    auto ptr = std::unique_ptr<DisparityBilateral>(new DisparityBilateral());
    int diameter = options["disparity-bilateral-diameter"].as<int>();
    double sigmaColor = options["disparity-bilateral-sigma-color"].as<double>();
    double sigmaSpace = options["disparity-bilateral-sigma-space"].as<double>();
    ptr->diameter(diameter);
    ptr->sigmaColor(sigmaColor);
    ptr->sigmaSpace(sigmaSpace);
    return ptr;
  }
  if (target == "disparity-morph-open") {
    auto ptr = std::make_unique<DisparityMorphology>();
    ptr->operation(DisparityMorphology::open);

    int diameter = options["disparity-morph-open-diameter"].as<int>();
    std::string shapeStr =
        options["disparity-morph-open-shape"].as<std::string>();
    DisparityMorphology::KernelShape shape = morphShapeToEnum(shapeStr);

    ptr->diameter(diameter);
    ptr->kernelShape(shape);
    return ptr;
  }
  if (target == "disparity-morph-close") {
    auto ptr = std::make_unique<DisparityMorphology>();
    ptr->operation(DisparityMorphology::open);

    int diameter = options["disparity-morph-close-diameter"].as<int>();
    std::string shapeStr =
        options["disparity-morph-close-shape"].as<std::string>();
    DisparityMorphology::KernelShape shape = morphShapeToEnum(shapeStr);

    ptr->diameter(diameter);
    ptr->kernelShape(shape);
    return ptr;
  }
  if (target == "background-subtraction") {
    std::string path;
    if (options.count("background-subtraction-cage-directory") == 0) {
      path = options["src-dir"].as<std::string>();
    } else {
      path = options["background-subtraction-cage-directory"].as<std::string>();
    }
    MatlabReader reader(path);
    reader.setBeginStream(options["first-stream"].as<int>());
    reader.setEndStream(options["last-stream"].as<int>() + 1);
    FrameWindow cageWindow = reader.frameWindow(
        options["background-subtraction-cage-frame"].as<int>());
    auto ptr = std::make_unique<BackgroundSubtraction>();
    ptr->cage_frame(cageWindow);
    return ptr;
  }
  if (target == "none") {
    return nullptr;
  }
  BOOST_LOG_TRIVIAL(info)
      << "Could not create frame window filter for unknown target " << target;
  return nullptr;
}

std::unique_ptr<Registration>
PipelineFactory::getRegistration(const op::variables_map &options) const {
  std::string target = options["pipeline-registration"].as<std::string>();
  if (target == "disparity") {
    return std::unique_ptr<Registration>(new DisparityRegistration());
  }
  if (target == "disparity-cpu-optimized") {
    return std::unique_ptr<Registration>(
        new DisparityRegistrationCpuOptimized());
  }
  return nullptr;
}

std::vector<std::unique_ptr<PointCloudFiltering>>
PipelineFactory::getCloudFilters(const op::variables_map &options) const {
  std::vector<std::unique_ptr<PointCloudFiltering>> filters;
  if (options.count("pipeline-point-cloud-filtering") == 0) {
    return filters;
  }
  std::vector<std::string> targets =
      options["pipeline-point-cloud-filtering"].as<std::vector<std::string>>();
  for (const auto &target : targets) {
    auto ptr = getCloudFiltering(target, options);
    if (ptr.get() == nullptr) {
      continue;
    }
    filters.push_back(std::move(ptr));
  }
  return filters;
}

std::unique_ptr<PointCloudFiltering>
PipelineFactory::getCloudFiltering(const std::string &target,
                                   const op::variables_map &options) const {
  if (target == "subsample") {
    int desired = options["subsample-to"].as<int>();
    return std::unique_ptr<PointCloudFiltering>(new SubSample(desired));
  }
  if (target == "statistical-outlier-removal") {
    double alpha = options["statistical-outlier-removal-alpha"].as<double>();
    int k = options["statistical-outlier-removal-k"].as<int>();
    return std::unique_ptr<PointCloudFiltering>(
        new StatisticalOutlierRemoval(alpha, k));
  }
  if (target == "none") {
    return nullptr;
  }
  BOOST_LOG_TRIVIAL(info) << "Could not create cloud filter for unknown target "
                          << target;
  return nullptr;
}

std::unique_ptr<Clustering>
PipelineFactory::getClustering(const op::variables_map &options) const {
  std::string target = options["pipeline-clustering"].as<std::string>();
  if (target == "mean-shift") {
    std::unique_ptr<MeanShift> ptr{
        new MeanShift(options["mean-shift-sigma"].as<double>())};
    ptr->setMaxIterations(options["mean-shift-max-iterations"].as<int>());
    ptr->setMergeThreshold(options["mean-shift-merge-threshold"].as<double>());
    ptr->setConvergenceThreshold(
        options["mean-shift-convergence-threshold"].as<double>());
    ptr->oracleFactory().desiredOracle(
        getOracle(options["mean-shift-oracle"].as<std::string>()));
    return ptr;
  } else if (target == "single-cluster") {
    std::unique_ptr<SingleCluster> ptr{new SingleCluster()};
    return ptr;
  }
  if (target == "mean-shift-cpu-optimized") {
    std::unique_ptr<MeanShiftCpuOptimized> ptr{
        new MeanShiftCpuOptimized(options["mean-shift-sigma"].as<double>())};
    ptr->setMaxIterations(options["mean-shift-max-iterations"].as<int>());
    ptr->setMergeThreshold(options["mean-shift-merge-threshold"].as<double>());
    ptr->setConvergenceThreshold(
        options["mean-shift-convergence-threshold"].as<double>());
    ptr->oracleFactory().desiredOracle(
        getOracle(options["mean-shift-oracle"].as<std::string>()));
    return ptr;
  }
  if (target == "kmeans") {
    std::unique_ptr<KMeans> ptr{new KMeans(10)};
    ptr->K(options["kmeans-k"].as<unsigned int>());
    ptr->oracleFactory().desiredOracle(
        getOracle(options["kmeans-oracle"].as<std::string>()));
    ptr->centroidThreshold(options["kmeans-centroid-threshold"].as<double>());
    ptr->assignmentThreshold(
        options["kmeans-assignment-threshold"].as<double>());
    BOOST_LOG_TRIVIAL(debug) << ptr->K();
    return ptr;
  }

  return nullptr;
}

PipelineFactory::OFactory::Oracles
PipelineFactory::getOracle(const std::string &oracleKey) const {
  if (oracleKey == "brute-force") {
    return OFactory::Oracles::BRUTE_FORCE;
  }
  if (oracleKey == "uniform-grid") {
    return OFactory::Oracles::UNIFORM_GRID;
  }
  if (oracleKey == "flann") {
    return OFactory::Oracles::FLANN;
  }
  BOOST_LOG_TRIVIAL(info) << "Unknown requested oracle \"" << oracleKey
                          << "\", using brute force.";
  return OFactory::Oracles::BRUTE_FORCE;
}

std::unique_ptr<Descripting>
PipelineFactory::getDescripting(const op::variables_map &options) const {
  std::string target = options["pipeline-descripting"].as<std::string>();
  if (target == "cog") {
    return std::unique_ptr<Descripting>(new CenterOfGravity());
  }
  return nullptr;
}

std::unique_ptr<Matching>
PipelineFactory::getMatching(const op::variables_map &options) const {
  std::string target = options["pipeline-matching"].as<std::string>();
  if (target == "nearest-neighbor") {
    return std::unique_ptr<Matching>(new NearestNeighbour());
  }
  return nullptr;
}

std::unique_ptr<TrajectoryBuilder>
PipelineFactory::getTrajectoryBuilder(const op::variables_map &options) const {
  std::string target = options["pipeline-trajectory-builder"].as<std::string>();
  if (target == "raw-cog") {
    return std::unique_ptr<TrajectoryBuilder>(new CogTrajectoryBuilder());
  }
  return nullptr;
}

} // namespace MouseTrack
