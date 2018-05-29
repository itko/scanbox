/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_factory.h"

#include "generic/read_csv.h"
#include "generic/resolve_symlink.h"

#include "frame_window_filtering/background_subtraction.h"
#include "frame_window_filtering/disparity_bilateral.h"
#include "frame_window_filtering/disparity_gaussian_blur.h"
#include "frame_window_filtering/disparity_median.h"
#include "frame_window_filtering/disparity_morphology.h"
#include "frame_window_filtering/hog_labeling.h"
#include "frame_window_filtering/strict_labeling.h"

#include "matching/nearest_neighbour.h"

#include "matlab_reader.h"
#include "matlab_reader_concurrent.h"

#if ENABLE_ROSBAG
#include "ros_bag_reader.h"
#endif

#include "registration/disparity_registration.h"
#include "registration/disparity_registration_cpu_optimized.h"

#include "point_cloud_filtering/statistical_outlier_removal.h"
#include "point_cloud_filtering/subsample.h"

#include "clustering/kmeans.h"
#include "clustering/label_clustering.h"
#include "clustering/mean_shift.h"
#include "clustering/mean_shift_cpu_optimized.h"
#include "clustering/single_cluster.h"

#include "descripting/cog.h"

#include "trajectory_builder/cog_trajectory_builder.h"

#include <boost/log/trivial.hpp>

namespace MouseTrack {

Pipeline
PipelineFactory::fromCliOptions(const op::variables_map &options) const {
  if (options.count("src") == 0) {
    BOOST_LOG_TRIVIAL(info) << "No command line source path given.";
  }
  BOOST_LOG_TRIVIAL(trace) << "Creating Reader";
  std::unique_ptr<Reader> reader = getReader(options);

  BOOST_LOG_TRIVIAL(trace) << "Creating FrameWindowFiltering";
  std::vector<std::unique_ptr<FrameWindowFiltering>> windowFiltering =
      getWindowFilters(options);

  BOOST_LOG_TRIVIAL(trace) << "Creating Registration";
  std::unique_ptr<Registration> registration{getRegistration(options)};

  BOOST_LOG_TRIVIAL(trace) << "Creating PointCloudFiltering";
  std::vector<std::unique_ptr<PointCloudFiltering>> cloudFiltering{
      getCloudFilters(options)};

  BOOST_LOG_TRIVIAL(trace) << "Creating Clustering";
  std::unique_ptr<Clustering> clustering{getClustering(options)};

  BOOST_LOG_TRIVIAL(trace) << "Creating Descripting";
  std::unique_ptr<Descripting> descripting{getDescripting(options)};

  BOOST_LOG_TRIVIAL(trace) << "Creating Matching";
  std::unique_ptr<Matching> matching{getMatching(options)};

  BOOST_LOG_TRIVIAL(trace) << "Creating Trajectory Builder";
  std::unique_ptr<TrajectoryBuilder> trajectoryBuilder{
      getTrajectoryBuilder(options)};
  BOOST_LOG_TRIVIAL(debug) << "Pipeline modules successfully created.";
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
  std::string src = options["src"].as<std::string>();
  target = chooseReaderTarget(target, src);
  if (target == "") {
    return nullptr;
  }
  if (target == "matlab" || target == "matlab-concurrent") {
    std::unique_ptr<MatlabReader> reader;
    if (target == "matlab") {
      reader = std::unique_ptr<MatlabReader>(new MatlabReader(src));
    } else {
      reader = std::unique_ptr<MatlabReader>(new MatlabReaderConcurrent(src));
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
  } else if (target == "ros-bag") {
#if ENABLE_ROSBAG
    if (options.count("camchain") == 0) {
      BOOST_LOG_TRIVIAL(warning)
          << "You've chosen to process a ROS bag file, "
             "please set --camchain=<YAML-Path> correspondingly.";
      return nullptr;
    }
    std::string camchain = options["camchain"].as<std::string>();
    std::unique_ptr<RosBagReader> reader =
        std::make_unique<RosBagReader>(src, camchain);
    if (options.count("first-frame")) {
      reader->setBeginFrame(
          std::max(reader->beginFrame(), options["first-frame"].as<int>()));
    }
    if (options.count("last-frame")) {
      reader->setEndFrame(
          std::min(reader->endFrame(), options["last-frame"].as<int>() + 1));
    }
    return reader;
#else
    BOOST_LOG_TRIVIAL(warning)
        << "Application was compiled without ROS-bag support.";
    return nullptr;
#endif
  }

  return nullptr;
}

std::string
PipelineFactory::chooseReaderTarget(const std::string &givenTarget,
                                    const std::string &givenSrc) const {
  std::string target = givenTarget;
  if (target == "auto") {
    fs::path sPath(givenSrc);
    if (fs::is_directory(sPath)) {
      target = "matlab-concurrent";
    } else if (fs::is_regular_file(sPath)) {
      target = "ros-bag";
    } else {
      BOOST_LOG_TRIVIAL(warning) << "PipelineFactory, Reader: Unexpected path "
                                    "type encountered, please "
                                    "add to case distinction or enter other "
                                    "source path.";
      return "";
    }
  }
  return target;
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
    std::string src;
    if (options.count("background-subtraction-cage-directory") == 0) {
      src = options["src"].as<std::string>();
    } else {
      src = options["background-subtraction-cage-directory"].as<std::string>();
    }
    FrameNumber desiredFrame =
        options["background-subtraction-cage-frame"].as<int>();
    std::unique_ptr<Reader> reader;
    {
      // create a reader
      std::string target;
      if (options.count("background-subtraction-cage-reader")) {
        target =
            options["background-subtraction-cage-reader"].as<std::string>();
      } else {
        target = options["pipeline-reader"].as<std::string>();
      }
      target = chooseReaderTarget(target, src);
      if (target == "") {
        return nullptr;
      }
      if (target == "matlab" || target == "matlab-concurrent") {
        std::unique_ptr<MatlabReader> mReader;
        if (target == "matlab") {
          mReader = std::unique_ptr<MatlabReader>(new MatlabReader(src));
        } else {
          mReader =
              std::unique_ptr<MatlabReader>(new MatlabReaderConcurrent(src));
        }
        if (options.count("first-stream")) {
          mReader->setBeginStream(std::max(mReader->beginStream(),
                                           options["first-stream"].as<int>()));
        }
        if (options.count("last-stream")) {
          mReader->setEndStream(std::min(mReader->endStream(),
                                         options["last-stream"].as<int>() + 1));
        }
        if (desiredFrame == -1) {
          desiredFrame = mReader->beginFrame();
        }
        if (desiredFrame < mReader->beginFrame() &&
            mReader->endFrame() <= desiredFrame) {
          BOOST_LOG_TRIVIAL(warning)
              << "Desired frame number " << desiredFrame
              << " not available, valid range for source: ["
              << mReader->beginFrame() << ", " << mReader->endFrame() << ")";
          return nullptr;
        }
        reader = std::move(mReader);
      } else if (target == "ros-bag") {
#if ENABLE_ROSBAG
        std::string camchain;
        if (options.count("background-subtraction-cage-camchain") > 0) {
          camchain =
              options["background-subtraction-cage-camchain"].as<std::string>();
        } else if (options.count("camchain") > 0) {
          camchain = options["camchain"].as<std::string>();
        } else {
          BOOST_LOG_TRIVIAL(warning)
              << "You've chosen to process a ROS bag file, "
                 "please set "
                 "--background-subtraction-cage-camchain=<YAML-Path> "
                 "correspondingly.";
          return nullptr;
        }
        std::unique_ptr<RosBagReader> rReader =
            std::make_unique<RosBagReader>(src, camchain);
        if (desiredFrame == -1) {
          desiredFrame = rReader->beginFrame();
        }
        if (desiredFrame < rReader->beginFrame() &&
            rReader->endFrame() <= desiredFrame) {
          BOOST_LOG_TRIVIAL(warning)
              << "Desired frame number " << desiredFrame
              << " not available, valid range for source: ["
              << rReader->beginFrame() << ", " << rReader->endFrame() << ")";
          return nullptr;
        }
        reader = std::move(rReader);
#else
        BOOST_LOG_TRIVIAL(warning) << "Background-subtraction: Application was "
                                      "compiled without ROS-bag support.";
        return nullptr;
#endif
      }
    }
    FrameWindow cageWindow = (*reader)(desiredFrame);
    auto ptr = std::make_unique<BackgroundSubtraction>();
    ptr->cage_frame(cageWindow);
    return ptr;
  }
  if (target == "strict-labeling") {
    return std::make_unique<StrictLabeling>();
  }
  if (target == "hog-labeling") {
    if (options.count("hog-labeling-train") == 0) {
      BOOST_LOG_TRIVIAL(info) << "HOG labeling needs training data, please "
                                 "provide a path via "
                                 "--hog-labeling-train=<path>";
      return nullptr;
    }
    std::string trainPath = options["hog-labeling-train"].as<std::string>();
    fs::path trainP(resolve_symlink(trainPath, 100));
    if (!fs::is_regular_file(trainP)) {
      BOOST_LOG_TRIVIAL(info)
          << "Provided path " << trainPath << " for training data resolved to "
          << trainP << ". Path must end at regular file.";
      return nullptr;
    }
    auto vecTrain = read_csv(trainPath);
    if (vecTrain.empty()) {
      BOOST_LOG_TRIVIAL(info) << "Training file is empty.";
      return nullptr;
    }
    // cols are samples, rows are dimensions
    int dimensions = vecTrain[0].size() - 1;
    int samples = vecTrain.size();
    BOOST_LOG_TRIVIAL(debug) << "Found " << samples << " training samples of "
                             << dimensions << " dimensions.";
    BOOST_LOG_TRIVIAL(trace) << "Reading X_train ...";
    HogLabeling::Mat X_train(dimensions, samples);
    for (int s = 0; s < samples; ++s) {
      for (int d = 0; d < dimensions; ++d) {
        const auto &str = vecTrain[s][d + 1];
        double v = std::stod(str);
        X_train(d, s) = v;
      }
    }
    BOOST_LOG_TRIVIAL(trace) << "Reading y_train ...";
    HogLabeling::Vec y_train(samples);
    for (int s = 0; s < samples; ++s) {
      const auto &str = vecTrain[s][0];
      double d = std::stod(str);
      int v = d;
      y_train[s] = v;
    }
    auto ptr = std::make_unique<HogLabeling>();
    ptr->train(X_train, y_train);
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
  if (target == "label-clustering") {
    BOOST_LOG_TRIVIAL(debug) << "Creating labelClustering";
    return std::make_unique<LabelClustering>();
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
