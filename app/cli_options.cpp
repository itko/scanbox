/// \file
/// Maintainer: Felice Serena
///
///

#include "cli_options.h"

namespace MouseTrack {

op::options_description cli_options() {
  op::options_description desc{"Options"};
  auto ad = desc.add_options();
  // clang-format off
  ad("help,h", "Help screen");
  ad("src,s", op::value<std::string>(), "Source directory/file to process");
  ad("camchain,y", op::value<std::string>(), "If the source points to a ROS bag file, set this variable to the corresponding YAML camchain file.");
  ad("out-dir,o", op::value<std::string>(), "Target directory for computed results. No results will be written, if not provided.");
  ad("cli,c", "Don't start the Graphical User Interface, run on command line.");
  ad("pipeline-timer", "Measures the execution time of each pipeline step end sends it's output to the log with debug priority.");
  ad("pipeline-timer-log", op::value<std::string>(), "If set, the pipeline times will be written to this file in csv format. (frame number, total frame time, READ_FRAME_WINDOW,FRAME_WINDOW_FILTERING,REGISTRATION,POINT_CLOUD_FILTERING,CLUSTERING,DESCRIPTING,MATCHING,CONTROL_POINTS)");
  ad("log,l", op::value<std::string>()->default_value("info"), "Set lowest log level to show. Possible options: trace, debug, info, warning, error, fatal, none. Default: info");
  ad("first-frame", op::value<int>(), "Desired lowest start frame (inclusive).");
  ad("last-frame", op::value<int>(), "Desired highest end frame (inclusive).");
  ad("first-stream", op::value<int>(), "Desired lowest start stream (inclusive).");
  ad("last-stream", op::value<int>(), "Desired highest end stream (inclusive).");

  // pipeline modules
  ad("pipeline-reader", op::value<std::string>()->default_value("auto"), "Which reader module to use. Valid values: auto, matlab, matlab-concurrent, ros-bag; auto picks 'matlab-concurrent' for source directories and 'ros-bag' in case a bag file is given");
  ad("pipeline-frame-window-filtering", op::value<std::vector<std::string>>()->multitoken(), "Which filtering modules to apply to a frame window. Valid values: none, disparity-gauss, disparity-median, disparity-bilateral, disparity-morph-open, disparity-morph-close, background-subtraction, hog-labeling, strict-labeling");
  ad("pipeline-registration", op::value<std::string>()->default_value("disparity-cpu-optimized"), "Which registration module to use. Valid values: none, disparity, disparity-cpu-optimized");
  ad("pipeline-point-cloud-filtering", op::value<std::vector<std::string>>()->multitoken(), "Which filtering modules to use. Valid values: none, subsample, statistical-outlier-removal");
  ad("pipeline-clustering", op::value<std::string>()->default_value("mean-shift"), "Which clustering module to use. Valid values: none, single-cluster, mean-shift, mean-shift-cpu-optimized, kmeans, label-clustering");
  ad("pipeline-descripting", op::value<std::string>()->default_value("cog"), "Which descripting module to use. Valid values: none, cog");
  ad("pipeline-matching", op::value<std::string>()->default_value("nearest-neighbor"), "Which matching module to use. Valid values: none, nearest-neighbor");
  ad("pipeline-trajectory-builder", op::value<std::string>()->default_value("raw-cog"), "Which matching module to use. Valid values: none, raw-cog");

  // module settings

  // frame window post processing
  ad("disparity-gauss-k", op::value<int>()->default_value(3), "Patch diameter in x/y direction, must be positive and integer. k = 0 results in a 1x1 patch size.");
  ad("disparity-gauss-sigma", op::value<double>(), "Gaussian standard deviation in x/y direction, must be positive, default: 0.3*k + 0.8 (Scale standard deviation proportional to kernel size).");
  ad("disparity-bilateral-diameter", op::value<int>()->default_value(2), "Patch diameter in x/y direction, must be positive and integer.");
  ad("disparity-bilateral-sigma-color", op::value<double>()->default_value(0.1), "Filter sigma in color space.");
  ad("disparity-bilateral-sigma-space", op::value<double>()->default_value(50), "Filter sigma in coordinate space.");
  ad("disparity-median-diameter", op::value<int>()->default_value(2), "Patch diameter in x/y direction, must be positive and integer.");
  ad("disparity-morph-open-diameter", op::value<int>()->default_value(2), "Patch diameter in x/y direction, must be positive and integer.");
  ad("disparity-morph-open-shape", op::value<std::string>()->default_value("rect"), "Shape of kernel for morphological operation. Valid values: rect, ellipse, cross");
  ad("disparity-morph-close-diameter", op::value<int>()->default_value(2), "Patch diameter in x/y direction, must be positive and integer.");
  ad("disparity-morph-close-shape", op::value<std::string>()->default_value("rect"), "Shape of kernel for morphological operation. Valid values: rect, ellipse, cross");
  ad("background-subtraction-threshold", op::value<double>()->default_value(0.01), "Threshold for background subtraction. (Representing an intensity value in [0,1])");
  ad("background-subtraction-cage-reader", op::value<std::string>()->default_value("auto"), "Which pipeline-reader should be used?");
  ad("background-subtraction-cage-frame", op::value<int>()->default_value(-1), "Number of a frame with empty cage. Default: choose first frame of input.");
  ad("background-subtraction-cage-directory", op::value<std::string>(), "Path to directory with empty cage. Default: src");
  ad("background-subtraction-cage-camchain", op::value<std::string>(), "Path to camchain file for empty cage.");
  ad("hog-labeling-train", op::value<std::string>(), "Path to training data for HOG labeling.");

  // point cloud post processing

  // subsample point cloud
  ad("subsample-to", op::value<int>()->default_value(100*1000), "Subsample the points cloud such that there are only <n> points.");

  // statistical outlier removal
  ad("statistical-outlier-removal-alpha", op::value<double>()->default_value(1.0), "Range within which points are inliers: [-alpha * stddev, alpha * stddev]");
  ad("statistical-outlier-removal-k", op::value<int>()->default_value(30), "K neighbors to take into account.");

  // clustering
  
  // mean-shift
  ad("mean-shift-sigma", op::value<double>()->default_value(0.02), "Sigma used for the mean-shift clustering.");
  ad("mean-shift-max-iterations", op::value<int>()->default_value(1000), "Maximum number of iterations for a point before it should converge.");
  ad("mean-shift-merge-threshold", op::value<double>()->default_value(0.01), "Maximum distance of two clusters such that they can still merge");
  ad("mean-shift-convergence-threshold", op::value<double>()->default_value(0.0001), "Maximum distance a point is allowed to travel in an iteration and still being classified as converged.");
  ad("mean-shift-oracle", op::value<std::string>()->default_value("uniform-grid"), "Which spatial acceleration should be used? valid values: brute-force, uniform-grid, flann");

  // k-means
  ad("kmeans-k", op::value<unsigned int>()->default_value(15), "Number of expected clusters.");
  ad("kmeans-centroid-threshold", op::value<double>()->default_value(0.01), "Total movement of cluster centers that should be classified as 'converged'.");
  ad("kmeans-assignment-threshold", op::value<double>()->default_value(0.02), "Percentage of points that changed clusters: if the percentage is below this threshold, convergence is assumed.");
  ad("kmeans-oracle", op::value<std::string>()->default_value("flann"), "Which spatial acceleration should be used? valid values: brute-force, uniform-grid, flann");

  // clang-format on
  return desc;
}

} // namespace MouseTrack
