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
  ad("src-dir,s", op::value<std::string>(), "Source directory to process");
  ad("out-dir,o", op::value<std::string>(), "Target directory for computed results. No results will be written, if not provided.");
  ad("cli,c", "Don't start the Graphical User Interface, run on command line.");
  ad("pipeline-timer", "Measures the execution time of each pipeline step end sends it's output to the log with debug priority.");
  ad("log,l", op::value<std::string>()->default_value("info"), "Set lowest log level to show. Possible options: trace, debug, info, warning, error, fatal, none. Default: info");
  ad("first-frame", op::value<int>(), "Desired lowest start frame (inclusive).");
  ad("last-frame", op::value<int>(), "Desired highest end frame (inclusive).");
  ad("first-stream", op::value<int>(), "Desired lowest start stream (inclusive).");
  ad("last-stream", op::value<int>(), "Desired highest end stream (inclusive).");

  // pipeline modules
  ad("pipeline-reader", op::value<std::string>()->default_value("matlab-concurrent"), "Which reader module to use. Valid values: none, matlab, matlab-concurrent");
  ad("pipeline-frame-window-filtering", op::value<std::vector<std::string>>()->multitoken(), "Which filtering modules to apply to a frame window. Valid values: none, disparity-gauss, disparity-median, disparity-bilateral, disparity-morphology");
  ad("pipeline-registration", op::value<std::string>()->default_value("disparity-cpu-optimized"), "Which registration module to use. Valid values: none, disparity, disparity-cpu-optimized");
  ad("pipeline-point-cloud-filtering", op::value<std::vector<std::string>>()->multitoken(), "Which filtering modules to use. Valid values: none, subsample, statistical-outlier-removal");
  ad("pipeline-clustering", op::value<std::string>()->default_value("mean-shift"), "Which clustering module to use. Valid values: none, mean-shift");
  ad("pipeline-descripting", op::value<std::string>()->default_value("cog"), "Which descripting module to use. Valid values: none, cog");
  ad("pipeline-matching", op::value<std::string>()->default_value("nearest-neighbor"), "Which matching module to use. Valid values: none, nearest-neighbor");
  ad("pipeline-trajectory-builder", op::value<std::string>()->default_value("raw-cog"), "Which matching module to use. Valid values: none, raw-cog");

  // module settings

  // frame window post processing
  ad("disparity-gauss-k", op::value<int>()->default_value(3), "Patch diameter in x/y direction, must be positive and integer. k = 0 results in a 1x1 patch size.");
  ad("disparity-gauss-sigma", op::value<double>()->default_value(0.5), "Gaussian standard deviation in x/y direction, must be positive.");
  ad("disparity-bilateral-diameter", op::value<int>()->default_value(2), "Patch diameter in x/y direction, must be positive and integer.");
  ad("disparity-bilateral-sigma-color", op::value<double>()->default_value(0.1), "Filter sigma in color space.");
  ad("disparity-bilateral-sigma-space", op::value<double>()->default_value(50), "Filter sigma in coordinate space.");
  ad("disparity-median-diameter", op::value<int>()->default_value(2), "Patch diameter in x/y direction, must be positive and integer.");
  ad("disparity-morphology-diameter", op::value<int>()->default_value(2), "Patch diameter in x/y direction, must be positive and integer.");
  ad("disparity-morphology-operation", op::value<std::string>()->default_value("open"), "Specific morphological operation. Valid values: open, close");
  ad("disparity-morphology-shape", op::value<std::string>()->default_value("rect"), "Shape of kernel for morphological operation. Valid values: rect, ellipse, cross");

  // point cloud post processing

  // subsample
  ad("subsample-to", op::value<int>()->default_value(100*000), "Subsample the points cloud such that there are only <n> points.");

  // statistical outlier removal
  ad("statistical-outlier-removal-alpha", op::value<double>()->default_value(1.0), "Range within which points are inliers: [-alpha * stddev, slpha * stddev]");
  ad("statistical-outlier-removal-k", op::value<int>()->default_value(30), "K neighbors to take into account.");

  // mean-shift
  ad("mean-shift-sigma", op::value<double>()->default_value(0.01), "Sigma used for the mean-shift clustering.");
  ad("mean-shift-max-iterations", op::value<int>()->default_value(1000), "Maximum number of iterations for a point before it should converge.");
  ad("mean-shift-merge-threshold", op::value<double>()->default_value(0.001), "Maximum distance of two clusters such that they can still merge");
  ad("mean-shift-convergence-threshold", op::value<double>()->default_value(0.0001), "Maximum distance a point is allowed to travel in an iteration and still being classified as converged.");
  // clang-format on
  return desc;
}

} // namespace MouseTrack
