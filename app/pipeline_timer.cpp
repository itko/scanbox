/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_timer.h"
#include <boost/log/trivial.hpp>

namespace MouseTrack {

void PipelineTimer::pipelineStarted() { startTimer(-1, "pipeline"); }

void PipelineTimer::pipelineTerminated() { stopTimer(-1, "pipeline"); }

void PipelineTimer::frameStart(FrameNumber f) { startTimer(f, "total"); }

void PipelineTimer::frameEnd(FrameNumber f) { stopTimer(f, "total"); }

void PipelineTimer::startFrameWindow(FrameNumber f) {
  startTimer(f, "readFrameWindow");
}

void PipelineTimer::newFrameWindow(FrameNumber f,
                                   std::shared_ptr<const FrameWindow>) {
  stopTimer(f, "readFrameWindow");
}

void PipelineTimer::startFrameWindowFiltering(FrameNumber f) {
  startTimer(f, "frameWindowFiltering");
}

void PipelineTimer::newFilteredFrameWindow(FrameNumber f,
                                           std::shared_ptr<const FrameWindow>) {
  stopTimer(f, "frameWindowFiltering");
}

void PipelineTimer::startRegistration(FrameNumber f) {
  startTimer(f, "registration");
}

void PipelineTimer::newRawPointCloud(FrameNumber f,
                                     std::shared_ptr<const PointCloud>) {
  stopTimer(f, "registration");
}

void PipelineTimer::startPointCloudFiltering(FrameNumber f) {
  startTimer(f, "point_cloud_filtering");
}

void PipelineTimer::newFilteredPointCloud(FrameNumber f,
                                          std::shared_ptr<const PointCloud>) {
  stopTimer(f, "point_cloud_filtering");
}

void PipelineTimer::startClustering(FrameNumber f) {
  startTimer(f, "clustering");
}

void PipelineTimer::newClusters(FrameNumber f,
                                std::shared_ptr<const std::vector<Cluster>>) {
  stopTimer(f, "clustering");
}

void PipelineTimer::startDescripting(FrameNumber f) {
  startTimer(f, "descripting");
}

void PipelineTimer::newDescriptors(
    FrameNumber f,
    std::shared_ptr<
        const std::vector<std::shared_ptr<const ClusterDescriptor>>>) {
  stopTimer(f, "descripting");
}

void PipelineTimer::startMatching(FrameNumber f) { startTimer(f, "matching"); }

void PipelineTimer::newMatches(FrameNumber f,
                               std::shared_ptr<const std::vector<long>>) {
  stopTimer(f, "matching");
}
void PipelineTimer::startControlPoints(FrameNumber f) {
  startTimer(f, "controlPoints");
}
void PipelineTimer::newControlPoints(
    FrameNumber f, std::shared_ptr<const std::vector<Eigen::Vector3d>>) {
  stopTimer(f, "controlPoints");
}

void PipelineTimer::startTimer(FrameNumber f, const std::string &key) {
  _starts[Key(f, key)] = std::chrono::system_clock::now();
}

void PipelineTimer::stopTimer(FrameNumber f, const std::string &key) {
  auto start = _starts.find(Key(f, key));
  if (start == _starts.end()) {
    BOOST_LOG_TRIVIAL(warning)
        << "Couldn't find timer start for key (" << f << ", " << key << ")";
    return;
  }
  auto duration = std::chrono::system_clock::now() - start->second;
  _starts.erase(start);
  int d =
      std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
  BOOST_LOG_TRIVIAL(info) << "(" << f << ", " << key << "): " << d / 1000.0
                          << " milliseconds";
}

} // namespace MouseTrack
