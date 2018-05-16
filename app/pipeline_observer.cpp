/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_observer.h"

namespace MouseTrack {

void PipelineObserver::pipelineStarted() {
  // empty
}

void PipelineObserver::pipelineTerminated() {
  // empty
}

void PipelineObserver::newClusterChains(
    std::shared_ptr<const std::vector<ClusterChain>>) {
  // empty
}

void PipelineObserver::frameStart(FrameNumber) {
  // empty
}

void PipelineObserver::frameEnd(FrameNumber) {
  // empty
}

void PipelineObserver::startFrameWindow(FrameNumber) {
  // empty
}

void PipelineObserver::newFrameWindow(FrameNumber,
                                      std::shared_ptr<const FrameWindow>) {
  // empty
}

void PipelineObserver::startFrameWindowFiltering(FrameNumber) {
  // empty
}

void PipelineObserver::newFilteredFrameWindow(
    FrameNumber, std::shared_ptr<const FrameWindow>) {
  // empty
}

void PipelineObserver::startRegistration(FrameNumber) {
  // empty
}

void PipelineObserver::newRawPointCloud(FrameNumber,
                                        std::shared_ptr<const PointCloud>) {
  // empty
}

void PipelineObserver::startPointCloudFiltering(FrameNumber) {
  // empty
}

void PipelineObserver::newFilteredPointCloud(
    FrameNumber, std::shared_ptr<const PointCloud>) {
  // empty
}

void PipelineObserver::startClustering(FrameNumber) {
  // empty
}

void PipelineObserver::newClusters(
    FrameNumber, std::shared_ptr<const std::vector<Cluster>>) {
  // empty
}

void PipelineObserver::startDescripting(FrameNumber) {
  // empty
}

void PipelineObserver::newDescriptors(
    FrameNumber,
    std::shared_ptr<
        const std::vector<std::shared_ptr<const ClusterDescriptor>>>) {
  // empty
}

void PipelineObserver::startMatching(FrameNumber) {
  // empty
}

void PipelineObserver::newMatches(FrameNumber,
                                  std::shared_ptr<const std::vector<long>>) {
  // empty
}

void PipelineObserver::startControlPoints(FrameNumber) {
  // empty
}

void PipelineObserver::newControlPoints(
    FrameNumber, std::shared_ptr<const std::vector<Eigen::Vector3d>>) {
  // empty
}

} // namespace MouseTrack
