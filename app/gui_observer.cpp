/// \file
/// Maintainer: Felice Serena
///
///

#include "gui_observer.h"

namespace MouseTrack {

GUIObserver::GUIObserver(MainWindow *window) { _window = window; }

void GUIObserver::pipelineStarted() {
  // empty
}

void GUIObserver::pipelineTerminated() {
  // empty
}

void GUIObserver::newClusterChains(
    std::shared_ptr<const std::vector<ClusterChain>> chains) {
  // empty
}

void GUIObserver::frameStart(FrameNumber frame) {
  // empty
}

void GUIObserver::frameEnd(FrameNumber frame) {
  // empty
}

void GUIObserver::startFrameWindow(FrameNumber f) {
  // empty
}

void GUIObserver::newFrameWindow(FrameNumber f,
                                 std::shared_ptr<const FrameWindow> window) {
  _window->setFrameNumber(f);
  _window->setFrameWindow(window);
}

void GUIObserver::startFrameWindowFiltering(FrameIndex f) {
  // empty
}

void GUIObserver::newFilteredFrameWindow(
    FrameIndex f, std::shared_ptr<const FrameWindow> window) {
  // empty
}

void GUIObserver::startRegistration(FrameIndex f) {
  // empty
}

void GUIObserver::newRawPointCloud(FrameNumber f,
                                   std::shared_ptr<const PointCloud> cloud) {
  // empty
}

void GUIObserver::startPointCloudFiltering(FrameNumber f) {
  // empty
}

void GUIObserver::newFilteredPointCloud(
    FrameNumber f, std::shared_ptr<const PointCloud> cloud) {
  // empty
}

void GUIObserver::startClustering(FrameNumber f) {
  // empty
}

void GUIObserver::newClusters(
    FrameNumber f, std::shared_ptr<const std::vector<Cluster>> cluster) {
  // empty
}

void GUIObserver::startDescripting(FrameNumber f) {
  // empty
}

void GUIObserver::newDescriptors(
    FrameNumber f,
    std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>>
        descriptors) {
  // empty
}

void GUIObserver::startMatching(FrameNumber f) {
  // empty
}

void GUIObserver::newMatches(FrameNumber f,
                             std::shared_ptr<const std::vector<long>> matches) {
  // empty
}

void GUIObserver::startControlPoints(FrameNumber f) {
  // empty
}

void GUIObserver::newControlPoints(
    FrameNumber f,
    std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints) {
  // empty
}

} // namespace MouseTrack
