/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "main_window.h"
#include "pipeline_observer.h"

namespace MouseTrack {

class GUIObserver : public PipelineObserver {
public:
  GUIObserver(MainWindow *window);

  /// Pipeline starts to process data
  void pipelineStarted();

  /// Pipeline finished all work.
  void pipelineTerminated();

  /// When the pipeline terminates, it delivers the complete cluster chain used
  /// internally.
  void
  newClusterChains(std::shared_ptr<const std::vector<ClusterChain>> chains);

  /// Work on frame `frame` started, it is inside the pipeline.
  void frameStart(FrameNumber frame);

  /// Work on frame `frame` finished, it is no longer processed by the pipeline.
  void frameEnd(FrameNumber frame);

  // see descriptions of the corresponding module interfaces

  // clang-format off
    void startFrameWindow     (FrameNumber f);
    void newFrameWindow       (FrameNumber f, std::shared_ptr<const FrameWindow> window);

    void startFrameWindowFiltering(FrameIndex f);
    void newFilteredFrameWindow(FrameIndex f, std::shared_ptr<const FrameWindow> window);

    void startRegistration    (FrameIndex f);
    void newRawPointCloud     (FrameIndex f, std::shared_ptr<const PointCloud> cloud);

    void startPointCloudFiltering(FrameNumber f);
    void newFilteredPointCloud(FrameNumber f, std::shared_ptr<const PointCloud> cloud);

    void startClustering      (FrameNumber f);
    void newClusters          (FrameNumber f, std::shared_ptr<const std::vector<Cluster>> clusters);

    void startDescripting     (FrameNumber f);
    void newDescriptors       (FrameNumber f, std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>> descriptors);

    void startMatching        (FrameNumber f);
    void newMatches           (FrameNumber f, std::shared_ptr<const std::vector<long>> matches);

    void startControlPoints   (FrameNumber f);
    void newControlPoints     (FrameNumber f, std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints);
  // clang-format on

private:
  MainWindow *_window;
};
} // namespace MouseTrack
