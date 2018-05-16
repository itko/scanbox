/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "pipeline_observer.h"
#include <chrono>

namespace MouseTrack {

class PipelineTimer : public PipelineObserver {
public:
  virtual void pipelineStarted();

  virtual void pipelineTerminated();

  virtual void frameStart(FrameNumber frame);

  virtual void frameEnd(FrameNumber frame);

  virtual void startFrameWindow(FrameNumber f);
  virtual void newFrameWindow(FrameNumber f,
                              std::shared_ptr<const FrameWindow> window);

  virtual void startFrameWindowFiltering(FrameNumber f);
  virtual void
  newFilteredFrameWindow(FrameNumber f,
                         std::shared_ptr<const FrameWindow> window);

  virtual void startRegistration(FrameNumber f);
  virtual void newRawPointCloud(FrameNumber f,
                                std::shared_ptr<const PointCloud> cloud);

  virtual void startPointCloudFiltering(FrameNumber f);
  virtual void newFilteredPointCloud(FrameNumber f,
                                     std::shared_ptr<const PointCloud> cloud);

  virtual void startClustering(FrameNumber f);
  virtual void
  newClusters(FrameNumber f,
              std::shared_ptr<const std::vector<Cluster>> clusters);

  virtual void startDescripting(FrameNumber f);
  virtual void newDescriptors(
      FrameNumber f,
      std::shared_ptr<
          const std::vector<std::shared_ptr<const ClusterDescriptor>>>
          descriptors);

  virtual void startMatching(FrameNumber f);
  virtual void newMatches(FrameNumber f,
                          std::shared_ptr<const std::vector<long>> matches);

  virtual void startControlPoints(FrameNumber f);
  virtual void newControlPoints(
      FrameNumber f,
      std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints);

private:
  typedef std::pair<FrameNumber, std::string> Key;
  void startTimer(FrameNumber f, const std::string &key);
  void stopTimer(FrameNumber f, const std::string &key);
  std::map<Key, std::chrono::system_clock::time_point> _starts;
};

} // namespace MouseTrack
