/// \file
/// Maintainer: Luzian Hug
///
///

#pragma once

#include "pipeline_observer.h"
#include "types.h"
#include <unordered_map>

namespace MouseTrack {

class PipelineWriter : public PipelineObserver {
public:
  PipelineWriter(fs::path targetDir);
  virtual void newRawPointCloud(FrameNumber f,
                                std::shared_ptr<const PointCloud> cloud);
  virtual void newFilteredPointCloud(FrameNumber f,
                                     std::shared_ptr<const PointCloud> cloud);
  virtual void
  newClusters(FrameNumber f,
              std::shared_ptr<const std::vector<Cluster>> clusters);
  virtual void newDescriptors(
      FrameNumber f,
      std::shared_ptr<
          const std::vector<std::shared_ptr<const ClusterDescriptor>>>
          descriptors);
  virtual void newMatches(FrameNumber f,
                          std::shared_ptr<const std::vector<long>> matches);
  virtual void newControlPoints(
      FrameNumber f,
      std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints);

  virtual void
  newClusterChains(std::shared_ptr<const std::vector<ClusterChain>> chains);

private:
  fs::path _outputDir;
  std::string _rawPointCloudPath;
  std::string _filteredPointCloudPath;
  std::string _clusteredPointCloudPath;
  std::string _clustersPath;
  std::string _descriptorsPath;
  std::string _matchesPath;
  std::string _controlPointsPath;
  std::string _chainedPointCloudPath;
  std::unordered_map<FrameNumber, std::shared_ptr<const PointCloud>> _clouds;
  std::unordered_map<FrameNumber, std::shared_ptr<const std::vector<Cluster>>>
      _clusters;

  std::string insertFrame(const std::string &templatePath, FrameNumber f) const;

  std::vector<std::vector<double>> nColors(int n) const;
};

} // namespace MouseTrack
