/// \file
/// Maintainer: Luzian Hug
///
///

#pragma once

#include "pipeline_observer.h"
#include "types.h"
#include <set>
#include <unordered_map>

namespace MouseTrack {

class PipelineWriter : public PipelineObserver {
public:
  PipelineWriter(fs::path targetDir);

  virtual void newFrameWindow(FrameNumber f,
                              std::shared_ptr<const FrameWindow> window);
  virtual void
  newFilteredFrameWindow(FrameNumber f,
                         std::shared_ptr<const FrameWindow> window);
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

  bool writeRawFrameWindow = true;
  bool writeFilteredFrameWindow = true;
  /// Should additional images also be written? Example: label overlays or
  /// masked label overlays
  bool writeFilteredFrameWindowExtensions = true;
  bool writeRawPointCloud = true;
  bool writeFilteredPointCloud = true;
  bool writeClusteredPointCloud = true;
  bool writeClusters = true;
  bool writeDescriptors = true;
  bool writeMatches = true;
  bool writeControlPoints = true;
  bool writeChainedPointCloud = true;

  std::vector<std::vector<double>> &forcedNColors();
  const std::vector<std::vector<double>> &forcedNColors() const;

  std::set<size_t> &labelsToIgnore();
  const std::set<size_t> &labelsToIgnore() const;

private:
  fs::path _outputDir;
  std::string _rawFrameWindowPath;
  std::string _rawFrameWindowDisparityPath;
  std::string _rawFrameWindowRawDisparityPath;
  std::string _rawFrameWindowReferencePath;
  std::string _rawFrameWindowParamsPath;
  std::string _filteredFrameWindowPath;
  std::string _filteredFrameWindowDisparityPath;
  std::string _filteredFrameWindowRawDisparityPath;
  std::string _filteredFrameWindowReferencePath;
  std::string _filteredFrameWindowParamsPath;
  std::string _filteredFrameWindowLabelsPath;
  std::string _filteredFrameWindowLabelsOverlayPath;
  std::string _filteredFrameWindowLabelsOverlayMaskedPath;
  std::string _rawPointCloudPath;
  std::string _rawPointCloudMetricsPath;
  std::string _filteredPointCloudPath;
  std::string _filteredPointCloudMetricsPath;
  std::string _clusteredPointCloudPath;
  std::string _clustersPath;
  std::string _clustersCoGsPath;
  std::string _descriptorsPath;
  std::string _matchesPath;
  std::string _controlPointsPath;
  std::string _chainedPointCloudPath;
  std::unordered_map<FrameNumber, std::shared_ptr<const PointCloud>> _clouds;
  std::unordered_map<FrameNumber, std::shared_ptr<const std::vector<Cluster>>>
      _clusters;

  std::string insertLabel(const std::string &templatePath, int l) const;
  std::string insertFrame(const std::string &templatePath, FrameNumber f) const;
  std::string insertStream(const std::string &templatePath,
                           StreamNumber f) const;

  /// only writes image, if not empty
  void writePng(const PictureD &pic, const std::string &path) const;

  std::vector<std::vector<double>> nColors(int n) const;

  /// force `nColors` to use these colors as first `_forcedNColors.size()`
  /// entries
  std::vector<std::vector<double>> _forcedNColors;

  std::set<size_t> _labelsToIgnore;
};

} // namespace MouseTrack
