/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_writer.h"
#include "color/color.h"
#include "generic/write_csv.h"
#include "generic/write_point_cloud.h"
#include <boost/algorithm/string/replace.hpp>
#include <boost/log/trivial.hpp>

namespace MouseTrack {

PipelineWriter::PipelineWriter(fs::path targetDir)
    : _outputDir(fs::absolute(targetDir)),
      _rawPointCloudPath("raw_point_cloud_<frameNumber>.ply"),
      _filteredPointCloudPath("filtered_point_cloud_<frameNumber>.ply"),
      _clusteredPointCloudPath("clustered_point_cloud_<frameNumber>.ply"),
      _clustersPath("clusters_<frameNumber>.csv"),
      _descriptorsPath("descriptors_<frameNumber>.csv"),
      _matchesPath("matches_<frameNumber>.csv"),
      _controlPointsPath("controlPoints_<frameNumber>.csv"),
      _chainedPointCloudPath("chained_point_cloud_<frameNumber>.ply") {
  if (!fs::exists(_outputDir)) {
    fs::create_directories(_outputDir);
  }

  if (!fs::is_directory(_outputDir)) {
    BOOST_LOG_TRIVIAL(info)
        << "Target output path is not a directory: " << _outputDir;
    throw "Target path not a directory.";
  }
}

void PipelineWriter::newRawPointCloud(FrameNumber f,
                                      std::shared_ptr<const PointCloud> cloud) {
  _clouds[f] = cloud;
  fs::path path = _outputDir / insertFrame(_rawPointCloudPath, f);
  write_point_cloud(path.string(), *cloud);
}

void PipelineWriter::newFilteredPointCloud(
    FrameNumber f, std::shared_ptr<const PointCloud> cloud) {
  // overwrite rawPointCloud
  _clouds[f] = cloud;

  fs::path path = _outputDir / insertFrame(_filteredPointCloudPath, f);
  write_point_cloud(path.string(), *cloud);
}

void PipelineWriter::newClusters(
    FrameNumber f, std::shared_ptr<const std::vector<Cluster>> clusters) {
  _clusters[f] = clusters;
  std::vector<std::vector<PointIndex>> tmp;
  tmp.reserve(clusters->size());
  for (const auto &c : *clusters) {
    tmp.push_back(c.points());
  }
  fs::path path = _outputDir / insertFrame(_clustersPath, f);
  write_csv(path.string(), tmp);

  // write clustered point cloud
  PointCloud cloud = *_clouds[f];
  BOOST_LOG_TRIVIAL(trace) << "Writing point cloud with " << cloud.size()
                           << " points.";
  std::vector<Cluster> largeClusters;
  for (auto &c : *clusters) {
    if (c.points().size() < 10) {
      continue;
    }
    largeClusters.push_back(c);
  }
  auto clusterColors = nColors(largeClusters.size());
  for (size_t ci = 0; ci < largeClusters.size(); ++ci) {
    const auto &cluster = largeClusters[ci];
    for (auto i : cluster.points()) {
      assert(i < cloud.size());
      auto color = clusterColors[ci];
      cloud[i].r(color[0]);
      cloud[i].g(color[1]);
      cloud[i].b(color[2]);
    }
  }
  fs::path cloudPath = _outputDir / insertFrame(_clusteredPointCloudPath, f);
  write_point_cloud(cloudPath.string(), cloud);
}

void PipelineWriter::newDescriptors(
    FrameNumber,
    std::shared_ptr<
        const std::vector<std::shared_ptr<const ClusterDescriptor>>>) {
  // empty
}

void PipelineWriter::newMatches(
    FrameNumber f, std::shared_ptr<const std::vector<long>> matches) {
  std::vector<std::vector<long>> tmp;
  tmp.push_back(*matches);

  fs::path path = _outputDir / insertFrame(_matchesPath, f);
  write_csv(path.string(), tmp);
}

void PipelineWriter::newControlPoints(
    FrameNumber f,
    std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints) {
  const int dimensions = 3;
  std::vector<std::vector<double>> tmp(controlPoints->size());
  for (size_t i = 0; i < controlPoints->size(); i += 1) {
    tmp[i].resize(dimensions);
    for (int j = 0; j < dimensions; j += 1) {
      tmp[i][j] = (*controlPoints)[i][j];
    }
  }
  fs::path path = _outputDir / insertFrame(_controlPointsPath, f);
  write_csv(path.string(), tmp);
}

std::string PipelineWriter::insertFrame(const std::string &templatePath,
                                        FrameNumber f) const {
  return boost::replace_all_copy(templatePath, "<frameNumber>",
                                 std::to_string(f));
}

void PipelineWriter::newClusterChains(
    std::shared_ptr<const std::vector<ClusterChain>> chains) {
  auto clusterColors = nColors(chains->size());
  for (auto cloudIt : _clouds) {
    FrameNumber f = cloudIt.first;
    PointCloud cloud = *cloudIt.second;
    for (size_t chainIndex = 0; chainIndex < chains->size(); ++chainIndex) {
      const ClusterChain &chain = (*chains)[chainIndex];
      const auto &clus = chain.clusters();
      auto cluster = clus.find(f);
      auto end = clus.end();
      if (cluster == end) {
        continue;
      }
      const Cluster &clu = *(cluster->second);
      for (auto i : clu.points()) {
        assert(i < cloud.size());
        auto color = clusterColors[chainIndex];
        cloud[i].r(color[0]);
        cloud[i].g(color[1]);
        cloud[i].b(color[2]);
      }
    }
    fs::path cloudPath = _outputDir / insertFrame(_chainedPointCloudPath, f);
    write_point_cloud(cloudPath.string(), cloud);
  }
}

std::vector<std::vector<double>> PipelineWriter::nColors(int n) const {
  auto cols = GenerateNColors(n);
  return cols;
}

} // namespace MouseTrack
