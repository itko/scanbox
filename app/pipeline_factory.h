/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "pipeline.h"
#include "spatial/oracle_factory.h"
#include "types.h"

namespace MouseTrack {

/// This class knows about all pipeline modules and how to connect them to
/// create a valid Pipeline.
class PipelineFactory {
public:
  /// temporary solution until we have some internal representation of the
  /// pipeline configuration
  Pipeline fromCliOptions(const op::variables_map &options) const;

private:
  typedef OracleFactoryXd OFactory;
  /// Depending on the given options, choose, create and return a reader
  std::unique_ptr<Reader> getReader(const op::variables_map &options) const;

  /// Depending on the option, create a list of frame window filters
  std::vector<std::unique_ptr<FrameWindowFiltering>>
  getWindowFilters(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a frame window
  /// filter
  std::unique_ptr<FrameWindowFiltering>
  getWindowFiltering(const std::string &target,
                     const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a registration
  /// module
  std::unique_ptr<Registration>
  getRegistration(const op::variables_map &options) const;

  /// Depending on the options, create a list of point cloud filters
  std::vector<std::unique_ptr<PointCloudFiltering>>
  getCloudFilters(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a point cloud
  /// filtering module
  std::unique_ptr<PointCloudFiltering>
  getCloudFiltering(const std::string &target,
                    const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a clustering
  /// module
  std::unique_ptr<Clustering>
  getClustering(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a descripting
  /// module
  std::unique_ptr<Descripting>
  getDescripting(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a matching
  /// module
  std::unique_ptr<Matching> getMatching(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a trajectory
  /// builder module
  std::unique_ptr<TrajectoryBuilder>
  getTrajectoryBuilder(const op::variables_map &options) const;

  /// Choose a value from the Oracles enum based on a given string
  OFactory::Oracles getOracle(const std::string &oracleKey) const;
};

} // namespace MouseTrack
