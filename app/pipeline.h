/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "clustering/clustering.h"
#include "descripting/descripting.h"
#include "frame_window_filtering/frame_window_filtering.h"
#include "matching/matching.h"
#include "pipeline_delegate.h"
#include "pipeline_observer.h"
#include "point_cloud_filtering/point_cloud_filtering.h"
#include "reader/reader.h"
#include "registration/registration.h"
#include "trajectory_builder/trajectory_builder.h"
#include <atomic>
#include <mutex>
#include <set>
#include <thread>

namespace MouseTrack {

/// This object is responsible to coordinate the processing steps
///
/// - It knows how to acquire new frames (reader or delegate)
/// - It knows how to push the frames through the pipeline steps
/// - It informs other objects about (inbetween-)results via an observer pattern
/// (PipelineObserver)
/// - An optional PipelineDelegate can be set to micromanage pipeline execution.
///
/// The pipeline runs one or multiple worker threads to process the frame stream
/// concurrently. Start the processing with `start()`. If you want to terminate
/// the pipeline early, call `stop()`, a signal will be sent to the pipeline, it
/// should stop as soon as possible. Always call `join` before terminating, it
/// will make sure that all worker threads terminated and delivered their data.
/// Also see `std::thread` for further information on threads.
class Pipeline {
public:
  /// Default constructor, create empty pipeline
  Pipeline();

  /// The first module pointer with a nullptr defines the end of the pipeline.
  Pipeline(std::unique_ptr<Reader> reader,
           std::vector<std::unique_ptr<FrameWindowFiltering>> &&windowFiltering,
           std::unique_ptr<Registration> registration,
           std::vector<std::unique_ptr<PointCloudFiltering>> &&cloudFiltering,
           std::unique_ptr<Clustering> clustering,
           std::unique_ptr<Descripting> descripting,
           std::unique_ptr<Matching> matching,
           std::unique_ptr<TrajectoryBuilder> trajectoryBuilder);

  /// Copy move constructor
  Pipeline(MouseTrack::Pipeline &&);

  /// Default destructor
  ~Pipeline() noexcept(true);

  /// Move assignment operator
  Pipeline &operator=(Pipeline &&);

  /// Start processing
  void start();

  /// Tell the pipeline to terminate as soon as possible.
  /// It will return immediately so you can to other things until the worker
  /// finished.
  void stop();

  /// Wait until the pipeline terminates, don't call concurrently
  /// If the worker thread already terminated or never started, it
  /// will return immediately.
  ///
  /// The destructor makes sure, worker thread is terminated properly.
  /// But you probably want to join manually to make sure, all data was
  /// delivered.
  ///
  /// Observers are guaranteed not to be called after `join()`returns,
  /// but they might be called while `join()` is executed.
  void join();

  /// Don't expect the observes to be called in a particular order.
  void addObserver(PipelineObserver *observer);

  void removeObserver(PipelineObserver *observer);

  /// Sets a pointer to a delegate object
  /// You can change the delegate at runtime (all operations are atomic) only to
  /// another valid object.
  ///
  /// Note: The new delegate will have to take over at an arbitrary point, so
  /// both delegates need to be synchronized. For your own safety, just pause
  /// the pipeline and restart it.
  ///
  /// Warning: You are not allowed to set the delegate to
  /// `nullptr`, while the pipeline is running. In this case, behavior is
  /// undefined.
  void setDelegate(PipelineDelegate *delegate);

private:
  /// This mutex regulates communication with the outside
  /// (observers and delegate)
  mutable std::mutex _observer_mutex;
  std::set<PipelineObserver *> _observers;
  PipelineDelegate *_delegate;

  /// main worker thread, coordinates work distribution and communicates with
  /// observer
  std::thread _controller;

  /// true as long as the controller is allowed to run
  /// setting it to false is a signal to stop
  std::atomic_bool _controller_should_run;

  /// true if the controller thread is running
  std::atomic_bool _controller_running;

  /// We joined the controller thread
  std::atomic_bool _controller_terminated;

  /// Access to start/stop of controller thread
  mutable std::mutex _controller_mutex;

  /// starting point for controller thread
  void controllerStart();

  /// coordinates pipeline for one frame
  void processFrame(FrameNumber f);

  /// Wraps `processFrame` and takes care of exception handling
  void processFrameSafe(FrameNumber f);

  template <typename Lambda> void forallObservers(Lambda lambda) {
    std::lock_guard<std::mutex> lock(_observer_mutex);
    for (PipelineObserver *o : _observers) {
      lambda(o);
    }
  }

  template <typename Lambda> auto askDelegate(Lambda lambda) {
    std::lock_guard<std::mutex> lock(_observer_mutex);
    return lambda(_delegate);
  }

  /// Helper for move operations: moves all relevant data members (not mutexes,
  /// not threads ...)
  void moveMembersFrom(Pipeline &p);

  /// Like `start()`, but does not lock thread
  void _start();

  /// Like `stop()`, but does not lock thread
  void _stop();

  /// Like `join()`, but does not lock thread
  void _join();

  // pipeline steps

  std::unique_ptr<Reader> _reader;
  std::vector<std::unique_ptr<FrameWindowFiltering>> _frameWindowFiltering;
  std::unique_ptr<Registration> _registration;
  std::vector<std::unique_ptr<PointCloudFiltering>> _cloudFiltering;
  std::unique_ptr<Clustering> _clustering;
  std::unique_ptr<Descripting> _descripting;
  std::unique_ptr<Matching> _matching;
  std::unique_ptr<TrajectoryBuilder> _trajectoryBuilder;
  std::vector<ClusterChain> _clusterChains;

  void runPipeline();

  bool terminateEarly();
};

} // namespace MouseTrack
