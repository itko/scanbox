/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline.h"
#include <boost/log/trivial.hpp>
#include <iostream>

namespace MouseTrack {

Pipeline::Pipeline() :
    _controller_should_run(false),
    _controller_running(false),
    _controller_terminated(true) {
    BOOST_LOG_TRIVIAL(debug) << "Constructing pipeline.";
}

Pipeline::Pipeline(
    std::unique_ptr<Reader> reader,
    std::unique_ptr<Registration> registration,
    std::unique_ptr<Clustering> clustering,
    std::unique_ptr<Descripting> descripting,
    std::unique_ptr<Matching> matching,
    std::unique_ptr<TrajectoryBuilder> trajectoryBuilder) :
    _controller_should_run(false),
    _controller_running(false),
    _controller_terminated(true),
    _reader(std::move(reader)),
    _registration(std::move(registration)),
    _clustering(std::move(clustering)),
    _descripting(std::move(descripting)),
    _matching(std::move(matching)),
    _trajectoryBuilder(std::move(trajectoryBuilder))
{
    // empty
}

Pipeline::~Pipeline() noexcept(true) {
    BOOST_LOG_TRIVIAL(trace) << "Deconstructing pipeline" << std::flush;
    join();
}

void Pipeline::moveMembersFrom(Pipeline& p) {
    _observers = std::move(p._observers);
    _reader = std::move(p._reader);
    _registration = std::move(p._registration);
    _clustering = std::move(p._clustering);
    _descripting = std::move(p._descripting);
    _matching = std::move(p._matching);
    _trajectoryBuilder = std::move(p._trajectoryBuilder);
}

Pipeline::Pipeline(MouseTrack::Pipeline&& p) {
    BOOST_LOG_TRIVIAL(trace) << "Move-constructing pipeline";
    if(this == &p){
        return;
    }

    std::lock_guard<std::mutex> lockC_rhs(p._controller_mutex);

    // wait for worker thread to terminate
    p._join();

    std::lock_guard<std::mutex> lockO_rhs(p._observer_mutex);
    moveMembersFrom(p);
}

Pipeline& Pipeline::operator=(Pipeline&& p) {
    BOOST_LOG_TRIVIAL(trace) << "Move-assigning pipeline";

    if(this == &p){
        return *this;
    }

    BOOST_LOG_TRIVIAL(trace) << "Moving pipeline: acquiring locks";
    std::unique_lock<std::mutex> lockC_lhs(_controller_mutex, std::defer_lock);
    std::unique_lock<std::mutex> lockC_rhs(p._controller_mutex, std::defer_lock);
    std::lock(lockC_lhs, lockC_rhs);

    // wait for worker threads to terminate
    _join();
    p._join();

    BOOST_LOG_TRIVIAL(trace) << "Moving pipeline: acquiring other locks";
    std::lock_guard<std::mutex> lockO_lhs(_observer_mutex);
    std::lock_guard<std::mutex> lockO_rhs(p._observer_mutex);
    BOOST_LOG_TRIVIAL(trace) << "Moving pipeline: moving";
    moveMembersFrom(p);
    return *this;
}

void Pipeline::start() {
    std::lock_guard<std::mutex> lock(_controller_mutex);
    _start();
}

void Pipeline::_start() {
    if(_controller_should_run || _controller_running || !_controller_terminated){
        BOOST_LOG_TRIVIAL(info) << "Controller thread not joined, ignoring start signal.";
        return;
    }
    BOOST_LOG_TRIVIAL(debug) << "Starting controller thread";
    _controller_should_run = true;
    _controller_running = true;
    _controller_terminated = false;
    _controller = std::thread([this](){controllerStart();});
}

void Pipeline::stop() {
    std::lock_guard<std::mutex> lock(_controller_mutex);
    _stop();
}

void Pipeline::_stop() {
    BOOST_LOG_TRIVIAL(debug) << "Stop signal sent to controller thread.";
    _controller_should_run = false;
}

void Pipeline::join() {
    std::lock_guard<std::mutex> lock(_controller_mutex);
    _join();
}

void Pipeline::_join(){
    BOOST_LOG_TRIVIAL(trace) << "Join of controller thread requested.";
    if(_controller_terminated && !_controller.joinable()){
        BOOST_LOG_TRIVIAL(trace) << "Controller thread already joined.";
        return;
    }
    BOOST_LOG_TRIVIAL(debug) << "Joining controller thread.";
    _controller.join();
    _controller_terminated = true;
}

// observer handling

void Pipeline::addObserver(PipelineObserver* observer) {
    std::lock_guard<std::mutex> lock(_observer_mutex);
    _observers.insert(observer);
}

void Pipeline::removeObserver(PipelineObserver* observer) {
    std::lock_guard<std::mutex> lock(_observer_mutex);
    _observers.erase(observer);
}

// controller thread

// the controller thread is not allowed to lock _controller_mutex
// the only communication happens via the two flag variables


void Pipeline::controllerStart() {
    BOOST_LOG_TRIVIAL(debug) << "Controller thread started";
    forallObservers( [](PipelineObserver* o){o->pipelineStarted();} );

    // do work
    runPipeline();

    forallObservers( [](PipelineObserver* o){o->pipelineTerminated();} );
    _controller_should_run = false;
    _controller_running = false;
    BOOST_LOG_TRIVIAL(debug) << "Controller thread terminated";
}


void Pipeline::runPipeline() {
    if(_reader == nullptr){
        BOOST_LOG_TRIVIAL(info) << "No reader set, terminating controller thread.";
        return;
    }

    _clusterChains.clear();

    for(FrameIndex f = _reader->beginFrame(); f < _reader->endFrame(); f += 1){
        if(terminateEarly()){
            return;
        }
        processFrame(f);
    }
}


bool Pipeline::terminateEarly() {
    if(!_controller_should_run){
        BOOST_LOG_TRIVIAL(debug) << "Terminate early.";
        return true;
    }
    return false;
}

void Pipeline::processFrame(FrameIndex f) {
    // tell everybody we're starting to process frame f
    BOOST_LOG_TRIVIAL(debug) << "Processing frame " << f;
    forallObservers( [=](PipelineObserver* o){o->frameStart(f);} );

    // Read frame data
    forallObservers( [=](PipelineObserver* o){o->startFrameWindow(f);} );
    std::unique_ptr<FrameWindow> windowPtr(new FrameWindow);
    (*windowPtr) = (*_reader)(f);
    std::shared_ptr<const FrameWindow> window{std::move(windowPtr)};
    forallObservers( [=](PipelineObserver* o){o->newFrameWindow(f, window);} );

    if(terminateEarly()){
        forallObservers( [=](PipelineObserver* o){o->frameEnd(f);} );
        return;
    }

    // get raw point cloud
    if(_registration == nullptr) {
        BOOST_LOG_TRIVIAL(info) << "No regsitration object set, stopping processing of frame " << f;
        forallObservers( [=](PipelineObserver* o){o->frameEnd(f);} );
        return;
    }
    forallObservers( [=](PipelineObserver* o){o->startRegistration(f);} );
    std::unique_ptr<PointCloud> rawPointCloudPtr(new PointCloud());
    (*rawPointCloudPtr) = (*_registration)(*window);
    std::shared_ptr<const PointCloud> rawPointCloud{std::move(rawPointCloudPtr)};
    forallObservers( [=](PipelineObserver* o){o->newRawPointCloud(f, rawPointCloud);} );

    if(terminateEarly()){
        forallObservers( [=](PipelineObserver* o){o->frameEnd(f);} );
        return;
    }

    // cluster the point cloud
    if(_clustering == nullptr) {
        BOOST_LOG_TRIVIAL(info) << "No clustering object set, stopping processing of frame " << f;
        forallObservers( [=](PipelineObserver* o){o->frameEnd(f);} );
        return;
    }
    forallObservers( [=](PipelineObserver* o){o->startClustering(f);} );
    std::unique_ptr<std::vector<Cluster>> clustersPtr(new std::vector<Cluster>());
    (*clustersPtr) = (*_clustering)(*rawPointCloud);
    std::shared_ptr<const std::vector<Cluster>> clusters(std::move(clustersPtr));
    forallObservers( [=](PipelineObserver* o){o->newClusters(f, clusters);} );

    if(terminateEarly()){
        forallObservers( [=](PipelineObserver* o){o->frameEnd(f);} );
        return;
    }

    // extract cluster descriptors

    if(_descripting == nullptr) {
        BOOST_LOG_TRIVIAL(info) << "No descripting object set, stopping processing of frame " << f;
        forallObservers( [=](PipelineObserver* o){o->frameEnd(f);} );
        return;
    }
    forallObservers( [=](PipelineObserver* o){o->startDescripting(f);} );
    std::unique_ptr<std::vector<std::shared_ptr<const ClusterDescriptor>>> descriptorsPtr(new std::vector<std::shared_ptr<const ClusterDescriptor>>());
    for(const Cluster& cluster : *clusters){
        std::shared_ptr<ClusterDescriptor> descriptor = (*_descripting)(cluster, *rawPointCloud);
        descriptorsPtr->push_back(descriptor);
    }
    std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>> descriptors(std::move(descriptorsPtr));
    forallObservers( [=](PipelineObserver* o){o->newDescriptors(f, descriptors);} );

    if(terminateEarly()){
        forallObservers( [=](PipelineObserver* o){o->frameEnd(f);} );
        return;
    }

    // Get matching indexes
    // matches: descriptorIndex -> clusterChain index, if no match: -1
    if(_matching == nullptr) {
        BOOST_LOG_TRIVIAL(info) << "No matching object set, stopping processing of frame " << f;
        forallObservers( [=](PipelineObserver* o){o->frameEnd(f);} );
        return;
    }
    forallObservers( [=](PipelineObserver* o){o->startMatching(f);} );
    std::unique_ptr<std::vector<long>> matchesPtr(new std::vector<long>());
    (*matchesPtr) = (*_matching)(*descriptors, _clusterChains);
    std::shared_ptr<const std::vector<long>> matches(std::move(matchesPtr));
    forallObservers( [=](PipelineObserver* o){o->newMatches(f, matches);} );

    if(terminateEarly()){
        forallObservers( [=](PipelineObserver* o){o->frameEnd(f);} );
        return;
    }

    if(_trajectoryBuilder == nullptr) {
        BOOST_LOG_TRIVIAL(info) << "No trajectory builder object set, stopping processing of frame " << f;
        forallObservers( [=](PipelineObserver* o){o->frameEnd(f);} );
        return;
    }
    forallObservers( [=](PipelineObserver* o){o->startControlPoints(f);} );
    // append descriptors and clusters to cluster chain according to matches vector
    for(size_t i = 0; i < descriptors->size(); i += 1){
        int chainIndex = (*matches)[i];
        if(chainIndex == -1){
            ClusterChain chain;
            chainIndex = _clusterChains.size();
            _clusterChains.push_back(chain);
        }
        auto& chain = _clusterChains[chainIndex];
        chain.clusters()[f] = &(*clusters)[i];
        chain.descriptors()[f] = (*descriptors)[i];
    }

    // get 3d control points for trajectory
    std::unique_ptr<std::vector<Eigen::Vector3d>> controlPointsPtr{new std::vector<Eigen::Vector3d>};
    for(const auto& chain : _clusterChains){
        const auto clusterIter = chain.clusters().find(f);
        if(chain.clusters().end() == clusterIter){
            // no entry for this frame
            controlPointsPtr->push_back(Eigen::Vector3d(0,0,0)); // better solution?
            continue;
        }
        const auto descriptorIter = chain.descriptors().find(f);
        const Cluster& cluster = *(clusterIter->second);
        const std::shared_ptr<const ClusterDescriptor> descriptor = (descriptorIter->second);
        controlPointsPtr->push_back((*_trajectoryBuilder)(descriptor, cluster, *rawPointCloud));
    }

    std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints{std::move(controlPointsPtr)};

    forallObservers( [=](PipelineObserver* o){o->newControlPoints(f, controlPoints);} );

    // tell everybody we're done
    BOOST_LOG_TRIVIAL(debug) << "Frame " << f << " processed.";
    forallObservers( [=](PipelineObserver* o){o->frameEnd(f);} );
}



} // MouseTrack
