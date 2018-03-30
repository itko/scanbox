#include "matlab_reader.h"
#include "generic/read_csv.h"
#include "generic/read_png.h"

#include <unordered_map>
#include <regex>
#include <set>
#include <fstream>
#include <boost/log/trivial.hpp>
#include <boost/range.hpp>

namespace MouseTrack {

/// Designed to capture upper and lower bounds for stream and frame numbers
struct IndexAggregateSF {
    StreamNumber firstStream = std::numeric_limits<FrameNumber>::max();
    StreamNumber lastStream = -1;
    FrameNumber firstFrame = std::numeric_limits<FrameNumber>::max();
    FrameNumber lastFrame = -1;
    std::set<std::string> suffixes;

    IndexAggregateSF() :
        firstStream(std::numeric_limits<StreamNumber>::max()),
        lastStream(-1),
        firstFrame(std::numeric_limits<FrameNumber>::max()),
        lastFrame(-1)
        {
        // empty
    }
    void addStream(StreamNumber s){
        firstStream = std::min(firstStream, s);
        lastStream = std::max(lastStream, s);
    }
    void addFrame(FrameNumber f){
        firstFrame = std::min(firstFrame, f);
        lastFrame = std::max(lastFrame, f);
    }
    void addSuffix(std::string suff) {
        suffixes.insert(suff);
    }
};


/// Designed to capture upper and lower bounds for frame numbers
struct IndexAggregateF {
    FrameNumber firstFrame = std::numeric_limits<FrameNumber>::max();
    FrameNumber lastFrame = -1;
    std::set<std::string> suffixes;

    IndexAggregateF() :
        firstFrame(std::numeric_limits<FrameNumber>::max()),
        lastFrame(-1)
        {
        // empty
    }
    void addFrame(FrameNumber f){
        firstFrame = std::min(firstFrame, f);
        lastFrame = std::max(lastFrame, f);
    }
    void addSuffix(std::string suff) {
        suffixes.insert(suff);
    }
};

// base file names for bag files
const std::string NORMALIZED_DISPARITY_KEY{"depth_normalized"};
const std::string RAW_DISPARITY_KEY{"depth"};
const std::string FRAME_PARAMETERS_KEY{"params"};
const std::string REFERENCE_PICTURE_KEY{"pic"};
const std::string ROTATION_CORRECTION_KEY{"params_R"};
const std::string CAMERA_CHAIN_KEY{"params_camchain"};

const std::set<std::string> EXPECTED_CHANNELS {NORMALIZED_DISPARITY_KEY, RAW_DISPARITY_KEY, FRAME_PARAMETERS_KEY, REFERENCE_PICTURE_KEY};
const std::set<std::string> EXPECTED_FILES {ROTATION_CORRECTION_KEY, CAMERA_CHAIN_KEY};

MatlabReader::MatlabReader(fs::path root_directory) : _root(fs::absolute(root_directory)), _valid(false) {
    if(!fs::exists(_root)){
        BOOST_LOG_TRIVIAL(info) << "Path for root directory " << root_directory.string() << "does not exist (absolute: " << _root << ")";
        throw "Root directory not found.";
    }
    if(!fs::is_directory(_root)){
        BOOST_LOG_TRIVIAL(info) << "Path for root directory does not point to a directory: " << _root;
        throw "Root path not a directory.";
    }

    preflight();
}
void MatlabReader::preflight() {

    /// We want to work out the index range for streams and frames.
    /// For this we iterate over each file, applying a regular
    /// expresson to extract the numbers. First we always widen
    /// the index range on a per channel basis.
    /// Then we take the smallest common range as final result.
    /// While iterating over all entries, we also store the file paths to the files
    /// for easy look up later on.
    /// We list the paths from the directory iterator in the ignored file list, but
    /// store the paths from following symlinks in all other cases.
    std::unordered_map<std::string, IndexAggregateSF> aggSF;
    std::unordered_map<std::string, IndexAggregateF> aggF;
    std::regex nameShape("^([A-Za-z0-9_-]+?)(?:_s_([0-9]+))?_f_([0-9]+)\\.([A-Za-z0-9]+)$", std::regex::ECMAScript);
    // TODO: Boost 1.54 does not support fs::directory_iterator as we would like to use it here
    // for(const auto& p : fs::directory_iterator(_root)){
    // Later versions do, but I can't make travis use the newer versions, hence this older code until an update is possible
    for(const auto& p : boost::make_iterator_range(fs::directory_iterator(_root), fs::directory_iterator())){
        fs::path path = p.path();
        BOOST_LOG_TRIVIAL(trace) << "Found path: " << path.string();
        int evaluatedSymlinks = 0;
        while(evaluatedSymlinks < _followSymlinkDepth && fs::is_symlink(path)){
            path = fs::read_symlink(path);
            BOOST_LOG_TRIVIAL(info) << "Followed symbolic link to: " << path.string();
            evaluatedSymlinks += 1;
        }
        if(!fs::is_regular_file(path)){
            // add original path to ignore list
            BOOST_LOG_TRIVIAL(info) << "Ignored non regular file: " << p.path().string();
            _ignoredPaths.push_back(p.path());
            continue;
        }
        // we have now two different paths:
        // p.path(): the representation we should use to recognize
        // path: path to the actual file, might be called anything, but this is the thing we want to store
        auto exp = EXPECTED_FILES.find(p.path().stem().string());
        if(exp != EXPECTED_FILES.end()){
            if(*exp == ROTATION_CORRECTION_KEY){
                BOOST_LOG_TRIVIAL(trace) << "Found rotations: " << p.path().string() << " at " << path.string();
                _files.rotationCorrectionsPath.insert(path);
            } else if (*exp == CAMERA_CHAIN_KEY) {
                BOOST_LOG_TRIVIAL(trace) << "Found camchain: " << p.path().string() << " at " << path.string();
                _files.camchainPath.insert(path);
            } else {
                // log: found internal inconsistency,
                BOOST_LOG_TRIVIAL(warning) << "Found internal inconsistency: expected file has no storage location in members: " << p.path().string() << " File ignored.";
                // file from expectedFiles has no storage location in members
                _ignoredPaths.push_back(p.path());
            }
            // no further checking, we know what it is
            continue;
        }
        std::cmatch cm;
        std::string channel;
        int stream, frame;

        std::string filename{p.path().filename().string()};
        std::regex_match(filename.c_str(), cm, nameShape);
        std::string suffix = p.path().extension().string();

        if(!cm[2].matched){
            BOOST_LOG_TRIVIAL(trace) << "Matched frame: " << p.path().string() << " at " << path.string() << ": " << cm[1] << ", " << cm[2] << ", " << cm[3];
            channel = cm[1];
            // internal key, not usable from outside, since we only allow non-negative integers
            stream = -1;
            frame = std::stoi(cm[3]);

            auto& a = aggF[channel];
            a.addSuffix(suffix);
            a.addFrame(frame);
        } else if (cm[1].matched) {
            BOOST_LOG_TRIVIAL(trace) << "Matched stream and frame: " << p.path().string() << " at " << path.string() << ": " << cm[1] << ", " << cm[2] << ", " << cm[3];
            channel = cm[1];
            stream = std::stoi(cm[2]);
            frame = std::stoi(cm[3]);

            auto& a = aggSF[channel];
            a.addSuffix(suffix);
            a.addStream(stream);
            a.addFrame(frame);
        } else {
            BOOST_LOG_TRIVIAL(info) << "Ignoring path " << p.path().string() << ", does not match any expected file name.";
            _ignoredPaths.push_back(p.path());
            continue;
        }
        BOOST_LOG_TRIVIAL(trace) << "Adding frame file " << path.string() << " from " << p.path().string();
        _files.frames[ElementKey(stream, frame, channel)].insert(path);
    }
    StreamNumber firstStream = std::numeric_limits<StreamNumber>::max();
    StreamNumber lastStream = -1;
    FrameNumber firstFrame = std::numeric_limits<StreamNumber>::max();
    FrameNumber lastFrame = -1;
    for(const auto& kv : aggSF){
        if(EXPECTED_CHANNELS.find(kv.first) == EXPECTED_CHANNELS.end()){
            _ignoredChannels.push_back(kv.first);
            continue;
        }
        firstStream = std::min(firstStream, kv.second.firstStream);
        lastStream = std::max(lastStream, kv.second.lastStream);
        firstFrame = std::min(firstFrame, kv.second.firstFrame);
        lastFrame = std::max(lastFrame, kv.second.lastFrame);
    }

    for(const auto& kv : aggF){
        if(EXPECTED_CHANNELS.find(kv.first) == EXPECTED_CHANNELS.end()){
            _ignoredChannels.push_back(kv.first);
        }
        firstFrame = std::min(firstFrame, kv.second.firstFrame);
        lastFrame = std::max(lastFrame, kv.second.lastFrame);
    }

    _valid = firstStream <= lastStream && firstFrame <= lastFrame;

    if(!_valid){
        _beginStream = 0;
        _endStream = 0;
        _beginFrame = 0;
        _endFrame = 0;
        return;
    } else {
        _beginStream = firstStream;
        _endStream = lastStream + 1;
        _beginFrame = firstFrame;
        _endFrame = lastFrame + 1;
    }

    _cache.rotationCorrections = readRotationCorrections();
    _cache.camchain = readCamchain();
}


bool MatlabReader::valid() const {
    return _valid;
}

StreamNumber MatlabReader::beginStream() const {
    return _beginStream;
}

StreamNumber MatlabReader::endStream() const {
    return _endStream;
}

FrameNumber MatlabReader::beginFrame() const {
    return _beginFrame;
}

FrameNumber MatlabReader::endFrame() const {
    return _endFrame;
}

const std::vector<fs::path>& MatlabReader::ignoredPaths() const {
    return _ignoredPaths;
}

const std::vector<std::string>& MatlabReader::ignoredChannels() const {
    return _ignoredChannels;
}

FrameWindow MatlabReader::frameWindow(FrameNumber f) const {
    unsigned int count = endStream() - beginStream();
    std::vector<Frame> frames{count};
    Eigen::MatrixXd params = channelParameters(f);
    for(int s = beginStream(); s < endStream(); s += 1){
        auto& frame = frames[s-beginStream()];
        // read stream dependent files
        frame.normalizedDisparityMap = normalizedDisparityMap(s, f);
        frame.rawDisparityMap = rawDisparityMap(s, f);
        frame.referencePicture = picture(s, f);
        // fread frame-only dependent files
        frame.focallength = params(s-1, 0);
        frame.baseline = params(s-1, 1);
        frame.ccx = params(s-1, 2);
        frame.ccy = params(s-1, 3);
        // read cached data
        frame.rotationCorrection = rotationCorrections()[s-1];
        frame.camChainPicture = camchain()[2*(s-1)];
        frame.camChainDisparity = camchain()[2*(s-1)+1];
    }
    FrameWindow window;
    window.frames() = std::move(frames);
    return window;
}

/// This layer decides, if a file needs to be touched, or if there's a cached version

DisparityMap MatlabReader::normalizedDisparityMap(StreamNumber s, FrameNumber f) const {
    return readNormalizedDisparityMap(s, f);
}

DisparityMap MatlabReader::rawDisparityMap(StreamNumber s, FrameNumber f) const {
    return readRawDisparityMap(s, f);
}

Eigen::MatrixXd MatlabReader::channelParameters(FrameNumber f) const {
    return readChannelParameters(f);
}

Eigen::MatrixXd MatlabReader::picture(StreamNumber s, FrameNumber f) const {
    return readPicture(s, f);
}

std::vector<Eigen::Matrix4d> MatlabReader::rotationCorrections() const {
    return _cache.rotationCorrections;
}

std::vector<Eigen::Matrix4d> MatlabReader::camchain() const {
    return _cache.camchain;
}

/// This layer touches files


/// Read normalized disparity map for frame f and stream s
DisparityMap MatlabReader::readNormalizedDisparityMap(StreamNumber s, FrameNumber f) const {
    const auto candidatesPtr = _files.frames.find(ElementKey(s, f, NORMALIZED_DISPARITY_KEY));
    if(candidatesPtr == _files.frames.end()){
        throw "No file found.";
    }
    fs::path p = chooseCandidate(candidatesPtr->second);
    BOOST_LOG_TRIVIAL(trace) << "readNormalizedDisparityMap: " << p.string();
    Picture map = read_png_normalized(p.string());
    DisparityMap result{std::move(map)};
    return result;
}

/// Read disparity map as stored in bag file for frame f and stream s
DisparityMap MatlabReader::readRawDisparityMap(StreamNumber s, FrameNumber f) const {
    const auto candidatesPtr = _files.frames.find(ElementKey(s, f, RAW_DISPARITY_KEY));
    if(candidatesPtr == _files.frames.end()){
        throw "No file found.";
    }
    fs::path p = chooseCandidate(candidatesPtr->second);
    BOOST_LOG_TRIVIAL(trace) << "readRawDisparityMap: " << p.string();
    Picture map = read_png_normalized(p.string());
    DisparityMap result{std::move(map)};
    return result;
}

/// read channel parameters (focallength, baseline, ccx, ccy)
Eigen::MatrixXd MatlabReader::readChannelParameters(FrameNumber f) const {
    const auto candidatesPtr = _files.frames.find(ElementKey(-1, f, FRAME_PARAMETERS_KEY));
    if(candidatesPtr == _files.frames.end()){
        throw "No file found.";
    }
    fs::path p = chooseCandidate(candidatesPtr->second);
    BOOST_LOG_TRIVIAL(trace) << "readChannelParameters: " << p.string();
    return readMatrix(p);
}

/// read left camera picture for frame f and stream s
Picture MatlabReader::readPicture(StreamNumber s, FrameNumber f) const {
    const auto candidatesPtr = _files.frames.find(ElementKey(s, f, REFERENCE_PICTURE_KEY));
    if(candidatesPtr == _files.frames.end()){
        throw "No file found.";
    }
    fs::path p = chooseCandidate(candidatesPtr->second);
    BOOST_LOG_TRIVIAL(trace) << "readPicture: " << p.string();
    return read_png_normalized(p.string());
}

/// Read rotation corrections for cameras
std::vector<Eigen::Matrix4d> MatlabReader::readRotationCorrections() const {
    const auto& candidates = _files.rotationCorrectionsPath;
    fs::path p = chooseCandidate(candidates);
    BOOST_LOG_TRIVIAL(trace) << "readRotationCorrections: " << p.string();
    return read4x4MatrixList(p);
}

/// Read the camera chain (8 rotation matrices that define how the cameras are positioned to each other)
std::vector<Eigen::Matrix4d> MatlabReader::readCamchain() const {
    const auto& candidates = _files.camchainPath;
    fs::path p = chooseCandidate(candidates);
    BOOST_LOG_TRIVIAL(trace) << "readCamchain: " << p.string();
    return read4x4MatrixList(p);
}


std::vector<Eigen::Matrix4d> MatlabReader::read4x4MatrixList(const fs::path& file) const {
    const auto fileContent = read_csv(file.string());
    // matlab reshapes matrices colum wise: [1 2; 3 4] -> reshape -> [1 3 2 4]
    const int rows = 4;
    const int cols = 4;
    std::vector<Eigen::Matrix4d> result(fileContent.size());
    for(size_t mat = 0; mat < fileContent.size(); mat += 1){
        for(size_t j = 0; j < rows; j += 1){
            for(size_t i = 0; i < cols; i += 1){
                double entry = std::stod(fileContent[mat][j*cols + i]);
                result[mat](i,j) = entry;
            }
        }
    }
    return result;
}

Eigen::MatrixXd MatlabReader::readMatrix(const fs::path& file) const {
    const auto fileContent = read_csv(file.string());
    // matlab reshapes matrices colum wise: [1 2; 3 4] -> reshape -> [1 3 2 4]
    Eigen::MatrixXd result;
    if(fileContent.empty()) {
        return result;
    }
    if(fileContent[0].empty()) {
        return result;
    }
    result.resize(fileContent.size(), fileContent[0].size());
    for(size_t i = 0; i < fileContent.size(); i += 1){
        for(size_t j = 0; j < fileContent[i].size(); j += 1){
            double entry = std::stod(fileContent[i][j]);
            result(i,j) = entry;
        }
    }
    return result;
}

fs::path MatlabReader::chooseCandidate(const std::set<fs::path>& candidates) const {
    fs::path p;
    for(const fs::path& c : candidates){
        // last chance, to choose something that exists
        // this check could be improved by taking file endings into account and favoring more efficient versions
        if(c.empty()){
            continue;
        }
        if(!fs::exists(c)){
            continue;
        }
        if(!fs::is_regular_file(c)){
            continue;
        }
        return c;
    }
    throw "no path found.";
}

// ElementKey

bool MatlabReader::ElementKey::operator<(const MatlabReader::ElementKey& other) const {
    if(stream == other.stream){
        if(frame == other.frame){
            return channel < other.channel;
        }
        return frame < other.frame;
    }
    return stream < other.stream;
}


} // MouseTrack
