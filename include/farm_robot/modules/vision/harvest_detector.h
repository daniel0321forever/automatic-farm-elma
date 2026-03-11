/**
 * @file harvest_detector.h
 * @brief Interface for harvest detection (YOLO keypoint/segment, OAK-D depth).
 *
 * Base interface for harvest detectors: OAK camera, depth-to-3D, camera-to-robot
 * transform, YOLO model loading. Implementations include mock, keypoint, segment.
 * C++ counterpart of modules/vision/domain/harvest_detector.py.
 */

#pragma once

#include <array>
#include <optional>
#include <memory>
#include <vector>

namespace farm_robot {

class IHarvestDetectorModule {
public:
    virtual ~IHarvestDetectorModule() = default;

    //! Connects to the detector (e.g. camera or mock).
    virtual void connect() = 0;
    //! Disconnects from the detector.
    virtual void disconnect() = 0;
    //! Loads the detection model from the given path.
    virtual void loadModel(const std::string& modelPath) = 0;
    //! Disposes of model resources.
    virtual void disposeModel() = 0;

    //! Runs detection and returns target (x,y,z) in robot frame if detected.
    virtual std::optional<std::array<double, 3>> detect() = 0;
};

}  // namespace farm_robot
