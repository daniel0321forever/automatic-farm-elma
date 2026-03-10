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

    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void loadModel(const std::string& modelPath) = 0;
    virtual void disposeModel() = 0;

    /**
     * Run detection and return target coordinates (x, y, z) in robot frame.
     * @return optional 3D coordinate if detected, empty otherwise
     */
    virtual std::optional<std::array<double, 3>> detect() = 0;
};

}  // namespace farm_robot
