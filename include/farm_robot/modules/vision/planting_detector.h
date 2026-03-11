/**
 * @file planting_detector.h
 * @brief Interface for planting detection (labels, DBSCAN clustering).
 *
 * Base interface for planting detectors: label detectors, stable detection
 * using DBSCAN clustering. Implementations include mock, center, extending.
 * C++ counterpart of modules/vision/domain/planting_detector.py.
 */

#pragma once

#include <array>
#include <vector>
#include <memory>

namespace farm_robot {

class IPlantingDetectorModule {
public:
    virtual ~IPlantingDetectorModule() = default;

    //! Connects to the detector (e.g. camera or mock).
    virtual void connect() = 0;
    //! Disconnects from the detector.
    virtual void disconnect() = 0;
    //! Loads the detection model from the given path.
    virtual void loadModel(const std::string& modelPath) = 0;
    //! Disposes of model resources.
    virtual void dispose() = 0;

    //! Runs detection and returns planting point coordinates (x,y,z) in robot frame.
    virtual std::vector<std::array<double, 3>> detect() = 0;
};

}  // namespace farm_robot
