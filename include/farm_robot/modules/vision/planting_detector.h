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

    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void loadModel(const std::string& modelPath) = 0;
    virtual void dispose() = 0;

    /**
     * Run detection and return planting point coordinates (x, y, z) in robot frame.
     * @return list of 3D coordinates
     */
    virtual std::vector<std::array<double, 3>> detect() = 0;
};

}  // namespace farm_robot
