/**
 * @file center_planting_detector.h
 * @brief Center-label planting detector (labels -> centers).
 *
 * Uses CenterLabelDetector for binary image, contours, DBSCAN. Stub implementation.
 * C++ counterpart of modules/vision/infra/planting/center/center_planting_detector.py.
 */

#pragma once

#include "farm_robot/modules/vision/planting_detector.h"

namespace farm_robot {

class CenterPlantingDetector : public IPlantingDetectorModule {
public:
    void connect() override;
    void disconnect() override;
    void loadModel(const std::string& modelPath) override;
    void dispose() override;
    std::vector<std::array<double, 3>> detect() override;
};

}  // namespace farm_robot
