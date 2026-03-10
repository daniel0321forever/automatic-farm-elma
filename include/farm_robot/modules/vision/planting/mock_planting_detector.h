/**
 * @file mock_planting_detector.h
 * @brief Mock planting detector for testing.
 *
 * Simulated planting detector returning dummy coordinates. C++ counterpart of
 * modules/vision/infra/planting/mock/mock_planting_module.py.
 */

#pragma once

#include "farm_robot/modules/vision/planting_detector.h"

namespace farm_robot {

class MockPlantingDetector : public IPlantingDetectorModule {
public:
    void connect() override;
    void disconnect() override;
    void loadModel(const std::string& modelPath) override;
    void dispose() override;
    std::vector<std::array<double, 3>> detect() override;
};

}  // namespace farm_robot
