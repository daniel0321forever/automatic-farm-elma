/**
 * @file mock_harvest_detector.h
 * @brief Mock harvest detector for testing.
 *
 * Simulated harvest detector returning dummy coordinates. C++ counterpart of
 * modules/vision/infra/harvest/mock/mock_harvest_detector.py.
 */

#pragma once

#include "farm_robot/modules/vision/harvest_detector.h"

namespace farm_robot {

class MockHarvestDetector : public IHarvestDetectorModule {
public:
    void connect() override;
    void disconnect() override;
    void loadModel(const std::string& modelPath) override;
    void disposeModel() override;
    std::optional<std::array<double, 3>> detect() override;

private:
    bool isConnected_{false};
};

}  // namespace farm_robot
