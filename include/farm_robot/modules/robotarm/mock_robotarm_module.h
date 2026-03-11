/**
 * @file mock_robotarm_module.h
 * @brief Mock robot arm module for testing.
 *
 * Simulated robot arm that accepts pick/plant and harvest commands.
 * C++ counterpart of modules/robotarm/infra/mock/mock_robotarm_module.py.
 */

#pragma once

#include "farm_robot/modules/robotarm/robotarm_module.h"

namespace farm_robot {

class MockRobotArmModule : public IRobotArmModule {
public:
    void initialize() override;
    void moveToStandby(const std::string& positionName, int speed) override;
    void close() override;
    //! Logs and returns true (mock success).
    bool pickAndPlant(const std::array<double, 3>& pickPos,
                      const std::array<double, 3>& plantPos) override;
    //! Logs and returns true (mock success).
    bool goToHarvest(const std::array<double, 3>& harvestPos) override;
};

}  // namespace farm_robot
