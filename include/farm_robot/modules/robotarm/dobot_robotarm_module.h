/**
 * @file dobot_robotarm_module.h
 * @brief Dobot robot arm module.
 *
 * Dobot arm control via API: pick/plant, harvest, standby positions.
 * C++ counterpart of modules/robotarm/infra/dobot/dobot_robotarm_module.py.
 */

#pragma once

#include "farm_robot/modules/robotarm/robotarm_module.h"

namespace farm_robot {

class DobotRobotArmModule : public IRobotArmModule {
public:
    void initialize() override;
    void moveToStandby(const std::string& positionName, int speed) override;
    void close() override;
    bool pickAndPlant(const std::array<double, 3>& pickPos,
                      const std::array<double, 3>& plantPos) override;
    bool goToHarvest(const std::array<double, 3>& harvestPos) override;
};

}  // namespace farm_robot
