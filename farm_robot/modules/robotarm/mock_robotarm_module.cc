/**
 * @file mock_robotarm_module.cc
 * @brief Mock robot arm implementation.
 *
 * C++ counterpart of modules/robotarm/infra/mock/mock_robotarm_module.py.
 */

#include <iostream>

#include "farm_robot/modules/robotarm/mock_robotarm_module.h"
#include "farm_robot/modules/robotarm/robotarm_module.h"

namespace farm_robot {

void MockRobotArmModule::initialize() {
    std::cout << "[MockRobotArmModule] Initializing the robot arm..."
              << std::endl;
}

void MockRobotArmModule::moveToStandby(const std::string& positionName,
                                       int /*speed*/) {
    std::cout << "[MockRobotArmModule] Moving to the standby position: "
              << positionName << std::endl;
}

void MockRobotArmModule::close() {
    std::cout << "[MockRobotArmModule] Closing the connection..." << std::endl;
}

bool MockRobotArmModule::pickAndPlant(const std::array<double, 3>& pickPos,
                                      const std::array<double, 3>& plantPos) {
    std::cout << "[MockRobotArmModule] Picking and planting the seedling: ("
              << pickPos[0] << "," << pickPos[1] << "," << pickPos[2]
              << "), (" << plantPos[0] << "," << plantPos[1] << ","
              << plantPos[2] << ")" << std::endl;
    return true;
}

bool MockRobotArmModule::goToHarvest(
    const std::array<double, 3>& harvestPos) {
    std::cout << "[MockRobotArmModule] Going to the harvest position: ("
              << harvestPos[0] << "," << harvestPos[1] << "," << harvestPos[2]
              << ")" << std::endl;
    return true;
}

}  // namespace farm_robot
