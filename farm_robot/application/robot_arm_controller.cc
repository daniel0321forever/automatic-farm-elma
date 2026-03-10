/**
 * @file robot_arm_controller.cc
 * @brief Robot arm controller implementation.
 *
 * C++ counterpart of controllers/robot_arm.py.
 */

#include "farm_robot/application/robot_arm_controller.h"
#include "farm_robot/modules/robotarm/robotarm_module.h"

namespace farm_robot {

RobotArmController::RobotArmController(
    std::shared_ptr<IRobotArmModule> robotArmModule)
    : robotArmModule_(std::move(robotArmModule)) {}

bool RobotArmController::plant(const std::array<double, 3>& coordinates) {
    return robotArmModule_->pickAndPlant(DEFAULT_PICK_POSITION, coordinates);
}

bool RobotArmController::harvest(const std::array<double, 3>& coordinates) {
    return robotArmModule_->goToHarvest(coordinates);
}

}  // namespace farm_robot
