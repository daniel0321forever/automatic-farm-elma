/**
 * @file robot_arm.h
 * @brief Robot arm controller for planting and harvesting.
 *
 * Invokes robot arm for plant (pick_and_plant) and harvest (go_to_harvest)
 * operations using target coordinates from SharedState. C++ counterpart of
 * controllers/robot_arm.py.
 */

#pragma once

#include <memory>
#include <array>

namespace farm_robot {

class SharedState;
class IRobotArmModule;

class RobotArmController {
public:
    RobotArmController(SharedState* sharedState,
                       std::shared_ptr<IRobotArmModule> robotArmModule);

    bool plant(const std::array<double, 3>& coordinates);
    bool harvest(const std::array<double, 3>& coordinates);

private:
    SharedState* sharedState_;
    std::shared_ptr<IRobotArmModule> robotArmModule_;
};

}  // namespace farm_robot
