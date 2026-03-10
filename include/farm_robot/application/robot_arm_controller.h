/**
 * @file robot_arm_controller.h
 * @brief Robot arm controller for planting and harvesting.
 *
 * C++ counterpart of controllers/robot_arm.py.
 */

#pragma once

#include <array>
#include <memory>

namespace farm_robot {

class IRobotArmModule;

class RobotArmController {
public:
    explicit RobotArmController(std::shared_ptr<IRobotArmModule> robotArmModule);

    bool plant(const std::array<double, 3>& coordinates);
    bool harvest(const std::array<double, 3>& coordinates);

private:
    static constexpr std::array<double, 3> DEFAULT_PICK_POSITION{-60.0, -482.0, 150.0};

    std::shared_ptr<IRobotArmModule> robotArmModule_;
};

}  // namespace farm_robot
