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
    //! Constructs controller with the given robot arm module.
    explicit RobotArmController(std::shared_ptr<IRobotArmModule> robotArmModule);

    //! Performs pick-and-plant from default pick position to given coordinates.
    bool plant(const std::array<double, 3>& coordinates);
    //! Moves arm to harvest at the given coordinates.
    bool harvest(const std::array<double, 3>& coordinates);

private:
    //! Default (pick) position used for planting.
    static constexpr std::array<double, 3> DEFAULT_PICK_POSITION{-60.0, -482.0, 150.0};

    std::shared_ptr<IRobotArmModule> robotArmModule_;
};

}  // namespace farm_robot
