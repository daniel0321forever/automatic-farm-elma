/**
 * @file enums.h
 * @brief Enumerations for vehicle control, AprilTag types, and vision tasks.
 *
 * Defines VehicleControlCommand for AGV motion, ApriltagType for tag classification,
 * and VisionTask for planting/harvesting/detecting modes.
 * C++ counterpart of cores/enums.py.
 */

#pragma once

namespace farm_robot {

//! Vehicle motion commands for AGV control.
enum class VehicleControlCommand {
    STOP = 0,
    TURN_RIGHT = 1,
    TURN_LEFT = 2,
    MOVE_FORWARD = 3,
    MOVE_BACKWARD = 4,
    TURN_RIGHT_AND_MOVE_FORWARD = 5,
    TURN_LEFT_AND_MOVE_FORWARD = 6,
    TURN_RIGHT_AND_MOVE_BACKWARD = 7,
    TURN_LEFT_AND_MOVE_BACKWARD = 8,
    ROTATE = 9
};

//! AprilTag classification (e.g. box).
enum class ApriltagType {
    BOX
};

//! Vision task mode (planting, harvesting, or detecting AprilTag).
enum class VisionTask {
    PLANTING,
    HARVESTING,
    DETECTING_APRILTAG
};

}  // namespace farm_robot
