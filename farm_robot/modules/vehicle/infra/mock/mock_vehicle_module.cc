/**
 * @file mock_vehicle_module.cc
 * @brief Mock vehicle module implementation.
 *
 * Simulated vehicle accepting commands without hardware. C++ counterpart of
 * modules/vehicle/infra/mock/mock_vehicle_module.py.
 */

#include <iostream>

#include "farm_robot/modules/vehicle/infra/mock/mock_vehicle_module.h"
#include "farm_robot/cores/enums.h"

namespace farm_robot {

namespace {

const char* commandToMessage(VehicleControlCommand cmd) {
    switch (cmd) {
        case VehicleControlCommand::STOP:
            return "Stopping...";
        case VehicleControlCommand::TURN_RIGHT:
            return "Turning right...";
        case VehicleControlCommand::TURN_LEFT:
            return "Turning left...";
        case VehicleControlCommand::MOVE_FORWARD:
            return "Moving forward...";
        case VehicleControlCommand::MOVE_BACKWARD:
            return "Moving backward...";
        case VehicleControlCommand::TURN_RIGHT_AND_MOVE_FORWARD:
            return "Turning right...";
        case VehicleControlCommand::TURN_LEFT_AND_MOVE_FORWARD:
            return "Turning left...";
        case VehicleControlCommand::TURN_RIGHT_AND_MOVE_BACKWARD:
            return "Turning right...";
        case VehicleControlCommand::TURN_LEFT_AND_MOVE_BACKWARD:
            return "Turning left...";
        case VehicleControlCommand::ROTATE:
            return "Rotating...";
        default:
            return "Stopping...";
    }
}

}  // namespace

void MockVehicleModule::connect() {
    std::cout << "[MockVehicleModule] Initializing MockVehicleModule..." << std::endl;
    std::cout << "[MockVehicleModule] Connecting to vehicle..." << std::endl;
}

void MockVehicleModule::disconnect() {
    std::cout << "[MockVehicleModule] Disconnecting from vehicle..." << std::endl;
}

bool MockVehicleModule::isChargingPointReached() {
    return false;
}

void MockVehicleModule::charge() {
    std::cout << "[MockVehicleModule] Charging..." << std::endl;
}

void MockVehicleModule::setSpeed(int speed) {
    std::cout << "[MockVehicleModule] Setting speed to " << speed << "..." << std::endl;
}

VehicleControlCommand MockVehicleModule::currentMotionStatus() {
    return cachedCommand_;
}

void MockVehicleModule::control(VehicleControlCommand command) {
    cachedCommand_ = command;
    std::cout << "[MockVehicleModule] " << commandToMessage(command) << std::endl;
}

}  // namespace farm_robot
