/**
 * @file vehicle_controller.cc
 * @brief Vehicle controller implementation.
 *
 * C++ counterpart of controllers/vehicle_control.py.
 */

#include <iostream>
#include <thread>
#include <vector>

#include "farm_robot/application/vehicle_controller.h"
#include "farm_robot/cores/config.h"
#include "farm_robot/cores/enums.h"
#include "farm_robot/cores/shared_state.h"
#include "farm_robot/modules/vehicle/domain/vehicle_module.h"

namespace farm_robot {

namespace {

std::vector<VehicleControlCommand> buildRoute() {
    return {
        VehicleControlCommand::MOVE_FORWARD,
        VehicleControlCommand::TURN_LEFT_AND_MOVE_FORWARD,
        VehicleControlCommand::MOVE_BACKWARD,
        VehicleControlCommand::TURN_RIGHT_AND_MOVE_FORWARD,
        VehicleControlCommand::TURN_LEFT_AND_MOVE_FORWARD,
        VehicleControlCommand::MOVE_FORWARD,
        VehicleControlCommand::TURN_LEFT_AND_MOVE_FORWARD,
        VehicleControlCommand::MOVE_BACKWARD,
        VehicleControlCommand::TURN_RIGHT_AND_MOVE_FORWARD,
        VehicleControlCommand::MOVE_BACKWARD,
    };
}

}  // namespace

VehicleController::VehicleController(SharedState* sharedState,
                                     std::shared_ptr<IVehicleModule> vehicleModule)
    : sharedState_(sharedState),
      vehicleModule_(std::move(vehicleModule)) {
    std::cout << "[Vehicle] Initializing VehicleController..." << std::endl;
}

VehicleControlCommand VehicleController::currentCommand() const {
    auto route = buildRoute();
    if (currentCheckpoint_ >= route.size()) {
        return VehicleControlCommand::STOP;
    }
    return route[currentCheckpoint_];
}

void VehicleController::startCycle() {
    std::cout << "[Vehicle] Starting cycle..." << std::endl;
    sharedState_->setRobotState(Config::ROBOT_STATE_CRUISING);
    auto route = buildRoute();
    vehicleModule_->control(route[0]);
}

void VehicleController::toNextCheckpoint() {
    std::cout << "[Vehicle] Cruising to next checking point..." << std::endl;
    currentCheckpoint_++;
    auto route = buildRoute();

    if (currentCheckpoint_ >= route.size()) {
        if (emitCallback_) {
            emitCallback_("going_to_charging_station");
        } else {
            sharedState_->setRobotState(Config::ROBOT_STATE_GOING_TO_CHARGING_STATION);
        }
        std::cout << "[Vehicle] Cruising to next checking point (last "
                     "segment, going to charging)."
                  << std::endl;
        return;
    }
    if (currentCheckpoint_ == route.size() - 1) {
        vehicleModule_->setSpeed(10);
        vehicleModule_->control(currentCommand());
        if (emitCallback_) {
            emitCallback_("going_to_charging_station");
        } else {
            sharedState_->setRobotState(Config::ROBOT_STATE_GOING_TO_CHARGING_STATION);
        }
        std::cout << "[Vehicle] Cruising to next checking point (last "
                     "segment, going to charging)."
                  << std::endl;
        return;
    }
    vehicleModule_->control(currentCommand());
    if (emitCallback_) {
        emitCallback_("cruising");
    } else {
        sharedState_->setRobotState(Config::ROBOT_STATE_CRUISING);
    }
    std::cout << "[Vehicle] Cruising to next checking point (segment "
              << currentCheckpoint_ << ")." << std::endl;
}

void VehicleController::stopAtNewBox(int boxId) {
    std::cout << "[Vehicle] Going to new box " << boxId << " and stopping."
              << std::endl;
    vehicleModule_->control(VehicleControlCommand::STOP);
    std::cout << "[Vehicle] New box " << boxId << " reached and stopped."
              << std::endl;
}

void VehicleController::goToTarget(int index) {
    std::cout << "[Vehicle] Going to target num " << index << " in box."
              << std::endl;
    vehicleModule_->control(vehicleModule_->currentMotionStatus());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    vehicleModule_->control(VehicleControlCommand::STOP);
    std::cout << "[Vehicle] Target number " << index << " in box reached."
              << std::endl;
}

void VehicleController::resume() {
    std::cout << "[Vehicle] Resuming..." << std::endl;
    vehicleModule_->control(vehicleModule_->currentMotionStatus());
    if (emitCallback_) {
        emitCallback_("cruising");
    } else {
        sharedState_->setRobotState(Config::ROBOT_STATE_CRUISING);
    }
}

void VehicleController::setEmitCallback(std::function<void(const std::string&)> cb) {
    emitCallback_ = std::move(cb);
}

}  // namespace farm_robot
