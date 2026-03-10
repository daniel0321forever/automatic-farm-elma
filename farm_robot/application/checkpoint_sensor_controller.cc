/**
 * @file checkpoint_sensor_controller.cc
 * @brief Checkpoint sensor controller implementation (runs on separate thread).
 *
 * C++ counterpart of controllers/checkpoint_sensor_controller.py.
 */

#include <chrono>
#include <iostream>

#include "farm_robot/application/checkpoint_sensor_controller.h"
#include "farm_robot/cores/config.h"
#include "farm_robot/cores/shared_state.h"
#include "farm_robot/modules/checkpoint_sensor/domain/checkpoint_sensor.h"

namespace farm_robot {

CheckpointSensorController::CheckpointSensorController(
    SharedState* sharedState,
    std::shared_ptr<ICheckpointSensorModule> checkpointSensor)
    : sharedState_(sharedState),
      checkpointSensor_(std::move(checkpointSensor)) {}

CheckpointSensorController::~CheckpointSensorController() {
    stop();
    join();
}

void CheckpointSensorController::start() {
    checkpointSensor_->connect();
    running_ = true;
    thread_ = std::thread(&CheckpointSensorController::run, this);
}

void CheckpointSensorController::stop() {
    running_ = false;
}

void CheckpointSensorController::join() {
    if (thread_.joinable()) {
        thread_.join();
    }
}

void CheckpointSensorController::run() {
    while (sharedState_->isRunning() && running_) {
        if (sharedState_->robotState() == Config::ROBOT_STATE_CRUISING &&
            checkpointSensor_->detect()) {
            std::cout << "[CheckpointSensorController] Checkpoint detected." << std::endl;
            sharedState_->setRequestedEvent("reach_checking_point");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}


}  // namespace farm_robot
