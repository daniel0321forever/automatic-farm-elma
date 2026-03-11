/**
 * @file checkpoint_sensor_controller.h
 * @brief Thread that detects checkpoints and updates robot state.
 *
 * CheckpointSensorController runs in a thread, detects checkpoint arrival,
 * and sets ROBOT_STATE_REACH_CHECKING_POINT in SharedState.
 * C++ counterpart of controllers/checkpoint_sensor_controller.py.
 */

#pragma once

#include <atomic>
#include <memory>
#include <thread>

namespace farm_robot {

class SharedState;
class ICheckpointSensorModule;

class CheckpointSensorController {
public:
    //! Constructs controller with shared state and checkpoint sensor module.
    CheckpointSensorController(SharedState* sharedState,
                               std::shared_ptr<ICheckpointSensorModule> checkpointSensor);

    ~CheckpointSensorController();
    //! Starts the sensor thread (calls connect() and run() in a thread).
    void start();
    //! Signals the thread to stop.
    void stop();
    //! Joins the sensor thread if it is joinable.
    void join();

private:
    //! Loop: while running, if cruising and sensor detects checkpoint, sets requested event.
    void run();

    SharedState* sharedState_;
    std::shared_ptr<ICheckpointSensorModule> checkpointSensor_;
    std::thread thread_;
    std::atomic<bool> running_{false};
};

}  // namespace farm_robot
