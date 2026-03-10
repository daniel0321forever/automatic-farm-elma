/**
 * @file checkpoint_sensor_controller.h
 * @brief Thread that detects checkpoints and updates robot state.
 *
 * CheckpointSensorController runs in a thread, detects checkpoint arrival,
 * and sets ROBOT_STATE_REACH_CHECKING_POINT in SharedState. C++ counterpart
 * of controllers/checkpoint_sensor_controller.py.
 */

#pragma once

#include <memory>
#include <thread>
#include <atomic>

namespace farm_robot {

class SharedState;
class ICheckpointSensorModule;

class CheckpointSensorController {
public:
    CheckpointSensorController(SharedState* sharedState,
                               std::shared_ptr<ICheckpointSensorModule> checkpointSensor);

    ~CheckpointSensorController();
    void start();
    void stop();
    void join();

private:
    void run();

    SharedState* sharedState_;
    std::shared_ptr<ICheckpointSensorModule> checkpointSensor_;
    std::thread thread_;
    std::atomic<bool> running_;
};

}  // namespace farm_robot
