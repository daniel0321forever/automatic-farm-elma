/**
 * @file shared_state.h
 * @brief Thread-safe shared state for coordination between controllers and threads.
 *
 * Holds system-level state (is_running, current_task, robot_state), position data
 * (detected AprilTag, box ID, target coordinates), and log messages. Uses std::mutex
 * for thread safety. C++ counterpart of cores/shared_state.py.
 */

#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <array>

namespace farm_robot {

class SharedState {
public:
    SharedState();

    // System state
    bool isRunning() const;
    void setIsRunning(bool value);
    std::string currentTask() const;
    void setCurrentTask(const std::string& task);
    std::string robotState() const;
    void setRobotState(const std::string& state);

    // Requested event (thread-safe; for checkpoint/other threads to request FSM transition)
    void setRequestedEvent(const std::string& eventName);
    std::optional<std::string> getAndClearRequestedEvent();

    // Position
    int detectedApriltagId() const;
    void setDetectedApriltagId(int id);
    int currentBoxId() const;
    void setCurrentBoxId(int id);
    int currentInBoxIndex() const;
    void setCurrentInBoxIndex(int index);

    // Target coordinates (x, y, z) in mm
    std::optional<std::array<double, 3>> targetCoordinates() const;
    void setTargetCoordinates(const std::array<double, 3>& coords);
    void clearTargetCoordinates();

    // Seedling index
    std::pair<int, int> seedlingIndex() const;
    void setSeedlingIndex(int row, int col);

    // Logging
    std::optional<std::string> getLog();
    void addLog(const std::string& message);

    // Operations
    void updateTask(const std::string& taskName);
    void stopAtNewBox(int boxId);
    void resumeVehicle();
    void startRobotArmTask(const std::array<double, 3>& coordinates,
                           const std::string& taskType = "");
    void clearCurrentBox();
    void clearApriltag();
    void stopSystem();

private:
    mutable std::mutex lock_;
    bool isRunning_;
    std::string currentTask_;
    std::string robotState_;
    int detectedApriltagId_;
    int currentBoxId_;
    int currentInBoxIndex_;
    std::optional<std::array<double, 3>> targetCoordinates_;
    std::pair<int, int> seedlingIndex_;
    std::optional<std::string> logMessage_;
    std::optional<std::string> requestedEvent_;
};

}  // namespace farm_robot
