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
    //! Constructs shared state with default values (running, charging state, -1 ids).
    SharedState();

    // System state
    //! Returns whether the system is considered running.
    bool isRunning() const;
    //! Sets the running flag.
    void setIsRunning(bool value);
    //! Returns current task name (e.g. planting, harvesting).
    std::string currentTask() const;
    //! Sets the current task name.
    void setCurrentTask(const std::string& task);
    //! Returns current robot state string.
    std::string robotState() const;
    //! Sets the robot state string.
    void setRobotState(const std::string& state);

    // Requested event (thread-safe; for checkpoint/other threads to request FSM transition)
    //! Sets a requested event name for the FSM to process.
    void setRequestedEvent(const std::string& eventName);
    //! Returns and clears the requested event if any.
    std::optional<std::string> getAndClearRequestedEvent();

    // Position
    //! Returns detected AprilTag id or -1.
    int detectedApriltagId() const;
    //! Sets the detected AprilTag id.
    void setDetectedApriltagId(int id);
    //! Returns current box id or -1.
    int currentBoxId() const;
    //! Sets the current box id.
    void setCurrentBoxId(int id);
    //! Returns current index within the box.
    int currentInBoxIndex() const;
    //! Sets the current in-box index.
    void setCurrentInBoxIndex(int index);

    // Target coordinates (x, y, z) in mm
    //! Returns target coordinates for arm if set.
    std::optional<std::array<double, 3>> targetCoordinates() const;
    //! Sets target coordinates for the robot arm.
    void setTargetCoordinates(const std::array<double, 3>& coords);
    //! Clears target coordinates.
    void clearTargetCoordinates();

    // Seedling index
    //! Returns seedling row and column index.
    std::pair<int, int> seedlingIndex() const;
    //! Sets seedling row and column index.
    void setSeedlingIndex(int row, int col);

    // Logging
    //! Returns and clears the next log message if any.
    std::optional<std::string> getLog();
    //! Adds a log message.
    void addLog(const std::string& message);

    // Operations
    //! Updates the current task name.
    void updateTask(const std::string& taskName);
    //! Records stop at new box (sets current box id).
    void stopAtNewBox(int boxId);
    //! Placeholder for resuming vehicle (no-op here).
    void resumeVehicle();
    //! Placeholder for starting a robot arm task (no-op here).
    void startRobotArmTask(const std::array<double, 3>& coordinates,
                           const std::string& taskType = "");
    //! Clears current box id.
    void clearCurrentBox();
    //! Clears detected AprilTag id.
    void clearApriltag();
    //! Stops the system (sets isRunning to false).
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
