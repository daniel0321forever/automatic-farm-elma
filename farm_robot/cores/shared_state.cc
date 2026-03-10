/**
 * @file shared_state.cc
 * @brief Thread-safe shared state implementation.
 *
 * C++ counterpart of cores/shared_state.py.
 */

#include "farm_robot/cores/config.h"
#include "farm_robot/cores/shared_state.h"

namespace farm_robot {

SharedState::SharedState()
    : isRunning_(true),
      currentTask_(),
      robotState_(Config::ROBOT_STATE_CHARGING),
      detectedApriltagId_(-1),
      currentBoxId_(-1),
      currentInBoxIndex_(-1),
      seedlingIndex_(0, 0) {}

bool SharedState::isRunning() const {
    std::lock_guard<std::mutex> guard(lock_);
    return isRunning_;
}

void SharedState::setIsRunning(bool value) {
    std::lock_guard<std::mutex> guard(lock_);
    isRunning_ = value;
}

std::string SharedState::currentTask() const {
    std::lock_guard<std::mutex> guard(lock_);
    return currentTask_;
}

void SharedState::setCurrentTask(const std::string& task) {
    std::lock_guard<std::mutex> guard(lock_);
    currentTask_ = task;
}

std::string SharedState::robotState() const {
    std::lock_guard<std::mutex> guard(lock_);
    return robotState_;
}

void SharedState::setRobotState(const std::string& state) {
    std::lock_guard<std::mutex> guard(lock_);
    robotState_ = state;
}

void SharedState::setRequestedEvent(const std::string& eventName) {
    std::lock_guard<std::mutex> guard(lock_);
    requestedEvent_ = eventName;
}

std::optional<std::string> SharedState::getAndClearRequestedEvent() {
    std::lock_guard<std::mutex> guard(lock_);
    auto ev = requestedEvent_;
    requestedEvent_.reset();
    return ev;
}

int SharedState::detectedApriltagId() const {
    std::lock_guard<std::mutex> guard(lock_);
    return detectedApriltagId_;
}

void SharedState::setDetectedApriltagId(int id) {
    std::lock_guard<std::mutex> guard(lock_);
    detectedApriltagId_ = id;
}

int SharedState::currentBoxId() const {
    std::lock_guard<std::mutex> guard(lock_);
    return currentBoxId_;
}

void SharedState::setCurrentBoxId(int id) {
    std::lock_guard<std::mutex> guard(lock_);
    currentBoxId_ = id;
}

int SharedState::currentInBoxIndex() const {
    std::lock_guard<std::mutex> guard(lock_);
    return currentInBoxIndex_;
}

void SharedState::setCurrentInBoxIndex(int index) {
    std::lock_guard<std::mutex> guard(lock_);
    currentInBoxIndex_ = index;
}

std::optional<std::array<double, 3>> SharedState::targetCoordinates() const {
    std::lock_guard<std::mutex> guard(lock_);
    return targetCoordinates_;
}

void SharedState::setTargetCoordinates(const std::array<double, 3>& coords) {
    std::lock_guard<std::mutex> guard(lock_);
    targetCoordinates_ = coords;
}

void SharedState::clearTargetCoordinates() {
    std::lock_guard<std::mutex> guard(lock_);
    targetCoordinates_.reset();
}

std::pair<int, int> SharedState::seedlingIndex() const {
    std::lock_guard<std::mutex> guard(lock_);
    return seedlingIndex_;
}

void SharedState::setSeedlingIndex(int row, int col) {
    std::lock_guard<std::mutex> guard(lock_);
    seedlingIndex_ = {row, col};
}

std::optional<std::string> SharedState::getLog() {
    std::lock_guard<std::mutex> guard(lock_);
    auto msg = logMessage_;
    logMessage_.reset();
    return msg;
}

void SharedState::addLog(const std::string& message) {
    std::lock_guard<std::mutex> guard(lock_);
    logMessage_ = message;
}

void SharedState::updateTask(const std::string& taskName) {
    setCurrentTask(taskName);
}

void SharedState::stopAtNewBox(int boxId) {
    setCurrentBoxId(boxId);
}

void SharedState::resumeVehicle() {}

void SharedState::startRobotArmTask(const std::array<double, 3>& /*coordinates*/,
                                    const std::string& /*taskType*/) {}

void SharedState::clearCurrentBox() {
    setCurrentBoxId(-1);
}

void SharedState::clearApriltag() {
    setDetectedApriltagId(-1);
}

void SharedState::stopSystem() {
    setIsRunning(false);
}

}  // namespace farm_robot
