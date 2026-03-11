/**
 * @file communicator.h
 * @brief Communicator for sending messages to greenhouse control (scaffold, charging, box).
 *
 * C++ counterpart of controllers/communicator.py.
 */

#pragma once

namespace farm_robot {

class SharedState;

class Communicator {
public:
    //! Constructs communicator with shared state (for box id and messaging).
    explicit Communicator(SharedState* sharedState);

    //! Sends current box id to greenhouse control and tags last box completed.
    void sendCurrentBox();
    //! Sends charging signal to greenhouse (blocks briefly for simulation).
    void sendChargingSignal();
    //! Sends finish-box signal for the current box.
    void sendFinishBoxSignal();
    //! Waits for scaffold rotation; returns true when complete (or simulated timeout).
    bool waitForScaffoldRotation();

private:
    SharedState* sharedState_;
};

}  // namespace farm_robot
