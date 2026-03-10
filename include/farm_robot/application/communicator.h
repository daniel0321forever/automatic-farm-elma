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
    explicit Communicator(SharedState* sharedState);

    void sendCurrentBox();
    void sendChargingSignal();
    void sendFinishBoxSignal();
    bool waitForScaffoldRotation();

private:
    SharedState* sharedState_;
};

}  // namespace farm_robot
