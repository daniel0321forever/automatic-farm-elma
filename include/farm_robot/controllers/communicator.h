/**
 * @file communicator.h
 * @brief Greenhouse communication (box update, charging, scaffold rotation).
 *
 * Sends signals to greenhouse control: box update, charging, finish box.
 * Waits for scaffold rotation signal. C++ counterpart of
 * controllers/communicator.py.
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
    void waitForScaffoldRotation();

private:
    SharedState* sharedState_;
};

}  // namespace farm_robot
