/**
 * @file communicator.cc
 * @brief Communicator implementation.
 *
 * C++ counterpart of controllers/communicator.py.
 */

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "farm_robot/application/communicator.h"
#include "farm_robot/cores/shared_state.h"

namespace farm_robot {

Communicator::Communicator(SharedState* sharedState) : sharedState_(sharedState) {
    std::cout << "[Communicator] Communicator initialized." << std::endl;
}

void Communicator::sendCurrentBox() {
    int boxId = sharedState_->currentBoxId();
    if (boxId >= 0) {
        std::cout << "[Communicator] Sending message to update current box: "
                  << boxId
                  << ", and tagged last box as completed" << std::endl;
    } else {
        std::cout << "[Communicator] No box found when trying to update current box." << std::endl;
    }
}

void Communicator::sendChargingSignal() {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "[Communicator] Sending message to charge" << std::endl;
}

void Communicator::sendFinishBoxSignal() {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "[Communicator] Sending message to finish box "
              << sharedState_->currentBoxId() << std::endl;
}

bool Communicator::waitForScaffoldRotation() {
    std::cout << "[Communicator] Waiting for scaffold rotation..." << std::endl;
    for (int waited = 0; waited < 5; ++waited) {
        if (std::rand() % 100 == 0) {
            std::cout << "[Communicator] Terminating wait for box rotation." << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "[Communicator] Box " << sharedState_->currentBoxId()
              << " rotation complete. Start moving to the first target." << std::endl;
    return true;
}

}  // namespace farm_robot
