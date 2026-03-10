/**
 * @file mock_checkpoint_sensor.cc
 * @brief Mock checkpoint sensor implementation.
 *
 * Simulated checkpoint detection. C++ counterpart of
 * modules/checkpoint_sensor/infra/mock/mock_checkpoint_sensor.py.
 */

#include <cstdlib>
#include <iostream>

#include "farm_robot/modules/checkpoint_sensor/infra/mock/mock_checkpoint_sensor.h"

namespace farm_robot {

void MockCheckpointSensor::connect() {
    std::cout << "[MockCheckpointSensorModule] Initializing checkpoint sensor..." << std::endl;
    std::cout << "[MockCheckpointSensorModule] Connecting to checkpoint sensor..." << std::endl;
}

bool MockCheckpointSensor::detect() {
    std::cout << "[MockCheckpointSensorModule] Detecting checkpoint..." << std::endl;
    return std::rand() % 10 == 0;
}

}  // namespace farm_robot
