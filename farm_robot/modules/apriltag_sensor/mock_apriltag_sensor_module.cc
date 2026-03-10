/**
 * @file mock_apriltag_sensor_module.cc
 * @brief Mock AprilTag sensor implementation.
 *
 * C++ counterpart of modules/apriltag_sensor_module/infra/mock/mock_apriltag_sensor_module.py.
 */

#include <cstdlib>
#include <iostream>

#include "farm_robot/cores/enums.h"
#include "farm_robot/models/apriltag_detection.h"
#include "farm_robot/modules/apriltag_sensor/apriltag_sensor_module.h"
#include "farm_robot/modules/apriltag_sensor/mock_apriltag_sensor_module.h"

namespace farm_robot {

namespace {

constexpr int BOX_TAGS[] = {1100, 1101, 1102, 1103};
constexpr size_t NUM_BOX_TAGS = sizeof(BOX_TAGS) / sizeof(BOX_TAGS[0]);

}  // namespace

void MockApriltagSensorModule::connect() {
    std::cout << "[MockApriltagSensorModule] Connecting to apriltag sensor... -> No need to connect" << std::endl;
}

void MockApriltagSensorModule::disconnect() {
    std::cout << "[MockApriltagSensorModule] Disconnecting from apriltag sensor... -> No need to disconnect" << std::endl;
}

std::optional<ApriltagDetection> MockApriltagSensorModule::detectBox() {
    std::cout << "[MockApriltagSensorModule] Detecting box..." << std::endl;
    if (std::rand() % 100 < 40)
        return std::nullopt;
    double validity = static_cast<double>(std::rand()) / RAND_MAX;
    int tagId = BOX_TAGS[static_cast<size_t>(std::rand()) % NUM_BOX_TAGS];
    return ApriltagDetection(tagId, ApriltagType::BOX, validity > 0.5, validity);
}

}  // namespace farm_robot
