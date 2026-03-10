/**
 * @file mock_apriltag_sensor_module.h
 * @brief Mock AprilTag sensor for testing.
 *
 * Simulated AprilTag detection. C++ counterpart of
 * modules/apriltag_sensor_module/infra/mock/mock_apriltag_sensor_module.py.
 */

#pragma once

#include "farm_robot/modules/apriltag_sensor/apriltag_sensor_module.h"

namespace farm_robot {

class MockApriltagSensorModule : public IApriltagSensorModule {
public:
    void connect() override;
    void disconnect() override;
    std::optional<ApriltagDetection> detectBox() override;
};

}  // namespace farm_robot
