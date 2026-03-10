/**
 * @file apriltag_sensor_module.h
 * @brief Interface for AprilTag/box detection.
 *
 * AprilTag sensor interface: connect, disconnect, detect_box. Returns
 * ApriltagDetection or empty. Implementations: mock, camera. C++ counterpart
 * of modules/apriltag_sensor_module/domain/apriltag_sensor_module.py.
 */

#pragma once

#include "farm_robot/models/apriltag_detection.h"
#include <memory>
#include <optional>

namespace farm_robot {

class IApriltagSensorModule {
public:
    virtual ~IApriltagSensorModule() = default;

    virtual void connect() = 0;
    virtual void disconnect() = 0;

    /**
     * Detect box AprilTag on the ground.
     * @return ApriltagDetection if detected, empty otherwise
     */
    virtual std::optional<ApriltagDetection> detectBox() = 0;
};

}  // namespace farm_robot
