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

    //! Connects to the AprilTag sensor (e.g. camera or mock).
    virtual void connect() = 0;
    //! Disconnects from the sensor.
    virtual void disconnect() = 0;

    //! Detects box AprilTag; returns detection if found, empty otherwise.
    virtual std::optional<ApriltagDetection> detectBox() = 0;
};

}  // namespace farm_robot
