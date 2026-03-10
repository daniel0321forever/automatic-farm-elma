/**
 * @file checkpoint_sensor.h
 * @brief Interface for checkpoint detection on the route.
 *
 * Checkpoint sensor interface: connect, detect. Implementations: mock,
 * elintech (uses vehicle status). C++ counterpart of
 * modules/checkpoint_sensor/domain/checkpoint_sensor.py.
 */

#pragma once

namespace farm_robot {

class ICheckpointSensorModule {
public:
    virtual ~ICheckpointSensorModule() = default;

    virtual void connect() = 0;

    /**
     * Detect if checkpoint is reached.
     * @return true if checkpoint reached, false otherwise
     */
    virtual bool detect() = 0;
};

}  // namespace farm_robot
