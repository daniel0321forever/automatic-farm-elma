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

    //! Connects to the checkpoint sensor (e.g. serial or mock).
    virtual void connect() = 0;

    //! Returns true if a checkpoint is detected, false otherwise.
    virtual bool detect() = 0;
};

}  // namespace farm_robot
