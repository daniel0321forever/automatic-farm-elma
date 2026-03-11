/**
 * @file mock_checkpoint_sensor.h
 * @brief Mock checkpoint sensor for testing.
 *
 * Simulated checkpoint detection. C++ counterpart of
 * modules/checkpoint_sensor/infra/mock/mock_checkpoint_sensor.py.
 */

#pragma once

#include "farm_robot/modules/checkpoint_sensor/domain/checkpoint_sensor.h"

namespace farm_robot {

class MockCheckpointSensor : public ICheckpointSensorModule {
public:
    //! No-op connect for mock.
    void connect() override;
    //! Returns true periodically to simulate checkpoint detection.
    bool detect() override;
};

}  // namespace farm_robot
