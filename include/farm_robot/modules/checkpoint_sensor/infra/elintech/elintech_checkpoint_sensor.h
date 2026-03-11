/**
 * @file elintech_checkpoint_sensor.h
 * @brief ElintTech checkpoint sensor (uses vehicle status).
 *
 * Uses ElintTech vehicle on_node for checkpoint detection. C++ counterpart of
 * modules/checkpoint_sensor/infra/elintech/elintech_checkpoint_sensor.py.
 */

#pragma once

#include "farm_robot/modules/checkpoint_sensor/domain/checkpoint_sensor.h"
#include "farm_robot/modules/vehicle/domain/vehicle_module.h"
#include <chrono>
#include <memory>

namespace farm_robot {

class ElintTechVehicleModule;

class ElintTechCheckpointSensor : public ICheckpointSensorModule {
public:
    //! Constructs sensor that uses the given vehicle module for node/checkpoint status.
    explicit ElintTechCheckpointSensor(std::shared_ptr<IVehicleModule> vehicleModule);
    void connect() override;
    //! Returns true when vehicle reports checkpoint (on_node); uses throttling.
    bool detect() override;
private:
    std::shared_ptr<IVehicleModule> vehicleModule_;
    std::chrono::steady_clock::time_point lastNodeDetectedTimestamp_;
    bool isDetectingAvailable_{true};
};

}  // namespace farm_robot
