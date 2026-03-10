/**
 * @file elintech_checkpoint_sensor.cc
 * @brief ElintTech checkpoint sensor (uses vehicle read_status, on_node).
 *
 * C++ counterpart of modules/checkpoint_sensor/infra/elintech/elintech_checkpoint_sensor.py.
 */

#include <chrono>
#include <iostream>

#include "farm_robot/modules/checkpoint_sensor/infra/elintech/elintech_checkpoint_sensor.h"
#include "farm_robot/modules/vehicle/infra/elintech/elintech_vehicle_module.h"

namespace farm_robot {

ElintTechCheckpointSensor::ElintTechCheckpointSensor(
    std::shared_ptr<IVehicleModule> vehicleModule)
    : vehicleModule_(std::move(vehicleModule)),
      lastNodeDetectedTimestamp_(std::chrono::steady_clock::time_point{}) {}

void ElintTechCheckpointSensor::connect() {
    ElintTechVehicleModule* v =
        dynamic_cast<ElintTechVehicleModule*>(vehicleModule_.get());
    if (v)
        v->connect();
    else
        std::cerr << "[ElintTechCheckpointSensor] Vehicle is not ElintTech, "
                     "cannot connect."
                  << std::endl;
}

bool ElintTechCheckpointSensor::detect() {
    ElintTechVehicleModule* v =
        dynamic_cast<ElintTechVehicleModule*>(vehicleModule_.get());
    if (!v)
        return false;

    nlohmann::json status = v->readStatus();
    if (status.is_null() || status.empty())
        return false;

    auto now = std::chrono::steady_clock::now();
    const auto cooldown = std::chrono::seconds(2);
    bool onNode = status.find("on_node") != status.end() &&
                  status["on_node"].get<bool>();

    if (onNode) {
        lastNodeDetectedTimestamp_ = now;
        if (isDetectingAvailable_) {
            isDetectingAvailable_ = false;
            return true;
        }
        return false;
    }

    if (now - lastNodeDetectedTimestamp_ > cooldown && !isDetectingAvailable_) {
        isDetectingAvailable_ = true;
    }
    return false;
}

}  // namespace farm_robot
