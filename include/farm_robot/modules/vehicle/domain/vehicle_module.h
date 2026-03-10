/**
 * @file vehicle_module.h
 * @brief Interface for vehicle/AGV control (ElintTech serial).
 *
 * Vehicle module interface: connect, control commands, charging status.
 * Implementations: mock, elintech (serial). C++ counterpart of
 * modules/vehicle/domain/vehicle_module.py.
 */

#pragma once

#include "farm_robot/cores/enums.h"

namespace farm_robot {

class IVehicleModule {
public:
    virtual ~IVehicleModule() = default;

    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isChargingPointReached() = 0;
    virtual void charge() = 0;
    virtual void setSpeed(int speed) = 0;
    virtual VehicleControlCommand currentMotionStatus() = 0;

    /**
     * Send control command to vehicle.
     */
    virtual void control(VehicleControlCommand command) = 0;
};

}  // namespace farm_robot
