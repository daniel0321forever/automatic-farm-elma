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

    //! Opens connection to the vehicle (e.g. serial port).
    virtual void connect() = 0;
    //! Closes connection to the vehicle.
    virtual void disconnect() = 0;
    //! Returns true if the vehicle has reached the charging point.
    virtual bool isChargingPointReached() = 0;
    //! Starts or continues charging.
    virtual void charge() = 0;
    //! Sets the vehicle speed (e.g. RPM).
    virtual void setSpeed(int speed) = 0;
    //! Returns the last commanded motion status.
    virtual VehicleControlCommand currentMotionStatus() = 0;

    //! Sends a control command to the vehicle.
    virtual void control(VehicleControlCommand command) = 0;
};

}  // namespace farm_robot
