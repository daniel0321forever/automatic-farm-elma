/**
 * @file vehicle_controller.h
 * @brief Vehicle controller for AGV cruising and checkpoints.
 *
 * Drives the vehicle along fixed routes, handles checkpoints, charging.
 * C++ counterpart of controllers/vehicle_control.py.
 */

#pragma once

#include <cstddef>
#include <functional>
#include <memory>

#include "farm_robot/cores/enums.h"

namespace farm_robot {

class SharedState;
class IVehicleModule;

class VehicleController {
public:
    //! Constructs controller with shared state and vehicle module.
    VehicleController(SharedState* sharedState,
                      std::shared_ptr<IVehicleModule> vehicleModule);

    //! Starts a cycle and sets vehicle to first route command.
    void startCycle();
    //! Advances to next checkpoint and updates vehicle command; may emit going_to_charging_station.
    void toNextCheckpoint();
    //! Stops the vehicle at a new box (e.g. for loading).
    void stopAtNewBox(int boxId);
    //! Drives to a target index within the current box.
    void goToTarget(int index);
    //! Resumes cruising after a stop.
    void resume();

    //! Sets callback used to emit state events (e.g. for FSM).
    void setEmitCallback(std::function<void(const std::string&)> cb);

private:
    //! Returns the current route command for the current checkpoint index.
    VehicleControlCommand currentCommand() const;

    SharedState* sharedState_;
    std::shared_ptr<IVehicleModule> vehicleModule_;
    size_t currentCheckpoint_{0};
    std::function<void(const std::string&)> emitCallback_;
};

}  // namespace farm_robot
