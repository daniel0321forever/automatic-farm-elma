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
    VehicleController(SharedState* sharedState,
                      std::shared_ptr<IVehicleModule> vehicleModule);

    void startCycle();
    void toNextCheckpoint();
    void stopAtNewBox(int boxId);
    void goToTarget(int index);
    void resume();

    void setEmitCallback(std::function<void(const std::string&)> cb);

private:
    VehicleControlCommand currentCommand() const;

    SharedState* sharedState_;
    std::shared_ptr<IVehicleModule> vehicleModule_;
    size_t currentCheckpoint_{0};
    std::function<void(const std::string&)> emitCallback_;
};

}  // namespace farm_robot
