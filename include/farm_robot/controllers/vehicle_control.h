/**
 * @file vehicle_control.h
 * @brief Vehicle controller for AGV cruising, checkpoints, and box handling.
 *
 * Drives the vehicle along fixed routes, handles checkpoints, charging,
 * stop-at-box and resume. Monitors charging point. C++ counterpart of
 * controllers/vehicle_control.py.
 */

#pragma once

#include <memory>

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
    void goToChargingStation();
    void charge();
    void resume();

private:
    SharedState* sharedState_;
    std::shared_ptr<IVehicleModule> vehicleModule_;
};

}  // namespace farm_robot
