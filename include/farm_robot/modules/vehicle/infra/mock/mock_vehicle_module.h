/**
 * @file mock_vehicle_module.h
 * @brief Mock vehicle module for testing.
 *
 * Simulated vehicle that accepts commands without hardware. C++ counterpart
 * of modules/vehicle/infra/mock/mock_vehicle_module.py.
 */

#pragma once

#include "farm_robot/modules/vehicle/domain/vehicle_module.h"

namespace farm_robot {

class MockVehicleModule : public IVehicleModule {
public:
    void connect() override;
    void disconnect() override;
    bool isChargingPointReached() override;
    void charge() override;
    void setSpeed(int speed) override;
    VehicleControlCommand currentMotionStatus() override;
    void control(VehicleControlCommand command) override;
private:
    VehicleControlCommand cachedCommand_{VehicleControlCommand::STOP};
};

}  // namespace farm_robot
