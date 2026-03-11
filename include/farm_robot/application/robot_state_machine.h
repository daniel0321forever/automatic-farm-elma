/**
 * @file robot_state_machine.h
 * @brief Elma StateMachine for farm robot state (cruising, checkpoint, box, charging, etc.).
 */

#pragma once

#include <memory>
#include <string>

#include "elma.h"

namespace farm_robot {

class CheckpointSensorController;
class Communicator;
class SharedState;
class VehicleController;
class RobotArmController;
class IVehicleModule;
class IApriltagSensorModule;
class IPlantingDetectorModule;
class IHarvestDetectorModule;
class IRobotArmModule;

class RobotStateMachine : public elma::StateMachine {
public:
    //! Constructs FSM with name and all required state/controller/module dependencies.
    RobotStateMachine(
        const std::string& name,
        SharedState* sharedState,
        VehicleController* vehicleController,
        CheckpointSensorController* checkpointController,
        RobotArmController* robotArmController,
        Communicator* communicator,
        std::shared_ptr<IVehicleModule> vehicleModule,
        std::shared_ptr<IApriltagSensorModule> apriltagModule,
        std::shared_ptr<IPlantingDetectorModule> plantingDetector,
        std::shared_ptr<IHarvestDetectorModule> harvestDetector,
        std::shared_ptr<IRobotArmModule> robotArmModule);

    //! Initializes modules, sets initial state and transitions, starts checkpoint controller.
    void init() override;
    void start() override;
    //! Delegates to StateMachine::update().
    void update() override;
    //! Stops checkpoint controller and disconnects modules.
    void stop() override;

    ~RobotStateMachine();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace farm_robot
