/**
 * @file robot_state_machine.cc
 * @brief Robot state machine implementation with Elma states.
 */

#include "farm_robot/application/robot_state_machine.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "farm_robot/application/checkpoint_sensor_controller.h"
#include "farm_robot/application/communicator.h"
#include "farm_robot/application/robot_arm_controller.h"
#include "farm_robot/application/vehicle_controller.h"
#include "farm_robot/cores/config.h"
#include "farm_robot/cores/enums.h"
#include "farm_robot/cores/shared_state.h"
#include "farm_robot/modules/apriltag_sensor/apriltag_sensor_module.h"
#include "farm_robot/modules/robotarm/robotarm_module.h"
#include "farm_robot/modules/vehicle/domain/vehicle_module.h"
#include "farm_robot/modules/vision/harvest_detector.h"
#include "farm_robot/modules/vision/planting_detector.h"

namespace farm_robot {

namespace {

struct FsmContext {
    SharedState* sharedState{nullptr};
    VehicleController* vehicleController{nullptr};
    CheckpointSensorController* checkpointController{nullptr};
    RobotArmController* robotArmController{nullptr};
    Communicator* communicator{nullptr};
    IVehicleModule* vehicleModule{nullptr};
    IApriltagSensorModule* apriltagModule{nullptr};
    IPlantingDetectorModule* plantingDetector{nullptr};
    IHarvestDetectorModule* harvestDetector{nullptr};
    IRobotArmModule* robotArmModule{nullptr};
    int updateCount{0};
    int goingToChargingCount{0};
};

// --- Cruising ---
class CruisingState : public elma::State {
public:
    explicit CruisingState(FsmContext* ctx) : elma::State(Config::ROBOT_STATE_CRUISING), ctx_(ctx) {}
    void entry(const elma::Event&) override {
        if (ctx_->sharedState) ctx_->sharedState->setRobotState(Config::ROBOT_STATE_CRUISING);
    }
    void during() override {
        if (!ctx_->apriltagModule || !ctx_->sharedState) return;
        ctx_->updateCount++;
        if (ctx_->updateCount % 10 == 0) {
            auto det = ctx_->apriltagModule->detectBox();
            if (det && det->onPosition) {
                ctx_->sharedState->setCurrentBoxId(det->tagId);
                std::cout << "[VisionThread] april tag for box " << det->tagId << " detected." << std::endl;
                emit(elma::Event("box_on_position"));
                return;
            }
        }
        auto ev = ctx_->sharedState->getAndClearRequestedEvent();
        if (ev && *ev == "reach_checking_point") {
            emit(elma::Event("reach_checking_point"));
        }
    }
    void exit(const elma::Event&) override {}
private:
    FsmContext* ctx_;
};

// --- ReachCheckingPoint ---
class ReachCheckingPointState : public elma::State {
public:
    explicit ReachCheckingPointState(FsmContext* ctx) : elma::State(Config::ROBOT_STATE_REACH_CHECKING_POINT), ctx_(ctx) {}
    void entry(const elma::Event&) override {
        if (ctx_->sharedState) ctx_->sharedState->setRobotState(Config::ROBOT_STATE_REACH_CHECKING_POINT);
        if (ctx_->vehicleController) ctx_->vehicleController->toNextCheckpoint();
    }
    void during() override {}
    void exit(const elma::Event&) override {}
private:
    FsmContext* ctx_;
};

// --- GoingToChargingStation ---
class GoingToChargingStationState : public elma::State {
public:
    explicit GoingToChargingStationState(FsmContext* ctx) : elma::State(Config::ROBOT_STATE_GOING_TO_CHARGING_STATION), ctx_(ctx) {}
    void entry(const elma::Event&) override {
        if (ctx_->sharedState) ctx_->sharedState->setRobotState(Config::ROBOT_STATE_GOING_TO_CHARGING_STATION);
        ctx_->goingToChargingCount = 0;
    }
    void during() override {
        ctx_->goingToChargingCount++;
        if (ctx_->goingToChargingCount >= 500) {
            if (ctx_->vehicleModule) ctx_->vehicleModule->control(VehicleControlCommand::STOP);
            if (ctx_->sharedState) ctx_->sharedState->setRobotState(Config::ROBOT_STATE_STOPPED_AT_CHARGING_STATION);
            std::cout << "[FSM] Stopped at charging station." << std::endl;
            emit(elma::Event("stopped_at_charging_station"));
        }
    }
    void exit(const elma::Event&) override {}
private:
    FsmContext* ctx_;
};

// --- StoppedAtChargingStation ---
class StoppedAtChargingStationState : public elma::State {
public:
    explicit StoppedAtChargingStationState(FsmContext* ctx) : elma::State(Config::ROBOT_STATE_STOPPED_AT_CHARGING_STATION), ctx_(ctx) {}
    void entry(const elma::Event&) override {
        if (ctx_->sharedState) ctx_->sharedState->setRobotState(Config::ROBOT_STATE_STOPPED_AT_CHARGING_STATION);
        if (ctx_->communicator) ctx_->communicator->sendChargingSignal();
        emit(elma::Event("charging"));
    }
    void during() override {}
    void exit(const elma::Event&) override {}
private:
    FsmContext* ctx_;
};

// --- Charging ---
class ChargingState : public elma::State {
public:
    explicit ChargingState(FsmContext* ctx) : elma::State(Config::ROBOT_STATE_CHARGING), ctx_(ctx) {}
    void entry(const elma::Event&) override {
        if (ctx_->sharedState) ctx_->sharedState->setRobotState(Config::ROBOT_STATE_CHARGING);
        if (ctx_->vehicleModule) ctx_->vehicleModule->charge();
    }
    void during() override {}
    void exit(const elma::Event&) override {}
private:
    FsmContext* ctx_;
};

// --- StoppingAtNewBox ---
class StoppingAtNewBoxState : public elma::State {
public:
    explicit StoppingAtNewBoxState(FsmContext* ctx) : elma::State(Config::ROBOT_STATE_STOPPING_AT_NEW_BOX), ctx_(ctx) {}
    void entry(const elma::Event&) override {
        if (ctx_->sharedState) ctx_->sharedState->setRobotState(Config::ROBOT_STATE_STOPPING_AT_NEW_BOX);
        int boxId = ctx_->sharedState ? ctx_->sharedState->currentBoxId() : -1;
        if (ctx_->vehicleController) ctx_->vehicleController->stopAtNewBox(boxId);
        if (ctx_->communicator) {
            ctx_->communicator->sendCurrentBox();
            ctx_->communicator->waitForScaffoldRotation();
        }
        std::cout << "[FSM] New box is ready." << std::endl;
        emit(elma::Event("new_box_is_ready"));
    }
    void during() override {}
    void exit(const elma::Event&) override {}
private:
    FsmContext* ctx_;
};

// --- NewBoxIsReady ---
class NewBoxIsReadyState : public elma::State {
public:
    explicit NewBoxIsReadyState(FsmContext* ctx) : elma::State(Config::ROBOT_STATE_NEW_BOX_IS_READY), ctx_(ctx) {}
    void entry(const elma::Event&) override {
        if (ctx_->sharedState) {
            ctx_->sharedState->setRobotState(Config::ROBOT_STATE_NEW_BOX_IS_READY);
            ctx_->sharedState->setCurrentInBoxIndex(0);
        }
        emit(elma::Event("going_to_next_target"));
    }
    void during() override {}
    void exit(const elma::Event&) override {}
private:
    FsmContext* ctx_;
};

// --- GoingToNextTarget ---
class GoingToNextTargetState : public elma::State {
public:
    explicit GoingToNextTargetState(FsmContext* ctx) : elma::State(Config::ROBOT_STATE_GOING_TO_NEXT_TARGET), ctx_(ctx) {}
    void entry(const elma::Event&) override {
        if (ctx_->sharedState) ctx_->sharedState->setRobotState(Config::ROBOT_STATE_GOING_TO_NEXT_TARGET);
        int index = ctx_->sharedState ? ctx_->sharedState->currentInBoxIndex() : 0;
        if (ctx_->vehicleController) ctx_->vehicleController->goToTarget(index);
        std::cout << "[FSM] Stopped at next target." << std::endl;
        emit(elma::Event("stopped_at_next_target"));
    }
    void during() override {}
    void exit(const elma::Event&) override {}
private:
    FsmContext* ctx_;
};

// --- StoppedAtNextTarget ---
class StoppedAtNextTargetState : public elma::State {
public:
    explicit StoppedAtNextTargetState(FsmContext* ctx) : elma::State(Config::ROBOT_STATE_STOPPED_AT_NEXT_TARGET), ctx_(ctx) {}
    void entry(const elma::Event&) override {
        if (ctx_->sharedState) ctx_->sharedState->setRobotState(Config::ROBOT_STATE_STOPPED_AT_NEXT_TARGET);
    }
    void during() override {
        if (!ctx_->sharedState || !ctx_->vehicleController) return;
        const std::string task = ctx_->sharedState->currentTask();
        if (task == "planting" && ctx_->plantingDetector) {
            auto coords = ctx_->plantingDetector->detect();
            if (!coords.empty()) {
                ctx_->sharedState->setTargetCoordinates(coords[0]);
                std::cout << "[FSM] Planting target set, performing task." << std::endl;
                emit(elma::Event("performing_task"));
                return;
            }
        } else if (task == "harvesting" && ctx_->harvestDetector) {
            auto coords = ctx_->harvestDetector->detect();
            if (coords) {
                ctx_->sharedState->setTargetCoordinates(*coords);
                std::cout << "[FSM] Harvest target set, performing task." << std::endl;
                emit(elma::Event("performing_task"));
                return;
            }
        }
        ctx_->vehicleController->resume();
        emit(elma::Event("cruising"));
    }
    void exit(const elma::Event&) override {}
private:
    FsmContext* ctx_;
};

// --- PerformingTask ---
class PerformingTaskState : public elma::State {
public:
    explicit PerformingTaskState(FsmContext* ctx) : elma::State(Config::ROBOT_STATE_PERFORMING_TASK), ctx_(ctx) {}
    void entry(const elma::Event&) override {
        if (ctx_->sharedState) ctx_->sharedState->setRobotState(Config::ROBOT_STATE_PERFORMING_TASK);
        auto coords = ctx_->sharedState ? ctx_->sharedState->targetCoordinates() : std::optional<std::array<double, 3>>();
        if (coords && ctx_->robotArmController && ctx_->vehicleController) {
            const std::string task = ctx_->sharedState->currentTask();
            if (task == "planting") ctx_->robotArmController->plant(*coords);
            else if (task == "harvesting") ctx_->robotArmController->harvest(*coords);
            ctx_->sharedState->clearTargetCoordinates();
            ctx_->vehicleController->resume();
        }
        std::cout << "[FSM] Task done, finishing box." << std::endl;
        emit(elma::Event("finish_box"));
    }
    void during() override {}
    void exit(const elma::Event&) override {}
private:
    FsmContext* ctx_;
};

// --- FinishBox ---
class FinishBoxState : public elma::State {
public:
    explicit FinishBoxState(FsmContext* ctx) : elma::State(Config::ROBOT_STATE_FINISH_BOX), ctx_(ctx) {}
    void entry(const elma::Event&) override {
        if (ctx_->sharedState) ctx_->sharedState->setRobotState(Config::ROBOT_STATE_FINISH_BOX);
        if (ctx_->communicator) ctx_->communicator->sendFinishBoxSignal();
        if (ctx_->sharedState) {
            ctx_->sharedState->clearCurrentBox();
            ctx_->sharedState->clearApriltag();
        }
        std::cout << "[FSM] Box finished, resuming cruising." << std::endl;
        emit(elma::Event("cruising"));
    }
    void during() override {}
    void exit(const elma::Event&) override {}
private:
    FsmContext* ctx_;
};

}  // namespace

struct RobotStateMachine::Impl {
    FsmContext ctx;
    CruisingState cruising;
    ReachCheckingPointState reachCheckingPoint;
    GoingToChargingStationState goingToChargingStation;
    StoppedAtChargingStationState stoppedAtChargingStation;
    ChargingState charging;
    StoppingAtNewBoxState stoppingAtNewBox;
    NewBoxIsReadyState newBoxIsReady;
    GoingToNextTargetState goingToNextTarget;
    StoppedAtNextTargetState stoppedAtNextTarget;
    PerformingTaskState performingTask;
    FinishBoxState finishBox;

    std::shared_ptr<IVehicleModule> vehicleModule;
    std::shared_ptr<IApriltagSensorModule> apriltagModule;
    std::shared_ptr<IPlantingDetectorModule> plantingDetector;
    std::shared_ptr<IHarvestDetectorModule> harvestDetector;
    std::shared_ptr<IRobotArmModule> robotArmModule;

    Impl(SharedState* sharedState,
         VehicleController* vehicleController,
         CheckpointSensorController* checkpointController,
         RobotArmController* robotArmController,
         Communicator* communicator,
         std::shared_ptr<IVehicleModule> vehicleModuleIn,
         std::shared_ptr<IApriltagSensorModule> apriltagModuleIn,
         std::shared_ptr<IPlantingDetectorModule> plantingDetectorIn,
         std::shared_ptr<IHarvestDetectorModule> harvestDetectorIn,
         std::shared_ptr<IRobotArmModule> robotArmModuleIn)
        : ctx{sharedState, vehicleController, checkpointController, robotArmController, communicator,
             vehicleModuleIn.get(), apriltagModuleIn.get(), plantingDetectorIn.get(), harvestDetectorIn.get(), robotArmModuleIn.get()},
          cruising(&ctx),
          reachCheckingPoint(&ctx),
          goingToChargingStation(&ctx),
          stoppedAtChargingStation(&ctx),
          charging(&ctx),
          stoppingAtNewBox(&ctx),
          newBoxIsReady(&ctx),
          goingToNextTarget(&ctx),
          stoppedAtNextTarget(&ctx),
          performingTask(&ctx),
          finishBox(&ctx),
          vehicleModule(std::move(vehicleModuleIn)),
          apriltagModule(std::move(apriltagModuleIn)),
          plantingDetector(std::move(plantingDetectorIn)),
          harvestDetector(std::move(harvestDetectorIn)),
          robotArmModule(std::move(robotArmModuleIn)) {}
};

RobotStateMachine::RobotStateMachine(
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
    std::shared_ptr<IRobotArmModule> robotArmModule)
    : elma::StateMachine(name),
      impl_(std::make_unique<Impl>(sharedState, vehicleController, checkpointController,
                                   robotArmController, communicator,
                                   vehicleModule, apriltagModule,
                                   plantingDetector, harvestDetector,
                                   robotArmModule)) {}

void RobotStateMachine::init() {
    std::srand(0);
    impl_->vehicleModule->connect();
    impl_->apriltagModule->connect();
    impl_->plantingDetector->connect();
    impl_->harvestDetector->connect();
    impl_->plantingDetector->loadModel("");
    impl_->harvestDetector->loadModel("");
    impl_->robotArmModule->initialize();
    if (impl_->ctx.sharedState) impl_->ctx.sharedState->setCurrentTask("planting");
    impl_->ctx.checkpointController->start();

    impl_->ctx.vehicleController->setEmitCallback([this](const std::string& e) { emit(elma::Event(e)); });
    set_initial(impl_->cruising);
    add_transition("reach_checking_point", impl_->cruising, impl_->reachCheckingPoint);
    add_transition("box_on_position", impl_->cruising, impl_->stoppingAtNewBox);
    add_transition("cruising", impl_->reachCheckingPoint, impl_->cruising);
    add_transition("going_to_charging_station", impl_->reachCheckingPoint, impl_->goingToChargingStation);
    add_transition("stopped_at_charging_station", impl_->goingToChargingStation, impl_->stoppedAtChargingStation);
    add_transition("charging", impl_->stoppedAtChargingStation, impl_->charging);
    add_transition("new_box_is_ready", impl_->stoppingAtNewBox, impl_->newBoxIsReady);
    add_transition("going_to_next_target", impl_->newBoxIsReady, impl_->goingToNextTarget);
    add_transition("stopped_at_next_target", impl_->goingToNextTarget, impl_->stoppedAtNextTarget);
    add_transition("performing_task", impl_->stoppedAtNextTarget, impl_->performingTask);
    add_transition("cruising", impl_->stoppedAtNextTarget, impl_->cruising);
    add_transition("finish_box", impl_->performingTask, impl_->finishBox);
    add_transition("cruising", impl_->finishBox, impl_->cruising);
    StateMachine::init();
}

RobotStateMachine::~RobotStateMachine() = default;

void RobotStateMachine::start() {
    StateMachine::start();
    if (impl_->ctx.vehicleController) impl_->ctx.vehicleController->startCycle();
    std::cout << "[Main] Starting RobotStateMachine..." << std::endl;
}

void RobotStateMachine::update() {
    StateMachine::update();
}

void RobotStateMachine::stop() {
    StateMachine::stop();
    if (impl_->ctx.checkpointController) {
        impl_->ctx.checkpointController->stop();
        impl_->ctx.checkpointController->join();
    }
    impl_->plantingDetector->disconnect();
    impl_->harvestDetector->disconnect();
    impl_->apriltagModule->disconnect();
    impl_->robotArmModule->close();
}

}  // namespace farm_robot
