#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>

#include "elma.h"
#include "farm_robot/application/checkpoint_sensor_controller.h"
#include "farm_robot/application/communicator.h"
#include "farm_robot/application/robot_arm_controller.h"
#include "farm_robot/application/robot_state_machine.h"
#include "farm_robot/application/vehicle_controller.h"
#include "farm_robot/cores/config.h"
#include "farm_robot/cores/shared_state.h"
#include "farm_robot/modules/apriltag_sensor/apriltag_sensor_module.h"
#include "farm_robot/modules/apriltag_sensor/mock_apriltag_sensor_module.h"
#include "farm_robot/modules/checkpoint_sensor/domain/checkpoint_sensor.h"
#include "farm_robot/modules/checkpoint_sensor/infra/elintech/elintech_checkpoint_sensor.h"
#include "farm_robot/modules/checkpoint_sensor/infra/mock/mock_checkpoint_sensor.h"
#include "farm_robot/modules/robotarm/mock_robotarm_module.h"
#include "farm_robot/modules/robotarm/robotarm_module.h"
#include "farm_robot/modules/vehicle/domain/vehicle_module.h"
#include "farm_robot/modules/vehicle/infra/elintech/elintech_vehicle_module.h"
#include "farm_robot/modules/vehicle/infra/mock/mock_vehicle_module.h"
#include "farm_robot/modules/vision/harvest_detector.h"
#include "farm_robot/modules/vision/harvest/mock_harvest_detector.h"
#include "farm_robot/modules/vision/planting_detector.h"
#include "farm_robot/modules/vision/planting/mock_planting_detector.h"

using namespace std::chrono;
using namespace elma;

static std::shared_ptr<farm_robot::IVehicleModule> createVehicle() {
    if (std::strcmp(farm_robot::Config::VEHICLE_MODULE_TYPE, "elintech") == 0) {
        return std::make_shared<farm_robot::ElintTechVehicleModule>(
            farm_robot::Config::VEHICLE_PORT,
            farm_robot::Config::VEHICLE_BAUDRATE);
    }
    return std::make_shared<farm_robot::MockVehicleModule>();
}

static std::shared_ptr<farm_robot::ICheckpointSensorModule> createCheckpoint(
    const std::shared_ptr<farm_robot::IVehicleModule>& vehicle) {
    if (std::strcmp(farm_robot::Config::CHECKPOINT_SENSOR_MODULE_TYPE,
                    "elintech") == 0) {
        return std::make_shared<farm_robot::ElintTechCheckpointSensor>(vehicle);
    }
    return std::make_shared<farm_robot::MockCheckpointSensor>();
}

static std::shared_ptr<farm_robot::IApriltagSensorModule> createApriltag() {
    return std::make_shared<farm_robot::MockApriltagSensorModule>();
}

static std::shared_ptr<farm_robot::IRobotArmModule> createRobotArm() {
    return std::make_shared<farm_robot::MockRobotArmModule>();
}

static std::shared_ptr<farm_robot::IPlantingDetectorModule> createPlantingDetector() {
    return std::make_shared<farm_robot::MockPlantingDetector>();
}

static std::shared_ptr<farm_robot::IHarvestDetectorModule> createHarvestDetector() {
    return std::make_shared<farm_robot::MockHarvestDetector>();
}

int main() {
    farm_robot::SharedState sharedState;

    std::cout << "[Main] Getting all the modules from container..." << std::endl;
    auto vehicle = createVehicle();
    auto checkpoint = createCheckpoint(vehicle);
    auto apriltag = createApriltag();
    auto robotArm = createRobotArm();
    auto plantingDetector = createPlantingDetector();
    auto harvestDetector = createHarvestDetector();

    farm_robot::VehicleController vehicleController(&sharedState, vehicle);
    farm_robot::CheckpointSensorController checkpointController(&sharedState,
                                                                checkpoint);
    farm_robot::RobotArmController robotArmController(robotArm);
    farm_robot::Communicator communicator(&sharedState);

    farm_robot::RobotStateMachine fsm(
        "RobotStateMachine",
        &sharedState,
        &vehicleController,
        &checkpointController,
        &robotArmController,
        &communicator,
        vehicle,
        apriltag,
        plantingDetector,
    harvestDetector,
    robotArm);

    std::cout << "[Main] Initializing controller..." << std::endl;
    Manager m;
    m.schedule(fsm, 1_ms).init().start().run(30_s);
}
