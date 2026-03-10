/**
 * @file container.h
 * @brief Dependency injection container for module implementations.
 *
 * Singleton factory that provides vehicle, robot arm, planting/harvest detectors,
 * checkpoint sensor, and AprilTag sensor modules according to config. Supports
 * mock, elintech, dobot, and other implementations. C++ counterpart of cores/container.py.
 */

#pragma once

#include <memory>

namespace farm_robot {

class IVehicleModule;
class IRobotArmModule;
class IHarvestDetectorModule;
class IPlantingDetectorModule;
class ICheckpointSensorModule;
class IApriltagSensorModule;

class ModuleContainer {
public:
    static ModuleContainer& instance();

    ModuleContainer(const ModuleContainer&) = delete;
    ModuleContainer& operator=(const ModuleContainer&) = delete;

    std::shared_ptr<IVehicleModule> vehicleModule();
    std::shared_ptr<IRobotArmModule> robotArmModule();
    std::shared_ptr<IHarvestDetectorModule> harvestDetector();
    std::shared_ptr<IPlantingDetectorModule> plantingDetector();
    std::shared_ptr<ICheckpointSensorModule> checkpointSensorModule();
    std::shared_ptr<IApriltagSensorModule> apriltagSensorModule();

private:
    ModuleContainer() = default;
};

inline ModuleContainer& getContainer() {
    return ModuleContainer::instance();
}

}  // namespace farm_robot
