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
    //! Returns the singleton container instance.
    static ModuleContainer& instance();

    ModuleContainer(const ModuleContainer&) = delete;
    ModuleContainer& operator=(const ModuleContainer&) = delete;

    //! Returns the configured vehicle module.
    std::shared_ptr<IVehicleModule> vehicleModule();
    //! Returns the configured robot arm module.
    std::shared_ptr<IRobotArmModule> robotArmModule();
    //! Returns the configured harvest detector module.
    std::shared_ptr<IHarvestDetectorModule> harvestDetector();
    //! Returns the configured planting detector module.
    std::shared_ptr<IPlantingDetectorModule> plantingDetector();
    //! Returns the configured checkpoint sensor module.
    std::shared_ptr<ICheckpointSensorModule> checkpointSensorModule();
    //! Returns the configured AprilTag sensor module.
    std::shared_ptr<IApriltagSensorModule> apriltagSensorModule();

private:
    ModuleContainer() = default;
};

//! Returns the global module container singleton.
inline ModuleContainer& getContainer() {
    return ModuleContainer::instance();
}

}  // namespace farm_robot
