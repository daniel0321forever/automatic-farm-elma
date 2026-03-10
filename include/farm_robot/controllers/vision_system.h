/**
 * @file vision_system.h
 * @brief Vision thread for planting, harvesting, and AprilTag detection.
 *
 * VisionThread switches between planting, harvesting, and AprilTag modes.
 * Calls detectors and updates SharedState with target coordinates and box
 * detection. C++ counterpart of controllers/vision_system.py.
 */

#pragma once

#include <memory>
#include <thread>
#include <atomic>

namespace farm_robot {

class SharedState;
class IHarvestDetectorModule;
class IPlantingDetectorModule;
class IApriltagSensorModule;

class VisionThread {
public:
    VisionThread(SharedState* sharedState,
                 std::shared_ptr<IPlantingDetectorModule> plantingDetector,
                 std::shared_ptr<IHarvestDetectorModule> harvestDetector,
                 std::shared_ptr<IApriltagSensorModule> apriltagSensor);

    ~VisionThread();
    void start();
    void stop();
    void join();

private:
    void run();

    SharedState* sharedState_;
    std::shared_ptr<IPlantingDetectorModule> plantingDetector_;
    std::shared_ptr<IHarvestDetectorModule> harvestDetector_;
    std::shared_ptr<IApriltagSensorModule> apriltagSensor_;
    std::thread thread_;
    std::atomic<bool> running_;
};

}  // namespace farm_robot
