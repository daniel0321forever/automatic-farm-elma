/**
 * @file mock_planting_detector.cc
 * @brief Mock planting detector implementation.
 *
 * C++ counterpart of modules/vision/infra/planting/mock/mock_planting_module.py.
 */

#include <iostream>

#include "farm_robot/modules/vision/planting_detector.h"
#include "farm_robot/modules/vision/planting/mock_planting_detector.h"

namespace farm_robot {

void MockPlantingDetector::connect() {
    std::cout << "[MockPlantingDetectorModule] Connecting to planting detector camera..." << std::endl;
}

void MockPlantingDetector::disconnect() {
    std::cout << "[MockPlantingDetectorModule] Disconnecting from planting detector camera..." << std::endl;
}

void MockPlantingDetector::loadModel(const std::string& /*modelPath*/) {
    std::cout << "[MockPlantingDetectorModule] No model to load for mocking" << std::endl;
}

void MockPlantingDetector::dispose() {
    std::cout << "[MockPlantingDetectorModule] Disposing detector..." << std::endl;
}

std::vector<std::array<double, 3>> MockPlantingDetector::detect() {
    std::cout << "[MockPlantingDetectorModule] Seeding detection is not implemented" << std::endl;
    return {};
}

}  // namespace farm_robot
