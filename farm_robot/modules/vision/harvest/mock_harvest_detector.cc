/**
 * @file mock_harvest_detector.cc
 * @brief Mock harvest detector implementation.
 *
 * C++ counterpart of modules/vision/infra/harvest/mock/mock_harvest_detector.py.
 */

#include <cstdlib>
#include <iostream>

#include "farm_robot/modules/vision/harvest_detector.h"
#include "farm_robot/modules/vision/harvest/mock_harvest_detector.h"

namespace farm_robot {

void MockHarvestDetector::connect() {
    if (isConnected_)
        return;
    isConnected_ = true;
    std::cout << "[MockHarvestDetectorModule] Connecting to harvest detector camera..." << std::endl;
}

void MockHarvestDetector::disconnect() {
    std::cout << "[MockHarvestDetectorModule] Disconnecting from harvest detector camera..." << std::endl;
}

void MockHarvestDetector::loadModel(const std::string& modelPath) {
    std::cout << "[MockHarvestDetectorModule] Mock harvest detector is loading model from " << modelPath << std::endl;
}

void MockHarvestDetector::disposeModel() {}

std::optional<std::array<double, 3>> MockHarvestDetector::detect() {
    std::cout << "[MockHarvestDetectorModule] Mock harvest detector is detecting..." << std::endl;
    std::array<double, 3> coords;
    coords[0] = static_cast<double>(std::rand()) / RAND_MAX;
    coords[1] = static_cast<double>(std::rand()) / RAND_MAX;
    coords[2] = static_cast<double>(std::rand()) / RAND_MAX;
    return coords;
}

}  // namespace farm_robot
