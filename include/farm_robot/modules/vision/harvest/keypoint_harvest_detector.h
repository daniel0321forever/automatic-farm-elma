/**
 * @file keypoint_harvest_detector.h
 * @brief YOLO keypoint-based harvest detector.
 *
 * YOLO keypoints + depth, OAK-D. Stub implementation. C++ counterpart of
 * modules/vision/infra/harvest/keypoint/keypoint_harvest_detector.py.
 */

#pragma once

#include "farm_robot/modules/vision/harvest_detector.h"

namespace farm_robot {

class KeypointHarvestDetector : public IHarvestDetectorModule {
public:
    void connect() override;
    void disconnect() override;
    void loadModel(const std::string& modelPath) override;
    void disposeModel() override;
    std::optional<std::array<double, 3>> detect() override;
};

}  // namespace farm_robot
