/**
 * @file apriltag_detection.h
 * @brief Data structure for AprilTag detection results.
 *
 * Holds tag ID, type, on-position flag, and validity for box detection on the
 * ground. C++ counterpart of models/apriltag_detection.py.
 */

#pragma once

#include "farm_robot/cores/enums.h"
#include <array>

namespace farm_robot {

struct ApriltagDetection {
    int tagId;
    ApriltagType tagType;
    bool onPosition;
    double validity;

    //! Constructs a detection with the given id, type, on-position flag, and validity.
    ApriltagDetection(int id, ApriltagType type, bool onPos, double val)
        : tagId(id), tagType(type), onPosition(onPos), validity(val) {}
};

}  // namespace farm_robot
