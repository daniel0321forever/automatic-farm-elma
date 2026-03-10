/**
 * @file config.h
 * @brief Central configuration for farm robot (work hours, aisles, module types).
 *
 * Holds constants for work schedule, farm structure, AprilTag IDs, camera
 * matrices, vehicle/robotarm/sensor module types, and robot state strings.
 * C++ counterpart of cores/config.py.
 */

#pragma once

#include <array>
#include <string>
#include <vector>
#include <unordered_map>

namespace farm_robot {

struct Config {
    // Working hours
    static constexpr int WORK_START_HOUR = 0;
    static constexpr int WORK_END_HOUR = 25;

    // Farm structure
    static constexpr int TOTAL_AISLES = 3;
    static constexpr int TOTAL_CYCLES_PER_TASK = 100;
    static constexpr int TOTAL_PLANTING_SPOT = 3;

    // Robot speed (RPM)
    static constexpr int MOVE_SPEED = 20;

    // AprilTag
    static constexpr const char* FAMILY = "tag5h13";
    static constexpr int FINAL_TAG_ID = 200;

    // Vehicle
    static constexpr const char* VEHICLE_PORT = "/dev/ttyUSB0";
    static constexpr int VEHICLE_BAUDRATE = 115200;

    // Log file
    static constexpr const char* LOG_FILE_PATH = "farm_operations.log";

    // Module types
    static constexpr const char* HARVEST_DETECTOR_MODULE_TYPE = "mock";
    static constexpr const char* PLANTING_DETECTOR_MODULE_TYPE = "mock";
    static constexpr const char* APRILTAG_SENSOR_MODULE_TYPE = "mock";
    static constexpr const char* ROBOTARM_MODULE_TYPE = "mock";
    static constexpr const char* VEHICLE_MODULE_TYPE = "mock";
    static constexpr const char* CHECKPOINT_SENSOR_MODULE_TYPE = "mock";

    // Task list
    static std::vector<std::string> tasks();

    // End-of-aisle tag IDs
    static std::unordered_map<int, int> endOfAisleTags();

    // Box tags
    static std::vector<int> boxTags();

    // Model paths
    static std::unordered_map<std::string, std::string> modelPaths();

    // Root directory
    static std::string rootDir();

    // Robot state strings
    static constexpr const char* ROBOT_STATE_CRUISING = "cruising";
    static constexpr const char* ROBOT_STATE_REACH_CHECKING_POINT = "reach_checking_point";
    static constexpr const char* ROBOT_STATE_GOING_TO_CHARGING_STATION = "going_to_charging_station";
    static constexpr const char* ROBOT_STATE_STOPPED_AT_CHARGING_STATION = "stopped_at_charging_station";
    static constexpr const char* ROBOT_STATE_CHARGING = "charging";
    static constexpr const char* ROBOT_STATE_STOPPING_AT_NEW_BOX = "going_to_new_box";
    static constexpr const char* ROBOT_STATE_NEW_BOX_IS_READY = "new_box_is_ready";
    static constexpr const char* ROBOT_STATE_GOING_TO_NEXT_TARGET = "going_to_next_target";
    static constexpr const char* ROBOT_STATE_STOPPED_AT_NEXT_TARGET = "stopped_at_next_target";
    static constexpr const char* ROBOT_STATE_PERFORMING_TASK = "performing_task";
    static constexpr const char* ROBOT_STATE_FINISH_BOX = "finish_box";
    static constexpr const char* ROBOT_STATE_CYCLE_END = "cycle_end";
};

}  // namespace farm_robot
