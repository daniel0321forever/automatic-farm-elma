/**
 * @file scheduler.h
 * @brief Work-time checker to determine if current time is within work hours.
 *
 * Uses system clock to decide whether the robot should run operations based on
 * configured work start/end hours. C++ counterpart of cores/scheduler.py.
 */

#pragma once

namespace farm_robot {

class Scheduler {
public:
    //! Returns true if the current time is within configured work hours.
    static bool isWorkTime();
};

}  // namespace farm_robot
