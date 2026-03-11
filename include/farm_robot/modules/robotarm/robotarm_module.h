/**
 * @file robotarm_module.h
 * @brief Interface for robot arm (planting, harvesting, standby).
 *
 * Robot arm interface: initialize, pick_and_plant, go_to_harvest, standby.
 * Implementations: mock, dobot. C++ counterpart of
 * modules/robotarm/domain/robotarm_module.py.
 */

#pragma once

#include <array>
#include <memory>

namespace farm_robot {

class IRobotArmModule {
public:
    virtual ~IRobotArmModule() = default;

    //! Initializes the robot arm (e.g. connect and home).
    virtual void initialize() = 0;
    //! Moves the arm to a named standby position at given speed.
    virtual void moveToStandby(const std::string& positionName, int speed) = 0;
    //! Closes and disconnects the robot arm.
    virtual void close() = 0;
    //! Picks at pickPos and plants at plantPos (x,y,z mm); returns true on success.
    virtual bool pickAndPlant(const std::array<double, 3>& pickPos,
                              const std::array<double, 3>& plantPos) = 0;
    //! Moves to harvest position and harvests; returns true on success.
    virtual bool goToHarvest(const std::array<double, 3>& harvestPos) = 0;
};

}  // namespace farm_robot
