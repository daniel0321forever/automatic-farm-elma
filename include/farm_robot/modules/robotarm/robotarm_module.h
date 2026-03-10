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

    /**
     * Initialize the robot arm module. 
     */
    virtual void initialize() = 0;

    /**
     * Move the robot arm to the standby position.
     * 
     * @param positionName The name of the standby position.
     * @param speed The speed of the robot arm.
     */
    virtual void moveToStandby(const std::string& positionName, int speed) = 0;

    /**
     * Close the robot arm. 
     */
    virtual void close() = 0;

    /**
     * Pick seedling at pickPos and plant at plantPos (x, y, z in mm).
     * @param pickPos The position to pick the seedling.
     * @param plantPos The position to plant the seedling.
     * @return True if the seedling is picked and planted successfully, false otherwise.
     */
    virtual bool pickAndPlant(const std::array<double, 3>& pickPos,
                              const std::array<double, 3>& plantPos) = 0;

    /**
     * Go to harvest position and harvest vegetable.
     * @param harvestPos The position to harvest the vegetable.
     * @return True if the vegetable is harvested successfully, false otherwise.
     */
    virtual bool goToHarvest(const std::array<double, 3>& harvestPos) = 0;
};

}  // namespace farm_robot
