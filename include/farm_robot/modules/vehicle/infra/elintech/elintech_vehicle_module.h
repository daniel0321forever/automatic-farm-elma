/**
 * @file elintech_vehicle_module.h
 * @brief ElintTech AGV module over serial.
 *
 * Vehicle control via serial port, JSON commands, CRC16. C++ counterpart of
 * modules/vehicle/infra/elintech/elintech_vehicle_module.py.
 */

#pragma once

#include "farm_robot/modules/vehicle/domain/vehicle_module.h"
#include "farm_robot/cores/config.h"
#include "json/json.h"
#include <mutex>
#include <string>

namespace farm_robot {

class ElintTechVehicleModule : public IVehicleModule {
public:
    ElintTechVehicleModule(const std::string& port, int baudrate);
    void connect() override;
    void disconnect() override;
    bool isChargingPointReached() override;
    void charge() override;
    void setSpeed(int speed) override;
    VehicleControlCommand currentMotionStatus() override;
    void control(VehicleControlCommand command) override;

    /** Thread-safe: returns status object from read_status, or empty {} on failure. */
    nlohmann::json readStatus();

private:
    uint16_t crc16(const std::string& data);
    std::string buildSimpleCmd(const std::string& cmd);
    std::string buildRunCmd(int move, int speed, int cnt = 0, int us = 1);
    nlohmann::json sendCmd(const std::string& cmd);

    int serial_fd_{-1};
    std::string port_;
    int baudrate_;
    bool isConnected_{false};
    int speed_{Config::MOVE_SPEED};
    std::mutex serialMutex_;
    VehicleControlCommand cachedCommand_{VehicleControlCommand::STOP};
};

}  // namespace farm_robot
