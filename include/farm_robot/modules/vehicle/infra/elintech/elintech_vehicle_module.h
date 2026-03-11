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
    //! Constructs module for the given serial port and baudrate.
    ElintTechVehicleModule(const std::string& port, int baudrate);
    void connect() override;
    void disconnect() override;
    bool isChargingPointReached() override;
    void charge() override;
    void setSpeed(int speed) override;
    VehicleControlCommand currentMotionStatus() override;
    void control(VehicleControlCommand command) override;

    //! Thread-safe: returns status JSON from device, or empty object on failure.
    nlohmann::json readStatus();

private:
    //! Computes CRC16 for the given string.
    uint16_t crc16(const std::string& data);
    //! Builds a simple command string.
    std::string buildSimpleCmd(const std::string& cmd);
    //! Builds a run command with move, speed, count, and timing.
    std::string buildRunCmd(int move, int speed, int cnt = 0, int us = 1);
    //! Sends a command and returns the response JSON.
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
