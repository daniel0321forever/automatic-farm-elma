/**
 * @file elintech_vehicle_module.cc
 * @brief ElintTech AGV module over serial (JSON + CRC16).
 *
 * C++ counterpart of modules/vehicle/infra/elintech/elintech_vehicle_module.py.
 */

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <unistd.h>

#include <termios.h>

#include "farm_robot/cores/config.h"
#include "farm_robot/cores/enums.h"
#include "farm_robot/modules/vehicle/infra/elintech/elintech_vehicle_module.h"

namespace farm_robot {

namespace {

speed_t baudrateToSpeedT(int baudrate) {
    switch (baudrate) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        default:
            return B115200;
    }
}

}  // namespace

ElintTechVehicleModule::ElintTechVehicleModule(const std::string& port,
                                               int baudrate)
    : port_(port),
      baudrate_(baudrate),
      speed_(Config::MOVE_SPEED) {}

uint16_t ElintTechVehicleModule::crc16(const std::string& data) {
    uint16_t crc = 0x0000;
    for (unsigned char byte : data) {
        crc ^= static_cast<uint16_t>(byte) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
            crc &= 0xFFFF;
        }
    }
    return crc;
}

std::string ElintTechVehicleModule::buildSimpleCmd(const std::string& cmd) {
    std::string jsonStr = "{\"id\":1,\"cmd\":\"" + cmd + "\"}";
    uint16_t c = crc16(jsonStr);
    char hex[5];
    snprintf(hex, sizeof(hex), "%04X", c);
    return jsonStr.substr(0, jsonStr.size() - 1) + ",\"crc\":\"" + hex + "\"}";
}

std::string ElintTechVehicleModule::buildRunCmd(int move, int speed,
                                                 int cnt, int us) {
    std::ostringstream oss;
    oss << "{\"id\":1,\"cmd\":\"run\",\"param\":{\"move\":"
        << move << ",\"speed\":" << speed
        << ",\"CNT\":" << cnt << ",\"US\":" << us << "}}";
    std::string jsonStr = oss.str();
    uint16_t c = crc16(jsonStr) ^ 0xFF8F;
    char hex[5];
    snprintf(hex, sizeof(hex), "%04X", c);
    return jsonStr.substr(0, jsonStr.size() - 1) + ",\"crc\":\"" + hex + "\"}";
}

nlohmann::json ElintTechVehicleModule::sendCmd(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(serialMutex_);
    if (!isConnected_ || serial_fd_ < 0)
        return nlohmann::json::object();

    const int maxWaitMs = 3000;
    const int retryMs = 100;
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(maxWaitMs);

    while (std::chrono::steady_clock::now() < deadline) {
        tcflush(serial_fd_, TCIOFLUSH);
        std::string line = cmd + "\n";
        ssize_t n = write(serial_fd_, line.data(), line.size());
        if (n != static_cast<ssize_t>(line.size()))
            break;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::string response;
        char buf[256];
        while (response.size() < sizeof(buf) - 1) {
            ssize_t r = read(serial_fd_, buf, sizeof(buf) - 1);
            if (r <= 0)
                break;
            buf[r] = '\0';
            response += buf;
            if (response.find('\n') != std::string::npos)
                break;
        }
        size_t nl = response.find('\n');
        if (nl != std::string::npos)
            response.resize(nl);
        while (!response.empty() && (response.back() == '\r' || response.back() == '\n'))
            response.pop_back();

        if (response.empty())
            continue;

        try {
            nlohmann::json j = nlohmann::json::parse(response);
            if (j.find("error") != j.end() && j["error"].get<std::string>() == "00")
                return j;
        } catch (...) {
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(retryMs));
    }
    return nlohmann::json::object();
}

nlohmann::json ElintTechVehicleModule::readStatus() {
    nlohmann::json res = sendCmd(buildSimpleCmd("read_status"));
    if (res.find("status") != res.end() && res["status"].is_object())
        return res["status"];
    return nlohmann::json::object();
}

void ElintTechVehicleModule::connect() {
    std::cout << "[Vehicle] Connecting to " << port_ << "..."
              << std::endl;
    if (isConnected_) {
        std::cout << "[Vehicle] Already connected to " << port_
                  << std::endl;
        return;
    }

    int fd = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "[Vehicle] Failed to open serial port: "
                  << port_ << std::endl;
        serial_fd_ = -1;
        return;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "[Vehicle] tcgetattr failed" << std::endl;
        close(fd);
        serial_fd_ = -1;
        return;
    }

    speed_t sp = baudrateToSpeedT(baudrate_);
    cfsetospeed(&tty, sp);
    cfsetispeed(&tty, sp);

    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "[Vehicle] tcsetattr failed" << std::endl;
        close(fd);
        serial_fd_ = -1;
        return;
    }

    serial_fd_ = fd;
    isConnected_ = true;
    std::cout << "[Vehicle] Connected to " << port_ << std::endl;
}

void ElintTechVehicleModule::disconnect() {
    std::lock_guard<std::mutex> lock(serialMutex_);
    if (serial_fd_ >= 0) {
        std::cout << "[Vehicle] Disconnected from " << port_
                  << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
    }
    isConnected_ = false;
}

bool ElintTechVehicleModule::isChargingPointReached() {
    nlohmann::json s = readStatus();
    if (s.find("mode") != s.end()) {
        int mode = s["mode"].get<int>();
        if (mode == 7)
            return true;
    }
    return false;
}

void ElintTechVehicleModule::charge() {
    std::cout << "[Vehicle] Charging..." << std::endl;
}

void ElintTechVehicleModule::setSpeed(int speed) {
    speed_ = std::max(1, std::min(100, speed));
    std::cout << "[Vehicle] Speed set to " << speed_ << std::endl;
}

VehicleControlCommand ElintTechVehicleModule::currentMotionStatus() {
    nlohmann::json s = readStatus();
    if (s.find("out_rpm") != s.end() && s["out_rpm"].is_array() &&
        s["out_rpm"].size() >= 2) {
        int l = s["out_rpm"][0].get<int>();
        int r = s["out_rpm"][1].get<int>();
        if (l > 1 && r > 1)
            return VehicleControlCommand::MOVE_FORWARD;
        if (l < -1 && r < -1)
            return VehicleControlCommand::MOVE_BACKWARD;
        if (l > 1 && std::abs(r) < 1)
            return VehicleControlCommand::TURN_RIGHT;
        if (std::abs(l) < 1 && r > 1)
            return VehicleControlCommand::TURN_LEFT;
    }
    return cachedCommand_;
}

void ElintTechVehicleModule::control(VehicleControlCommand command) {
    cachedCommand_ = command;
    std::string cmd;
    switch (command) {
        case VehicleControlCommand::MOVE_FORWARD:
            std::cout << "[Vehicle] Moving forward..." << std::endl;
            cmd = buildRunCmd(1, speed_, 0, 1);
            break;
        case VehicleControlCommand::MOVE_BACKWARD:
            std::cout << "[Vehicle] Moving backward..." << std::endl;
            cmd = buildRunCmd(2, speed_, 0, 1);
            break;
        case VehicleControlCommand::TURN_LEFT:
            std::cout << "[Vehicle] Turning left..." << std::endl;
            cmd = buildRunCmd(3, speed_, 1, 1);
            break;
        case VehicleControlCommand::TURN_RIGHT:
            std::cout << "[Vehicle] Turning right..." << std::endl;
            cmd = buildRunCmd(4, speed_, 1, 1);
            break;
        case VehicleControlCommand::STOP:
            std::cout << "[Vehicle] Stopping..." << std::endl;
            cmd = buildSimpleCmd("stop");
            break;
        case VehicleControlCommand::TURN_LEFT_AND_MOVE_FORWARD:
        case VehicleControlCommand::TURN_RIGHT_AND_MOVE_FORWARD:
        case VehicleControlCommand::TURN_LEFT_AND_MOVE_BACKWARD:
        case VehicleControlCommand::TURN_RIGHT_AND_MOVE_BACKWARD:
        case VehicleControlCommand::ROTATE:
        default:
            return;
    }
    nlohmann::json res = sendCmd(cmd);
    if (res.find("error") != res.end() && res["error"].get<std::string>() == "00") {
        if (command != VehicleControlCommand::STOP)
            std::cout << "[Vehicle] Command sent (speed="
                      << speed_ << ")" << std::endl;
        else
            std::cout << "[Vehicle] Stopped" << std::endl;
    } else if (!cmd.empty()) {
        std::cerr << "[Vehicle] Command failed: "
                  << res.dump() << std::endl;
    }
}

}  // namespace farm_robot
