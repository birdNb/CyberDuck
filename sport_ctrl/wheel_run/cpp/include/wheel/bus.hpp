#pragma once

#include "wheel/math.hpp"

#include <cstdint>
#include <string>

namespace wheel {

struct Feedback {
    int16_t raw_pos = 0;
    float position = 0;
    float velocity = 0;
    float torque = 0;
    float voltage = 0;
    float temperature = 0;
    uint8_t mode = 0;
    uint8_t fault = 0;
    uint64_t t_pos_ns = 0;
    uint64_t t_rx_ns = 0;
};

struct Frame {
    int id = 0;
    uint8_t data[8]{};
    int dlc = 0;
};

struct DecodeResult {
    bool any = false;
    bool position = false;
};

DecodeResult decode_feedback(Feedback& fb, const uint8_t* data, int dlc, uint64_t now_ns);

class Bus {
public:
    explicit Bus(const std::string& iface);
    ~Bus();
    Bus(const Bus&) = delete;
    Bus& operator=(const Bus&) = delete;

    bool ok() const { return fd_ >= 0; }
    const std::string& error() const { return error_; }

    bool send(int id, const uint8_t* data, int len);
    bool send_stop(int id);
    bool send_zero(int id);
    // 默认 query=0x00：位置指令不回包。query=0x0E 会让每条 0x1F 都回状态；全关节 50Hz 会把 PCAN-USB 灌爆。
    bool send_position(int id, int16_t raw_pos, float vel_rad_s, float torque_nm, uint8_t query = 0x00);
    bool send_velocity(int id, float vel_rad_s);
    bool send_status(int id);
    bool send_temp(int id);
    bool send_voltage(int id);
    bool send_ping(int id);
    bool send_periodic_off(int id);
    bool poll(Frame& out, int timeout_ms);
    void drain();

private:
    int fd_ = -1;
    std::string error_;
};

}  // namespace wheel
