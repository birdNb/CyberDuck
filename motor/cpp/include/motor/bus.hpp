#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace motor {

struct State {
    int id = 0;
    int mode = 0;
    double position = 0;
    double velocity = 0;
    double torque = 0;
    double voltage = 0;
    bool fresh = false;
};

class Bus {
public:
    explicit Bus(std::string iface);
    ~Bus();
    Bus(const Bus&) = delete;
    Bus& operator=(const Bus&) = delete;

    void add(int id, std::string type, std::string name);
    void set_mode(int id, int mode);
    void enable_position(int id);
    void disable(int id);
    void set_timeout_ms(int id, int timeout_ms);
    void read_status(int id);
    void command_position(int id, double position_rad, double vel_limit_rad_s, double torque_nm);
    void poll(double timeout_s);
    const State* state(int id) const;

private:
    void send(uint32_t can_id, const uint8_t* data, int len, bool extended);
    int fd_ = -1;
    std::string iface_;
    std::unordered_map<int, std::string> types_;
    std::unordered_map<int, State> states_;
};

}  // namespace motor
