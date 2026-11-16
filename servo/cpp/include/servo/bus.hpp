#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace servo {

struct State {
    int position_raw = 0;
    int speed_raw = 0;
};

class Bus {
public:
    Bus(std::string port, int baudrate, int timeout_ms);
    ~Bus();
    Bus(const Bus&) = delete;
    Bus& operator=(const Bus&) = delete;

    State read_state(int id);
    void set_torque(int id, bool enabled);
    void set_motion(int id, int speed, int acceleration, int torque_limit);
    void set_goal_position(int id, int position_raw);

private:
    std::vector<uint8_t> transact(int id, uint8_t instruction, const std::vector<uint8_t>& params,
                                  size_t response_size);
    void write_packet(int id, uint8_t instruction, const std::vector<uint8_t>& params);
    int fd_ = -1;
    int timeout_ms_ = 4;
};

}  // namespace servo
