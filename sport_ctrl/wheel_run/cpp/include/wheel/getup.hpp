#pragma once

#include <string>

namespace wheel {

struct GetupSense {
    float gyro[3]{};
    float grav[3]{0.f, 0.f, -1.f};
    float pos[16]{};
    float vel[16]{};
    float servo_yaw = 0.f;
    float servo_roll = 0.f;
    bool lb_rise = false;
    bool allow = false;
};

struct GetupGoal {
    bool active = false;
    float deg[16]{};
    float tq[16]{};
    float yaw = 0.f;
    float roll = 0.f;
    bool say_fall = false;
    bool say_start = false;
    bool say_stop = false;
    bool say_refuse = false;
};

class Getup {
public:
    bool load(const std::string& path);
    bool ready() const { return ready_; }
    const float* last_action() const { return last_action_; }
    const std::string& reason() const { return reason_; }
    void step(float dt, const GetupSense& in, GetupGoal& out);

private:
    bool ready_ = false;
    std::string reason_;
    bool active_ = false;
    bool fallen_latched_ = false;
    float fallen_for_ = 0.f;
    bool have_prev_ = false;
    float q_prev_[14]{};
    float last_action_[14]{};
    float mean_[46]{};
    float std_[46]{};
    float w0_[512 * 46]{};
    float b0_[512]{};
    float w1_[256 * 512]{};
    float b1_[256]{};
    float w2_[128 * 256]{};
    float b2_[128]{};
    float w3_[14 * 128]{};
    float b3_[14]{};
};

}  // namespace wheel
