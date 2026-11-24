#pragma once

#include <string>
#include <vector>

namespace wheel {

struct RollSense {
    float gyro[3]{};
    float grav[3]{0.f, 0.f, -1.f};
    float pos[16]{};
    float vel[16]{};
    float servo_yaw = 0.f;
    float servo_roll = 0.f;
    bool chord_rise = false;
    bool allow = false;
    bool other_busy = false;
};

struct RollGoal {
    bool active = false;
    float deg[16]{};
    float tq[16]{};
    float yaw = 0.f;
    float roll = 0.f;
    float motor_left = 0.f;
    float motor_right = 0.f;
    float cmd_vel = 6.f;
    bool say_start = false;
    bool say_ready = false;  // 初始姿态到位，开始推理
    bool say_stop = false;
    bool say_done = false;
    bool say_busy = false;
    bool say_refuse = false;
};

class Roll {
public:
    bool load(const std::string& path);
    bool load_csv(const std::string& path);
    bool ready() const { return ready_; }
    bool csv_mode() const { return csv_mode_; }
    bool approaching() const { return active_ && phase_ == PhaseApproach; }
    int frame_count() const { return static_cast<int>(frames_.size()); }
    int miss_frames() const { return miss_frames_; }
    int frame_index() const { return frame_i_; }
    const std::string& reason() const { return reason_; }
    const std::string& source() const { return source_; }
    void step(float dt, const RollSense& in, RollGoal& out);

private:
    enum Phase : int { PhaseIdle = 0, PhaseApproach = 1, PhaseRun = 2 };

    struct Frame {
        float t_s = 0.f;
        float act[14]{};
    };

    void emit_action(const float act[14], RollGoal& out) const;
    void emit_default_pose(RollGoal& out) const;
    bool default_pose_arrived(const RollSense& in) const;
    bool frame_arrived(const float act[14], const RollSense& in) const;

    bool ready_ = false;
    bool csv_mode_ = false;
    std::string reason_;
    std::string source_;
    bool active_ = false;
    int phase_ = PhaseIdle;
    bool have_prev_ = false;
    float elapsed_ = 0.f;
    float angle_ = 0.f;
    float hold_s_ = 0.f;
    float approach_s_ = 0.f;
    int frame_i_ = 0;
    int miss_frames_ = 0;
    float q_prev_[14]{};
    float last_action_[14]{};
    std::vector<Frame> frames_;
    float mean_[48]{};
    float std_[48]{};
    float w0_[512 * 48]{};
    float b0_[512]{};
    float w1_[256 * 512]{};
    float b1_[256]{};
    float w2_[128 * 256]{};
    float b2_[128]{};
    float w3_[14 * 128]{};
    float b3_[14]{};
};

}  // namespace wheel
