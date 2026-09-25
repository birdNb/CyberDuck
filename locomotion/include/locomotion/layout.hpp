#pragma once

#include <array>
#include <string>

namespace loco {

constexpr int kObsLen = 61;
constexpr int kActionLen = 14;
constexpr int kCommandLen = 13;
constexpr double kHz = 50.0;

struct Joint {
    std::string name;
    int can_id = 0;
    double home = 0;
};

inline const std::array<const char*, kActionLen> kJointNames = {
    "left_hip_yaw",  "left_hip_roll",  "left_hip_pitch", "left_knee",    "left_ankle",
    "neck_pitch",    "head_pitch",     "head_yaw",       "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee",  "right_ankle",
};

inline const std::array<double, kActionLen> kMicroduckHome = {
    0.0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530,
};

struct Command {
    float vx = 0;
    float vy = 0;
    float vyaw = 0;
};

struct Config {
    double hz = kHz;
    std::string can = "can0";
    std::string engine = "auto";
    std::string model;
    std::string ort;
    std::string qnn_backend = "libQnnHtp.so";
    std::string gamepad = "/dev/input/js0";
    std::string motor_type = "4538_19";
    double action_scale = 0.25;
    double vel_limit = 1.0;
    double torque_nm = 1.2;
    double max_vx = 0.5;
    double max_vx_back = 0.3;
    double max_vy = 0.3;
    double max_vyaw = 0.8;
    double deadzone = 0.12;
    double print_hz = 2.0;
    double ramp_s = 2.0;
    int motor_timeout_ms = 100;
    std::array<Joint, kActionLen> joints{};
};

Config load_config(const std::string& path);
void build_obs(const float gyro[3], const float gravity[3],
               const double pos[kActionLen], const double vel[kActionLen],
               const double home[kActionLen], const float last_action[kActionLen],
               const Command& cmd, float out[kObsLen]);

}  // namespace loco
