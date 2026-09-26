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

// 数组下标是策略顺序，见 htdk_policy_joint_order.md。can_id 是实机电机号。
// 右腿 1–5、左腿 6–10：踝、膝、髋 pitch、髋 roll、髋 yaw。
// 头颈：head pitch 11，neck pitch 12，head roll 13，head yaw 14。嘴是 15，不在这 14 维里。
struct JointSpec {
    const char* name;
    int can_id;
    double home;
    double scale;
    double lo;
    double hi;
    double kp;
    double tau;
    bool leg;
};

inline constexpr int kMouthCanId = 15;

inline constexpr JointSpec kJointSpec[kActionLen] = {
    {"l_hip_yaw", 10, 0.0, 0.25, -0.436, 0.176, 60.0, 0.5, true},
    {"l_hip_roll", 9, 0.0, 0.10, -0.384, 0.080, 60.0, 0.5, true},
    {"l_hip_pitch", 8, 0.175, 0.50, -1.571, 1.571, 60.0, 0.5, true},
    {"l_calf", 7, 0.0, 0.50, -1.571, 1.571, 60.0, 0.5, true},
    {"l_ankle", 6, -0.175, 0.40, -1.571, 1.571, 60.0, 0.5, true},
    {"neck_pitch", 12, 0.0, 0.40, -1.200, 1.571, 20.0, 0.5, false},
    {"head_pitch", 11, 0.0, 0.40, -1.240, 1.571, 20.0, 0.5, false},
    {"head_yaw", 14, 0.0, 0.50, -2.967, 2.967, 8.0, 0.3, false},
    {"head_roll", 13, 0.0, 0.20, -0.436, 0.436, 8.0, 0.3, false},
    {"r_hip_yaw", 5, 0.0, 0.25, -0.176, 0.436, 60.0, 0.5, true},
    {"r_hip_roll", 4, 0.0, 0.10, -0.080, 0.384, 60.0, 0.5, true},
    {"r_hip_pitch", 3, 0.175, 0.50, -1.571, 1.571, 60.0, 0.5, true},
    {"r_calf", 2, 0.0, 0.50, -1.571, 1.571, 60.0, 0.5, true},
    {"r_ankle", 1, -0.175, 0.40, -1.571, 1.571, 60.0, 0.5, true},
};

enum class MotorGoal { Home, Hold, Track };

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
    std::string imu_mount = "config/imu_mount.json";
    double tau_max_leg = 0.5;
    double cmd_scale = 0.25;
    double vel_limit = 1.0;
    double max_vx = 0.4;
    double max_vx_back = 0.4;
    double max_vy = 0.3;
    double max_vyaw = 1.0;
    double deadzone = 0.12;
    double print_hz = 2.0;
    double ramp_s = 2.0;
    int motor_timeout_ms = 100;
    std::array<Joint, kActionLen> joints{};
};

Config load_config(const std::string& path);
bool load_imu_mount(const std::string& path, float R[9]);
// 腿：q_des = clamp(HOME + scale * action)。颈和头目标锁在 HOME。
// |q_des - q| 不超过当前力矩 / kp。Hold 时停在当前角度。
void motor_targets(const float action[kActionLen], const double q[kActionLen],
                   const double home[kActionLen], MotorGoal goal, double ramp,
                   double tau_max_leg, double q_des[kActionLen], double tau[kActionLen]);
void build_obs(const float gyro[3], const float gravity[3],
               const double pos[kActionLen], const double vel[kActionLen],
               const double home[kActionLen], const float last_action[kActionLen],
               const Command& cmd, float out[kObsLen]);

}  // namespace loco
