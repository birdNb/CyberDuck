#pragma once

#include <string>

namespace wheel {

struct Args {
    std::string config = "/home/aidlux/cyberDuck/sport_ctrl/pose_contrl/pose_config.yaml";
    std::string iface = "can0";
    std::string pose = "drive";
    float vmax = 10.f;
    float turn = 6.f;
    float spin = 4.f;
    float accel = 25.f;
    float smooth = 0.04f;
    float torque = 1.f;
    float hip_torque = 1.f;
    float wheel_torque = 3.f;
    float height_rate = 0.15f;
    float balance_kp = 1.f;
    float balance_kd = 0.1f;
    float balance_max = 40.f;
    bool balance_flip = false;
    bool no_balance = false;
    bool no_wait_y = false;
    bool swap_wheels = false;
    int wheel_hz = 200;
    int body_hz = 100;
};

int run(const Args& args);
int self_test(const std::string& config_path);

}  // namespace wheel
