#include "locomotion/layout.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace loco {

Config load_config(const std::string& path) {
    Config cfg;
    for (int i = 0; i < kActionLen; ++i) {
        cfg.joints[i].name = kJointNames[i];
        cfg.joints[i].can_id = i + 1;
        cfg.joints[i].home = kMicroduckHome[i];
    }
    std::ifstream in(path);
    if (!in) throw std::runtime_error("打不开配置 " + path);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string key;
        ss >> key;
        if (key == "hz") ss >> cfg.hz;
        else if (key == "can") ss >> cfg.can;
        else if (key == "engine") ss >> cfg.engine;
        else if (key == "model") ss >> cfg.model;
        else if (key == "ort") ss >> cfg.ort;
        else if (key == "qnn_backend") ss >> cfg.qnn_backend;
        else if (key == "gamepad") ss >> cfg.gamepad;
        else if (key == "motor_type") ss >> cfg.motor_type;
        else if (key == "action_scale") ss >> cfg.action_scale;
        else if (key == "vel_limit") ss >> cfg.vel_limit;
        else if (key == "torque_nm") ss >> cfg.torque_nm;
        else if (key == "max_vx") ss >> cfg.max_vx;
        else if (key == "max_vx_back") ss >> cfg.max_vx_back;
        else if (key == "max_vy") ss >> cfg.max_vy;
        else if (key == "max_vyaw") ss >> cfg.max_vyaw;
        else if (key == "deadzone") ss >> cfg.deadzone;
        else if (key == "print_hz") ss >> cfg.print_hz;
        else if (key == "ramp_s") ss >> cfg.ramp_s;
        else if (key == "motor_timeout_ms") ss >> cfg.motor_timeout_ms;
        else if (key == "joint") {
            std::string name;
            int id = 0;
            double home = 0;
            ss >> name >> id >> home;
            for (auto& j : cfg.joints) {
                if (j.name == name) {
                    j.can_id = id;
                    j.home = home;
                }
            }
        }
    }
    return cfg;
}

void build_obs(const float gyro[3], const float gravity[3],
               const double pos[kActionLen], const double vel[kActionLen],
               const double home[kActionLen], const float last_action[kActionLen],
               const Command& cmd, float out[kObsLen]) {
    int i = 0;
    for (int k = 0; k < 3; ++k) out[i++] = gyro[k];
    for (int k = 0; k < 3; ++k) out[i++] = gravity[k];
    for (int k = 0; k < kActionLen; ++k) out[i++] = static_cast<float>(pos[k] - home[k]);
    for (int k = 0; k < kActionLen; ++k) out[i++] = static_cast<float>(vel[k]);
    for (int k = 0; k < kActionLen; ++k) out[i++] = last_action[k];
    out[i++] = cmd.vx;
    out[i++] = cmd.vy;
    out[i++] = cmd.vyaw;
    for (int k = 0; k < 4; ++k) out[i++] = 0;
    out[i++] = 0;
    out[i++] = 0;
    out[i++] = 0;
    out[i++] = 0;
    out[i++] = 0;
    out[i++] = 0;
}

}  // namespace loco
