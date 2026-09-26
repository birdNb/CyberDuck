#include "locomotion/layout.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>

namespace loco {

Config load_config(const std::string& path) {
    Config cfg;
    for (int i = 0; i < kActionLen; ++i) {
        cfg.joints[i].name = kJointSpec[i].name;
        cfg.joints[i].can_id = kJointSpec[i].can_id;
        cfg.joints[i].home = kJointSpec[i].home;
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
        else if (key == "imu_mount") ss >> cfg.imu_mount;
        else if (key == "tau_max_leg") ss >> cfg.tau_max_leg;
        else if (key == "cmd_scale") ss >> cfg.cmd_scale;
        else if (key == "vel_limit") ss >> cfg.vel_limit;
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
    cfg.tau_max_leg = std::clamp(cfg.tau_max_leg, 0.0, 3.7);
    cfg.cmd_scale = std::clamp(cfg.cmd_scale, 0.0, 1.0);
    return cfg;
}

bool load_imu_mount(const std::string& path, float R[9]) {
    std::ifstream in(path);
    if (!in) return false;
    std::string text((std::istreambuf_iterator<char>(in)), {});
    auto key = text.find("\"R\"");
    if (key == std::string::npos) return false;
    auto lb = text.find('[', key);
    if (lb == std::string::npos) return false;
    const char* p = text.c_str() + lb;
    for (int i = 0; i < 9; ++i) {
        while (*p && !((*p >= '0' && *p <= '9') || *p == '-' || *p == '+')) ++p;
        if (!*p) return false;
        char* end = nullptr;
        R[i] = std::strtof(p, &end);
        if (end == p) return false;
        p = end;
    }
    float det = R[0] * (R[4] * R[8] - R[5] * R[7]) - R[1] * (R[3] * R[8] - R[5] * R[6]) +
                R[2] * (R[3] * R[7] - R[4] * R[6]);
    return det > 0.5f && det < 1.5f;
}

void motor_targets(const float action[kActionLen], const double q[kActionLen],
                   const double home[kActionLen], MotorGoal goal, double ramp,
                   double tau_max_leg, double q_des[kActionLen], double tau[kActionLen]) {
    ramp = std::clamp(ramp, 0.0, 1.0);
    tau_max_leg = std::clamp(tau_max_leg, 0.0, 3.7);
    for (int i = 0; i < kActionLen; ++i) {
        const auto& spec = kJointSpec[i];
        double tau_full = spec.leg ? tau_max_leg : spec.tau;
        tau[i] = tau_full * ramp;
        if (goal == MotorGoal::Hold || tau[i] <= 0) {
            q_des[i] = q[i];
            continue;
        }
        double target = home[i];
        if (goal == MotorGoal::Track && spec.leg) {
            target = home[i] + spec.scale * static_cast<double>(action[i]);
        }
        target = std::clamp(target, spec.lo, spec.hi);
        double max_err = tau[i] / spec.kp;
        q_des[i] = std::clamp(target, q[i] - max_err, q[i] + max_err);
        q_des[i] = std::clamp(q_des[i], spec.lo, spec.hi);
    }
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
