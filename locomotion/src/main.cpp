#include "imu/lsm6.hpp"
#include "locomotion/engine.hpp"
#include "locomotion/gamepad.hpp"
#include "locomotion/layout.hpp"
#include "motor/bus.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <time.h>

namespace {
volatile std::sig_atomic_t g_stop = 0;
void on_stop(int) { g_stop = 1; }

bool self_test() {
    float gyro[3] = {1, 2, 3};
    float grav[3] = {4, 5, 6};
    double pos[loco::kActionLen]{};
    double vel[loco::kActionLen]{};
    double home[loco::kActionLen]{};
    float last[loco::kActionLen]{};
    pos[0] = 0.25;
    home[0] = 0;
    last[0] = -0.5f;
    last[loco::kActionLen - 1] = 0.75f;
    loco::Command cmd;
    cmd.vx = 0.1f;
    cmd.vy = 0.2f;
    cmd.vyaw = 0.3f;
    float obs[loco::kObsLen];
    loco::build_obs(gyro, grav, pos, vel, home, last, cmd, obs);
    bool ok = obs[0] == 1 && obs[3] == 4 && std::fabs(obs[6] - 0.25f) < 1e-6f && obs[20] == 0 &&
              obs[34] == -0.5f && obs[47] == 0.75f && obs[48] == 0.1f && obs[55] == 0 && obs[60] == 0;
    bool motor_ok = true;
    float action[loco::kActionLen]{};
    double q[loco::kActionLen]{};
    for (int i = 0; i < loco::kActionLen; ++i) {
        home[i] = loco::kJointSpec[i].home;
        q[i] = home[i];
    }
    action[0] = 1.f;
    action[1] = 1.f;
    action[2] = 1.f;
    action[5] = 1.f;
    action[7] = 1.f;
    double q_des[loco::kActionLen]{};
    double tau[loco::kActionLen]{};
    loco::motor_targets(action, q, home, loco::MotorGoal::Track, 1.0, 0.5, q_des, tau);
    auto near = [](double a, double b) { return std::fabs(a - b) < 1e-6; };
    motor_ok &= near(q_des[0], 0.5 / 60.0);
    motor_ok &= near(q_des[2], 0.68 + 0.5 / 60.0);
    motor_ok &= near(q_des[5], 0.52);
    motor_ok &= near(tau[0], 0.5) && near(tau[7], 0.3);
    q[1] = 0.079;
    loco::motor_targets(action, q, home, loco::MotorGoal::Track, 1.0, 0.5, q_des, tau);
    motor_ok &= near(q_des[1], 0.080);
    loco::motor_targets(action, q, home, loco::MotorGoal::Hold, 1.0, 0.5, q_des, tau);
    motor_ok &= near(q_des[1], 0.079);
    bool map_ok = loco::kJointSpec[0].can_id == 10 && loco::kJointSpec[4].can_id == 6 &&
                  loco::kJointSpec[9].can_id == 5 && loco::kJointSpec[13].can_id == 1 &&
                  loco::kJointSpec[5].can_id == 12 && loco::kJointSpec[6].can_id == 11 &&
                  loco::kJointSpec[7].can_id == 14 && loco::kJointSpec[8].can_id == 13 &&
                  loco::kMouthCanId == 15;
    std::cout << (ok ? "obs 布局自检通过\n" : "obs 布局自检失败\n");
    std::cout << (map_ok ? "电机号映射自检通过\n" : "电机号映射自检失败\n");
    std::cout << (motor_ok ? "电机目标自检通过\n" : "电机目标自检失败\n");
    return ok && motor_ok && map_ok;
}
}  // namespace

int main(int argc, char** argv) {
    std::string config = "config/locomotion.cfg";
    bool apply = false;
    bool test = false;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--apply")) apply = true;
        else if (!std::strcmp(argv[i], "--self-test")) test = true;
        else if (!std::strcmp(argv[i], "--config") && i + 1 < argc) config = argv[++i];
        else if (!std::strcmp(argv[i], "--help")) {
            std::cout << "用法: duck_locomotion [--config 文件] [--apply] [--self-test]\n"
                         "默认只闭环观测和推理，不给电机上力矩。--apply 才发位置指令。\n";
            return 0;
        }
    }
    if (test) return self_test() ? 0 : 1;

    loco::Config cfg = loco::load_config(config);
    std::string note;
    auto engine = loco::make_engine(cfg, note);
    std::cout << "引擎 " << engine->name() << "  " << note << "\n";

    float mount[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    bool mount_ok = loco::load_imu_mount(cfg.imu_mount, mount);
    if (!mount_ok && apply) {
        std::cerr << "没有可用的 IMU 安装旋转 " << cfg.imu_mount << "，拒绝 --apply\n";
        return 1;
    }
    imu::Lsm6 imu;
    if (mount_ok) imu.set_mount(mount);
    bool imu_ok = imu.open();
    std::cout << (imu_ok ? "IMU 已打开\n" : "IMU 未打开，gyro/gravity 保持 0\n");
    std::cout << (mount_ok ? "IMU 安装旋转已加载 " : "IMU 安装旋转未加载 ") << cfg.imu_mount << "\n";
    std::cout << "腿力矩上限 " << cfg.tau_max_leg << " N·m，指令缩放 " << cfg.cmd_scale
              << "。颈、头和嘴锁在 0，不跟策略。\n";
    for (int i = 0; i < loco::kActionLen; ++i) {
        std::cout << "  [" << i << "] " << cfg.joints[i].name << " -> 电机 " << cfg.joints[i].can_id
                  << "\n";
    }
    std::cout << "  嘴 -> 电机 " << loco::kMouthCanId << "\n";

    loco::Gamepad pad(cfg.gamepad);
    std::cout << (pad.open() ? "手柄已打开 " : "手柄未连接 ") << cfg.gamepad << "\n";

    std::unique_ptr<motor::Bus> bus;
    double measured[loco::kActionLen]{};
    double home[loco::kActionLen]{};
    try {
        bus = std::make_unique<motor::Bus>(cfg.can);
        for (const auto& j : cfg.joints) bus->add(j.can_id, cfg.motor_type, j.name);
        bus->add(loco::kMouthCanId, cfg.motor_type, "mouth");
        for (const auto& j : cfg.joints) bus->read_status(j.can_id);
        bus->read_status(loco::kMouthCanId);
        bus->poll(0.05);
        for (int i = 0; i < loco::kActionLen; ++i) {
            home[i] = cfg.joints[i].home;
            if (const auto* s = bus->state(cfg.joints[i].can_id); s && s->fresh) measured[i] = s->position;
            else measured[i] = home[i];
        }
        if (apply) {
            for (const auto& j : cfg.joints) {
                bus->set_timeout_ms(j.can_id, cfg.motor_timeout_ms);
                bus->enable_position(j.can_id);
            }
            bus->set_timeout_ms(loco::kMouthCanId, cfg.motor_timeout_ms);
            bus->enable_position(loco::kMouthCanId);
            std::cout << "电机已进位置模式。颈、头和嘴锁在 0。Ctrl+C 会卸力。\n";
        } else {
            std::cout << "干跑：不使能电机。确认关节映射后再加 --apply。\n";
        }
    } catch (const std::exception& e) {
        std::cout << "CAN 未接入（" << e.what() << "），关节读数保持 0\n";
        bus.reset();
        for (int i = 0; i < loco::kActionLen; ++i) {
            home[i] = cfg.joints[i].home;
            measured[i] = 0;
        }
    }

    std::signal(SIGINT, on_stop);
    std::signal(SIGTERM, on_stop);

    float last[loco::kActionLen]{};
    double q_dot_prev[loco::kActionLen]{};
    const double period = 1.0 / cfg.hz;
    const int print_every = std::max(1, static_cast<int>(cfg.hz / std::max(cfg.print_hz, 0.1)));
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    int tick = 0;
    int late_streak = 0;
    auto t0 = std::chrono::steady_clock::now();
    auto prev_tick = t0;
    auto grav_bad_since = t0;
    bool printed_joints = false;

    while (!g_stop) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - prev_tick).count();
        prev_tick = now;
        bool kill_torque = tick > 0 && dt > 0.1;
        if (tick > 0 && dt > 0.03) ++late_streak;
        else late_streak = 0;
        bool late = late_streak >= 2;

        float gyro[3]{};
        float grav[3]{0, 0, -1};
        if (imu_ok) {
            auto s = imu.read();
            if (s.ok) {
                std::memcpy(gyro, s.gyro, sizeof(gyro));
                std::memcpy(grav, s.gravity, sizeof(grav));
            }
        }
        double upright = grav[0] * grav[0] + grav[1] * grav[1] + (grav[2] + 1.0) * (grav[2] + 1.0);
        bool grav_now = upright < 0.0194;  // 与 (0,0,-1) 夹角约 8°
        if (!grav_now) grav_bad_since = now;
        bool grav_ok = grav_now && std::chrono::duration<double>(now - grav_bad_since).count() >= 0.5;

        double pos[loco::kActionLen];
        double vel[loco::kActionLen]{};
        std::memcpy(pos, measured, sizeof(pos));
        if (bus) {
            for (const auto& j : cfg.joints) bus->read_status(j.can_id);
            bus->poll(0.004);
            for (int i = 0; i < loco::kActionLen; ++i) {
                if (const auto* s = bus->state(cfg.joints[i].can_id); s && s->fresh) {
                    pos[i] = s->position;
                    vel[i] = s->velocity;
                    measured[i] = s->position;
                }
            }
        }
        if (!printed_joints) {
            std::cout << "q-home";
            for (int i = 0; i < loco::kActionLen; ++i) std::cout << " " << (pos[i] - home[i]);
            std::cout << "\n";
            printed_joints = true;
        }
        bool near_home = true;
        for (int i = 0; i < loco::kActionLen; ++i) {
            if (std::fabs(pos[i] - home[i]) > 0.35) near_home = false;
        }
        double elapsed = std::chrono::duration<double>(now - t0).count();
        double ramp = 0;
        if (apply && cfg.ramp_s > 0) ramp = std::min(1.0, elapsed / cfg.ramp_s);
        else if (apply) ramp = 1;
        bool ramp_done = !apply || ramp >= 1.0;
        loco::MotorGoal goal = loco::MotorGoal::Hold;
        if (apply && (kill_torque || late)) goal = loco::MotorGoal::Hold;
        else if (apply && !ramp_done) goal = loco::MotorGoal::Home;
        else if (apply && mount_ok && near_home && grav_ok) goal = loco::MotorGoal::Track;

        loco::Command cmd = pad.poll(cfg);
        float obs[loco::kObsLen];
        loco::build_obs(gyro, grav, pos, q_dot_prev, home, last, cmd, obs);
        float action[loco::kActionLen]{};
        auto infer_t0 = std::chrono::steady_clock::now();
        bool finite = true;
        for (float v : obs)
            if (!std::isfinite(v)) finite = false;
        double infer_us = 0;
        if (goal == loco::MotorGoal::Track) {
            if (!finite || !engine->infer(obs, action)) {
                for (float& a : action) a = 0;
                goal = loco::MotorGoal::Hold;
            } else {
                std::memcpy(last, action, sizeof(last));
            }
            infer_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - infer_t0)
                           .count();
        }
        double q_des[loco::kActionLen];
        double tau[loco::kActionLen];
        loco::motor_targets(action, pos, home, goal, kill_torque ? 0.0 : ramp, cfg.tau_max_leg, q_des,
                            tau);
        if (bus && apply) {
            for (int i = 0; i < loco::kActionLen; ++i) {
                bus->command_position(cfg.joints[i].can_id, q_des[i], cfg.vel_limit, tau[i]);
            }
            double mouth_tau = kill_torque ? 0.0 : 0.3 * ramp;
            bus->command_position(loco::kMouthCanId, 0.0, cfg.vel_limit, mouth_tau);
        }
        std::memcpy(q_dot_prev, vel, sizeof(q_dot_prev));
        if (tick % print_every == 0) {
            const char* mode = goal == loco::MotorGoal::Track ? "track" : goal == loco::MotorGoal::Home ? "home" : "hold";
            std::cout << mode << "  gyro " << gyro[0] << " " << gyro[1] << " " << gyro[2]
                      << "  gravity " << grav[0] << " " << grav[1] << " " << grav[2]
                      << "  cmd " << cmd.vx << " " << cmd.vy << " " << cmd.vyaw
                      << "  tau0 " << tau[0] << "  infer " << infer_us << " us\n";
        }
        ++tick;
        next.tv_nsec += static_cast<long>(period * 1e9);
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }

    if (bus && apply) {
        for (const auto& j : cfg.joints) bus->disable(j.can_id);
        bus->disable(loco::kMouthCanId);
        std::cout << "\n电机已卸力\n";
    }
    return 0;
}
