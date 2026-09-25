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
    std::cout << (ok ? "obs 布局自检通过\n" : "obs 布局自检失败\n");
    return ok;
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

    imu::Lsm6 imu;
    bool imu_ok = imu.open();
    std::cout << (imu_ok ? "IMU 已打开\n" : "IMU 未打开，gyro/gravity 保持 0\n");

    loco::Gamepad pad(cfg.gamepad);
    std::cout << (pad.open() ? "手柄已打开 " : "手柄未连接 ") << cfg.gamepad << "\n";

    std::unique_ptr<motor::Bus> bus;
    double measured[loco::kActionLen]{};
    double home[loco::kActionLen]{};
    try {
        bus = std::make_unique<motor::Bus>(cfg.can);
        for (const auto& j : cfg.joints) bus->add(j.can_id, cfg.motor_type, j.name);
        for (const auto& j : cfg.joints) bus->read_status(j.can_id);
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
            std::cout << "电机已进位置模式。Ctrl+C 会卸力。\n";
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
    const double period = 1.0 / cfg.hz;
    const int print_every = std::max(1, static_cast<int>(cfg.hz / std::max(cfg.print_hz, 0.1)));
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    int tick = 0;
    auto t0 = std::chrono::steady_clock::now();

    while (!g_stop) {
        float gyro[3]{};
        float grav[3]{0, 0, -1};
        if (imu_ok) {
            auto s = imu.read();
            if (s.ok) {
                std::memcpy(gyro, s.gyro, sizeof(gyro));
                std::memcpy(grav, s.gravity, sizeof(grav));
            }
        }
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
                }
            }
        }
        loco::Command cmd = pad.poll(cfg);
        float obs[loco::kObsLen];
        loco::build_obs(gyro, grav, pos, vel, home, last, cmd, obs);
        float action[loco::kActionLen]{};
        auto infer_t0 = std::chrono::steady_clock::now();
        bool finite = true;
        for (float v : obs)
            if (!std::isfinite(v)) finite = false;
        if (!finite || !engine->infer(obs, action)) {
            for (float& a : action) a = 0;
        } else {
            std::memcpy(last, action, sizeof(last));
        }
        double infer_us = std::chrono::duration<double, std::micro>(
                              std::chrono::steady_clock::now() - infer_t0)
                              .count();
        double alpha = 1;
        if (apply && cfg.ramp_s > 0) {
            double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            alpha = std::min(1.0, elapsed / cfg.ramp_s);
        }
        if (bus && apply) {
            for (int i = 0; i < loco::kActionLen; ++i) {
                float a = std::clamp(action[i], -1.f, 1.f);
                double target = home[i] + cfg.action_scale * a;
                double cmd_pos = measured[i] + alpha * (target - measured[i]);
                bus->command_position(cfg.joints[i].can_id, cmd_pos, cfg.vel_limit, cfg.torque_nm);
            }
        }
        if (tick % print_every == 0) {
            std::cout << "gyro " << gyro[0] << " " << gyro[1] << " " << gyro[2]
                      << "  gravity " << grav[0] << " " << grav[1] << " " << grav[2]
                      << "  cmd " << cmd.vx << " " << cmd.vy << " " << cmd.vyaw
                      << "  infer " << infer_us << " us  " << engine->name() << "\n";
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
        std::cout << "\n电机已卸力\n";
    }
    return 0;
}
