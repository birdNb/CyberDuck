#include "wheel/config.hpp"
#include "wheel/math.hpp"
#include "wheel/plant.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

namespace {

void usage() {
    std::cout
        << "用法: wheel_drive [选项]\n"
        << "  站立后手柄轮行。CAN 线程独占总线并先发两轮，轮子和腿在各自的实时线程里。\n"
        << "  --config 文件       姿态配置，默认 pose_config.yaml\n"
        << "  --iface can0\n"
        << "  --pose drive\n"
        << "  --vmax 10           按住 LT 的全速 rad/s，不按只有一半\n"
        << "  --turn 6 --spin 4   转弯和自转上限，同样按 LT 才到全额\n"
        << "  --accel 25          轮速斜坡 rad/s^2\n"
        << "  --speed-smooth 0.04 轮速低通时间常数秒\n"
        << "  --tqe 1             腿的力矩上限 N·m\n"
        << "  --hip-tqe 1         自平衡时髋 pitch 力矩上限\n"
        << "  --wheel-tqe 3       轮子追速力矩上限\n"
        << "  --height-rate 0.15  升降全行程速率 1/s\n"
        << "  --wheel-hz 200      轮子和 CAN 节拍\n"
        << "  --body-hz 100       腿、侧倾、平衡节拍\n"
        << "  --no-balance        不打开 IMU\n"
        << "  --balance-kp 1 --balance-kd 0.1 --balance-max 40\n"
        << "  --balance-flip      平衡方向反了时翻转\n"
        << "  --swap-wheels       交换左右轮角色\n"
        << "  --no-wait-y         扫描后立刻进入姿态\n"
        << "  --self-test         只检查换算、摇杆和姿态文件，不打开电机\n";
}

bool take_float(int& i, int argc, char** argv, float& dst) {
    if (i + 1 >= argc) return false;
    dst = std::strtof(argv[++i], nullptr);
    return true;
}

bool near(float a, float b) { return std::fabs(a - b) < 1e-3f; }

}  // namespace

namespace wheel {

int self_test(const std::string& config_path) {
    int fail = 0;
    auto expect = [&](bool ok, const char* name) {
        if (ok) return;
        std::cerr << "自检失败: " << name << "\n";
        ++fail;
    };
    expect(pos_to_int(0) == 0, "位置零点");
    expect(tqe_to_int(1.0) == 100, "1 N·m");
    expect(vel_to_int(0) == 0, "速度零点");
    expect(near(deadzone(0.12f), 0.f) && near(deadzone(1.f), 1.f), "死区");
    expect(near(smootherstep(0.f), 0.f) && near(smootherstep(1.f), 1.f) && near(smootherstep(0.5f), 0.5f),
           "平滑插值");
    expect(near(ramp(0.f, 10.f, 25.f, 0.2f), 5.f), "斜坡");
    expect(lowpass(0.05f, 0.f, 0.04f, 0.2f) == 0.f, "低通回零");
    auto fwd = wheel_targets(1.f, 0.f, 0.f, 10.f, 6.f, 4.f);
    expect(near(fwd.left, 10.f) && near(fwd.right, 10.f), "直行");
    auto pivot = wheel_targets(0.f, 1.f, 0.f, 10.f, 6.f, 4.f);
    expect(near(pivot.left, 0.f) && near(pivot.right, 6.f), "单轮转弯");
    auto half = wheel_targets(1.f, 0.f, 0.f, 5.f, 3.f, 2.f);
    expect(near(half.left, 5.f) && near(half.right, 5.f), "半速");
    expect(sticks_bogus(0.5f, -0.5f, 0.1f) && !sticks_bogus(0.2f, 0.f, 0.f), "错误手柄");
    Chase chase = wheel_chase(32000, 1.f);
    expect(chase.raw == static_cast<int16_t>(-31636), "追速回绕");
    expect(wheel_chase(1000, 0.f).raw == 1000, "零速保持当前角");
    expect(wheel_sign(1) == 1.f && wheel_sign(6) == -1.f, "轮子方向");

    YamlCfg cfg;
    std::string err;
    expect(load_yaml(config_path, cfg, err), err.c_str());
    auto drive = cfg.poses.find("drive");
    auto ext = cfg.poses.find("extended");
    auto lean = cfg.poses.find("lean_right");
    auto unload = cfg.poses.find("unload");
    expect(drive != cfg.poses.end() && drive->second.has(1), "drive 姿态");
    if (drive != cfg.poses.end()) expect(near(drive->second.deg[1], -32.83f), "drive 1 号角度");
    expect(ext != cfg.poses.end() && ext->second.has(2) && near(ext->second.deg[2], 0.f), "伸直姿态");
    expect(lean != cfg.poses.end() && lean->second.has(4) && near(lean->second.deg[4], 20.f), "右侧倾");
    expect(unload != cfg.poses.end() && unload->second.has(7) && near(unload->second.deg[7], 122.f), "卸力姿态");
    if (fail) {
        std::cerr << "自检未通过，" << fail << " 项\n";
        return 1;
    }
    std::cout << "轮行自检通过\n";
    return 0;
}

}  // namespace wheel

int main(int argc, char** argv) {
    wheel::Args args;
    bool test = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            usage();
            return 0;
        } else if (a == "--self-test") {
            test = true;
        } else if (a == "--config" && i + 1 < argc) {
            args.config = argv[++i];
        } else if (a == "--iface" && i + 1 < argc) {
            args.iface = argv[++i];
        } else if (a == "--pose" && i + 1 < argc) {
            args.pose = argv[++i];
        } else if (a == "--vmax" && i + 1 < argc) {
            take_float(i, argc, argv, args.vmax);
        } else if (a == "--turn" && i + 1 < argc) {
            take_float(i, argc, argv, args.turn);
        } else if (a == "--spin" && i + 1 < argc) {
            take_float(i, argc, argv, args.spin);
        } else if (a == "--accel" && i + 1 < argc) {
            take_float(i, argc, argv, args.accel);
        } else if (a == "--speed-smooth" && i + 1 < argc) {
            take_float(i, argc, argv, args.smooth);
        } else if (a == "--tqe" && i + 1 < argc) {
            take_float(i, argc, argv, args.torque);
        } else if (a == "--hip-tqe" && i + 1 < argc) {
            take_float(i, argc, argv, args.hip_torque);
        } else if (a == "--wheel-tqe" && i + 1 < argc) {
            take_float(i, argc, argv, args.wheel_torque);
        } else if (a == "--height-rate" && i + 1 < argc) {
            take_float(i, argc, argv, args.height_rate);
        } else if (a == "--balance-kp" && i + 1 < argc) {
            take_float(i, argc, argv, args.balance_kp);
        } else if (a == "--balance-kd" && i + 1 < argc) {
            take_float(i, argc, argv, args.balance_kd);
        } else if (a == "--balance-max" && i + 1 < argc) {
            take_float(i, argc, argv, args.balance_max);
        } else if (a == "--wheel-hz" && i + 1 < argc) {
            args.wheel_hz = std::atoi(argv[++i]);
        } else if (a == "--body-hz" && i + 1 < argc) {
            args.body_hz = std::atoi(argv[++i]);
        } else if (a == "--no-balance") {
            args.no_balance = true;
        } else if (a == "--balance-flip") {
            args.balance_flip = true;
        } else if (a == "--swap-wheels") {
            args.swap_wheels = true;
        } else if (a == "--no-wait-y") {
            args.no_wait_y = true;
        } else {
            std::cerr << "不认识的参数: " << a << "\n";
            usage();
            return 2;
        }
    }
    if (args.vmax <= 0.f || args.vmax > 25.f || args.accel <= 0.f || args.wheel_hz < 50 || args.wheel_hz > 500 ||
        args.body_hz < 20 || args.body_hz > 500 || args.torque <= 0.f || args.wheel_torque <= 0.f) {
        std::cerr << "速度、力矩或节拍超出允许范围\n";
        return 2;
    }
    if (test) return wheel::self_test(args.config);
    return wheel::run(args);
}
