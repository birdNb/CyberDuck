#include "imu/lsm6.hpp"

extern "C" {
#include "i2c_gpio.h"
}

#include <chrono>
#include <cmath>
#include <cstdint>
#include <unistd.h>

namespace imu {
namespace {
constexpr int kChip = 4;
constexpr int kSda = 24;
constexpr int kScl = 25;
constexpr int kWho = 0x0F;
constexpr int kWhoVal = 0x70;
constexpr double kAccLsb = 0.000244;
constexpr float kG = 9.80665f;
constexpr double kGyroRad = 0.070 * 3.141592653589793 / 180.0;
constexpr double kGravityTau = 0.5;

void mul_mount(const float* R, const float v[3], float out[3]) {
    out[0] = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
    out[1] = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
    out[2] = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
}

void normalize(float v[3], float fallback_z) {
    float n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n < 1e-6f) {
        v[0] = 0;
        v[1] = 0;
        v[2] = fallback_z;
        return;
    }
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
}

int16_t s16(uint8_t lo, uint8_t hi) {
    int v = lo | (hi << 8);
    return v & 0x8000 ? static_cast<int16_t>(v - 0x10000) : static_cast<int16_t>(v);
}
}  // namespace

bool Lsm6::write_reg(int reg, int val) {
    uint8_t w[2] = {static_cast<uint8_t>(reg), static_cast<uint8_t>(val)};
    return i2c_gpio_xfer(static_cast<uint8_t>(addr_), w, 2, nullptr, 0) == 0;
}

bool Lsm6::read_regs(int reg, uint8_t* out, int n) {
    uint8_t w = static_cast<uint8_t>(reg);
    return i2c_gpio_xfer(static_cast<uint8_t>(addr_), &w, 1, out, n) == 0;
}

bool Lsm6::open() {
    if (i2c_gpio_open(kChip, kSda, kScl, 0) != 0) return false;
    opened_ = true;
    uint8_t who = 0;
    bool found = false;
    for (int addr = 0x6B; addr >= 0x6A; --addr) {
        addr_ = addr;
        if (read_regs(kWho, &who, 1) && who == kWhoVal) {
            found = true;
            break;
        }
    }
    if (!found) {
        close();
        return false;
    }
    write_reg(0x12, 1);
    usleep(10000);
    write_reg(0x12, 1 << 7);
    usleep(20000);
    write_reg(0x12, (1 << 6) | (1 << 2));
    write_reg(0x18, 0x02);
    write_reg(0x10, (0x03 << 4) | 0x07);
    write_reg(0x11, (0x03 << 4) | 0x07);
    write_reg(0x15, 0x04 | 0x01);
    write_reg(0x17, 0x02);
    usleep(20000);
    return true;
}

void Lsm6::close() {
    if (opened_) i2c_gpio_close();
    opened_ = false;
}

void Lsm6::set_mount(const float R[9]) {
    for (int i = 0; i < 9; ++i) mount_[i] = R[i];
    grav_init_ = false;
}

PolicySample Lsm6::read() {
    PolicySample s;
    uint8_t buf[12];
    if (!opened_ || !read_regs(0x22, buf, 12)) return s;
    float gyro_s[3] = {s16(buf[0], buf[1]) * static_cast<float>(kGyroRad),
                       s16(buf[2], buf[3]) * static_cast<float>(kGyroRad),
                       s16(buf[4], buf[5]) * static_cast<float>(kGyroRad)};
    float acc_s[3] = {s16(buf[6], buf[7]) * static_cast<float>(kAccLsb) * kG,
                      s16(buf[8], buf[9]) * static_cast<float>(kAccLsb) * kG,
                      s16(buf[10], buf[11]) * static_cast<float>(kAccLsb) * kG};
    mul_mount(mount_, gyro_s, s.gyro);
    float acc_b[3];
    mul_mount(mount_, acc_s, acc_b);

    float measured[3] = {-acc_b[0], -acc_b[1], -acc_b[2]};
    normalize(measured, -1.f);
    double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    double dt = last_read_s_ < 0 ? 0 : now - last_read_s_;
    last_read_s_ = now;
    if (!grav_init_ || dt > 0.2) {
        grav_[0] = measured[0];
        grav_[1] = measured[1];
        grav_[2] = measured[2];
        grav_init_ = true;
    } else if (dt > 0) {
        float wx = s.gyro[0], wy = s.gyro[1], wz = s.gyro[2];
        float gx = grav_[0], gy = grav_[1], gz = grav_[2];
        grav_[0] -= static_cast<float>(dt) * (wy * gz - wz * gy);
        grav_[1] -= static_cast<float>(dt) * (wz * gx - wx * gz);
        grav_[2] -= static_cast<float>(dt) * (wx * gy - wy * gx);
        normalize(grav_, -1.f);
        float an = std::sqrt(acc_b[0] * acc_b[0] + acc_b[1] * acc_b[1] + acc_b[2] * acc_b[2]);
        if (an > 7.f && an < 13.f) {
            float a = static_cast<float>(1.0 - std::exp(-dt / kGravityTau));
            grav_[0] = (1.f - a) * grav_[0] + a * measured[0];
            grav_[1] = (1.f - a) * grav_[1] + a * measured[1];
            grav_[2] = (1.f - a) * grav_[2] + a * measured[2];
            normalize(grav_, -1.f);
        }
    }
    s.gravity[0] = grav_[0];
    s.gravity[1] = grav_[1];
    s.gravity[2] = grav_[2];
    s.ok = true;
    return s;
}

}  // namespace imu
