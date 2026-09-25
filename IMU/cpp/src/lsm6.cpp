#include "imu/lsm6.hpp"

extern "C" {
#include "i2c_gpio.h"
}

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
constexpr double kGyroRad = 0.070 * 3.141592653589793 / 180.0;

void rotate(float x, float y, float z, float out[3]) {
    out[0] = z;
    out[1] = y;
    out[2] = -x;
}

void normalize_neg(const float in[3], float out[3]) {
    float x = -in[0], y = -in[1], z = -in[2];
    float n = std::sqrt(x * x + y * y + z * z);
    if (n < 1e-9f) {
        out[0] = 0;
        out[1] = 0;
        out[2] = -1;
        return;
    }
    out[0] = x / n;
    out[1] = y / n;
    out[2] = z / n;
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

PolicySample Lsm6::read() {
    PolicySample s;
    uint8_t buf[12];
    if (!opened_ || !read_regs(0x22, buf, 12)) return s;
    float gx = s16(buf[0], buf[1]) * kGyroRad;
    float gy = s16(buf[2], buf[3]) * kGyroRad;
    float gz = s16(buf[4], buf[5]) * kGyroRad;
    float ax = s16(buf[6], buf[7]) * kAccLsb;
    float ay = s16(buf[8], buf[9]) * kAccLsb;
    float az = s16(buf[10], buf[11]) * kAccLsb;
    rotate(gx, gy, gz, s.gyro);
    float acc_t[3];
    rotate(ax, ay, az, acc_t);
    normalize_neg(acc_t, s.gravity);
    s.ok = true;
    return s;
}

}  // namespace imu
