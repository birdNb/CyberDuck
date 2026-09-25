#pragma once

#include <cstdint>

namespace imu {

struct PolicySample {
    float gyro[3];     // trunk, rad/s
    float gravity[3];  // trunk, 单位向量
    bool ok = false;
};

class Lsm6 {
public:
    bool open();
    void close();
    PolicySample read();
    ~Lsm6() { close(); }

private:
    bool opened_ = false;
    int addr_ = 0x6B;
    bool write_reg(int reg, int val);
    bool read_regs(int reg, uint8_t* out, int n);
};

}  // namespace imu
