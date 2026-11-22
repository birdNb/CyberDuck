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
    // 行主序。R @ v_sensor = v_base_link。未调用时为单位阵。
    void set_mount(const float R[9]);
    PolicySample read();
    // 这一拍的陀螺和加速度方向。不改 read() 里的重力滤波状态。
    PolicySample read_raw();
    ~Lsm6() { close(); }

private:
    bool opened_ = false;
    int addr_ = 0x6B;
    float mount_[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    bool grav_init_ = false;
    float grav_[3] = {0, 0, -1};
    double last_read_s_ = -1;
    bool write_reg(int reg, int val);
    bool read_regs(int reg, uint8_t* out, int n);
};

}  // namespace imu
