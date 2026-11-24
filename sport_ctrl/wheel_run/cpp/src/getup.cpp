#include "wheel/getup.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>

namespace wheel {
namespace {

constexpr float kFallenGz = -0.7f;
constexpr float kFallenHoldS = 0.3f;
constexpr float kNormEps = 1e-2f;
constexpr float kWheelLimit = 2.f;
constexpr uint32_t kMagic = 0x50554443u;

struct Joint {
    int can;
    bool servo;
    float lo;
    float hi;
    float latent;
    float sign;
};

constexpr Joint kJoints[12] = {
    {5, false, -0.436f, 0.436f, 0.f, -1.f},
    {4, false, -0.384f, 0.384f, 0.f, -1.f},
    {3, false, -1.571f, 1.571f, 0.f, -1.f},
    {2, false, -1.571f, 1.571f, 0.f, 1.f},
    {10, false, -0.436f, 0.436f, 0.f, -1.f},
    {9, false, -0.384f, 0.384f, 0.f, 1.f},
    {8, false, -1.571f, 1.571f, 0.f, 1.f},
    {7, false, -1.571f, 1.571f, 0.f, -1.f},
    // 12 实机方向和行走策略表相反。11 保持原方向。
    {12, false, -1.571f, 1.877f, -0.08898119628429413f, -1.f},
    {11, false, -1.571f, 1.571f, 0.f, 1.f},
    {13, true, -2.967f, 2.967f, 0.f, 1.f},
    {14, true, -0.208f, 0.208f, 0.f, 1.f},
};

constexpr int kWheelCan[2] = {1, 6};
constexpr float kWheelSign[2] = {-1.f, 1.f};

float artanh(float x) {
    if (x > 0.999999f) x = 0.999999f;
    if (x < -0.999999f) x = -0.999999f;
    return 0.5f * std::log((1.f + x) / (1.f - x));
}

float encode_pos(float q, float lo, float hi, float latent) {
    float mid = 0.5f * (lo + hi);
    float half = 0.5f * (hi - lo);
    if (half < 1e-6f) return 0.f;
    return artanh((q - mid) / half) - latent;
}

float decode_pos(float action, float lo, float hi, float latent) {
    float mid = 0.5f * (lo + hi);
    float half = 0.5f * (hi - lo);
    float q = mid + half * std::tanh(action + latent);
    if (q < lo) q = lo;
    if (q > hi) q = hi;
    return q;
}

void linear(const float* w, const float* b, const float* x, float* y, int out, int in, bool elu) {
    for (int r = 0; r < out; ++r) {
        const float* row = w + static_cast<size_t>(r) * in;
        float sum = b[r];
        for (int c = 0; c < in; ++c) sum += row[c] * x[c];
        if (elu) sum = sum > 0.f ? sum : std::expm1(sum);
        y[r] = sum;
    }
}

bool read_exact(std::ifstream& in, void* dst, size_t n) {
    in.read(reinterpret_cast<char*>(dst), static_cast<std::streamsize>(n));
    return static_cast<size_t>(in.gcount()) == n;
}

}  // namespace

bool Getup::load(const std::string& path) {
    ready_ = false;
    reason_.clear();
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        reason_ = "权重文件打不开";
        return false;
    }
    uint32_t magic = 0, obs = 0, h0 = 0, h1 = 0, h2 = 0, act = 0;
    if (!read_exact(in, &magic, 4) || !read_exact(in, &obs, 4) || !read_exact(in, &h0, 4) || !read_exact(in, &h1, 4) ||
        !read_exact(in, &h2, 4) || !read_exact(in, &act, 4)) {
        reason_ = "权重头不完整";
        return false;
    }
    if (magic != kMagic || obs != 46 || h0 != 512 || h1 != 256 || h2 != 128 || act != 14) {
        reason_ = "权重尺寸和起身策略不一致";
        return false;
    }
    if (!read_exact(in, mean_, sizeof(mean_)) || !read_exact(in, std_, sizeof(std_)) || !read_exact(in, w0_, sizeof(w0_)) ||
        !read_exact(in, b0_, sizeof(b0_)) || !read_exact(in, w1_, sizeof(w1_)) || !read_exact(in, b1_, sizeof(b1_)) ||
        !read_exact(in, w2_, sizeof(w2_)) || !read_exact(in, b2_, sizeof(b2_)) || !read_exact(in, w3_, sizeof(w3_)) ||
        !read_exact(in, b3_, sizeof(b3_))) {
        reason_ = "权重数据不完整";
        return false;
    }
    ready_ = true;
    return true;
}

void Getup::step(float dt, const GetupSense& in, GetupGoal& out) {
    out = {};
    out.active = active_;
    if (!ready_) return;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;
    float gz = in.grav[2];
    if (gz > kFallenGz) {
        fallen_for_ += dt;
        if (fallen_for_ > kFallenHoldS) fallen_for_ = kFallenHoldS;
    } else {
        fallen_for_ = 0.f;
        fallen_latched_ = false;
    }
    bool fallen = fallen_for_ >= kFallenHoldS;
    if (fallen && !fallen_latched_ && !active_) {
        fallen_latched_ = true;
        out.say_fall = true;
    }
    if (active_ && in.lb_rise) {
        active_ = false;
        out.say_stop = true;
    } else if (!active_ && in.lb_rise) {
        if (!in.allow || !fallen) {
            out.say_refuse = true;
        } else {
            active_ = true;
            out.say_start = true;
        }
    }
    if (!in.allow && active_) {
        active_ = false;
        out.say_stop = true;
    }

    float q[14]{};
    float qd[14]{};
    for (int i = 0; i < 12; ++i) {
        const Joint& j = kJoints[i];
        float motor = j.servo ? (j.can == 13 ? in.servo_yaw : in.servo_roll) : in.pos[j.can];
        if (j.servo) motor = motor * 0.0174532925f;
        q[i] = j.sign * motor;
    }
    for (int i = 0; i < 2; ++i) q[12 + i] = kWheelSign[i] * in.pos[kWheelCan[i]];
    if (!have_prev_) {
        for (int i = 0; i < 14; ++i) q_prev_[i] = q[i];
        have_prev_ = true;
    }
    for (int i = 0; i < 12; ++i) {
        const Joint& j = kJoints[i];
        if (j.servo) qd[i] = (q[i] - q_prev_[i]) / dt;
        else qd[i] = j.sign * in.vel[j.can];
    }
    for (int i = 0; i < 2; ++i) qd[12 + i] = kWheelSign[i] * in.vel[kWheelCan[i]];
    for (int i = 0; i < 14; ++i) q_prev_[i] = q[i];

    float obs[46]{};
    obs[0] = in.gyro[0];
    obs[1] = in.gyro[1];
    obs[2] = in.gyro[2];
    obs[3] = in.grav[0];
    obs[4] = in.grav[1];
    obs[5] = in.grav[2];
    for (int i = 0; i < 12; ++i) obs[6 + i] = encode_pos(q[i], kJoints[i].lo, kJoints[i].hi, kJoints[i].latent);
    for (int i = 0; i < 14; ++i) obs[18 + i] = qd[i];
    for (int i = 0; i < 14; ++i) obs[32 + i] = last_action_[i];
    for (int i = 0; i < 46; ++i) obs[i] = (obs[i] - mean_[i]) / (std_[i] + kNormEps);

    float h0[512];
    float h1[256];
    float h2[128];
    float act[14];
    linear(w0_, b0_, obs, h0, 512, 46, true);
    linear(w1_, b1_, h0, h1, 256, 512, true);
    linear(w2_, b2_, h1, h2, 128, 256, true);
    linear(w3_, b3_, h2, act, 14, 128, false);
    if (act[12] > kWheelLimit) act[12] = kWheelLimit;
    if (act[12] < -kWheelLimit) act[12] = -kWheelLimit;
    if (act[13] > kWheelLimit) act[13] = kWheelLimit;
    if (act[13] < -kWheelLimit) act[13] = -kWheelLimit;
    for (int i = 0; i < 14; ++i) last_action_[i] = act[i];

    out.active = active_;
    if (!active_) return;
    for (int i = 0; i < 12; ++i) {
        const Joint& j = kJoints[i];
        float q_tgt = decode_pos(act[i], j.lo, j.hi, j.latent);
        float deg = j.sign * q_tgt * 57.2957795f;
        if (j.servo) {
            if (j.can == 13) out.yaw = deg;
            else out.roll = deg;
        } else if (j.can >= 1 && j.can <= 15) {
            out.deg[j.can] = deg;
            // 0x1F 必须带力矩字段。300 N·m 顶到协议量程，等于不施加软件力矩限制。
            out.tq[j.can] = 300.f;
        }
    }
}

}  // namespace wheel
