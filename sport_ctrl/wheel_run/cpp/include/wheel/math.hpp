#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace wheel {

constexpr double kTwoPi = 6.283185307179586;
constexpr float kDeadzone = 0.12f;
constexpr float kSpeedArmS = 3.f;
constexpr float kYHoldS = 1.f;
constexpr float kRollRateDegS = 24.f;
constexpr int kChaseLead = 1900;

inline int16_t clip_i16(int value) {
    if (value > 32700) return 32700;
    if (value < -32700) return -32700;
    return static_cast<int16_t>(value);
}

inline int16_t pos_to_int(double rad) {
    return clip_i16(static_cast<int>(rad / kTwoPi * 10000.0));
}

inline int16_t vel_to_int(double rad_s) {
    return clip_i16(static_cast<int>(rad_s / kTwoPi * 4000.0));
}

inline int16_t tqe_to_int(double nm) {
    return clip_i16(static_cast<int>(nm * 100.0));
}

inline double pos_from_int(int16_t raw) { return raw * kTwoPi / 10000.0; }
inline double vel_from_int(int16_t raw) { return raw * kTwoPi / 4000.0; }
inline double tqe_from_int(int16_t raw) { return raw / 100.0; }

inline float rad2deg(float rad) { return rad * 57.2957795f; }
inline float deg2rad(float deg) { return deg * 0.0174532925f; }

inline float deadzone(float v) {
    v = std::clamp(v, -1.f, 1.f);
    if (std::fabs(v) < kDeadzone) return 0.f;
    float mag = (std::fabs(v) - kDeadzone) / (1.f - kDeadzone);
    return v > 0.f ? mag : -mag;
}

inline float ramp(float cur, float goal, float accel, float dt) {
    float dv = std::fabs(accel) * dt;
    if (cur < goal) return std::min(goal, cur + dv);
    if (cur > goal) return std::max(goal, cur - dv);
    return goal;
}

inline float lowpass(float cur, float goal, float tau, float dt) {
    if (tau <= 1e-4f) return goal;
    float alpha = dt / (tau + dt);
    alpha = std::clamp(alpha, 0.f, 1.f);
    float next = cur + (goal - cur) * alpha;
    if (std::fabs(goal) < 1e-4f && std::fabs(next) < 0.02f) return 0.f;
    return next;
}

inline float smootherstep(float u) {
    u = std::clamp(u, 0.f, 1.f);
    return u * u * u * (u * (u * 6.f - 15.f) + 10.f);
}

struct WheelPair {
    float left = 0;
    float right = 0;
};

inline WheelPair wheel_targets(float fwd_stick, float turn_stick, float spin_stick, float vmax, float turn_max,
                               float spin_max) {
    float fwd = deadzone(fwd_stick) * vmax;
    float turn = deadzone(turn_stick) * turn_max;
    float spin = deadzone(spin_stick) * spin_max;
    WheelPair out;
    if (std::fabs(fwd) < 1e-9f && spin == 0.f && turn != 0.f) {
        if (turn > 0.f) {
            out.left = 0.f;
            out.right = turn;
        } else {
            out.left = -turn;
            out.right = 0.f;
        }
        return out;
    }
    float diff = turn + spin;
    float cap = std::max(vmax, spin_max);
    out.left = std::clamp(fwd - diff, -cap, cap);
    out.right = std::clamp(fwd + diff, -cap, cap);
    return out;
}

inline float wheel_sign(int motor_id) { return motor_id == 1 ? 1.f : -1.f; }

struct Chase {
    int16_t raw = 0;
    float vel_limit = 2.f;
};

inline int16_t wrap_sum(int raw, int lead) {
    int target = raw + lead;
    if (target > 32767) target -= 65536;
    else if (target < -32768) target += 65536;
    return static_cast<int16_t>(target);
}

inline Chase wheel_chase(int16_t raw_pos, float vel_rad_s) {
    Chase c;
    if (std::fabs(vel_rad_s) < 0.08f) {
        c.raw = raw_pos;
        c.vel_limit = 2.f;
        return c;
    }
    int lead = vel_rad_s > 0.f ? kChaseLead : -kChaseLead;
    c.raw = wrap_sum(raw_pos, lead);
    c.vel_limit = std::max(std::fabs(vel_rad_s), 0.8f);
    return c;
}

inline bool sticks_bogus(float fwd, float turn, float spin) {
    int half = 0;
    for (float v : {fwd, turn, spin}) {
        if (std::fabs(std::fabs(v) - 0.5f) < 0.06f) ++half;
    }
    return half >= 2;
}

}  // namespace wheel
