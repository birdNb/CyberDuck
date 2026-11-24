#include "wheel/roll.hpp"

#include "wheel/math.hpp"
#include "wheel/policy_log.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace wheel {
namespace {

constexpr float kNormEps = 1e-2f;
constexpr float kWheelLimit = 2.f;
constexpr float kEpisodeS = 5.f;
// 与仿真 effort_limit / hard150 对齐。
constexpr float kLimitTorqueNm = 2.f;
// 仿真硬裁剪 150 RPM ≈ 15.708 rad/s。
constexpr float kCmdVelRadS = 15.708f;
constexpr float kArriveTolRad = 0.05236f;  // ~3°
constexpr float kMinHoldS = 0.08f;
constexpr float kMaxHoldS = 4.f;
constexpr float kApproachMaxS = 8.f;
constexpr uint32_t kMagic = 0x4C4C4F52u;

struct Joint {
    int can;
    bool servo;
    float lo;
    float hi;
    float latent;
    float home;  // 仿真默认角（翻滚前先走到这里）
    float sign;  // 与 check_joint_signs / locomotion 对齐
};

// 顺序 = 策略动作 0..11。SIGN 来自 check_joint_signs；latent/home 来自 checkpoint contract。
constexpr Joint kJoints[12] = {
    {5, false, -0.436f, 0.436f, 0.f, 0.f, 1.f},
    {4, false, -0.384f, 0.384f, 0.f, 0.f, -1.f},
    {3, false, -1.571f, 1.571f, 0.3226523995399475f, 0.49f, -1.f},
    {2, false, -1.571f, 1.571f, -0.2905299961566925f, -0.444f, 1.f},
    {10, false, -0.436f, 0.436f, 0.f, 0.f, -1.f},
    {9, false, -0.384f, 0.384f, 0.f, 0.f, 1.f},
    {8, false, -1.571f, 1.571f, 0.3226523995399475f, 0.49f, 1.f},
    {7, false, -1.571f, 1.571f, -0.2905299961566925f, -0.444f, -1.f},
    {12, false, -1.571f, 1.877f, -0.3068446218967438f, -0.36f, 1.f},
    {11, false, -1.571f, 1.571f, 0.26374778151512146f, 0.405f, 1.f},
    {13, true, -2.967f, 2.967f, 0.f, 0.f, 1.f},
    {14, true, -0.208f, 0.208f, 0.05293400213122368f, 0.011f, 1.f},
};

constexpr int kWheelCan[2] = {1, 6};
constexpr float kWheelSign[2] = {-1.f, 1.f};  // CAN1 右、CAN6 左，与 check_joint_signs 踝 SIGN 一致

constexpr const char* kActCols[14] = {
    "act_r_hip_yaw", "act_r_hip_roll", "act_r_hip_pitch", "act_r_calf",
    "act_l_hip_yaw", "act_l_hip_roll", "act_l_hip_pitch", "act_l_calf",
    "act_neck_pitch", "act_head_pitch", "act_head_yaw", "act_head_roll", "act_r_ankle", "act_l_ankle",
};

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

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::string cell;
    std::stringstream ss(line);
    while (std::getline(ss, cell, ',')) {
        while (!cell.empty() && (cell.back() == '\r' || cell.back() == ' ')) cell.pop_back();
        out.push_back(cell);
    }
    return out;
}

}  // namespace

constexpr const char* kRollObs[48] = {
    "gyro_x", "gyro_y", "gyro_z", "grav_x", "grav_y", "grav_z",
    "q_r_hip_yaw", "q_r_hip_roll", "q_r_hip_pitch", "q_r_calf",
    "q_l_hip_yaw", "q_l_hip_roll", "q_l_hip_pitch", "q_l_calf",
    "q_neck_pitch", "q_head_pitch", "q_head_yaw", "q_head_roll",
    "qd_r_hip_yaw", "qd_r_hip_roll", "qd_r_hip_pitch", "qd_r_calf",
    "qd_l_hip_yaw", "qd_l_hip_roll", "qd_l_hip_pitch", "qd_l_calf",
    "qd_neck_pitch", "qd_head_pitch", "qd_head_yaw", "qd_head_roll", "qd_r_ankle", "qd_l_ankle",
    "prev_r_hip_yaw", "prev_r_hip_roll", "prev_r_hip_pitch", "prev_r_calf",
    "prev_l_hip_yaw", "prev_l_hip_roll", "prev_l_hip_pitch", "prev_l_calf",
    "prev_neck_pitch", "prev_head_pitch", "prev_head_yaw", "prev_head_roll", "prev_r_ankle", "prev_l_ankle",
    "phase_0", "phase_1",
};
constexpr const char* kRollAct[14] = {
    "act_r_hip_yaw", "act_r_hip_roll", "act_r_hip_pitch", "act_r_calf",
    "act_l_hip_yaw", "act_l_hip_roll", "act_l_hip_pitch", "act_l_calf",
    "act_neck_pitch", "act_head_pitch", "act_head_yaw", "act_head_roll", "act_r_ankle", "act_l_ankle",
};

// CSV 慢放日志：动作后附带帧进度和物理角，方便和仿真对齐。
constexpr const char* kCsvOut[14 + 6 + 24] = {
    "act_r_hip_yaw", "act_r_hip_roll", "act_r_hip_pitch", "act_r_calf",
    "act_l_hip_yaw", "act_l_hip_roll", "act_l_hip_pitch", "act_l_calf",
    "act_neck_pitch", "act_head_pitch", "act_head_yaw", "act_head_roll", "act_r_ankle", "act_l_ankle",
    "frame_i", "hold_s", "arrived", "timed_out", "csv_t_s", "q_err_max",
    "meas_r_hip_yaw", "meas_r_hip_roll", "meas_r_hip_pitch", "meas_r_calf",
    "meas_l_hip_yaw", "meas_l_hip_roll", "meas_l_hip_pitch", "meas_l_calf",
    "meas_neck_pitch", "meas_head_pitch", "meas_head_yaw", "meas_head_roll",
    "tgt_r_hip_yaw", "tgt_r_hip_roll", "tgt_r_hip_pitch", "tgt_r_calf",
    "tgt_l_hip_yaw", "tgt_l_hip_roll", "tgt_l_hip_pitch", "tgt_l_calf",
    "tgt_neck_pitch", "tgt_head_pitch", "tgt_head_yaw", "tgt_head_roll",
};

void Roll::emit_action(const float act[14], RollGoal& out) const {
    out.motor_left = 0.f;
    out.motor_right = 0.f;
    out.cmd_vel = kCmdVelRadS;
    out.active = true;
    for (int i = 0; i < 12; ++i) {
        const Joint& j = kJoints[i];
        float q_tgt = decode_pos(act[i], j.lo, j.hi, j.latent);
        float deg = j.sign * q_tgt * 57.2957795f;
        if (j.servo) {
            if (j.can == 13) out.yaw = deg;
            else out.roll = deg;
        } else if (j.can >= 1 && j.can <= 15) {
            out.deg[j.can] = deg;
            out.tq[j.can] = kLimitTorqueNm;
        }
    }
}

void Roll::emit_default_pose(RollGoal& out) const {
    out.motor_left = 0.f;
    out.motor_right = 0.f;
    out.cmd_vel = kCmdVelRadS;
    out.active = true;
    for (int i = 0; i < 12; ++i) {
        const Joint& j = kJoints[i];
        float deg = j.sign * j.home * 57.2957795f;
        if (j.servo) {
            if (j.can == 13) out.yaw = deg;
            else out.roll = deg;
        } else if (j.can >= 1 && j.can <= 15) {
            out.deg[j.can] = deg;
            out.tq[j.can] = kLimitTorqueNm;
        }
    }
}

bool Roll::default_pose_arrived(const RollSense& in) const {
    for (int i = 0; i < 12; ++i) {
        const Joint& j = kJoints[i];
        float motor = j.servo ? (j.can == 13 ? in.servo_yaw : in.servo_roll) : in.pos[j.can];
        if (j.servo) motor = motor * 0.0174532925f;
        float q = j.sign * motor;
        if (std::fabs(q - j.home) > kArriveTolRad) return false;
    }
    return true;
}

bool Roll::frame_arrived(const float act[14], const RollSense& in) const {
    for (int i = 0; i < 12; ++i) {
        const Joint& j = kJoints[i];
        float motor = j.servo ? (j.can == 13 ? in.servo_yaw : in.servo_roll) : in.pos[j.can];
        if (j.servo) motor = motor * 0.0174532925f;
        float q = j.sign * motor;
        float tgt = decode_pos(act[i], j.lo, j.hi, j.latent);
        if (std::fabs(q - tgt) > kArriveTolRad) return false;
    }
    return true;
}

bool Roll::load_csv(const std::string& path) {
    ready_ = false;
    csv_mode_ = false;
    frames_.clear();
    reason_.clear();
    source_.clear();
    std::ifstream in(path);
    if (!in) {
        reason_ = "CSV 打不开";
        return false;
    }
    std::string header;
    if (!std::getline(in, header)) {
        reason_ = "CSV 没有表头";
        return false;
    }
    auto cols = split_csv(header);
    int t_col = -1;
    int act_col[14];
    for (int i = 0; i < 14; ++i) act_col[i] = -1;
    for (int i = 0; i < static_cast<int>(cols.size()); ++i) {
        if (cols[i] == "t_s") t_col = i;
        for (int a = 0; a < 14; ++a) {
            if (cols[i] == kActCols[a]) act_col[a] = i;
        }
    }
    if (t_col < 0) {
        reason_ = "CSV 缺少 t_s";
        return false;
    }
    for (int a = 0; a < 14; ++a) {
        if (act_col[a] < 0) {
            reason_ = std::string("CSV 缺少 ") + kActCols[a];
            return false;
        }
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        auto cells = split_csv(line);
        int need = t_col;
        for (int a = 0; a < 14; ++a) need = std::max(need, act_col[a]);
        if (static_cast<int>(cells.size()) <= need) continue;
        Frame fr;
        fr.t_s = static_cast<float>(std::atof(cells[t_col].c_str()));
        for (int a = 0; a < 14; ++a) fr.act[a] = static_cast<float>(std::atof(cells[act_col[a]].c_str()));
        frames_.push_back(fr);
    }
    if (frames_.empty()) {
        reason_ = "CSV 没有动作行";
        return false;
    }
    ready_ = true;
    csv_mode_ = true;
    source_ = path;
    return true;
}

bool Roll::load(const std::string& path) {
    ready_ = false;
    csv_mode_ = false;
    active_ = false;
    phase_ = PhaseIdle;
    frames_.clear();
    reason_.clear();
    source_.clear();
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
    if (magic != kMagic || obs != 48 || h0 != 512 || h1 != 256 || h2 != 128 || act != 14) {
        reason_ = "权重尺寸和向前翻滚策略不一致";
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
    source_ = path;
    return true;
}

void Roll::step(float dt, const RollSense& in, RollGoal& out) {
    out = {};
    out.active = active_;
    if (!ready_) return;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;

    if (active_ && in.chord_rise) {
        active_ = false;
        phase_ = PhaseIdle;
        out.active = false;
        out.say_stop = true;
        return;
    }
    if (!active_ && in.chord_rise) {
        if (in.other_busy) {
            out.say_busy = true;
        } else if (!in.allow) {
            out.say_refuse = true;
        } else {
            active_ = true;
            phase_ = PhaseApproach;
            elapsed_ = 0.f;
            angle_ = 0.f;
            hold_s_ = 0.f;
            approach_s_ = 0.f;
            frame_i_ = 0;
            miss_frames_ = 0;
            have_prev_ = false;
            for (int i = 0; i < 14; ++i) last_action_[i] = 0.f;
            if (csv_mode_) log_policy_close("roll_csv");
            else log_policy_close("roll");
            out.say_start = true;
        }
    }
    if (!in.allow && active_) {
        active_ = false;
        phase_ = PhaseIdle;
        out.active = false;
        out.say_stop = true;
        return;
    }
    if (!active_) {
        out.active = false;
        return;
    }

    if (phase_ == PhaseApproach) {
        approach_s_ += dt;
        emit_default_pose(out);
        bool arrived = default_pose_arrived(in);
        if (arrived || approach_s_ >= kApproachMaxS) {
            phase_ = PhaseRun;
            elapsed_ = 0.f;
            angle_ = 0.f;
            hold_s_ = 0.f;
            have_prev_ = false;
            for (int i = 0; i < 14; ++i) last_action_[i] = 0.f;
            out.say_ready = true;
        }
        return;
    }

    elapsed_ += dt;

    if (csv_mode_) {
        if (frame_i_ >= static_cast<int>(frames_.size())) {
            active_ = false;
            phase_ = PhaseIdle;
            out.active = false;
            out.say_done = true;
            return;
        }
        const Frame& fr = frames_[frame_i_];
        float prev_act[14];
        for (int i = 0; i < 14; ++i) prev_act[i] = last_action_[i];
        float act[14];
        for (int i = 0; i < 14; ++i) act[i] = fr.act[i];
        if (act[12] > kWheelLimit) act[12] = kWheelLimit;
        if (act[12] < -kWheelLimit) act[12] = -kWheelLimit;
        if (act[13] > kWheelLimit) act[13] = kWheelLimit;
        if (act[13] < -kWheelLimit) act[13] = -kWheelLimit;
        for (int i = 0; i < 14; ++i) last_action_[i] = act[i];
        emit_action(act, out);
        hold_s_ += dt;
        bool arrived = frame_arrived(act, in);
        bool min_hold = hold_s_ >= kMinHoldS;
        bool timed_out = hold_s_ >= kMaxHoldS;

        float q[14]{};
        float qd[14]{};
        float q_tgt[12]{};
        float q_err_max = 0.f;
        for (int i = 0; i < 12; ++i) {
            const Joint& j = kJoints[i];
            float motor = j.servo ? (j.can == 13 ? in.servo_yaw : in.servo_roll) : in.pos[j.can];
            if (j.servo) motor = motor * 0.0174532925f;
            q[i] = j.sign * motor;
            q_tgt[i] = decode_pos(act[i], j.lo, j.hi, j.latent);
            float err = std::fabs(q[i] - q_tgt[i]);
            if (err > q_err_max) q_err_max = err;
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

        float progress = 0.f;
        if (!frames_.empty()) {
            float span = frames_.back().t_s;
            if (span < 1e-3f) span = 1.f;
            progress = fr.t_s / span;
            if (progress < 0.f) progress = 0.f;
            if (progress > 1.f) progress = 1.f;
        }

        float raw[48]{};
        raw[0] = in.gyro[0];
        raw[1] = in.gyro[1];
        raw[2] = in.gyro[2];
        raw[3] = in.grav[0];
        raw[4] = in.grav[1];
        raw[5] = in.grav[2];
        for (int i = 0; i < 12; ++i) raw[6 + i] = encode_pos(q[i], kJoints[i].lo, kJoints[i].hi, kJoints[i].latent);
        for (int i = 0; i < 14; ++i) raw[18 + i] = qd[i];
        for (int i = 0; i < 14; ++i) raw[32 + i] = prev_act[i];
        raw[46] = progress;
        raw[47] = progress;

        float out_row[14 + 6 + 24]{};
        for (int i = 0; i < 14; ++i) out_row[i] = act[i];
        out_row[14] = static_cast<float>(frame_i_);
        out_row[15] = hold_s_;
        out_row[16] = arrived ? 1.f : 0.f;
        out_row[17] = timed_out ? 1.f : 0.f;
        out_row[18] = fr.t_s;
        out_row[19] = q_err_max;
        for (int i = 0; i < 12; ++i) {
            out_row[20 + i] = q[i];
            out_row[32 + i] = q_tgt[i];
        }
        log_policy_run("roll_csv", kRollObs, 48, raw, kCsvOut, 14 + 6 + 24, out_row);

        if ((arrived && min_hold) || timed_out) {
            if (timed_out && !arrived) ++miss_frames_;
            ++frame_i_;
            hold_s_ = 0.f;
            if (frame_i_ >= static_cast<int>(frames_.size())) {
                active_ = false;
                phase_ = PhaseIdle;
                out.active = false;
                out.say_done = true;
                return;
            }
        }
        return;
    }

    if (elapsed_ >= kEpisodeS) {
        active_ = false;
        phase_ = PhaseIdle;
        out.active = false;
        out.say_done = true;
        return;
    }
    angle_ += in.gyro[1] * dt;
    float progress = angle_ / static_cast<float>(kTwoPi);
    if (progress < 0.f) progress = 0.f;
    if (progress > 1.f) progress = 1.f;

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

    float obs[48]{};
    obs[0] = in.gyro[0];
    obs[1] = in.gyro[1];
    obs[2] = in.gyro[2];
    obs[3] = in.grav[0];
    obs[4] = in.grav[1];
    obs[5] = in.grav[2];
    for (int i = 0; i < 12; ++i) obs[6 + i] = encode_pos(q[i], kJoints[i].lo, kJoints[i].hi, kJoints[i].latent);
    for (int i = 0; i < 14; ++i) obs[18 + i] = qd[i];
    for (int i = 0; i < 14; ++i) obs[32 + i] = last_action_[i];
    obs[46] = progress;
    obs[47] = progress;
    float raw[48];
    for (int i = 0; i < 48; ++i) raw[i] = obs[i];
    for (int i = 0; i < 48; ++i) obs[i] = (obs[i] - mean_[i]) / (std_[i] + kNormEps);

    float h0[512];
    float h1[256];
    float h2[128];
    float act[14];
    linear(w0_, b0_, obs, h0, 512, 48, true);
    linear(w1_, b1_, h0, h1, 256, 512, true);
    linear(w2_, b2_, h1, h2, 128, 256, true);
    linear(w3_, b3_, h2, act, 14, 128, false);
    if (act[12] > kWheelLimit) act[12] = kWheelLimit;
    if (act[12] < -kWheelLimit) act[12] = -kWheelLimit;
    if (act[13] > kWheelLimit) act[13] = kWheelLimit;
    if (act[13] < -kWheelLimit) act[13] = -kWheelLimit;
    for (int i = 0; i < 14; ++i) last_action_[i] = act[i];
    log_policy_run("roll", kRollObs, 48, raw, kRollAct, 14, act);
    emit_action(act, out);
}

}  // namespace wheel
