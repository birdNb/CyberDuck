#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "wheel/getup.hpp"
#include "wheel/plant.hpp"
#include "wheel/roll.hpp"

#include "imu/lsm6.hpp"
#include "wheel/bus.hpp"
#include "wheel/can_log.hpp"
#include "wheel/config.hpp"
#include "wheel/math.hpp"

#include <linux/joystick.h>

#include <pthread.h>
#include <sched.h>
#include <poll.h>
#include <spawn.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <time.h>

#include <csignal>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>

extern char** environ;

namespace wheel {

int bring_up_can(const std::string& iface, const std::string& config);

namespace {

volatile std::sig_atomic_t g_sig = 0;
void on_signal(int) { g_sig = static_cast<std::sig_atomic_t>(g_sig + 1); }

enum Phase : uint32_t { PhBoot = 0, PhWait = 1, PhSlew = 2, PhSettle = 3, PhDrive = 4, PhCut = 5, PhStop = 6, PhExit = 7, PhHold = 8 };

struct Slot {
    uint8_t kind = 0;  // 1 = 关节位置 0x1F，2 = 轮子速度 0x1C
    uint8_t use_raw = 0;
    uint8_t zero = 0;
    int16_t raw = 0;
    float pos = 0;
    float vel = 0;
    float tq = 0;
};

struct ShowCmd {
    uint64_t t_ns = 0;
    bool neck = false;
    float n11 = 0;
    float n12 = 0;
    float nvel = 1.2f;
    float ntq = 1.f;
    bool lock_wheels = false;
    bool has_servo = false;
    float servo_yaw = 0;
    float servo_roll = 0;
    uint16_t mask = 0;
    float deg[16]{};
    float vel[16]{};
    float tq[16]{};
};

struct PadSnap {
    float axis[8]{};
    float fwd = 0;
    float turn = 0;
    float spin = 0;
    float height = 0;
    int gear = 0;
    bool lt = false;
    bool connected = false;
    bool bogus = false;
    bool xbox = false;
    bool y_held = false;
    bool lb_held = false;
    uint32_t edge_b = 0;
    uint32_t edge_y = 0;
    char path[64]{};
};

struct ImuSnap {
    float gyro[3]{};
    float grav[3]{0, 0, -1};
    bool ok = false;
};

struct Job {
    double t0 = 0;
    double duration = 1;
    float start_deg[16]{};
    float goal_deg[16]{};
    uint16_t mask = 0;
    float torque = 1;
    float vel = 1.5f;
};

struct PiMutex {
    pthread_mutex_t mu;
    PiMutex() {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
        if (pthread_mutex_init(&mu, &attr) != 0) pthread_mutex_init(&mu, nullptr);
        pthread_mutexattr_destroy(&attr);
    }
    ~PiMutex() { pthread_mutex_destroy(&mu); }
    void lock() { pthread_mutex_lock(&mu); }
    void unlock() { pthread_mutex_unlock(&mu); }
};

struct Lock {
    PiMutex& m;
    explicit Lock(PiMutex& x) : m(x) { m.lock(); }
    ~Lock() { m.unlock(); }
};

struct State {
    Args args;
    YamlCfg cfg;
    Pose stand;
    Pose extended;
    Pose unload;
    Pose lean_l;
    Pose lean_r;
    int left_id = 6;
    int right_id = 1;
    uint32_t online = 0;
    bool lift_ok = false;
    float slew_vel = 1.5f;
    uint64_t can_period_ns = 5000000;
    uint64_t body_period_ns = 10000000;

    std::unique_ptr<Bus> bus;
    std::unique_ptr<imu::Lsm6> imu;

    PiMutex mu_cmd;
    PiMutex mu_fb;
    PiMutex mu_pad;
    PiMutex mu_imu;
    PiMutex mu_job;
    PiMutex mu_show;
    ShowCmd show;
    Getup getup;
    GetupGoal getup_goal;
    bool getup_lb = false;
    uint64_t getup_prev_ns = 0;
    std::atomic<uint8_t> getup_on{0};
    Roll roll;
    RollGoal roll_goal;
    bool roll_chord = false;
    uint64_t roll_prev_ns = 0;
    std::atomic<uint8_t> roll_on{0};
    Slot slot[16]{};
    Feedback fb[16]{};
    PadSnap pad{};
    ImuSnap imu_snap{};
    Job job{};

    std::atomic<uint32_t> phase{PhBoot};
    std::atomic<uint8_t> estop{0};
    std::atomic<uint8_t> balance_on{0};
    std::atomic<uint8_t> unload_hold{0};
    std::atomic<uint8_t> torque_off{0};
    std::atomic<uint8_t> link_recover{0};
    std::atomic<uint64_t> link_quiet_until_ns{0};
    std::atomic<uint32_t> sample_mask{0};
    std::atomic<uint32_t> sample_gen{0};
    std::atomic<uint64_t> can_hb_ns{0};
    std::atomic<uint64_t> wheel_hb_ns{0};
    std::atomic<uint64_t> drive_t0_ns{0};
    std::atomic<uint64_t> cmd_ready_ns{0};
    std::atomic<uint32_t> tx_fail{0};
    std::atomic<uint32_t> late_can{0};
    std::atomic<uint32_t> late_wheel{0};
    std::atomic<uint32_t> late_body{0};
    std::atomic<uint32_t> can_us{0};
    std::atomic<uint32_t> wheel_us{0};
    std::atomic<uint32_t> cmd_l_bits{0};
    std::atomic<uint32_t> cmd_r_bits{0};
    std::atomic<uint32_t> height_bits{0};
    std::atomic<uint32_t> roll_bits{0};
    std::atomic<uint32_t> lean_bits{0};
    std::atomic<uint32_t> hip_bits{0};
    std::atomic<int> hip_frozen{0};
    std::atomic<int> lean_dir{0};
    std::atomic<int> fifo_can{0};
    std::atomic<int> fifo_wheel{0};
    std::atomic<int> fifo_body{0};
    std::atomic<int> fifo_imu{0};
    std::atomic<int> fifo_pad{0};
    std::atomic<int> fifo_wd{0};
};

uint64_t mono_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

double mono_s() { return static_cast<double>(mono_ns()) * 1e-9; }

bool age_exceeds(uint64_t now, uint64_t stamp, uint64_t limit_ns) {
    return stamp != 0 && now >= stamp && now - stamp > limit_ns;
}

bool iface_up(const std::string& iface) {
    std::string path = "/sys/class/net/" + iface + "/flags";
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return false;
    unsigned flags = 0;
    int n = std::fscanf(f, "%x", &flags);
    std::fclose(f);
    return n == 1 && (flags & 1u) != 0;
}

uint64_t read_rx_packets(const std::string& iface) {
    std::string path = "/sys/class/net/" + iface + "/statistics/rx_packets";
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return 0;
    unsigned long long v = 0;
    int n = std::fscanf(f, "%llu", &v);
    std::fclose(f);
    return n == 1 ? static_cast<uint64_t>(v) : 0;
}

void add_ns(timespec& t, uint64_t ns) {
    t.tv_sec += static_cast<time_t>(ns / 1000000000ull);
    t.tv_nsec += static_cast<long>(ns % 1000000000ull);
    if (t.tv_nsec >= 1000000000L) {
        t.tv_nsec -= 1000000000L;
        t.tv_sec += 1;
    }
}

void pace(timespec& next, uint64_t period_ns, std::atomic<uint32_t>* late) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    add_ns(next, period_ns);
    uint64_t now = mono_ns();
    uint64_t nxt = static_cast<uint64_t>(next.tv_sec) * 1000000000ull + static_cast<uint64_t>(next.tv_nsec);
    if (now > nxt) {
        if (late) late->fetch_add(1, std::memory_order_relaxed);
        if (now > nxt + period_ns) {
            clock_gettime(CLOCK_MONOTONIC, &next);
            add_ns(next, period_ns);
        }
    }
}

bool set_fifo(int prio) {
    int maxp = sched_get_priority_max(SCHED_FIFO);
    int minp = sched_get_priority_min(SCHED_FIFO);
    if (maxp > 0 && prio > maxp) prio = maxp;
    if (minp > 0 && prio < minp) prio = minp;
    sched_param sp{};
    sp.sched_priority = prio;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == 0) return true;
    int policy = 0;
    if (pthread_getschedparam(pthread_self(), &policy, &sp) == 0 && policy == SCHED_FIFO) return true;
    return false;
}

void store_f(std::atomic<uint32_t>& dst, float v) {
    uint32_t u = 0;
    std::memcpy(&u, &v, sizeof(u));
    dst.store(u, std::memory_order_relaxed);
}

float load_f(const std::atomic<uint32_t>& src) {
    uint32_t u = src.load(std::memory_order_relaxed);
    float v = 0;
    std::memcpy(&v, &u, sizeof(v));
    return v;
}

bool is_wheel(int id) { return id == 1 || id == 6; }
bool is_lift(int id) { return id == 2 || id == 3 || id == 7 || id == 8; }
uint32_t bit(int id) { return 1u << id; }

std::string mask_text(uint32_t mask) {
    std::string s;
    for (int id = 1; id <= 15; ++id) {
        if (!(mask & bit(id))) continue;
        if (!s.empty()) s += ",";
        s += std::to_string(id);
    }
    return s.empty() ? "无" : s;
}

PadSnap copy_pad(State* s) {
    Lock lk(s->mu_pad);
    return s->pad;
}

Feedback copy_fb(State* s, int id) {
    Lock lk(s->mu_fb);
    if (id < 0 || id >= 16) return {};
    return s->fb[id];
}

ImuSnap copy_imu(State* s) {
    Lock lk(s->mu_imu);
    return s->imu_snap;
}

ShowCmd copy_show(State* s) {
    Lock lk(s->mu_show);
    return s->show;
}

bool show_live(const ShowCmd& cmd) {
    return cmd.t_ns != 0 && !age_exceeds(mono_ns(), cmd.t_ns, 300000000ull);
}

bool is_wheel(int id);
uint32_t bit(int id);

void put_deg(State* s, int id, float deg, float vel, float tq) {
    if (id < 1 || id > 15 || !(s->online & bit(id)) || is_wheel(id)) return;
    Slot sl;
    sl.kind = 1;
    sl.pos = deg2rad(deg);
    sl.vel = vel;
    sl.tq = tq;
    s->slot[id] = sl;
}

bool start_thread(pthread_t& thread, void* (*fn)(void*), void* arg, const char* name) {
    int rc = pthread_create(&thread, nullptr, fn, arg);
    if (rc != 0) return false;
    pthread_setname_np(thread, name);
    return true;
}

bool hard_stop(State* s) {
    if (g_sig < 2) return false;
    s->estop.store(1);
    s->phase.store(PhStop);
    timespec ts{0, 80000000};
    nanosleep(&ts, nullptr);
    return true;
}

bool wait_for(State* s, double seconds) {
    double end = mono_s() + seconds;
    while (mono_s() < end) {
        if (hard_stop(s)) return false;
        timespec ts{0, 20000000};
        nanosleep(&ts, nullptr);
    }
    return true;
}

void ingest(State* s) {
    if (!s->bus) return;
    for (int n = 0; n < 48; ++n) {
        Frame frame;
        if (!s->bus->poll(frame, 0)) return;
        if (frame.id <= 0 || frame.id >= 16) continue;
        DecodeResult dec;
        {
            Lock lk(s->mu_fb);
            dec = decode_feedback(s->fb[frame.id], frame.data, frame.dlc, mono_ns());
        }
        // 采样期间每来一帧 0x0E 就销掉这一台。不要只认一次发出去的查询，丢一帧就再也不问。
        if (dec.position && (s->sample_mask.load() & bit(frame.id))) s->sample_mask.fetch_and(~bit(frame.id));
    }
}

bool sample_motors(State* s, uint32_t mask, int timeout_ms) {
    mask &= s->online;
    if (mask == 0) return true;
    s->sample_mask.store(0);
    s->sample_gen.fetch_add(1);
    s->sample_mask.store(mask);
    double end = mono_s() + timeout_ms / 1000.0;
    while (s->sample_mask.load() != 0) {
        if (hard_stop(s)) {
            s->sample_mask.store(0);
            return false;
        }
        if (mono_s() >= end) break;
        timespec ts{0, 10000000};
        nanosleep(&ts, nullptr);
    }
    uint32_t left = s->sample_mask.load();
    // 超时必须清掉，否则 CAN 线程会一直按 mask 狂发 0x0E，把 PCAN-USB 灌死。
    s->sample_mask.store(0);
    if (left) std::fprintf(stderr, "这些关节没有读到角度: %s\n", mask_text(left).c_str());
    return left == 0;
}

void* pad_thread(void* arg) {
    auto* s = static_cast<State*>(arg);
    s->fifo_pad.store(set_fifo(20) ? 1 : -1);
    int fd = -1;
    bool xbox = false;
    float axis[8]{};
    bool seen[8]{};
    bool buttons[16]{};
    axis[4] = axis[5] = -1.f;
    uint32_t edge_b = 0;
    uint32_t edge_y = 0;
    char path[64]{};
    double next_open = 0;
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    add_ns(next, 4000000);
    auto publish = [&](bool connected) {
        float fwd = connected ? axis[1] : 0.f;
        float turn = connected ? axis[0] : 0.f;
        float spin = connected ? axis[2] : 0.f;
        float height = connected ? -axis[3] : 0.f;
        bool lt = connected && (buttons[6] || (seen[5] && axis[5] > 0.3f));
        bool bogus = connected && sticks_bogus(fwd, turn, spin);
        PadSnap snap;
        std::memcpy(snap.axis, axis, sizeof(axis));
        snap.connected = connected;
        snap.xbox = xbox;
        snap.bogus = bogus;
        snap.y_held = connected && buttons[xbox ? 3 : 4];
        snap.lb_held = connected && buttons[xbox ? 4 : 6];
        snap.edge_b = edge_b;
        snap.edge_y = edge_y;
        std::snprintf(snap.path, sizeof(snap.path), "%s", path);
        if (bogus || !connected) {
            snap.fwd = snap.turn = snap.spin = snap.height = 0;
            snap.gear = 0;
            snap.lt = false;
        } else {
            snap.fwd = fwd;
            snap.turn = turn;
            snap.spin = spin;
            snap.height = height;
            snap.lt = lt;
            snap.gear = lt ? 1 : 0;
        }
        {
            Lock lk(s->mu_pad);
            s->pad = snap;
        }
        uint32_t ph = s->phase.load();
        bool live = ph == PhDrive || ph == PhSlew || ph == PhSettle;
        if (connected && buttons[1] && live) s->estop.store(1);
    };
    auto drop = [&]() {
        if (fd >= 0) ::close(fd);
        fd = -1;
        std::memset(buttons, 0, sizeof(buttons));
        std::memset(seen, 0, sizeof(seen));
        std::memset(axis, 0, sizeof(axis));
        axis[4] = axis[5] = -1.f;
        path[0] = 0;
        publish(false);
    };
    while (s->phase.load() != PhExit) {
        if (fd < 0 && mono_s() >= next_open) {
            next_open = mono_s() + 1.0;
            const char* paths[] = {"/dev/input/js0", "/dev/input/js1", "/dev/input/js2", "/dev/input/js3"};
            for (const char* p : paths) {
                int opened = ::open(p, O_RDONLY | O_NONBLOCK);
                if (opened < 0) continue;
                fd = opened;
                std::snprintf(path, sizeof(path), "%s", p);
                const char* base = std::strrchr(path, '/');
                base = base ? base + 1 : path;
                std::string name_path = std::string("/sys/class/input/") + base + "/device/name";
                xbox = false;
                if (FILE* f = std::fopen(name_path.c_str(), "r")) {
                    char name[128]{};
                    if (std::fgets(name, sizeof(name), f)) {
                        for (char* c = name; *c; ++c)
                            if (*c >= 'A' && *c <= 'Z') *c = static_cast<char>(*c - 'A' + 'a');
                        xbox = std::strstr(name, "xbox") || std::strstr(name, "x-box");
                    }
                    std::fclose(f);
                }
                std::memset(buttons, 0, sizeof(buttons));
                std::memset(seen, 0, sizeof(seen));
                std::memset(axis, 0, sizeof(axis));
                axis[4] = axis[5] = -1.f;
                break;
            }
        }
        if (fd >= 0) {
            while (true) {
                js_event ev{};
                ssize_t n = ::read(fd, &ev, sizeof(ev));
                if (n == static_cast<ssize_t>(sizeof(ev))) {
                    bool init = ev.type & JS_EVENT_INIT;
                    uint8_t kind = ev.type & ~JS_EVENT_INIT;
                    if (kind == JS_EVENT_AXIS && ev.number < 8) {
                        axis[ev.number] = std::clamp(ev.value / 32767.f, -1.f, 1.f);
                        seen[ev.number] = true;
                    } else if (kind == JS_EVENT_BUTTON && ev.number < 16) {
                        buttons[ev.number] = ev.value != 0;
                        if (!init && ev.value == 1) {
                            bool bogus = sticks_bogus(axis[1], axis[0], axis[2]);
                            if (ev.number == 1) edge_b++;
                            if (!bogus && ((xbox && ev.number == 3) || (!xbox && ev.number == 4))) edge_y++;
                        }
                    }
                    continue;
                }
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
                drop();
                next_open = mono_s() + 0.5;
                break;
            }
        }
        publish(fd >= 0);
        pace(next, 4000000, nullptr);
    }
    if (fd >= 0) ::close(fd);
    return nullptr;
}

void* imu_thread(void* arg) {
    auto* s = static_cast<State*>(arg);
    s->fifo_imu.store(set_fifo(30) ? 1 : -1);
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    add_ns(next, 5000000);
    while (s->phase.load() != PhExit) {
        if (s->imu) {
            auto sample = s->imu->read_raw();
            Lock lk(s->mu_imu);
            if (sample.ok) {
                std::memcpy(s->imu_snap.gyro, sample.gyro, sizeof(sample.gyro));
                std::memcpy(s->imu_snap.grav, sample.gravity, sizeof(sample.gravity));
                s->imu_snap.ok = true;
            } else {
                s->imu_snap.ok = false;
            }
        }
        pace(next, 5000000, nullptr);
    }
    return nullptr;
}

struct Balance {
    float kp = 1;
    float kd = 0.1f;
    float max_deg = 40;
    float flip = 1;
    float lean = 0;
    float hold = 0;
    float rate_f = 0;
    float delta = 0;
    bool init = false;
    bool hold_init = false;

    void update(float dt, const ImuSnap& imu, bool active) {
        dt = std::clamp(dt, 0.001f, 0.05f);
        if (!imu.ok) return;
        float meas = std::atan2(imu.grav[0], std::hypot(imu.grav[1], imu.grav[2])) * 57.2957795f;
        float rate = imu.gyro[1] * 57.2957795f;
        if (!init) {
            lean = meas;
            init = true;
        } else {
            lean += rate * dt;
            lean += (meas - lean) * (dt / 0.8f);
        }
        float a = dt / (0.08f + dt);
        rate_f += a * (rate - rate_f);
        float rate_d = std::fabs(rate_f) < 4.f ? 0.f : rate_f;
        if (!hold_init) {
            hold = lean;
            hold_init = true;
        } else {
            float b = dt / (0.7f + dt);
            hold += b * (lean - hold);
        }
        float hold_eff = std::fabs(hold) > 1.5f ? hold - std::copysign(1.5f, hold) : 0.f;
        float raw = std::clamp((kp * hold_eff + kd * rate_d) * flip, -max_deg, max_deg);
        if (!active) raw = 0.f;
        float step = 50.f * dt;
        delta += std::clamp(raw - delta, -step, step);
    }
};

void publish_idle_body(State* s) {
    Lock lk(s->mu_cmd);
    for (int id = 1; id <= 15; ++id) {
        if (is_wheel(id)) continue;
        s->slot[id] = {};
    }
}

void* body_thread(void* arg) {
    auto* s = static_cast<State*>(arg);
    s->fifo_body.store(set_fifo(40) ? 1 : -1);
    Balance bal;
    bal.kp = s->args.balance_kp;
    bal.kd = s->args.balance_kd;
    bal.max_deg = s->args.balance_max;
    bal.flip = s->args.balance_flip ? -1.f : 1.f;
    float height_u = 0;
    float roll_u = 0;
    float prev_lean = 0;
    float hip_hold = 0;
    bool frozen = false;
    bool holding = false;
    uint64_t prev = mono_ns();
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    add_ns(next, s->body_period_ns);
    while (s->phase.load() != PhExit) {
        uint64_t now = mono_ns();
        float dt = std::clamp(static_cast<float>(now - prev) * 1e-9f, 0.001f, 0.05f);
        prev = now;
        uint32_t ph = s->phase.load();
        if (ph == PhWait || ph == PhCut || ph == PhStop || ph == PhBoot) {
            if (holding) publish_idle_body(s);
            holding = false;
            if (ph == PhWait) {
                height_u = 0;
                roll_u = 0;
            }
        } else if (ph == PhHold) {
            // 采样和过渡前继续发上一帧位置，不能把站立的腿空出一段没有指令的时间。
        } else if (ph == PhSlew || ph == PhSettle) {
            holding = true;
            Job job;
            {
                Lock lk(s->mu_job);
                job = s->job;
            }
            float u = ph == PhSettle ? 1.f : static_cast<float>((mono_s() - job.t0) / std::max(0.05, job.duration));
            float k = smootherstep(u);
            Lock lk(s->mu_cmd);
            for (int id = 1; id <= 15; ++id) {
                if (!(job.mask & bit(id)) || is_wheel(id)) continue;
                float deg = job.start_deg[id] + (job.goal_deg[id] - job.start_deg[id]) * k;
                Slot sl;
                sl.kind = 1;
                sl.pos = deg2rad(deg);
                sl.vel = job.vel;
                sl.tq = job.torque;
                s->slot[id] = sl;
            }
        } else if (ph == PhDrive) {
            holding = true;
            PadSnap pad = copy_pad(s);
            ImuSnap imu = copy_imu(s);
            bool active = s->balance_on.load() != 0 && s->imu != nullptr;
            bal.update(dt, imu, active);
            if (s->lift_ok && pad.connected && !pad.bogus) {
                height_u = std::clamp(height_u + deadzone(pad.height) * s->args.height_rate * dt, 0.f, 1.f);
            }
            float hat = pad.connected ? pad.axis[6] : 0.f;
            int dir = 0;
            if (pad.connected && !pad.bogus) {
                if (hat >= 0.5f) dir = 1;
                else if (hat <= -0.5f) dir = -1;
            }
            float scale = pad.lt ? 1.f : 0.5f;
            float span = 1.f;
            for (int id : {4, 9}) {
                float base = s->stand.has(id) ? s->stand.deg[id] : 0.f;
                float goal = s->lean_r.has(id) ? s->lean_r.deg[id] : 20.f;
                span = std::max(span, std::fabs(goal - base));
            }
            roll_u = ramp(roll_u, static_cast<float>(dir) * scale, kRollRateDegS / span, dt);
            float lean_a = std::fabs(roll_u);
            const Pose& lean = roll_u >= 0.f ? s->lean_r : s->lean_l;
            float d_hip = bal.delta;
            float hip_tor = 0;
            for (int id : {3, 8}) hip_tor = std::max(hip_tor, std::fabs(copy_fb(s, id).torque));
            float hip_limit = active ? s->args.hip_torque : s->args.torque;
            bool growing = std::fabs(bal.lean) >= std::fabs(prev_lean) - 0.3f;
            if (active && hip_tor >= 0.85f * hip_limit && growing && std::fabs(d_hip) > hip_hold) {
                d_hip = std::copysign(hip_hold, d_hip);
                bal.delta = d_hip;
                frozen = true;
            } else {
                hip_hold = std::fabs(d_hip);
                frozen = false;
            }
            prev_lean = bal.lean;
            ShowCmd show = copy_show(s);
            {
                Lock lk(s->mu_cmd);
                for (int id = 1; id <= 15; ++id) {
                    if (!(s->online & bit(id)) || is_wheel(id) || !s->stand.has(id)) continue;
                    float deg = s->stand.deg[id];
                    if (is_lift(id) && s->lift_ok && s->extended.has(id)) {
                        deg = s->stand.deg[id] + (s->extended.deg[id] - s->stand.deg[id]) * height_u;
                    }
                    if (lean_a > 0.02f && lean.has(id)) deg += (lean.deg[id] - deg) * lean_a;
                    if (id == 8) deg += d_hip;
                    if (id == 3) deg -= d_hip;
                    Slot sl;
                    sl.kind = 1;
                    sl.pos = deg2rad(deg);
                    sl.vel = 6.f;
                    sl.tq = (id == 3 || id == 8) ? hip_limit : s->args.torque;
                    s->slot[id] = sl;
                }
                if (show_live(show)) {
                    if (show.mask != 0) {
                        for (int id = 1; id <= 15; ++id) {
                            if (!(show.mask & bit(id))) continue;
                            put_deg(s, id, show.deg[id], show.vel[id], show.tq[id]);
                        }
                    } else if (show.neck && !s->getup_on.load()) {
                        put_deg(s, 11, show.n11, show.nvel, show.ntq);
                        put_deg(s, 12, show.n12, show.nvel, show.ntq);
                    }
                }
                if (s->getup_on.load() && !s->roll_on.load()) {
                    const GetupGoal& goal = s->getup_goal;
                    for (int id = 1; id <= 15; ++id) {
                        if (goal.tq[id] <= 0.f) continue;
                        put_deg(s, id, goal.deg[id], 6.f, goal.tq[id]);
                    }
                }
                if (s->roll_on.load()) {
                    const RollGoal& goal = s->roll_goal;
                    float vel = goal.cmd_vel > 0.1f ? goal.cmd_vel : 6.f;
                    for (int id = 1; id <= 15; ++id) {
                        if (goal.tq[id] <= 0.f) continue;
                        put_deg(s, id, goal.deg[id], vel, goal.tq[id]);
                    }
                }
            }
            store_f(s->height_bits, height_u);
            store_f(s->roll_bits, roll_u);
            store_f(s->lean_bits, bal.lean);
            store_f(s->hip_bits, d_hip);
            s->hip_frozen.store(frozen ? 1 : 0);
            s->lean_dir.store(dir);
        }
        pace(next, s->body_period_ns, &s->late_body);
    }
    return nullptr;
}

void* wheel_thread(void* arg) {
    auto* s = static_cast<State*>(arg);
    s->fifo_wheel.store(set_fifo(50) ? 1 : -1);
    float cmd_l = 0, cmd_r = 0, spd_l = 0, spd_r = 0;
    uint64_t prev = mono_ns();
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    add_ns(next, s->can_period_ns);
    while (s->phase.load() != PhExit) {
        uint64_t t0 = mono_ns();
        float dt = std::clamp(static_cast<float>(t0 - prev) * 1e-9f, 0.001f, 0.05f);
        prev = t0;
        s->wheel_hb_ns.store(t0, std::memory_order_relaxed);
        uint32_t ph = s->phase.load();
        bool estop = s->estop.load() != 0;
        PadSnap pad = copy_pad(s);
        bool run = ph == PhDrive && !estop && pad.connected;
        if (!run) {
            cmd_l = cmd_r = spd_l = spd_r = 0;
        } else {
            float scale = pad.gear ? 1.f : 0.5f;
            float cap = s->args.vmax * scale;
            WheelPair tgt = wheel_targets(pad.fwd, pad.turn, pad.spin, cap, s->args.turn * scale, s->args.spin * scale);
            ShowCmd show = copy_show(s);
            if (show_live(show) && show.lock_wheels) tgt.left = tgt.right = 0;
            if (s->getup_on.load()) tgt.left = tgt.right = 0;
            if (t0 - s->drive_t0_ns.load() < static_cast<uint64_t>(kSpeedArmS * 1e9f)) tgt.left = tgt.right = 0;
            cmd_l = ramp(cmd_l, tgt.left, s->args.accel, dt);
            cmd_r = ramp(cmd_r, tgt.right, s->args.accel, dt);
            spd_l = lowpass(spd_l, cmd_l, s->args.smooth, dt);
            spd_r = lowpass(spd_r, cmd_r, s->args.smooth, dt);
        }
        auto set_wheel = [&](int id, float spd) {
            Slot sl;
            sl.kind = 2;
            sl.vel = spd * wheel_sign(id);
            return sl;
        };
        {
            Lock lk(s->mu_cmd);
            s->slot[s->left_id] = set_wheel(s->left_id, spd_l);
            s->slot[s->right_id] = set_wheel(s->right_id, spd_r);
        }
        store_f(s->cmd_l_bits, spd_l);
        store_f(s->cmd_r_bits, spd_r);
        s->wheel_us.store(static_cast<uint32_t>(mono_ns() - t0), std::memory_order_relaxed);
        pace(next, s->can_period_ns, &s->late_wheel);
    }
    return nullptr;
}

uint32_t scan_bus(Bus& bus, int begin, int end, Feedback* fb);
bool pose_ready(const State& s);

void recover_bus(State* s) {
    std::printf("关节失控：卸力，并重新接通 %s。\n", s->args.iface.c_str());
    {
        Lock lk(s->mu_cmd);
        s->roll_goal = {};
        s->getup_goal = {};
    }
    s->roll_on.store(0);
    s->getup_on.store(0);
    s->balance_on.store(0);
    s->torque_off.store(1);
    s->sample_mask.store(0);
    s->phase.store(PhWait);
    if (s->bus && s->bus->ok()) {
        for (int round = 0; round < 4; ++round) {
            for (int id = 1; id <= 12; ++id) s->bus->send_stop(id);
        }
    }
    s->bus.reset();
    // PCAN-USB 常出现口还 UP、但 RX 已卡死；强制 down/up 清掉。
    bring_up_can(s->args.iface, s->args.config);
    for (int attempt = 0; attempt < 2 && g_sig == 0; ++attempt) {
        s->can_hb_ns.store(mono_ns(), std::memory_order_relaxed);
        if (!iface_up(s->args.iface)) bring_up_can(s->args.iface, s->args.config);
        auto bus = std::make_unique<Bus>(s->args.iface);
        if (!bus->ok()) {
            std::fprintf(stderr, "重连失败：%s\n", bus->error().c_str());
            if (!wait_for(s, 0.4)) break;
            continue;
        }
        Feedback found_fb[16]{};
        uint32_t found = scan_bus(*bus, s->cfg.start_id, s->cfg.end_id, found_fb);
        s->online = found;
        if (!pose_ready(*s)) {
            std::fprintf(stderr, "重连后关节还没齐（在线 %s）。\n", mask_text(found).c_str());
            if (!wait_for(s, 0.5)) break;
            continue;
        }
        for (int id = 1; id <= 15; ++id) {
            if (!(found & bit(id))) continue;
            Lock lk(s->mu_fb);
            s->fb[id] = found_fb[id];
        }
        s->bus = std::move(bus);
        s->link_quiet_until_ns.store(mono_ns() + 5000000000ull);
        s->cmd_ready_ns.store(mono_ns() + static_cast<uint64_t>(kBootArmS * 1e9f));
        s->link_recover.store(0);
        s->can_hb_ns.store(mono_ns(), std::memory_order_relaxed);
        std::printf("关节已重连，在线 %s。已卸力，再等 %.0f 秒后长按 Y 进入。\n", mask_text(found).c_str(),
                    kBootArmS);
        return;
    }
    s->link_quiet_until_ns.store(mono_ns() + 5000000000ull);
    s->link_recover.store(0);
    s->can_hb_ns.store(mono_ns(), std::memory_order_relaxed);
    std::fprintf(stderr, "这次没接上，五秒后再试。\n");
}

void* can_thread(void* arg) {
    auto* s = static_cast<State*>(arg);
    s->fifo_can.store(set_fifo(60) ? 1 : -1);
    int rr_stat = 1;
    int rr_cmd = 1;
    uint64_t last_wheel_q = 0;
    uint64_t last_wheel_cmd = 0;
    constexpr uint64_t kWheelCmdPeriodNs = 100000000ull;  // 轮速最多 10 Hz，防止 PCAN 缓冲爆掉
    struct Track {
        float deg = 0;
        uint64_t ns = 0;
        bool valid = false;
    } track[16];
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    add_ns(next, s->can_period_ns);
    auto fail = [&](bool ok) {
        if (!ok) s->tx_fail.fetch_add(1, std::memory_order_relaxed);
    };
    while (s->phase.load() != PhExit) {
        uint64_t t0 = mono_ns();
        if (!s->bus || s->link_recover.load() != 0) {
            if (s->link_recover.load() != 0 || mono_ns() >= s->link_quiet_until_ns.load()) recover_bus(s);
            s->can_hb_ns.store(mono_ns(), std::memory_order_relaxed);
            pace(next, s->can_period_ns, &s->late_can);
            continue;
        }
        ingest(s);
        uint32_t ph = s->phase.load();
        bool estop = s->estop.load() != 0;
        Slot slots[16];
        GetupGoal getup_goal;
        RollGoal roll_goal;
        {
            Lock lk(s->mu_cmd);
            std::memcpy(slots, s->slot, sizeof(slots));
            getup_goal = s->getup_goal;
            roll_goal = s->roll_goal;
        }
        if (ph == PhWait) {
            ShowCmd show = copy_show(s);
            if (roll_goal.active) getup_goal = {};
            // 翻滚/起身：位置指令用 0x00（不回包）。若 0x1F+0x0E 全关节齐发，PCAN-USB 缓冲会爆，
            // 观测角冻住 → 策略以为已站直 → 动作僵住，随后整口掉线。
            if (roll_goal.active) {
                for (int id = 1; id <= 15; ++id) {
                    if (!(s->online & bit(id)) || is_wheel(id) || roll_goal.tq[id] <= 0.f) continue;
                    float deg = roll_goal.deg[id];
                    float vel = roll_goal.cmd_vel > 0.1f ? roll_goal.cmd_vel : 6.f;
                    if (s->bus->send_position(id, pos_to_int(deg2rad(deg)), vel, roll_goal.tq[id])) {
                        track[id].deg = deg;
                        track[id].ns = t0;
                        track[id].valid = true;
                    } else {
                        s->tx_fail.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            } else if (getup_goal.active) {
                for (int id = 1; id <= 15; ++id) {
                    if (!(s->online & bit(id)) || is_wheel(id) || getup_goal.tq[id] <= 0.f) continue;
                    float deg = getup_goal.deg[id];
                    if (s->bus->send_position(id, pos_to_int(deg2rad(deg)), 6.f, getup_goal.tq[id])) {
                        track[id].deg = deg;
                        track[id].ns = t0;
                        track[id].valid = true;
                    } else {
                        s->tx_fail.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
            bool neck = show_live(show) && show.neck && show.mask == 0 && !getup_goal.active && !roll_goal.active;
            if (neck) {
                if (s->online & bit(11))
                    fail(s->bus->send_position(11, pos_to_int(deg2rad(show.n11)), show.nvel, show.ntq));
                if (s->online & bit(12))
                    fail(s->bus->send_position(12, pos_to_int(deg2rad(show.n12)), show.nvel, show.ntq));
            }
        } else if (ph == PhSlew || ph == PhSettle || ph == PhHold) {
            // 过渡到站立：一拍发齐，查询码 0x00。
            for (int id = 1; id <= 15; ++id) {
                if (!(s->online & bit(id)) || is_wheel(id) || slots[id].kind != 1) continue;
                float deg = rad2deg(slots[id].pos);
                if (s->bus->send_position(id, pos_to_int(slots[id].pos), slots[id].vel, slots[id].tq)) {
                    track[id].deg = deg;
                    track[id].ns = t0;
                    track[id].valid = true;
                } else {
                    s->tx_fail.fetch_add(1, std::memory_order_relaxed);
                }
            }
        } else if (ph == PhDrive) {
            // 驾驶：身体最多每拍 4 路轮询，避免和轮速一起把 PCAN 灌满。
            int budget = 4;
            for (int n = 0; n < 16 && budget > 0; ++n) {
                int id = rr_cmd;
                rr_cmd = id % 15 + 1;
                if (!(s->online & bit(id)) || is_wheel(id) || slots[id].kind != 1) continue;
                float deg = rad2deg(slots[id].pos);
                bool due = !track[id].valid || std::fabs(deg - track[id].deg) >= 0.05f ||
                           t0 - track[id].ns >= 100000000ull;
                if (!due) continue;
                if (s->bus->send_position(id, pos_to_int(slots[id].pos), slots[id].vel, slots[id].tq)) {
                    track[id].deg = deg;
                    track[id].ns = t0;
                    track[id].valid = true;
                    --budget;
                } else {
                    s->tx_fail.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }
        const bool skill = roll_goal.active || getup_goal.active;
        if (ph == PhDrive) s->torque_off.store(0);
        bool wheels_quiet = ph == PhStop || (s->torque_off.load() != 0 && ph != PhDrive);
        auto send_wheel = [&](int id) {
            if (!(s->online & bit(id)) || wheels_quiet) return;
            const Slot& sl = slots[id];
            uint64_t wh = s->wheel_hb_ns.load(std::memory_order_relaxed);
            bool wheel_stale = age_exceeds(mono_ns(), wh, 150000000ull);
            float vel = 0.f;
            if (ph == PhDrive && !estop && !wheel_stale && sl.kind == 2) vel = sl.vel;
            fail(s->bus->send_velocity(id, vel));
        };
        // 轮速指令封顶 10 Hz（左右各一帧），其余节拍留给身体位置和状态回包。
        if (t0 - last_wheel_cmd >= kWheelCmdPeriodNs) {
            send_wheel(s->left_id);
            send_wheel(s->right_id);
            last_wheel_cmd = t0;
        }
        const uint64_t wheel_q_period = 100000000ull;
        if (t0 - last_wheel_q >= wheel_q_period) {
            if (s->online & bit(s->left_id)) fail(s->bus->send_status(s->left_id));
            if (s->online & bit(s->right_id)) fail(s->bus->send_status(s->right_id));
            last_wheel_q = t0;
        }
        // 身体状态单独稀疏问：策略时每拍 3 路（约 15 Hz/关节），平时 1 路。绝不再用位置帧带 0x0E 灌回包。
        int stat_budget = skill ? 3 : 1;
        for (int n = 0; n < 16 && stat_budget > 0; ++n) {
            int id = rr_stat;
            rr_stat = id % 15 + 1;
            if ((s->online & bit(id)) && !is_wheel(id)) {
                fail(s->bus->send_status(id));
                --stat_budget;
            }
        }
        uint32_t want = s->sample_mask.load();
        int sample_budget = 2;
        for (int id = 1; id <= 15 && sample_budget > 0; ++id) {
            if (!(want & bit(id))) continue;
            fail(s->bus->send_status(id));
            --sample_budget;
        }
        ingest(s);
        if (ph == PhStop) {
            for (int id = 1; id <= 15; ++id)
                if (s->online & bit(id)) fail(s->bus->send_stop(id));
        }
        s->can_hb_ns.store(mono_ns(), std::memory_order_relaxed);
        s->can_us.store(static_cast<uint32_t>(mono_ns() - t0), std::memory_order_relaxed);
        pace(next, s->can_period_ns, &s->late_can);
    }
    return nullptr;
}

void* wd_thread(void* arg) {
    auto* s = static_cast<State*>(arg);
    s->fifo_wd.store(set_fifo(70) ? 1 : -1);
    Bus bus(s->args.iface);
    bool warned = false;
    bool latched = false;
    uint64_t lost_since = 0;
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    add_ns(next, 20000000);
    while (s->phase.load() != PhExit) {
        uint64_t now = mono_ns();
        uint32_t ph = s->phase.load();
        bool drive = ph == PhDrive;
        uint64_t wh = s->wheel_hb_ns.load(std::memory_order_relaxed);
        uint64_t ch = s->can_hb_ns.load(std::memory_order_relaxed);
        bool wheel_bad = drive && age_exceeds(now, wh, 150000000ull);
        bool can_bad = drive && age_exceeds(now, ch, 200000000ull);
        if ((wheel_bad || can_bad) && bus.ok()) {
            if (!latched) {
                std::fprintf(stderr, "看门：%s心跳中断。不切停止模式，轮子保持最后的位置指令。\n", can_bad ? "CAN" : "轮子");
                latched = true;
            }
        } else if (!wheel_bad && !can_bad) {
            latched = false;
        } else if (!bus.ok() && !warned) {
            std::fprintf(stderr, "看门没有自己的 CAN 口：%s\n", bus.error().c_str());
            warned = true;
        }
        bool watch = ph == PhWait || ph == PhSlew || ph == PhSettle || ph == PhDrive || ph == PhHold;
        static uint64_t last_rx = 0;
        static uint64_t rx_flat_since = 0;
        if (!watch || s->link_recover.load() != 0 || now < s->link_quiet_until_ns.load()) {
            lost_since = 0;
            rx_flat_since = 0;
            last_rx = read_rx_packets(s->args.iface);
        } else {
            bool down = !iface_up(s->args.iface);
            // 口还 UP 但电机回包计数长时间不动：PCAN RX 卡死或总线被灌死。
            // 不用“反馈变稀”判断，避免 RB 张望误触发；看内核 rx_packets。
            uint64_t rx = read_rx_packets(s->args.iface);
            bool rx_dead = false;
            if (rx > last_rx) {
                last_rx = rx;
                rx_flat_since = 0;
            } else if (rx_flat_since == 0) {
                rx_flat_since = now;
            } else if (now - rx_flat_since >= (down ? 600000000ull : 3000000000ull)) {
                // 口还 UP 时多等一会：翻滚高峰回包会短暂变稀，1.5s 误判会直接卸力僵住。
                rx_dead = true;
            }
            bool lost = down || rx_dead;
            if (!lost) {
                lost_since = 0;
            } else if (lost_since == 0) {
                lost_since = now;
            } else if (now - lost_since >= (down ? 600000000ull : 1500000000ull)) {
                s->link_recover.store(1);
                lost_since = 0;
                rx_flat_since = 0;
                std::fprintf(stderr, "关节失控：%s。准备卸力并重连。\n",
                             down ? "CAN 口掉了" : "电机回包中断");
            }
        }
        pace(next, 20000000, nullptr);
    }
    return nullptr;
}

uint32_t scan_bus(Bus& bus, int begin, int end, Feedback* fb) {
    uint32_t found = 0;
    auto once = [&](int id, int wait_ms) {
        bus.drain();
        // 用 0x0E 状态查询，不用 0x04 版本 ping：右轮 1 号对 ping 经常不回。
        bus.send_status(id);
        uint64_t t0 = mono_ns();
        while (mono_ns() - t0 < static_cast<uint64_t>(wait_ms) * 1000000ull) {
            Frame frame;
            if (!bus.poll(frame, 10)) continue;
            // 晚到的其它 ID 回包也收下，避免漏扫。
            if (frame.id < 1 || frame.id > 15) continue;
            decode_feedback(fb[frame.id], frame.data, frame.dlc, mono_ns());
            found |= bit(frame.id);
            if (frame.id == id) return;
        }
    };
    for (int id = begin; id <= end; ++id) once(id, 60);
    for (int pass = 0; pass < 3; ++pass) {
        for (int id = begin; id <= end; ++id)
            if (!(found & bit(id))) once(id, 120);
    }
    return found;
}

bool pose_ready(const State& s) {
    if (!(s.online & bit(1)) || !(s.online & bit(6))) return false;
    for (int id = s.cfg.start_id; id <= s.cfg.end_id; ++id) {
        if (is_wheel(id) || !s.stand.has(id)) continue;
        if (!(s.online & bit(id))) return false;
    }
    return true;
}

uint32_t body_mask(const State& s, const Pose& pose) {
    uint32_t mask = 0;
    for (int id = 1; id <= 15; ++id) {
        if (is_wheel(id) || !(s.online & bit(id)) || !pose.has(id)) continue;
        mask |= bit(id);
    }
    return mask;
}

bool fill_job(State* s, const Pose& pose, float torque, float vel, double min_s) {
    uint32_t mask = body_mask(*s, pose);
    if (!sample_motors(s, mask, 800)) {
        std::printf("有关节没读到新角度，用正在保持的位置继续走。\n");
    }
    Job job;
    job.t0 = mono_s();
    job.torque = torque;
    job.vel = vel;
    float span = 0;
    for (int id = 1; id <= 15; ++id) {
        if (!(mask & bit(id))) continue;
        Feedback fb = copy_fb(s, id);
        bool fresh = fb.t_pos_ns != 0 && !age_exceeds(mono_ns(), fb.t_pos_ns, 1000000000ull);
        float start = 0;
        if (fresh) {
            start = rad2deg(fb.position);
        } else {
            bool have_cmd = false;
            {
                Lock lk(s->mu_cmd);
                if (s->slot[id].kind == 1) {
                    start = rad2deg(s->slot[id].pos);
                    have_cmd = true;
                }
            }
            if (!have_cmd) {
                mask &= ~bit(id);
                continue;
            }
        }
        job.start_deg[id] = start;
        job.goal_deg[id] = pose.deg[id];
        span = std::max(span, std::fabs(job.goal_deg[id] - job.start_deg[id]));
    }
    job.mask = mask;
    if (mask == 0) {
        std::printf("没有可用的关节角度，这段过渡跳过。\n");
        return false;
    }
    job.duration = std::max(min_s, static_cast<double>(span / 40.f));
    {
        Lock lk(s->mu_job);
        s->job = job;
    }
    s->phase.store(PhSlew);
    std::printf("过渡 %.1f 秒，最大行程 %.1f°，力矩 %.1f N·m\n", job.duration, span, torque);
    return wait_for(s, job.duration);
}

bool legs_at_unload(State* s) {
    const int ids[] = {2, 3, 4, 5, 7, 8, 9, 10};
    for (int id : ids) {
        if (!s->unload.has(id) || !(s->online & bit(id))) continue;
        Feedback fb = copy_fb(s, id);
        if (fb.t_pos_ns == 0 || age_exceeds(mono_ns(), fb.t_pos_ns, 500000000ull)) return false;
        float tol = id == 7 ? 5.f : 2.f;
        if (std::fabs(rad2deg(fb.position) - s->unload.deg[id]) > tol) return false;
    }
    return true;
}

void cancel_skills(State* s) {
    {
        Lock lk(s->mu_cmd);
        s->roll_goal = {};
        s->getup_goal = {};
    }
    s->roll_on.store(0);
    s->getup_on.store(0);
}

bool release_torque(State* s) {
    if (hard_stop(s)) return false;
    cancel_skills(s);
    s->unload_hold.store(0);
    s->torque_off.store(1);
    s->estop.store(1);
    s->phase.store(PhStop);
    if (!wait_for(s, 0.25)) return false;
    s->estop.store(0);
    s->phase.store(PhWait);
    std::printf("已卸力。长按 Y %.0f 秒再进入。再按一次 Ctrl+C 退出。\n", kYHoldS);
    return true;
}

bool do_unload(State* s) {
    std::printf("停车：轮速归零。腿走到卸力姿态，到位后才卸力。\n");
    cancel_skills(s);
    s->estop.store(1);
    s->balance_on.store(0);
    s->unload_hold.store(0);
    s->phase.store(PhHold);
    if (!wait_for(s, 0.15)) return false;
    if (s->unload.mask == 0) {
        std::printf("没有卸力姿态，腿继续保持现在的角度。再按 B 就在当前位置卸力。\n");
        s->unload_hold.store(1);
        return true;
    }
    uint32_t rolls = body_mask(*s, s->unload) & (bit(4) | bit(9));
    if (rolls) {
        bool far = false;
        float peak = 0;
        for (int id : {4, 9}) {
            if (!(rolls & bit(id))) continue;
            Feedback fb = copy_fb(s, id);
            float now_deg = 0;
            bool known = fb.t_pos_ns != 0 && !age_exceeds(mono_ns(), fb.t_pos_ns, 1000000000ull);
            if (known) now_deg = rad2deg(fb.position);
            else {
                Lock lk(s->mu_cmd);
                if (s->slot[id].kind != 1) continue;
                now_deg = rad2deg(s->slot[id].pos);
            }
            float err = std::fabs(now_deg - s->unload.deg[id]);
            peak = std::max(peak, err);
            if (err > 2.f) far = true;
        }
        if (far) {
            Pose only;
            for (int id : {4, 9}) {
                if (!(rolls & bit(id))) continue;
                only.deg[id] = s->unload.deg[id];
                only.mask = static_cast<uint16_t>(only.mask | bit(id));
            }
            std::printf("先收髋 roll，最大还差 %.1f°。\n", peak);
            fill_job(s, only, s->args.torque, 2.5f, std::max(0.6, static_cast<double>(peak / 40.f)));
        }
    }
    if (!fill_job(s, s->unload, s->args.torque, 2.f, std::max(0.6, s->cfg.transition_s))) {
        std::printf("卸力姿态没能发出去，腿继续保持现在的角度。再按 B 就在当前位置卸力。\n");
        s->phase.store(PhHold);
        s->unload_hold.store(1);
        return true;
    }
    s->phase.store(PhSettle);
    s->unload_hold.store(1);
    std::printf("腿还在出力。到位后卸力。没到位时再按 B，就在当前位置卸力。\n");
    return true;
}

int read_counter(const std::string& iface, const char* name) {
    std::string path = "/sys/class/net/" + iface + "/statistics/" + name;
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return -1;
    long v = -1;
    if (std::fscanf(f, "%ld", &v) != 1) v = -1;
    std::fclose(f);
    return static_cast<int>(v);
}

const char* fifo_text(int v) { return v > 0 ? "实时" : (v < 0 ? "普通" : "未起"); }

const char* phase_name(uint32_t ph) {
    switch (ph) {
    case PhBoot: return "boot";
    case PhWait: return "wait";
    case PhSlew: return "slew";
    case PhSettle: return "settle";
    case PhDrive: return "drive";
    case PhCut: return "cut";
    case PhStop: return "stop";
    case PhHold: return "hold";
    default: return "exit";
    }
}

int write_packet(int fd, const char* data, size_t n) {
    size_t off = 0;
    while (off < n) {
        ssize_t w = ::send(fd, data + off, n - off, MSG_NOSIGNAL);
        if (w < 0) {
            if (errno == EINTR) continue;
            if ((errno == EAGAIN || errno == EWOULDBLOCK) && off == 0) return 0;
            return -1;
        }
        if (w == 0) return -1;
        off += static_cast<size_t>(w);
    }
    return 1;
}

void apply_show_line(ShowCmd& cmd, const char* line) {
    if (std::strncmp(line, "N ", 2) == 0) {
        int on = 0;
        float a = 0, b = 0, vel = 1.2f, tq = 1.f;
        if (std::sscanf(line, "N %d %f %f %f %f", &on, &a, &b, &vel, &tq) >= 1) {
            cmd.neck = on != 0;
            if (cmd.neck) {
                cmd.n11 = a;
                cmd.n12 = b;
                cmd.nvel = vel;
                cmd.ntq = tq;
            }
        }
        return;
    }
    if (std::strncmp(line, "W ", 2) == 0) {
        int lock = 0;
        if (std::sscanf(line, "W %d", &lock) == 1) cmd.lock_wheels = lock != 0;
        return;
    }
    if (std::strncmp(line, "F ", 2) == 0) {
        float yaw = 0, roll = 0;
        if (std::sscanf(line, "F %f %f", &yaw, &roll) == 2) {
            cmd.has_servo = true;
            cmd.servo_yaw = yaw;
            cmd.servo_roll = roll;
        }
        return;
    }
    if (std::strncmp(line, "J ", 2) == 0) {
        int id = 0;
        float deg = 0, vel = 6.f, tq = 2.f;
        if (std::sscanf(line, "J %d %f %f %f", &id, &deg, &vel, &tq) == 4 && id >= 1 && id <= 15) {
            cmd.mask = static_cast<uint16_t>(cmd.mask | bit(id));
            cmd.deg[id] = deg;
            cmd.vel[id] = vel;
            cmd.tq[id] = tq;
        }
    }
}

int recv_show(int fd, std::string& acc, ShowCmd& cmd) {
    char buf[1024];
    double deadline = mono_s() + 0.03;
    while (mono_s() < deadline) {
        ssize_t r = ::recv(fd, buf, sizeof(buf), 0);
        if (r < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                timespec ts{0, 2000000};
                nanosleep(&ts, nullptr);
                continue;
            }
            return -1;
        }
        if (r == 0) return -1;
        acc.append(buf, buf + r);
        size_t at = 0;
        bool done = false;
        while (true) {
            size_t nl = acc.find('\n', at);
            if (nl == std::string::npos) {
                acc.erase(0, at);
                break;
            }
            std::string line = acc.substr(at, nl - at);
            at = nl + 1;
            if (line == ".") {
                done = true;
                break;
            }
            if (!line.empty()) apply_show_line(cmd, line.c_str());
        }
        if (done) {
            acc.erase(0, at);
            return 1;
        }
    }
    return 0;
}

void* show_thread(void* arg) {
    auto* s = static_cast<State*>(arg);
    const char* sock_path = "/tmp/cyberduck-wheel-show.sock";
    const char* script = "/home/aidlux/cyberDuck/sport_ctrl/wheel_run/cpp/show_bridge.py";
    ::unlink(sock_path);
    int listen_fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        std::fprintf(stderr, "表演通道打不开。\n");
        return nullptr;
    }
    int fd_flags = fcntl(listen_fd, F_GETFD, 0);
    if (fd_flags >= 0) fcntl(listen_fd, F_SETFD, fd_flags | FD_CLOEXEC);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", sock_path);
    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 || ::listen(listen_fd, 1) < 0) {
        std::fprintf(stderr, "表演通道绑定失败。\n");
        ::close(listen_fd);
        ::unlink(sock_path);
        return nullptr;
    }
    pid_t child = -1;
    double respawn_at = 0;
    auto spawn = [&]() {
        if (child > 0) return;
        char* argv[] = {const_cast<char*>("python3"), const_cast<char*>("-u"), const_cast<char*>(script),
                        const_cast<char*>(sock_path), const_cast<char*>(s->args.config.c_str()), nullptr};
        int rc = ::posix_spawnp(&child, "python3", nullptr, nullptr, argv, ::environ);
        if (rc != 0) {
            child = -1;
            respawn_at = mono_s() + 2.0;
            std::fprintf(stderr, "表演程序没能启动：%s\n", std::strerror(rc));
            return;
        }
        std::printf("表演程序已启动。RB 头部张望，右扳机鸭叫，X 切灯，十字键上下调音量，LB 起身，LT+上向前翻滚。\n");
    };
    spawn();
    int client = -1;
    std::string acc;
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    add_ns(next, 20000000);
    while (s->phase.load() != PhExit) {
        if (child > 0) {
            int st = 0;
            pid_t got = ::waitpid(child, &st, WNOHANG);
            if (got == child) {
                std::fprintf(stderr, "表演程序退出了，稍后再开。\n");
                child = -1;
                respawn_at = mono_s() + 2.0;
                if (client >= 0) {
                    ::close(client);
                    client = -1;
                }
            }
        } else if (mono_s() >= respawn_at) {
            spawn();
        }
        if (client < 0) {
            pollfd pfd{};
            pfd.fd = listen_fd;
            pfd.events = POLLIN;
            if (::poll(&pfd, 1, 0) > 0) {
                client = ::accept(listen_fd, nullptr, nullptr);
                if (client >= 0) {
                    timeval tv{0, 20000};
                    ::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
                    ::setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
                    int cflags = fcntl(client, F_GETFD, 0);
                    if (cflags >= 0) fcntl(client, F_SETFD, cflags | FD_CLOEXEC);
                    acc.clear();
                    std::printf("表演程序已接上。\n");
                }
            }
        }
        if (client >= 0) {
            char out[4096];
            int n = 0;
            GetupGoal goal;
            {
                Lock lk(s->mu_cmd);
                goal = s->getup_goal;
            }
            RollGoal roll_goal;
            {
                Lock lk(s->mu_cmd);
                roll_goal = s->roll_goal;
            }
            if (roll_goal.active) {
                n += std::snprintf(out + n, sizeof(out) - static_cast<size_t>(n), "G 1 %.3f %.3f\n", roll_goal.yaw, roll_goal.roll);
            } else if (goal.active) {
                n += std::snprintf(out + n, sizeof(out) - static_cast<size_t>(n), "G 1 %.3f %.3f\n", goal.yaw, goal.roll);
            } else {
                n += std::snprintf(out + n, sizeof(out) - static_cast<size_t>(n), "G 0\n");
            }
            n += std::snprintf(out + n, sizeof(out) - static_cast<size_t>(n), "P %s\n", phase_name(s->phase.load()));
            ImuSnap imu = copy_imu(s);
            if (imu.ok) {
                n += std::snprintf(out + n, sizeof(out) - static_cast<size_t>(n), "I 1 %.5f %.5f %.5f %.5f %.5f %.5f\n",
                                   imu.gyro[0], imu.gyro[1], imu.gyro[2], imu.grav[0], imu.grav[1], imu.grav[2]);
            } else {
                n += std::snprintf(out + n, sizeof(out) - static_cast<size_t>(n), "I 0\n");
            }
            uint64_t now = mono_ns();
            for (int id = 1; id <= 15 && n < 3600; ++id) {
                if (!(s->online & bit(id))) continue;
                Feedback fb = copy_fb(s, id);
                int age = -1;
                if (fb.t_pos_ns != 0 && now >= fb.t_pos_ns) age = static_cast<int>((now - fb.t_pos_ns) / 1000000ull);
                n += std::snprintf(out + n, sizeof(out) - static_cast<size_t>(n), "M %d %.5f %.5f %.4f %d\n", id, fb.position,
                                   fb.velocity, fb.torque, age);
            }
            n += std::snprintf(out + n, sizeof(out) - static_cast<size_t>(n), ".\n");
            int wrote = n < 0 ? -1 : write_packet(client, out, static_cast<size_t>(n));
            if (wrote < 0) {
                ::close(client);
                client = -1;
            } else if (wrote > 0) {
                ShowCmd cmd;
                int rc = recv_show(client, acc, cmd);
                if (rc < 0) {
                    ::close(client);
                    client = -1;
                } else if (rc > 0) {
                    cmd.t_ns = mono_ns();
                    Lock lk(s->mu_show);
                    s->show = cmd;
                }
            }
        }
        pace(next, 20000000, nullptr);
    }
    if (client >= 0) ::close(client);
    ::close(listen_fd);
    ::unlink(sock_path);
    if (child > 0) {
        ::kill(child, SIGTERM);
        for (int i = 0; i < 10; ++i) {
            if (::waitpid(child, nullptr, WNOHANG) == child) {
                child = -1;
                break;
            }
            ::usleep(100000);
        }
        if (child > 0) {
            ::kill(child, SIGKILL);
            ::waitpid(child, nullptr, 0);
        }
    }
    return nullptr;
}

}  // namespace

std::string setup_script_path(const std::string& config) {
    std::string from_config;
    const auto slash = config.rfind("/sport_ctrl/");
    if (slash != std::string::npos) from_config = config.substr(0, slash) + "/motor/setup_can0.sh";
    const char* candidates[] = {from_config.c_str(), "/home/aidlux/cyberDuck/motor/setup_can0.sh",
                                "motor/setup_can0.sh"};
    for (const char* path : candidates) {
        if (path != nullptr && path[0] != '\0' && ::access(path, X_OK) == 0) return path;
    }
    return "/home/aidlux/cyberDuck/motor/setup_can0.sh";
}

int bring_up_can(const std::string& iface, const std::string& config) {
    const std::string script = setup_script_path(config);
    std::printf("%s 处于 DOWN，正在执行 %s。\n", iface.c_str(), script.c_str());
    std::fflush(stdout);
    pid_t child = 0;
    char* argv[] = {const_cast<char*>(script.c_str()), const_cast<char*>(iface.c_str()), nullptr};
    int rc = ::posix_spawn(&child, script.c_str(), nullptr, nullptr, argv, ::environ);
    if (rc != 0) {
        std::fprintf(stderr, "拉起 %s 失败：%s\n", iface.c_str(), std::strerror(rc));
        return -1;
    }
    int status = 0;
    if (::waitpid(child, &status, 0) < 0) {
        std::fprintf(stderr, "拉起 %s 失败：%s\n", iface.c_str(), std::strerror(errno));
        return -1;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        std::fprintf(stderr, "拉起 %s 失败，脚本退出码 %d。\n", iface.c_str(),
                     WIFEXITED(status) ? WEXITSTATUS(status) : -1);
        return -1;
    }
    std::printf("%s 已拉起。\n", iface.c_str());
    return 0;
}

void tick_roll(State* s) {
    if (!s->roll.ready()) return;
    uint64_t now = mono_ns();
    float dt = 0.02f;
    if (s->roll_prev_ns != 0 && now > s->roll_prev_ns) dt = static_cast<float>(now - s->roll_prev_ns) * 1e-9f;
    s->roll_prev_ns = now;
    uint32_t ph = s->phase.load();
    bool allow = ph == PhDrive || ph == PhWait;
    PadSnap pad = copy_pad(s);
    bool up = pad.connected && pad.axis[7] <= -0.5f;
    bool chord = up && pad.lt;
    bool rise = chord && !s->roll_chord;
    s->roll_chord = chord;
    ImuSnap imu = copy_imu(s);
    if (!imu.ok) {
        if (rise) std::printf("没有 IMU，向前翻滚不能跑。\n");
        if (s->roll_on.load()) {
            Lock lk(s->mu_cmd);
            s->roll_goal = {};
            s->roll_on.store(0);
        }
        return;
    }
    RollSense sense;
    for (int i = 0; i < 3; ++i) {
        sense.gyro[i] = imu.gyro[i];
        sense.grav[i] = imu.grav[i];
    }
    for (int id = 1; id <= 12; ++id) {
        Feedback fb = copy_fb(s, id);
        sense.pos[id] = fb.position;
        sense.vel[id] = fb.velocity;
    }
    ShowCmd show = copy_show(s);
    if (show.has_servo) {
        sense.servo_yaw = show.servo_yaw;
        sense.servo_roll = show.servo_roll;
    }
    if (rise && s->cmd_ready_ns.load() != 0 && mono_ns() < s->cmd_ready_ns.load()) {
        std::printf("还要再等 %.1f 秒再翻滚，CAN 刚起来先别推指令。\n",
                    static_cast<double>(s->cmd_ready_ns.load() - mono_ns()) * 1e-9);
        rise = false;
    }
    sense.chord_rise = rise;
    sense.allow = allow;
    sense.other_busy = s->getup_on.load() != 0;
    RollGoal goal;
    s->roll.step(dt, sense, goal);
    if (goal.say_start) {
        if (s->roll.csv_mode()) {
            std::printf("LT+上：CSV 慢放，先走到仿真默认姿态，再按帧播放。力矩 2 N·m。按 B 立刻卸力。\n");
        } else {
            std::printf("LT+上：先走到翻滚初始姿态（仿真默认角），到位后再推理。力矩 2 N·m，速度上限 150 RPM。按 B 立刻卸力，再按 LT+上退出。\n");
        }
    }
    if (goal.say_ready) {
        if (s->roll.csv_mode()) std::printf("初始姿态到位，开始 CSV 慢放。\n");
        else std::printf("初始姿态到位，开始 hard150/2997 翻滚推理。日志 logs/policy_roll_*.csv。\n");
    }
    if (goal.say_stop) std::printf("LT+上：退出向前翻滚。\n");
    if (goal.say_done) {
        if (s->roll.csv_mode()) {
            std::printf("CSV 慢放结束。超时未到位 %d 帧。\n", s->roll.miss_frames());
        } else {
            std::printf("向前翻滚已跑满 5 秒，退出。\n");
        }
    }
    if (goal.say_busy) std::printf("起身还在执行，向前翻滚不启动。\n");
    if (goal.say_refuse) std::printf("现在不能翻滚。先长按 Y 站起来，或停在等待。\n");
    {
        Lock lk(s->mu_cmd);
        s->roll_goal = goal;
    }
    s->roll_on.store(goal.active ? 1 : 0);
}

void tick_getup(State* s) {
    if (!s->getup.ready()) return;
    uint64_t now = mono_ns();
    float dt = 0.02f;
    if (s->getup_prev_ns != 0 && now > s->getup_prev_ns) dt = static_cast<float>(now - s->getup_prev_ns) * 1e-9f;
    s->getup_prev_ns = now;
    uint32_t ph = s->phase.load();
    bool allow = ph == PhDrive || ph == PhWait;
    PadSnap pad = copy_pad(s);
    bool rise = pad.connected && pad.lb_held && !s->getup_lb && s->roll_on.load() == 0;
    s->getup_lb = pad.connected && pad.lb_held;
    if (rise && s->cmd_ready_ns.load() != 0 && mono_ns() < s->cmd_ready_ns.load()) {
        std::printf("还要再等 %.1f 秒再起身，CAN 刚起来先别推指令。\n",
                    static_cast<double>(s->cmd_ready_ns.load() - mono_ns()) * 1e-9);
        rise = false;
    }
    ImuSnap imu = copy_imu(s);
    if (!imu.ok) return;
    GetupSense sense;
    for (int i = 0; i < 3; ++i) {
        sense.gyro[i] = imu.gyro[i];
        sense.grav[i] = imu.grav[i];
    }
    for (int id = 1; id <= 12; ++id) {
        Feedback fb = copy_fb(s, id);
        sense.pos[id] = fb.position;
        sense.vel[id] = fb.velocity;
    }
    ShowCmd show = copy_show(s);
    if (show.has_servo) {
        sense.servo_yaw = show.servo_yaw;
        sense.servo_roll = show.servo_roll;
    }
    sense.lb_rise = rise;
    sense.allow = allow;
    GetupGoal goal;
    s->getup.step(dt, sense, goal);
    if (goal.say_fall) std::printf("已倒地。起身策略在推理，按 LB 才执行。\n");
    if (goal.say_refuse) std::printf("还没倒地，起身策略不执行。\n");
    if (goal.say_start) std::printf("LB：已倒地，起身策略开始执行。腿、头和脖子一起动，不限力矩，再按 LB 退出。\n");
    if (goal.say_stop) std::printf("LB：退出起身策略。推理继续，不执行。\n");
    {
        Lock lk(s->mu_cmd);
        s->getup_goal = goal;
    }
    s->getup_on.store(goal.active ? 1 : 0);
}

int run(const Args& args) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::setvbuf(stderr, nullptr, _IOLBF, 0);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    if (args.can_log) can_log_open();
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::printf("内存没有锁住（%s）。实时节拍仍按时钟走。\n", std::strerror(errno));
    }
    auto state = std::make_unique<State>();
    State* s = state.get();
    s->args = args;
    std::string err;
    if (!load_yaml(args.config, s->cfg, err)) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 2;
    }
    auto it = s->cfg.poses.find(args.pose);
    if (it == s->cfg.poses.end() || it->second.mask == 0) {
        std::fprintf(stderr, "姿态 %s 没有角度\n", args.pose.c_str());
        return 2;
    }
    s->stand = it->second;
    if (auto ext = s->cfg.poses.find("extended"); ext != s->cfg.poses.end()) s->extended = ext->second;
    if (auto un = s->cfg.poses.find("unload"); un != s->cfg.poses.end()) s->unload = un->second;
    if (auto l = s->cfg.poses.find("lean_left"); l != s->cfg.poses.end()) s->lean_l = l->second;
    if (auto r = s->cfg.poses.find("lean_right"); r != s->cfg.poses.end()) s->lean_r = r->second;
    s->lift_ok = true;
    for (int id : {2, 3, 7, 8}) s->lift_ok = s->lift_ok && s->stand.has(id) && s->extended.has(id);
    s->left_id = args.swap_wheels ? 1 : 6;
    s->right_id = args.swap_wheels ? 6 : 1;
    s->slew_vel = static_cast<float>(std::max(1.5, std::fabs(s->cfg.vel_limit)));
    int wheel_hz = std::clamp(args.wheel_hz, 50, 500);
    int body_hz = std::clamp(args.body_hz, 20, 500);
    s->can_period_ns = 1000000000ull / static_cast<uint64_t>(wheel_hz);
    s->body_period_ns = 1000000000ull / static_cast<uint64_t>(body_hz);
    s->args.iface = args.iface;

    std::printf("轮行实时线程：看门 70、CAN 60、轮子 %d Hz 优先级 50、身体 %d Hz 优先级 40、IMU 30、手柄 20。\n", wheel_hz,
                body_hz);
    std::printf("CAN 线程独占总线。主线程只切换状态和写日志。\n");
    std::printf("RB 头部张望、嘴、灯环、音量、起身和翻滚走伴随程序，关节角仍由这条 CAN 线程发出。\n");

    pthread_t th_pad{};
    pthread_t th_imu{};
    pthread_t th_body{};
    pthread_t th_wheel{};
    pthread_t th_can{};
    pthread_t th_wd{};
    pthread_t th_show{};
    bool up_pad = false, up_imu = false, up_body = false, up_wheel = false, up_can = false, up_wd = false, up_show = false;
    up_pad = start_thread(th_pad, pad_thread, s, "wheel-pad");
    if (!up_pad) {
        std::fprintf(stderr, "手柄线程没起来\n");
        return 1;
    }

    while (g_sig == 0) {
        auto bus = std::make_unique<Bus>(args.iface);
        if (!bus->ok()) {
            if (bus->error().find("处于 DOWN") != std::string::npos && bring_up_can(args.iface, args.config) == 0) {
                continue;
            }
            std::fprintf(stderr, "%s。继续等。\n", bus->error().c_str());
            if (!wait_for(s, 1.0)) break;
            continue;
        }
        Feedback found_fb[16]{};
        uint32_t found = scan_bus(*bus, s->cfg.start_id, s->cfg.end_id, found_fb);
        s->online = found;
        if (!pose_ready(*s)) {
            std::fprintf(stderr, "关节还没齐（在线 %s）。\n", mask_text(found).c_str());
            if (!wait_for(s, 0.8)) break;
            continue;
        }
        for (int id = 1; id <= 15; ++id) {
            if (!(found & bit(id))) continue;
            s->fb[id] = found_fb[id];
        }
        ::usleep(20000);
        s->bus = std::move(bus);
        break;
    }
    if (!s->bus || g_sig) {
        s->phase.store(PhExit);
        if (up_pad) pthread_join(th_pad, nullptr);
        return g_sig ? 0 : 1;
    }
    std::printf("关节在线 %s。左轮 %d，右轮 %d。轮子速度模式，不设圈数。不按 LT ±%.0f rad/s，按住 ±%.0f。\n",
                mask_text(s->online).c_str(), s->left_id, s->right_id, s->args.vmax * 0.5f, s->args.vmax);
    if (!s->lift_ok) std::printf("没有完整的伸直姿态，升降锁定。\n");

    if (!args.no_balance) {
        s->imu = std::make_unique<imu::Lsm6>();
        if (!s->imu->open()) {
            std::printf("IMU 没打开，自平衡不可用。\n");
            s->imu.reset();
        } else {
            std::printf("IMU 已打开。自平衡默认关，驾驶中短按 Y 切换。\n");
        }
    }
    if (s->getup.load("/home/aidlux/cyberDuck/sport_ctrl/policies/倒地起身/actor_1999.bin")) {
        std::printf("起身策略已载入本程序。推理一直在跑。倒地后按 LB 才执行，再按 LB 退出。\n");
    } else {
        std::printf("起身策略没载入（%s）。\n", s->getup.reason().c_str());
    }
    if (s->roll.load("/home/aidlux/cyberDuck/sport_ctrl/policies/cyberduck_v2_roulade_hard150_2997/actor_2997.bin")) {
        std::printf("向前翻滚 hard150/2997 已载入。符号对齐 check_joint_signs。按住 LT+上：先到仿真默认姿态，再推理 5 秒。力矩 2 N·m，速度 150 RPM。\n");
    } else {
        std::printf("向前翻滚没载入（%s）。\n", s->roll.reason().c_str());
    }
    std::printf("驾驶和翻滚中摇杆、B 卸力保持有效。只有 CAN 口掉线才自动重连。\n");
    s->phase.store(PhWait);
    // 扫完关节后总线还要喘口气；马上推轮速/翻滚容易把 PCAN-USB 冲挂。
    s->cmd_ready_ns.store(mono_ns() + static_cast<uint64_t>(kBootArmS * 1e9f));
    if (s->imu) up_imu = start_thread(th_imu, imu_thread, s, "wheel-imu");
    up_body = start_thread(th_body, body_thread, s, "wheel-body");
    up_wheel = start_thread(th_wheel, wheel_thread, s, "wheel-speed");
    up_can = start_thread(th_can, can_thread, s, "wheel-can");
    up_wd = start_thread(th_wd, wd_thread, s, "wheel-wd");
    up_show = start_thread(th_show, show_thread, s, "wheel-show");
    if (!up_body || !up_wheel || !up_can || !up_wd) {
        std::fprintf(stderr, "实时线程没起齐，退出。\n");
        s->phase.store(PhExit);
    } else {
        wait_for(s, 0.05);
        std::printf("优先级：CAN %s，轮子 %s，身体 %s，IMU %s，手柄 %s，看门 %s。\n", fifo_text(s->fifo_can.load()),
                    fifo_text(s->fifo_wheel.load()), fifo_text(s->fifo_body.load()), fifo_text(s->fifo_imu.load()),
                    fifo_text(s->fifo_pad.load()), fifo_text(s->fifo_wd.load()));
        if (s->fifo_can.load() < 0 || s->fifo_wheel.load() < 0) {
            std::printf("没有拿到 SCHED_FIFO。用 root 启动才能保证电机节拍不被日志和手柄抢走。\n");
        }
        std::printf("上电后先等 %.0f 秒再给指令（Y / LT+上 / LB），等 PCAN 稳定。\n", kBootArmS);
    }

    bool announced = false;
    bool y_down = false;
    double y_t0 = 0;
    bool y_block = false;
    bool auto_once = false;
    bool speed_checked = false;
    bool arm_announced = false;
    int last_gear = -1;
    int last_frozen = 0;
    int last_lean = 0;
    uint32_t last_edge_b = 0;
    uint32_t last_edge_y = 0;
    bool was_connected = false;
    bool was_bogus = false;
    double next_print = 0;
    double next_log = 0;
    // 策略主循环锁 50 Hz，避免 nanosleep(20ms) 叠加上次耗时掉到 ~43 Hz。
    constexpr uint64_t kPolicyPeriodNs = 20000000ull;
    timespec policy_next{};
    clock_gettime(CLOCK_MONOTONIC, &policy_next);
    add_ns(policy_next, kPolicyPeriodNs);
    FILE* log = nullptr;
    auto close_log = [&]() {
        if (!log) return;
        std::fflush(log);
        std::fclose(log);
        log = nullptr;
    };
    auto cmds_ready = [&]() -> bool {
        uint64_t ready = s->cmd_ready_ns.load();
        if (ready == 0 || mono_ns() >= ready) return true;
        double left = static_cast<double>(ready - mono_ns()) * 1e-9;
        std::printf("还要再等 %.1f 秒，CAN 刚起来先别推指令。\n", left);
        return false;
    };
    auto start_slew = [&]() -> bool {
        if (!cmds_ready()) return false;
        std::printf("开始进入轮行姿态 %s。\n", args.pose.c_str());
        s->estop.store(1);
        if (!fill_job(s, s->stand, args.torque, s->slew_vel, std::max(0.5, s->cfg.transition_s))) {
            s->estop.store(0);
            if (s->phase.load() != PhStop) s->phase.store(PhWait);
            return false;
        }
        if (g_sig) return false;
        s->phase.store(PhSettle);
        std::printf("保持目标角。\n");
        if (!wait_for(s, 0.6)) return false;
        if (g_sig) return false;
        s->estop.store(0);
        s->drive_t0_ns.store(mono_ns());
        speed_checked = false;
        arm_announced = false;
        last_gear = -1;
        s->phase.store(PhDrive);
        std::string dir = "/home/aidlux/cyberDuck/sport_ctrl/wheel_run/logs";
        ::mkdir(dir.c_str(), 0755);
        std::time_t wall = std::time(nullptr);
        std::tm tm{};
        localtime_r(&wall, &tm);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);
        std::string path = dir + "/drive_cpp_" + stamp + ".csv";
        close_log();
        log = std::fopen(path.c_str(), "w");
        if (log) {
            std::fprintf(log, "t_s,fwd,turn,spin,gear,height_u,roll_u,cmd_l,cmd_r,fb_l,fb_r,tor_l,tor_r,voltage,lean,hip,balance,can_us,wheel_us,late_can,late_wheel,tx_fail\n");
            std::printf("驾驶日志 %s\n", path.c_str());
        }
        std::printf("已站稳。前 %.0f 秒轮速为 0。左摇杆前进和转弯，右摇杆左右自转、上下升降。\n", kSpeedArmS);
        std::printf("RB 开关头部张望。右扳机张嘴并鸭叫。X 切灯。十字键上下调音量，左右侧倾。LB 起身，LT+上向前翻滚。短按 Y 切换平衡。\n");
        std::printf("默认半速，按住 LT 全速。十字键左右侧倾。短按 Y 开关自平衡。B 卸力。Ctrl+C 退出。\n");
        next_print = mono_s();
        next_log = mono_s();
        return true;
    };

    while (s->phase.load() != PhExit && g_sig < 2) {
        if (g_sig >= 1) {
            if (s->phase.load() != PhWait && s->phase.load() != PhStop) do_unload(s);
            break;
        }
        tick_roll(s);
        tick_getup(s);
        uint32_t ph = s->phase.load();
        PadSnap pad = copy_pad(s);
        if (pad.connected != was_connected) {
            was_connected = pad.connected;
            if (pad.connected) std::printf("手柄已接入 %s（%s）。\n", pad.path, pad.xbox ? "Xbox" : "北通");
            else std::printf("手柄断开，轮子保持当前位置。\n");
        }
        if (pad.bogus != was_bogus) {
            was_bogus = pad.bogus;
            std::printf("%s\n", pad.bogus ? "手柄模式异常，本帧指令作废。" : "手柄摇杆已恢复。");
        }
        if (ph == PhWait) {
            if (!announced) {
                std::printf("已卸力。长按 Y %.0f 秒进入轮行，Ctrl+C 退出。\n", kYHoldS);
                announced = true;
            }
            if (args.no_wait_y && !auto_once) {
                auto_once = true;
                if (!start_slew()) announced = false;
                continue;
            }
            if (pad.edge_b != last_edge_b) {
                last_edge_b = pad.edge_b;
                std::printf("B：再卸一次力。\n");
                release_torque(s);
                announced = false;
                continue;
            }
            if (pad.y_held && !y_block) {
                if (!y_down) {
                    y_down = true;
                    y_t0 = mono_s();
                } else if (mono_s() - y_t0 >= kYHoldS) {
                    y_down = false;
                    y_block = true;
                    last_edge_y = pad.edge_y;
                    if (!start_slew()) announced = false;
                }
            } else {
                y_down = false;
                if (!pad.y_held) y_block = false;
            }
        } else if (s->unload_hold.load() && (ph == PhSettle || ph == PhHold)) {
            if (legs_at_unload(s)) {
                std::printf("卸力姿态已到位。\n");
                release_torque(s);
                announced = false;
                continue;
            }
            if (pad.edge_b != last_edge_b) {
                last_edge_b = pad.edge_b;
                std::printf("再按 B：腿还没到卸力姿态，在当前位置卸力。\n");
                release_torque(s);
                announced = false;
                continue;
            }
        } else if (ph == PhDrive) {
            if (pad.edge_b != last_edge_b) {
                last_edge_b = pad.edge_b;
                close_log();
                if (s->roll_on.load() || s->getup_on.load()) {
                    std::printf("B：退出策略，立刻卸力。\n");
                    release_torque(s);
                } else {
                    std::printf("B：卸力停下。\n");
                    do_unload(s);
                }
                if (g_sig) break;
                announced = false;
                continue;
            }
            if (pad.edge_y != last_edge_y) {
                last_edge_y = pad.edge_y;
                if (s->imu) {
                    uint8_t on = s->balance_on.load() ? 0 : 1;
                    s->balance_on.store(on);
                    std::printf("%s\n", on ? "Y：进入自平衡。" : "Y：退出自平衡，补偿回零。");
                } else {
                    std::printf("没有 IMU，自平衡不能开。\n");
                }
            }
            double now = mono_s();
            if (!speed_checked && now - static_cast<double>(s->drive_t0_ns.load()) * 1e-9 > 0.6) {
                Feedback fl = copy_fb(s, s->left_id);
                Feedback fr = copy_fb(s, s->right_id);
                bool fresh = fl.t_pos_ns && fr.t_pos_ns && mono_ns() - fl.t_pos_ns < 300000000ull &&
                             mono_ns() - fr.t_pos_ns < 300000000ull;
                if (fresh) {
                    speed_checked = true;
                    float vl = fl.velocity * wheel_sign(s->left_id);
                    float vr = fr.velocity * wheel_sign(s->right_id);
                    if (std::fabs(vl) > 1.f || std::fabs(vr) > 1.f) {
                        std::fprintf(stderr, "轮子没有停住：左%+.2f 右%+.2f rad/s，取消驾驶。\n", vl, vr);
                        close_log();
                        if (!do_unload(s) || g_sig) break;
                        announced = false;
                        continue;
                    }
                    std::printf("轮子已停住（左%+.2f 右%+.2f rad/s）。\n", vl, vr);
                } else if (now - static_cast<double>(s->drive_t0_ns.load()) * 1e-9 > 1.5) {
                    speed_checked = true;
                    std::printf("还没读到轮速反馈，继续保持，摇杆仍要等满 %.0f 秒。\n", kSpeedArmS);
                }
            }
            if (!arm_announced && mono_ns() - s->drive_t0_ns.load() >= static_cast<uint64_t>(kSpeedArmS * 1e9f)) {
                arm_announced = true;
                std::printf("轮速已解锁。\n");
            }
            if (pad.gear != last_gear) {
                last_gear = pad.gear;
                std::printf("%s：上限 ±%.0f rad/s\n", pad.gear ? "全速" : "半速", args.vmax * (pad.gear ? 1.f : 0.5f));
            }
            int frozen = s->hip_frozen.load();
            if (frozen != last_frozen) {
                last_frozen = frozen;
                if (frozen) std::printf("髋力矩已顶住，补偿停在 %+.1f°。\n", load_f(s->hip_bits));
            }
            int lean = s->lean_dir.load();
            if (lean != last_lean) {
                last_lean = lean;
                if (lean == 0) std::printf("侧倾回中。\n");
                else std::printf("侧倾：%s（%s）。\n", lean > 0 ? "向右" : "向左", pad.lt ? "满幅" : "半幅");
            }
            if (log && now >= next_log) {
                Feedback fl = copy_fb(s, s->left_id);
                Feedback fr = copy_fb(s, s->right_id);
                float volts = copy_fb(s, 6).voltage;
                std::fprintf(log, "%.3f,%.3f,%.3f,%.3f,%d,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d,%u,%u,%u,%u,%u\n",
                             now - static_cast<double>(s->drive_t0_ns.load()) * 1e-9, pad.fwd, pad.turn, pad.spin, pad.gear,
                             load_f(s->height_bits), load_f(s->roll_bits), load_f(s->cmd_l_bits), load_f(s->cmd_r_bits),
                             fl.velocity * wheel_sign(s->left_id), fr.velocity * wheel_sign(s->right_id),
                             fl.torque * wheel_sign(s->left_id), fr.torque * wheel_sign(s->right_id), volts, load_f(s->lean_bits),
                             load_f(s->hip_bits), s->balance_on.load() ? 1 : 0, s->can_us.load(), s->wheel_us.load(),
                             s->late_can.load(), s->late_wheel.load(), s->tx_fail.load());
                next_log = now + 0.05;
            }
            if (now >= next_print) {
                Feedback fl = copy_fb(s, s->left_id);
                Feedback fr = copy_fb(s, s->right_id);
                float volts = copy_fb(s, 6).voltage;
                std::printf("t=%5.1f  杆 %+.2f/%+.2f/%+.2f  升降 %.2f 侧倾 %+.2f 前倾 %+.1f 补偿 %+.1f  %s  "
                            "轮令 %+.2f/%+.2f  反馈 %+.2f/%+.2f  %.1fV  CAN %u us 迟到 %u/%u 发送失败 %u  tx %d\n",
                            now - static_cast<double>(s->drive_t0_ns.load()) * 1e-9, pad.fwd, pad.turn, pad.spin,
                            load_f(s->height_bits), load_f(s->roll_bits), load_f(s->lean_bits), load_f(s->hip_bits),
                            s->balance_on.load() ? "平衡开" : "平衡关", load_f(s->cmd_l_bits), load_f(s->cmd_r_bits),
                            fl.velocity * wheel_sign(s->left_id), fr.velocity * wheel_sign(s->right_id), volts, s->can_us.load(),
                            s->late_can.load(), s->late_wheel.load(), s->tx_fail.load(), read_counter(args.iface, "tx_packets"));
                next_print = now + 0.5;
            }
        }
        pace(policy_next, kPolicyPeriodNs, nullptr);
    }

    close_log();
    if (s->unload_hold.load() && g_sig < 2) {
        double t0 = mono_s();
        while (g_sig < 2 && mono_s() - t0 < 8.0 && !legs_at_unload(s)) {
            timespec ts{0, 50000000};
            nanosleep(&ts, nullptr);
        }
    }
    if (s->phase.load() != PhStop) s->phase.store(PhStop);
    timespec stop_ts{0, 100000000};
    nanosleep(&stop_ts, nullptr);
    s->phase.store(PhExit);
    if (up_wheel) pthread_join(th_wheel, nullptr);
    if (up_body) pthread_join(th_body, nullptr);
    if (up_imu) pthread_join(th_imu, nullptr);
    if (up_pad) pthread_join(th_pad, nullptr);
    if (up_can) pthread_join(th_can, nullptr);
    if (up_wd) pthread_join(th_wd, nullptr);
    if (up_show) pthread_join(th_show, nullptr);
    can_log_close();
    std::printf("已退出。\n");
    return 0;
}

}  // namespace wheel
