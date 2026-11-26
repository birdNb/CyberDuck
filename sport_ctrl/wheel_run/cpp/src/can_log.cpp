#include "wheel/can_log.hpp"

#include "wheel/math.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sys/stat.h>
#include <time.h>

namespace wheel {
namespace {

std::mutex g_mu;
FILE* g_fp = nullptr;
std::atomic<bool> g_on{false};
uint64_t g_t0 = 0;
uint64_t g_rows = 0;
constexpr uint64_t kMaxRows = 200000;

uint64_t mono_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

void hex_bytes(char* out, size_t cap, const uint8_t* d, int n) {
    size_t o = 0;
    for (int i = 0; i < n && o + 3 < cap; ++i) {
        if (i) out[o++] = ' ';
        std::snprintf(out + o, cap - o, "%02X", d[i]);
        o += 2;
    }
    out[o] = 0;
}

const char* kind_of(const uint8_t* d, int dlc, bool rx) {
    if (dlc < 1) return "EMPTY";
    if (rx) {
        if (d[0] == 0x0E) return "FB_STAT";
        if (d[0] == 0x04) return "FB_PING";
        if (d[0] == 0x0C) return "FB_TEMP";
        if (d[0] == 0x0D) return "FB_VOLT";
        return "FB_OTHER";
    }
    if (d[0] == 0x1F) return "TX_POS";
    if (d[0] == 0x1C) return "TX_VEL";
    if (d[0] == 0x01) return "TX_STOP";
    if (d[0] == 0x00 && dlc >= 2) {
        if (d[1] == 0x0E) return "TX_QSTAT";
        if (d[1] == 0x04) return "TX_QPING";
        if (d[1] == 0x0C) return "TX_QTEMP";
        if (d[1] == 0x0D) return "TX_QVOLT";
        return "TX_QUERY";
    }
    return "TX_OTHER";
}

void write_row(const char* dir, int mid, uint32_t arb, const uint8_t* d, int dlc) {
    if (!g_on.load(std::memory_order_relaxed) || g_fp == nullptr) return;
    if (g_rows >= kMaxRows) {
        g_on.store(false, std::memory_order_relaxed);
        std::fprintf(g_fp, "# truncated at %llu rows\n", static_cast<unsigned long long>(g_rows));
        std::fflush(g_fp);
        return;
    }
    uint64_t now = mono_ns();
    if (g_t0 == 0) g_t0 = now;
    double t = static_cast<double>(now - g_t0) * 1e-9;
    char hex[3 * 8 + 1];
    hex_bytes(hex, sizeof(hex), d, dlc);
    const char* kind = kind_of(d, dlc, dir[0] == 'R');
    int query = (dlc >= 2) ? static_cast<int>(d[1]) : -1;
    char pos_b[32] = "", vel_b[32] = "", tqe_b[32] = "", fault_b[16] = "";
    if (d[0] == 0x1F && dlc >= 8) {
        std::snprintf(pos_b, sizeof(pos_b), "%.6f", pos_from_int(static_cast<int16_t>(d[2] | (d[3] << 8))));
        std::snprintf(vel_b, sizeof(vel_b), "%.6f", vel_from_int(static_cast<int16_t>(d[4] | (d[5] << 8))));
        std::snprintf(tqe_b, sizeof(tqe_b), "%.3f", tqe_from_int(static_cast<int16_t>(d[6] | (d[7] << 8))));
    } else if (d[0] == 0x1C && dlc >= 4) {
        std::snprintf(vel_b, sizeof(vel_b), "%.6f", vel_from_int(static_cast<int16_t>(d[2] | (d[3] << 8))));
    } else if (d[0] == 0x0E && dlc >= 8) {
        std::snprintf(fault_b, sizeof(fault_b), "%u", d[1]);
        std::snprintf(pos_b, sizeof(pos_b), "%.6f", pos_from_int(static_cast<int16_t>(d[2] | (d[3] << 8))));
        std::snprintf(vel_b, sizeof(vel_b), "%.6f", vel_from_int(static_cast<int16_t>(d[4] | (d[5] << 8))));
        std::snprintf(tqe_b, sizeof(tqe_b), "%.3f", tqe_from_int(static_cast<int16_t>(d[6] | (d[7] << 8))));
    }
    std::fprintf(g_fp, "%.6f,%s,%d,0x%05X,%d,%s,%s,%d,%s,%s,%s,%s\n", t, dir, mid, arb, dlc, hex, kind, query, pos_b,
                 vel_b, tqe_b, fault_b);
    ++g_rows;
    if ((g_rows & 255ull) == 0) std::fflush(g_fp);
}

}  // namespace

void can_log_open() {
    std::lock_guard<std::mutex> lk(g_mu);
    if (g_fp) return;
    ::mkdir("/home/aidlux/cyberDuck/sport_ctrl/wheel_run/logs", 0755);
    time_t now = time(nullptr);
    tm tm_now{};
    localtime_r(&now, &tm_now);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_now);
    char path[256];
    std::snprintf(path, sizeof(path), "/home/aidlux/cyberDuck/sport_ctrl/wheel_run/logs/can_mix_%s.csv", stamp);
    g_fp = std::fopen(path, "w");
    if (!g_fp) return;
    setvbuf(g_fp, nullptr, _IOFBF, 1 << 16);
    std::fprintf(g_fp, "t_s,dir,id,arb,dlc,hex,kind,query,pos_rad,vel_rad_s,tqe_nm,fault\n");
    std::fprintf(g_fp, "# TX arb=0x18000|id  RX arb=0x10000|(id<<8)  POS=0x1F VEL=0x1C QSTAT=00 0E\n");
    std::fflush(g_fp);
    g_t0 = 0;
    g_rows = 0;
    g_on.store(true, std::memory_order_relaxed);
    std::fprintf(stderr, "CAN 报文记录：%s\n", path);
}

void can_log_close() {
    std::lock_guard<std::mutex> lk(g_mu);
    g_on.store(false, std::memory_order_relaxed);
    if (!g_fp) return;
    std::fprintf(g_fp, "# closed rows=%llu\n", static_cast<unsigned long long>(g_rows));
    std::fflush(g_fp);
    std::fclose(g_fp);
    g_fp = nullptr;
}

void can_log_enable(bool on) { g_on.store(on, std::memory_order_relaxed); }

bool can_log_enabled() { return g_on.load(std::memory_order_relaxed) && g_fp != nullptr; }

void can_log_tx(int motor_id, const uint8_t* data, int dlc) {
    if (!g_on.load(std::memory_order_relaxed) || data == nullptr || dlc < 1) return;
    std::lock_guard<std::mutex> lk(g_mu);
    uint32_t arb = 0x18000u | (static_cast<uint32_t>(motor_id) & 0x7Fu);
    write_row("TX", motor_id, arb, data, dlc);
}

void can_log_rx(int motor_id, uint32_t arb, const uint8_t* data, int dlc) {
    if (!g_on.load(std::memory_order_relaxed) || data == nullptr || dlc < 1) return;
    std::lock_guard<std::mutex> lk(g_mu);
    write_row("RX", motor_id, arb, data, dlc);
}

}  // namespace wheel
