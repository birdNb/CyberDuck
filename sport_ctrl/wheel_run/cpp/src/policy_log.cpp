#include "wheel/policy_log.hpp"

#include <cstdint>
#include <ctime>

#include <sys/stat.h>

#include <cstdio>
#include <cstring>

namespace wheel {
namespace {

struct LogFile {
    const char* name = nullptr;
    FILE* file = nullptr;
    uint64_t t0_ns = 0;
};

uint64_t mono_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

LogFile* slot_for(const char* name) {
    static LogFile slots[4];
    for (auto& slot : slots) {
        if (slot.name != nullptr && std::strcmp(slot.name, name) == 0) return &slot;
    }
    for (auto& slot : slots) {
        if (slot.name == nullptr) {
            slot.name = name;
            return &slot;
        }
    }
    return nullptr;
}

bool open_log(LogFile& slot, const char* const* input_names, int n_in, const char* const* output_names, int n_out) {
    ::mkdir("/home/aidlux/cyberDuck/sport_ctrl/wheel_run/logs", 0755);
    std::time_t wall = std::time(nullptr);
    std::tm tm{};
    localtime_r(&wall, &tm);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);
    char path[256];
    std::snprintf(path, sizeof(path), "/home/aidlux/cyberDuck/sport_ctrl/wheel_run/logs/policy_%s_%s.csv", slot.name, stamp);
    slot.file = std::fopen(path, "w");
    if (!slot.file) {
        std::fprintf(stderr, "策略日志打不开：%s\n", path);
        return false;
    }
    std::fprintf(slot.file, "t_s");
    for (int i = 0; i < n_in; ++i) std::fprintf(slot.file, ",%s", input_names[i]);
    for (int i = 0; i < n_out; ++i) std::fprintf(slot.file, ",%s", output_names[i]);
    std::fprintf(slot.file, "\n");
    std::fflush(slot.file);
    slot.t0_ns = mono_ns();
    std::printf("策略日志 %s\n", path);
    return true;
}

}  // namespace

void log_policy_close(const char* name) {
    if (name == nullptr) return;
    LogFile* slot = slot_for(name);
    if (slot == nullptr || slot->file == nullptr) return;
    std::fflush(slot->file);
    std::fclose(slot->file);
    slot->file = nullptr;
    slot->t0_ns = 0;
}

void log_policy_run(const char* name, const char* const* input_names, int n_in, const float* input,
                     const char* const* output_names, int n_out, const float* output) {
    if (name == nullptr || input == nullptr || output == nullptr || n_in < 1 || n_out < 1) return;
    LogFile* slot = slot_for(name);
    if (slot == nullptr) return;
    if (slot->file == nullptr && !open_log(*slot, input_names, n_in, output_names, n_out)) return;
    uint64_t now = mono_ns();
    double t = static_cast<double>(now - slot->t0_ns) * 1e-9;
    std::fprintf(slot->file, "%.4f", t);
    for (int i = 0; i < n_in; ++i) std::fprintf(slot->file, ",%.6g", input[i]);
    for (int i = 0; i < n_out; ++i) std::fprintf(slot->file, ",%.6g", output[i]);
    std::fprintf(slot->file, "\n");
    std::fflush(slot->file);
}

}  // namespace wheel
