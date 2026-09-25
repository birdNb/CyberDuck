#include "motor/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace motor {
namespace {
const std::unordered_map<std::string, double> kAdj = {
    {"3536_32", 0.4581}, {"4538_19", 0.4450}, {"5046_20", 0.5280},
    {"5047_9", 0.5330},  {"5047_36", 0.4938}, {"5047_36_2", 0.8030},
    {"4438_30", 0.5256}, {"4438_32", 0.5584}, {"6056_36", 0.6770},
    {"7256_35", 0.6770}, {"60SG_35", 0.7942}, {"60BM_35", 0.7942},
    {"5043_20", 0.9660}, {"General", 0.5000}, {"NONE", 1.0000},
    {"HT4538", 0.4450},  {"HT5046", 0.5280},  {"HT5047_36", 0.8030},
    {"HT5047_9", 0.5330},
};
}

double torque_adj(const std::string& type) {
    auto it = kAdj.find(type);
    if (it == kAdj.end()) {
        throw std::runtime_error("未知电机型号: " + type);
    }
    return it->second;
}

int16_t clip_i16(int value) {
    return static_cast<int16_t>(std::clamp(value, -32700, 32700));
}

int16_t pos_to_int(double rad) {
    return clip_i16(static_cast<int>(rad / kTwoPi * 10000.0));
}

int16_t vel_to_int(double rad_s) {
    return clip_i16(static_cast<int>(rad_s / kTwoPi * 4000.0));
}

int16_t tqe_to_int(double nm, const std::string& type) {
    return clip_i16(static_cast<int>(nm / (torque_adj(type) * 0.01)));
}

double pos_from_int(int16_t raw) { return raw * kTwoPi / 10000.0; }
double vel_from_int(int16_t raw) { return raw * kTwoPi / 4000.0; }

double tqe_from_int(int16_t raw, const std::string& type) {
    auto it = kAdj.find(type);
    double adj = it == kAdj.end() ? 0.5 : it->second;
    return raw * (adj * 0.01);
}

uint32_t command_can_id(int motor_id, bool reply) {
    uint32_t id = static_cast<uint32_t>(motor_id) & 0xFFu;
    if (reply) id |= 0x8000u;
    return id;
}

int motor_id_from_can(uint32_t arb_id) {
    int src = static_cast<int>((arb_id >> 8) & 0x7F);
    if (src >= 1 && src <= 127) return src;
    return static_cast<int>(arb_id & 0xFF);
}

}  // namespace motor
