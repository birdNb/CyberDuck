#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace wheel {

struct Pose {
    float deg[16]{};
    uint16_t mask = 0;
    bool has(int id) const { return id > 0 && id < 16 && (mask & (1u << id)); }
};

struct YamlCfg {
    std::string iface = "can0";
    int start_id = 1;
    int end_id = 12;
    double hz = 50;
    double transition_s = 3;
    double vel_limit = 1;
    double torque = 1;
    std::unordered_map<std::string, Pose> poses;
};

bool load_yaml(const std::string& path, YamlCfg& out, std::string& err);

}  // namespace wheel
