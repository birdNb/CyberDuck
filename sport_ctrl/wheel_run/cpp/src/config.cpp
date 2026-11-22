#include "wheel/config.hpp"

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace wheel {
namespace {

std::string strip_comment(const std::string& in) {
    bool sq = false;
    bool dq = false;
    for (size_t i = 0; i < in.size(); ++i) {
        char c = in[i];
        if (c == '\'' && !dq) sq = !sq;
        else if (c == '"' && !sq) dq = !dq;
        else if (c == '#' && !sq && !dq) return in.substr(0, i);
    }
    return in;
}

std::string trim(const std::string& in) {
    size_t a = 0;
    while (a < in.size() && (in[a] == ' ' || in[a] == '\t' || in[a] == '\r')) ++a;
    size_t b = in.size();
    while (b > a && (in[b - 1] == ' ' || in[b - 1] == '\t' || in[b - 1] == '\r')) --b;
    return in.substr(a, b - a);
}

std::string unquote(std::string key) {
    if (key.size() >= 2) {
        char a = key.front();
        char b = key.back();
        if ((a == '\'' && b == '\'') || (a == '"' && b == '"')) return key.substr(1, key.size() - 2);
    }
    return key;
}

int leading_indent(const std::string& line) {
    int n = 0;
    for (char c : line) {
        if (c == ' ') n += 1;
        else if (c == '\t') n += 2;
        else break;
    }
    return n;
}

}  // namespace

bool load_yaml(const std::string& path, YamlCfg& out, std::string& err) {
    std::ifstream in(path);
    if (!in) {
        err = "找不到配置 " + path;
        return false;
    }
    bool in_poses = false;
    bool in_joints = false;
    int joints_indent = 0;
    std::string pose;
    std::string line;
    while (std::getline(in, line)) {
        std::string body = trim(strip_comment(line));
        if (body.empty()) continue;
        int indent = leading_indent(line);
        auto colon = body.find(':');
        if (colon == std::string::npos) continue;
        std::string key = unquote(trim(body.substr(0, colon)));
        std::string val = trim(body.substr(colon + 1));
        if (!val.empty() && val.front() == '"' && val.back() == '"' && val.size() >= 2) val = val.substr(1, val.size() - 2);

        if (indent == 0) {
            in_poses = key == "poses";
            in_joints = false;
            pose.clear();
            if (in_poses) continue;
            if (key == "interface" && !val.empty()) out.iface = val;
            else if (key == "start_id") out.start_id = std::atoi(val.c_str());
            else if (key == "end_id") out.end_id = std::atoi(val.c_str());
            else if (key == "hz") out.hz = std::atof(val.c_str());
            else if (key == "transition_s") out.transition_s = std::atof(val.c_str());
            else if (key == "vel_limit") out.vel_limit = std::atof(val.c_str());
            else if (key == "torque_nm") out.torque = std::atof(val.c_str());
            continue;
        }
        if (!in_poses) continue;
        if (in_joints && indent < joints_indent) in_joints = false;
        if (!in_joints && indent <= 2) {
            pose = key;
            out.poses[pose];
            continue;
        }
        if (!pose.empty() && key == "joints") {
            in_joints = true;
            joints_indent = indent + 2;
            continue;
        }
        if (!in_joints || pose.empty()) continue;
        int id = std::atoi(key.c_str());
        if (id < 1 || id > 15 || val.empty()) continue;
        out.poses[pose].deg[id] = std::strtof(val.c_str(), nullptr);
        out.poses[pose].mask = static_cast<uint16_t>(out.poses[pose].mask | (1u << id));
    }
    if (out.start_id < 1) out.start_id = 1;
    if (out.end_id > 15) out.end_id = 15;
    if (out.end_id < out.start_id) {
        err = "电机扫描范围无效";
        return false;
    }
    return true;
}

}  // namespace wheel
