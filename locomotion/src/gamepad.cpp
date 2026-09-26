#include "locomotion/gamepad.hpp"

#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include <cmath>

namespace loco {

Gamepad::Gamepad(std::string path) : path_(std::move(path)) {}

Gamepad::~Gamepad() {
    if (fd_ >= 0) ::close(fd_);
}

bool Gamepad::open() {
    fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
    return fd_ >= 0;
}

Command Gamepad::poll(const Config& cfg) {
    if (fd_ >= 0) {
        js_event ev{};
        while (::read(fd_, &ev, sizeof(ev)) == sizeof(ev)) {
            if ((ev.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS && ev.number < 8) {
                axis_[ev.number] = ev.value / 32767.0f;
            }
        }
    }
    auto dz = [&](float v) { return std::fabs(v) < cfg.deadzone ? 0.f : v; };
    float lx = dz(axis_[0]);
    float ly = dz(-axis_[1]);
    float rx = dz(axis_[3]);
    Command c;
    float scale = static_cast<float>(cfg.cmd_scale);
    c.vx = ly * static_cast<float>(ly >= 0 ? cfg.max_vx : cfg.max_vx_back) * scale;
    c.vy = -lx * static_cast<float>(cfg.max_vy) * scale;
    c.vyaw = -rx * static_cast<float>(cfg.max_vyaw) * scale;
    return c;
}

}  // namespace loco
