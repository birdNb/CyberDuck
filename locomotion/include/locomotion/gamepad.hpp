#pragma once

#include "locomotion/layout.hpp"

#include <string>

namespace loco {

class Gamepad {
public:
    explicit Gamepad(std::string path);
    ~Gamepad();
    bool open();
    Command poll(const Config& cfg);

private:
    std::string path_;
    int fd_ = -1;
    float axis_[8]{};
};

}  // namespace loco
