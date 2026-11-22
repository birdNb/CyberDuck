#pragma once

#include "locomotion/layout.hpp"

#include <memory>
#include <string>

namespace loco {

class Engine {
public:
    virtual ~Engine() = default;
    virtual const char* name() const = 0;
    virtual bool infer(const float obs[kObsLen], float action[kActionLen]) = 0;
};

std::unique_ptr<Engine> make_engine(const Config& cfg, std::string& note);

}  // namespace loco
