#pragma once

#include <cstdint>
#include <string>

namespace motor {

constexpr double kTwoPi = 6.283185307179586;
constexpr int16_t kMagicPos = -32768;

double torque_adj(const std::string& type);
int16_t clip_i16(int value);
int16_t pos_to_int(double rad);
int16_t vel_to_int(double rad_s);
int16_t tqe_to_int(double nm, const std::string& type);
double pos_from_int(int16_t raw);
double vel_from_int(int16_t raw);
double tqe_from_int(int16_t raw, const std::string& type);

uint32_t command_can_id(int motor_id, bool reply);
int motor_id_from_can(uint32_t arb_id);

}  // namespace motor
