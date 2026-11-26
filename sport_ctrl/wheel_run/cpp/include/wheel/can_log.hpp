#pragma once

#include <cstdint>

namespace wheel {

// 混合轮速(0x1C)+位置(0x1F)+状态(0x0E) 的 CAN 收发落盘。
// 文件：sport_ctrl/wheel_run/logs/can_mix_*.csv
void can_log_open();
void can_log_close();
void can_log_enable(bool on);
bool can_log_enabled();
void can_log_tx(int motor_id, const uint8_t* data, int dlc);
void can_log_rx(int motor_id, uint32_t arb, const uint8_t* data, int dlc);

}  // namespace wheel
