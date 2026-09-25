#include "motor/bus.hpp"

#include "motor/protocol.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <sys/select.h>

namespace motor {
namespace {
void put_i16(uint8_t* p, int16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
int16_t get_i16(const uint8_t* p) {
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}
}

Bus::Bus(std::string iface) : iface_(std::move(iface)) {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error("打开 CAN 失败");
    ifreq ifr{};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", iface_.c_str());
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(fd_);
        throw std::runtime_error("找不到 CAN 接口 " + iface_);
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd_);
        throw std::runtime_error("绑定 " + iface_ + " 失败");
    }
    int loopback = 0;
    setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
}

Bus::~Bus() {
    if (fd_ >= 0) ::close(fd_);
}

void Bus::add(int id, std::string type, std::string name) {
    types_[id] = std::move(type);
    State s;
    s.id = id;
    states_[id] = s;
    (void)name;
}

void Bus::send(uint32_t can_id, const uint8_t* data, int len, bool extended) {
    can_frame frame{};
    frame.can_id = can_id & CAN_EFF_MASK;
    if (extended) frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = static_cast<uint8_t>(len);
    std::memcpy(frame.data, data, len);
    for (int i = 0; i < 8; ++i) {
        if (::write(fd_, &frame, sizeof(frame)) == sizeof(frame)) return;
        if (errno != ENOBUFS) throw std::runtime_error("CAN 发送失败");
        usleep(2000);
    }
    throw std::runtime_error("CAN 发送缓冲满");
}

void Bus::set_mode(int id, int mode) {
    uint8_t d[8] = {0x01, 0x00, static_cast<uint8_t>(mode), 0x50, 0x50, 0x50, 0x50, 0x50};
    send(command_can_id(id, false), d, 8, true);
}

void Bus::enable_position(int id) { set_mode(id, 0x0A); }
void Bus::disable(int id) { set_mode(id, 0x00); }

void Bus::set_timeout_ms(int id, int timeout_ms) {
    uint8_t d[6] = {0x05, 0xB4, 0x02, 0x00, 0, 0};
    put_i16(d + 4, static_cast<int16_t>(timeout_ms));
    send(command_can_id(id, false), d, 6, true);
}

void Bus::read_status(int id) {
    uint8_t d[2] = {0x17, 0x01};
    send(command_can_id(id, true), d, 2, true);
}

void Bus::command_position(int id, double position_rad, double vel_limit_rad_s, double torque_nm) {
    auto it = types_.find(id);
    if (it == types_.end()) return;
    uint8_t d[8] = {0x07, 0x07};
    put_i16(d + 2, pos_to_int(position_rad));
    put_i16(d + 4, vel_to_int(vel_limit_rad_s));
    put_i16(d + 6, tqe_to_int(torque_nm, it->second));
    send(command_can_id(id, false), d, 8, true);
}

void Bus::poll(double timeout_s) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout_s);
    tv.tv_usec = static_cast<suseconds_t>((timeout_s - tv.tv_sec) * 1e6);
    if (select(fd_ + 1, &rfds, nullptr, nullptr, &tv) <= 0) return;
    while (true) {
        fd_set one;
        FD_ZERO(&one);
        FD_SET(fd_, &one);
        timeval zero{};
        can_frame frame{};
        if (select(fd_ + 1, &one, nullptr, nullptr, &zero) <= 0) break;
        ssize_t n = ::read(fd_, &frame, sizeof(frame));
        if (n != sizeof(frame)) break;
        if (frame.can_id & CAN_ERR_FLAG) continue;
        int id = motor_id_from_can(frame.can_id & CAN_EFF_MASK);
        auto it = states_.find(id);
        if (it == states_.end() || frame.can_dlc < 8) continue;
        const uint8_t cmd = frame.data[0];
        int16_t raw_pos, raw_vel, raw_tqe;
        if (cmd == 0x27 || cmd == 0x17) {
            raw_pos = get_i16(frame.data + 2);
            raw_vel = get_i16(frame.data + 4);
            raw_tqe = get_i16(frame.data + 6);
        } else {
            raw_pos = get_i16(frame.data);
            raw_vel = get_i16(frame.data + 2);
            raw_tqe = get_i16(frame.data + 4);
        }
        auto type = types_[id];
        it->second.position = pos_from_int(raw_pos);
        it->second.velocity = vel_from_int(raw_vel);
        it->second.torque = tqe_from_int(raw_tqe, type);
        it->second.fresh = true;
    }
}

const State* Bus::state(int id) const {
    auto it = states_.find(id);
    return it == states_.end() ? nullptr : &it->second;
}

}  // namespace motor
