#include "wheel/bus.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <fcntl.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace wheel {
namespace {

int16_t read_i16(const uint8_t* p) {
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}

void write_i16(uint8_t* p, int16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

int reply_id(uint32_t arb) {
    if ((arb & ~0x7F00u) != 0x10000u) return 0;
    return static_cast<int>((arb >> 8) & 0x7F);
}

}  // namespace

DecodeResult decode_feedback(Feedback& fb, const uint8_t* data, int dlc, uint64_t now_ns) {
    DecodeResult r;
    if (data == nullptr || dlc < 1) return r;
    fb.t_rx_ns = now_ns;
    const uint8_t cmd = data[0];
    if (cmd == 0x0E && dlc >= 8) {
        fb.fault = data[1];
        fb.raw_pos = read_i16(data + 2);
        fb.position = static_cast<float>(pos_from_int(fb.raw_pos));
        fb.velocity = static_cast<float>(vel_from_int(read_i16(data + 4)));
        fb.torque = static_cast<float>(tqe_from_int(read_i16(data + 6)));
        fb.t_pos_ns = now_ns;
        r.any = true;
        r.position = true;
        return r;
    }
    if (cmd == 0x04 || cmd == 0x05 || cmd == 0x07 || cmd == 0x03) {
        r.any = true;
        return r;
    }
    return r;
}

Bus::Bus(const std::string& iface) {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        error_ = "打开 CAN 失败";
        return;
    }
    ifreq ifr{};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", iface.c_str());
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        error_ = iface + " 不存在（PCAN-USB 未识别到）";
        ::close(fd_);
        fd_ = -1;
        return;
    }
    if (ioctl(fd_, SIOCGIFFLAGS, &ifr) == 0 && !(ifr.ifr_flags & IFF_UP)) {
        error_ = iface + " 处于 DOWN，先执行 motor/setup_can0.sh";
        ::close(fd_);
        fd_ = -1;
        return;
    }
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        error_ = "读取 " + iface + " 索引失败";
        ::close(fd_);
        fd_ = -1;
        return;
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        error_ = "绑定 " + iface + " 失败";
        ::close(fd_);
        fd_ = -1;
        return;
    }
    int loopback = 0;
    setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags >= 0) fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
}

Bus::~Bus() {
    if (fd_ >= 0) ::close(fd_);
}

bool Bus::send(int id, const uint8_t* data, int len) {
    if (fd_ < 0 || data == nullptr || len < 1 || len > 8) return false;
    can_frame frame{};
    frame.can_id = (0x18000u | (static_cast<uint32_t>(id) & 0x7Fu)) | CAN_EFF_FLAG;
    frame.can_dlc = static_cast<uint8_t>(len);
    std::memcpy(frame.data, data, len);
    for (int i = 0; i < 3; ++i) {
        ssize_t n = ::write(fd_, &frame, sizeof(frame));
        if (n == static_cast<ssize_t>(sizeof(frame))) return true;
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOBUFS) return false;
        ::usleep(100);
    }
    return false;
}

bool Bus::send_stop(int id) {
    // 只在卸力姿态到位后使用。查询码 0x00，避免停止帧把状态回包灌满总线。
    const uint8_t d[2] = {0x01, 0x00};
    return send(id, d, 2);
}

bool Bus::send_zero(int id) {
    // 重设零位的首字节是 0x03，会离开位置模式。新电机一切模式就报错，所以不发。
    (void)id;
    return true;
}

bool Bus::send_position(int id, int16_t raw_pos, float vel_rad_s, float torque_nm) {
    uint8_t d[8] = {0x1F, 0x00};
    write_i16(d + 2, raw_pos);
    write_i16(d + 4, vel_to_int(vel_rad_s));
    write_i16(d + 6, tqe_to_int(torque_nm));
    return send(id, d, 8);
}

bool Bus::send_velocity(int id, float vel_rad_s) {
    // 速度模式只有 4 字节。多出来的字节整帧会被丢掉。查询码 0x00，不让电机每帧回包。
    uint8_t d[4] = {0x1C, 0x00};
    write_i16(d + 2, vel_to_int(vel_rad_s));
    return send(id, d, 4);
}

bool Bus::send_status(int id) {
    const uint8_t d[2] = {0x00, 0x0E};
    return send(id, d, 2);
}

bool Bus::send_temp(int id) {
    const uint8_t d[2] = {0x00, 0x0C};
    return send(id, d, 2);
}

bool Bus::send_voltage(int id) {
    const uint8_t d[2] = {0x00, 0x0D};
    return send(id, d, 2);
}

bool Bus::send_ping(int id) {
    const uint8_t d[2] = {0x00, 0x04};
    return send(id, d, 2);
}

bool Bus::send_periodic_off(int id) {
    // 关闭周期返回的首字节也是 0x03。不发，避免开机时先切一次模式。
    (void)id;
    return true;
}

bool Bus::poll(Frame& out, int timeout_ms) {
    if (fd_ < 0) return false;
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    int rc = ::poll(&pfd, 1, timeout_ms);
    if (rc <= 0) return false;
    can_frame frame{};
    ssize_t n = ::read(fd_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) return false;
    if (frame.can_id & CAN_ERR_FLAG) {
        out.id = -1;
        out.dlc = 0;
        return true;
    }
    out.id = reply_id(frame.can_id & CAN_EFF_MASK);
    out.dlc = frame.can_dlc > 8 ? 8 : frame.can_dlc;
    std::memcpy(out.data, frame.data, out.dlc);
    return true;
}

void Bus::drain() {
    Frame frame;
    for (int i = 0; i < 64; ++i) {
        if (!poll(frame, 0)) return;
    }
}

}  // namespace wheel
