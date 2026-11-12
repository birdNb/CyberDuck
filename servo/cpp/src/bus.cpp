#include "servo/bus.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace servo {
namespace {
constexpr uint8_t kHeader = 0xff;
constexpr uint8_t kRead = 0x02;
constexpr uint8_t kWrite = 0x03;
constexpr uint8_t kTorqueEnable = 0x28;
constexpr uint8_t kAcceleration = 0x29;
constexpr uint8_t kGoalPosition = 0x2a;
constexpr uint8_t kGoalSpeed = 0x2e;
constexpr uint8_t kTorqueLimit = 0x30;
constexpr uint8_t kPresentPosition = 0x38;
constexpr size_t kFeedbackSize = 19;

speed_t baud_constant(int baudrate) {
    switch (baudrate) {
        case 38400: return B38400;
        case 57600: return B57600;
#ifdef B76800
        case 76800: return B76800;
#endif
        case 115200: return B115200;
#ifdef B128000
        case 128000: return B128000;
#endif
#ifdef B250000
        case 250000: return B250000;
#endif
#ifdef B500000
        case 500000: return B500000;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
        default: throw std::runtime_error("不支持的舵机波特率");
    }
}

uint8_t packet_checksum(const std::vector<uint8_t>& body) {
    unsigned int sum = 0;
    for (auto byte : body) sum += byte;
    return static_cast<uint8_t>(~sum);
}

void append_u16(std::vector<uint8_t>& data, int value) {
    data.push_back(static_cast<uint8_t>(value & 0xff));
    data.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
}

int read_u16(const std::vector<uint8_t>& data, size_t offset) {
    return data.at(offset) | (static_cast<int>(data.at(offset + 1)) << 8);
}

int read_exact(int fd, uint8_t* data, size_t size, int timeout_ms) {
    size_t received = 0;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (received < size) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                             deadline - std::chrono::steady_clock::now())
                             .count();
        if (remaining <= 0) return -1;
        pollfd pfd{fd, POLLIN, 0};
        int ready = ::poll(&pfd, 1, static_cast<int>(remaining));
        if (ready <= 0) return -1;
        ssize_t n = ::read(fd, data + received, size - received);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return -1;
        received += static_cast<size_t>(n);
    }
    return 0;
}
}  // namespace

Bus::Bus(std::string port, int baudrate, int timeout_ms) : timeout_ms_(timeout_ms) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("打开舵机串口失败: " + port + ": " + std::strerror(errno));

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("读取舵机串口配置失败");
    }
    cfmakeraw(&tty);
    speed_t speed = baud_constant(baudrate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("设置舵机串口配置失败");
    }
    tcflush(fd_, TCIOFLUSH);
}

Bus::~Bus() {
    if (fd_ >= 0) ::close(fd_);
}

void Bus::write_packet(int id, uint8_t instruction, const std::vector<uint8_t>& params) {
    if (id < 0 || id > 253) throw std::runtime_error("舵机 ID 超出范围");
    std::vector<uint8_t> body{static_cast<uint8_t>(id), static_cast<uint8_t>(params.size() + 2),
                              instruction};
    body.insert(body.end(), params.begin(), params.end());
    std::vector<uint8_t> packet{kHeader, kHeader};
    packet.insert(packet.end(), body.begin(), body.end());
    packet.push_back(packet_checksum(body));

    tcflush(fd_, TCIFLUSH);
    size_t sent = 0;
    while (sent < packet.size()) {
        ssize_t n = ::write(fd_, packet.data() + sent, packet.size() - sent);
        if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
        if (n <= 0) throw std::runtime_error("舵机串口写入失败");
        sent += static_cast<size_t>(n);
    }
    if (tcdrain(fd_) != 0) throw std::runtime_error("等待舵机串口发送完成失败");
}

std::vector<uint8_t> Bus::transact(int id, uint8_t instruction, const std::vector<uint8_t>& params,
                                  size_t response_size) {
    write_packet(id, instruction, params);
    std::vector<uint8_t> packet(response_size + 6);
    if (read_exact(fd_, packet.data(), 4, timeout_ms_) != 0)
        throw std::runtime_error("等待舵机响应超时");
    if (packet[0] != kHeader || packet[1] != kHeader || packet[2] != id)
        throw std::runtime_error("舵机响应头或 ID 不匹配");
    size_t length = packet[3];
    if (length != response_size + 2 || length < 2)
        throw std::runtime_error("舵机响应长度不匹配");
    if (read_exact(fd_, packet.data() + 4, length, timeout_ms_) != 0)
        throw std::runtime_error("舵机响应数据超时");
    unsigned int sum = 0;
    for (size_t i = 2; i + 1 < packet.size(); ++i) sum += packet[i];
    if (static_cast<uint8_t>(~sum) != packet.back())
        throw std::runtime_error("舵机响应校验和错误");
    if (packet[4] != 0) throw std::runtime_error("舵机返回错误码");
    return std::vector<uint8_t>(packet.begin() + 5, packet.end() - 1);
}

State Bus::read_state(int id) {
    auto data = transact(id, kRead, {kPresentPosition, static_cast<uint8_t>(kFeedbackSize)},
                         kFeedbackSize);
    State state;
    state.position_raw = read_u16(data, 0) & 0x0fff;
    state.speed_raw = read_u16(data, 2);
    return state;
}

void Bus::set_torque(int id, bool enabled) {
    write_packet(id, kWrite, {kTorqueEnable, static_cast<uint8_t>(enabled ? 1 : 0)});
}

void Bus::set_motion(int id, int speed, int acceleration, int torque_limit) {
    write_packet(id, kWrite, {kTorqueLimit, static_cast<uint8_t>(torque_limit & 0xff),
                              static_cast<uint8_t>((torque_limit >> 8) & 0xff)});
    write_packet(id, kWrite, {kAcceleration, static_cast<uint8_t>(acceleration)});
    uint16_t signed_speed = static_cast<uint16_t>(speed);
    write_packet(id, kWrite, {kGoalSpeed, static_cast<uint8_t>(signed_speed & 0xff),
                              static_cast<uint8_t>((signed_speed >> 8) & 0xff)});
}

void Bus::set_goal_position(int id, int position_raw) {
    std::vector<uint8_t> params{kGoalPosition};
    append_u16(params, position_raw);
    write_packet(id, kWrite, params);
}

}  // namespace servo
