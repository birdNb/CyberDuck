#!/usr/bin/env python3
"""HighTorque 关节电机 — 经典 CAN / SocketCAN（PCAN-USB）。

协议要点（29-bit 扩展帧，1 Mbps，单帧 ≤ 8 字节，小端 int16）：
  发送 CAN ID = 0x18000 | motor_id
  MIT  CAN ID = 0x58000 | motor_id
  返回 CAN ID = 0x10000 | (motor_id << 8)
  控制帧 = 模式码 + 查询码 + 参数；状态查询码固定 0x0E
  换算：pos 10000/圈，vel 4000/(圈/s)，tqe 100/Nm（无型号力矩系数）
"""

from __future__ import annotations

import select
import socket
import struct
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence

CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_FLAG = getattr(socket, "CAN_EFF_FLAG", 0x80000000)
CAN_ERR_FLAG = getattr(socket, "CAN_ERR_FLAG", 0x20000000)
CAN_EFF_MASK = getattr(socket, "CAN_EFF_MASK", 0x1FFFFFFF)
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_ERR_FILTER = 2
CAN_RAW_RECV_OWN_MSGS = 4
CAN_ERR_MASK = 0x1FFFFFFF
CAN_ERR_TX_TIMEOUT = 0x00000001
CAN_ERR_LOSTARB = 0x00000002
CAN_ERR_CRTL = 0x00000004
CAN_ERR_PROT = 0x00000008
CAN_ERR_TRX = 0x00000010
CAN_ERR_ACK = 0x00000020
CAN_ERR_BUSOFF = 0x00000040
CAN_ERR_BUSERROR = 0x00000080
CAN_ERR_RESTARTED = 0x00000100

TWO_PI = 6.28318530717
POS_SCALE = 10000.0
VEL_SCALE = 4000.0
TQE_SCALE = 100.0
ACC_SCALE = 1000.0

CAN_TITLE_INT16 = 0x18000
CAN_TITLE_MIT = 0x58000
CAN_TITLE_REPLY = 0x10000

QUERY_STATUS = 0x0E
QUERY_FW = 0x04
QUERY_HW = 0x05
QUERY_MODEL = 0x07
QUERY_SYS = 0x03
QUERY_TEMP = 0x0C
QUERY_VOLTAGE = 0x0D

MODE_STOP = 0x01
MODE_BRAKE = 0x18
MODE_VOLTAGE = 0x19
MODE_CURRENT = 0x1A
MODE_TORQUE = 0x1B
MODE_VELOCITY = 0x1C
MODE_POSITION = 0x1D
MODE_VEL_ACC = 0x1E
MODE_POS_VEL_TQE = 0x1F
MODE_POS_VEL_ACC = 0x20
MODE_MIT = 0x21

SYS_CMD = 0x03
SYS_REBOOT = 0x01
SYS_SAVE = 0x02
SYS_ZERO = 0x03
SYS_SET_ID = 0x04
SYS_PERIODIC = 0x05

# 兼容旧代码里的占位名；新协议速度模式不再需要 MAGIC 位置字。
MAGIC_POS = -32768

MOTOR_TQE_ADJ = {
    "3532_0": 1.0,
    "3536_32": 1.0,
    "4538_19": 1.0,
    "5046_20": 1.0,
    "5047_9": 1.0,
    "5047_36": 1.0,
    "5047_36_2": 1.0,
    "4438_30": 1.0,
    "4438_32": 1.0,
    "6056_36": 1.0,
    "7256_35": 1.0,
    "60SG_35": 1.0,
    "60BM_35": 1.0,
    "5043_20": 1.0,
    "General": 1.0,
    "NONE": 1.0,
    "HT4538": 1.0,
    "HT5046": 1.0,
    "HT5047_36": 1.0,
    "HT5047_9": 1.0,
}

MODE_NAMES = {
    MODE_STOP: "停止",
    MODE_BRAKE: "刹车",
    MODE_TORQUE: "力矩",
    MODE_VELOCITY: "速度",
    MODE_POSITION: "位置",
    MODE_VEL_ACC: "速加",
    MODE_POS_VEL_TQE: "位速矩",
    MODE_POS_VEL_ACC: "位速加",
    MODE_MIT: "MIT",
}


def _clip_i16(value: int) -> int:
    if value >= 32700:
        return 32700
    if value <= -32700:
        return -32700
    return int(value)


def pos_to_int(rad: float) -> int:
    """rad → 0.0001 圈。"""
    return _clip_i16(int(rad / TWO_PI * POS_SCALE))


def vel_to_int(rad_s: float) -> int:
    """rad/s → 0.00025 圈/s。"""
    return _clip_i16(int(rad_s / TWO_PI * VEL_SCALE))


def tqe_to_int(nm: float, motor_type: str = "3532_0") -> int:
    """Nm → raw，1 LSB = 0.01 Nm。motor_type 保留兼容，新协议不再乘型号系数。"""
    _ = motor_type
    return _clip_i16(int(nm * TQE_SCALE))


def pos_from_int(raw: int) -> float:
    return raw * TWO_PI / POS_SCALE


def vel_from_int(raw: int) -> float:
    return raw * TWO_PI / VEL_SCALE


def tqe_from_int(raw: int, motor_type: str = "3532_0") -> float:
    _ = motor_type
    return raw / TQE_SCALE


def motor_can_id(motor_id: int, reply: bool = True, mit: bool = False) -> int:
    """发送帧 CAN ID。reply 参数保留兼容（新协议靠查询码回传）。"""
    _ = reply
    title = CAN_TITLE_MIT if mit else CAN_TITLE_INT16
    return title | (motor_id & 0x7F)


def motor_id_from_can_id(can_id: int) -> int:
    """只认回复帧 0x10000|(id<<8)。

    发送帧是 0x18000|id。适配器有时把发送帧折回接收，
    低 7 位正好是电机号，不能把它当成这台电机的回包。
    """
    cid = can_id & CAN_EFF_MASK
    if (cid & ~0x7F00) != CAN_TITLE_REPLY:
        return 0
    return (cid >> 8) & 0x7F


def i16_le(value: int) -> bytes:
    return struct.pack("<h", _clip_i16(value))


def unpack_i16_le(data: bytes, offset: int = 0) -> int:
    return struct.unpack_from("<h", data, offset)[0]


@dataclass
class MotorState:
    motor_id: int = 0
    mode: int = 0
    fault: int = 0
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    voltage: float = 0.0
    raw_pos: int = 0
    raw_vel: int = 0
    raw_tqe: int = 0
    can_id: int = 0
    payload: bytes = b""
    t_rx: float = 0.0
    t_pos: float = 0.0
    fw_version: str = ""
    model: str = ""


@dataclass
class CanFrame:
    can_id: int
    data: bytes
    extended: bool = True
    error: bool = False

    @property
    def arb_id(self) -> int:
        return self.can_id & CAN_EFF_MASK


def describe_error_frame(frame: "CanFrame") -> str:
    cid = frame.can_id
    reasons = []
    if cid & CAN_ERR_ACK:
        reasons.append("ACK缺失")
    if cid & CAN_ERR_BUSOFF:
        reasons.append("BUS-OFF")
    if cid & CAN_ERR_CRTL:
        reasons.append("控制器")
    if cid & CAN_ERR_PROT:
        reasons.append("协议/位错误")
    if cid & CAN_ERR_TRX:
        reasons.append("收发器")
    if cid & CAN_ERR_LOSTARB:
        reasons.append("仲裁丢失")
    if cid & CAN_ERR_TX_TIMEOUT:
        reasons.append("TX超时")
    if cid & CAN_ERR_BUSERROR:
        reasons.append("总线错误")
    if cid & CAN_ERR_RESTARTED:
        reasons.append("已重启")
    return ",".join(reasons) or f"err=0x{cid:08X}"


class SocketCan:
    def __init__(self, iface: str = "can0", recv_own: bool = False):
        self.iface = iface
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setsockopt(
            SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack("I", CAN_ERR_MASK)
        )
        self.sock.setsockopt(
            SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, struct.pack("i", 1 if recv_own else 0)
        )
        self.sock.bind((iface,))
        self.sock.setblocking(False)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def __enter__(self) -> "SocketCan":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def fileno(self) -> int:
        return self.sock.fileno()

    def send(self, can_id: int, data: bytes, extended: bool = True) -> None:
        payload = bytes(data[:8]).ljust(8, b"\x00")
        cid = can_id & CAN_EFF_MASK
        if extended:
            cid |= CAN_EFF_FLAG
        frame = struct.pack(CAN_FRAME_FMT, cid, min(len(data), 8), payload)
        n = self._send_frame(frame)
        if n is None:
            self._drop_pending()
            n = self._send_frame(frame)
        if n is None:
            raise OSError(
                105,
                "CAN 发送缓冲满。位置保持时刷新太密，或总线暂时发不出去。",
            )
        if n != CAN_FRAME_SIZE:
            raise OSError(f"CAN 发送不完整: {n}/{CAN_FRAME_SIZE}")

    def _send_frame(self, frame: bytes) -> Optional[int]:
        try:
            return self.sock.send(frame)
        except OSError as exc:
            if exc.errno in (11, 105):
                return None
            raise

    def recv(self, timeout: Optional[float] = 0.05) -> Optional[CanFrame]:
        if timeout is not None:
            ready, _, _ = select.select([self.sock], [], [], timeout)
            if not ready:
                return None
        try:
            raw = self.sock.recv(CAN_FRAME_SIZE)
        except BlockingIOError:
            return None
        if len(raw) < CAN_FRAME_SIZE:
            return None
        cid, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
        return CanFrame(
            can_id=cid & CAN_EFF_MASK,
            data=data[:dlc],
            extended=bool(cid & CAN_EFF_FLAG),
            error=bool(cid & CAN_ERR_FLAG),
        )

    def flush_rx(self, limit: int = 128) -> int:
        """读掉已经到达的帧。控制环每拍先清接收，避免回包堵住发送。"""
        n = 0
        for _ in range(limit):
            if self.recv(timeout=0) is None:
                break
            n += 1
        return n

    def _drop_pending(self, limit: int = 64) -> None:
        self.flush_rx(limit)

    def drain(self, window_s: float = 0.02) -> List[CanFrame]:
        frames: List[CanFrame] = []
        deadline = time.monotonic() + window_s
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                break
            frame = self.recv(timeout=remain)
            if frame is None:
                break
            frames.append(frame)
        return frames


class Motor:
    def __init__(self, motor_id: int, motor_type: str = "3532_0", name: str = ""):
        self.id = int(motor_id)
        self.type = motor_type
        self.name = name or f"motor{self.id}"
        self.state = MotorState(motor_id=self.id)
        self._torque_limit_nm = 0.5

    def update_from_status(self, frame: CanFrame) -> bool:
        data = frame.data
        self.state.motor_id = self.id
        self.state.can_id = frame.arb_id
        self.state.payload = data
        self.state.t_rx = time.monotonic()
        if not data:
            return False
        cmd = data[0]

        # 0x0E 标准状态：query, fault, pos, vel, tqe
        if cmd == QUERY_STATUS and len(data) >= 8:
            self.state.fault = data[1]
            raw_pos = unpack_i16_le(data, 2)
            raw_vel = unpack_i16_le(data, 4)
            raw_tqe = unpack_i16_le(data, 6)
            self.state.raw_pos = raw_pos
            self.state.raw_vel = raw_vel
            self.state.raw_tqe = raw_tqe
            self.state.position = pos_from_int(raw_pos)
            self.state.velocity = vel_from_int(raw_vel)
            self.state.torque = tqe_from_int(raw_tqe, self.type)
            self.state.t_pos = time.monotonic()
            return True

        # 0x0C：mode, fault, temp(0.1°C), pos...
        if cmd == QUERY_TEMP and len(data) >= 5:
            self.state.mode = data[1]
            self.state.fault = data[2]
            self.state.temperature = unpack_i16_le(data, 3) * 0.1
            if len(data) >= 7:
                raw_pos = unpack_i16_le(data, 5)
                self.state.raw_pos = raw_pos
                self.state.position = pos_from_int(raw_pos)
                self.state.t_pos = time.monotonic()
            return True

        # 0x0D：mode, fault, voltage(0.1V)
        if cmd == QUERY_VOLTAGE and len(data) >= 5:
            self.state.mode = data[1]
            self.state.fault = data[2]
            self.state.voltage = unpack_i16_le(data, 3) * 0.1
            return True

        # 固件版本：04 patch minor major
        if cmd == QUERY_FW and len(data) >= 4:
            self.state.fw_version = f"{data[3]}.{data[2]}.{data[1]}"
            return True

        # 硬件版本
        if cmd == QUERY_HW and len(data) >= 4:
            return True

        # 型号：07 len ascii...
        if cmd == QUERY_MODEL and len(data) >= 2:
            mlen = data[1]
            self.state.model = data[2 : 2 + mlen].decode("ascii", errors="replace")
            if self.state.model:
                self.type = self.state.model
            return True

        # 系统命令应答：03 result
        if cmd == QUERY_SYS and len(data) >= 2:
            return True

        return False


class MotorBus:
    def __init__(self, iface: str = "can0"):
        self.can = SocketCan(iface)
        self.motors: Dict[int, Motor] = {}

    def close(self) -> None:
        self.can.close()

    def __enter__(self) -> "MotorBus":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def add_motor(self, motor: Motor) -> Motor:
        self.motors[motor.id] = motor
        return motor

    def send_cmd(self, motor_id: int, data: bytes, reply: bool = True) -> None:
        self.can.send(motor_can_id(motor_id, reply=reply), data, extended=True)

    def set_mode(self, motor_id: int, mode: int) -> None:
        """新协议模式写在每帧 Byte0；这里发一帧无参数模式切换（停止/刹车）或空速度。"""
        mode = mode & 0xFF
        if mode in (MODE_STOP, 0x00):
            self.send_cmd(motor_id, bytes([MODE_STOP, QUERY_STATUS]), reply=False)
        elif mode == MODE_BRAKE:
            self.send_cmd(motor_id, bytes([MODE_BRAKE, QUERY_STATUS]), reply=False)
        elif mode in (MODE_VELOCITY, 0x0B):
            self.send_cmd(
                motor_id,
                bytes([MODE_VELOCITY, QUERY_STATUS]) + i16_le(0),
                reply=False,
            )
        elif mode in (MODE_POSITION, 0x0A):
            motor = self.motors.get(motor_id)
            pos = pos_to_int(motor.state.position if motor else 0.0)
            tqe = tqe_to_int(motor._torque_limit_nm if motor else 0.5)
            self.send_cmd(
                motor_id,
                bytes([MODE_POS_VEL_TQE, QUERY_STATUS])
                + i16_le(pos)
                + i16_le(vel_to_int(1.0))
                + i16_le(tqe),
                reply=False,
            )
        else:
            self.send_cmd(motor_id, bytes([mode, QUERY_STATUS]), reply=False)

    def enable(self, motor_id: int) -> None:
        """兼容旧接口：不主动发力。真正出力只走 command_pos_vel_tqe / 速度帧。"""
        _ = motor_id

    def disable(self, motor_id: int) -> None:
        """三相断开，卸力停转。连发几帧，避免仍停在位置保持。"""
        frame = bytes([MODE_STOP, QUERY_STATUS])
        for _ in range(3):
            self.send_cmd(motor_id, frame, reply=False)

    def read_mode(self, motor_id: int) -> None:
        """0x0C 回包带当前模式。0x0E 状态帧不更新模式。"""
        self.send_cmd(motor_id, bytes([0x00, QUERY_TEMP]))

    def motors_not_stopped(self, motor_ids: Iterable[int]) -> List[int]:
        """只认本轮 0x0C 回包。mode 仍是 0 表示没读到，不能当成已经停止。"""
        ids = list(motor_ids)
        for mid in ids:
            self.motors[mid].state.mode = 0
            self.read_mode(mid)
        self.collect(max(0.12, 0.015 * len(ids)))
        return [mid for mid in ids if self.motors[mid].state.mode != MODE_STOP]

    def disable_all(self, motor_ids: Optional[Iterable[int]] = None, rounds: int = 5) -> List[int]:
        """反复发停止，并用 0x0C 回读模式。力矩为 0 仍可能停在位置模式。"""
        ids = list(motor_ids) if motor_ids is not None else list(self.motors)
        pending = list(ids)
        frame = bytes([MODE_STOP, QUERY_STATUS])
        for _ in range(max(1, rounds)):
            if not pending:
                break
            for mid in pending:
                self.send_cmd(mid, frame, reply=False)
            time.sleep(0.03)
            pending = self.motors_not_stopped(pending)
        # 给发送队列时间把停止帧推上总线，避免 close() 把队尾丢掉。
        time.sleep(0.08)
        return pending

    def set_torque_limit(self, motor_id: int, nm: float) -> None:
        """新协议力矩上限随控制帧下发；这里缓存，供后续位置/速度指令使用。"""
        motor = self.motors.get(motor_id)
        if motor is not None:
            motor._torque_limit_nm = abs(float(nm))

    def stop(self, motor_id: int) -> None:
        self.disable(motor_id)

    def ping(self, motor_id: int) -> None:
        """固件版本查询，用于在线探测。"""
        self.send_cmd(motor_id, bytes([0x00, QUERY_FW]))

    def read_status(self, motor_id: int) -> None:
        """纯状态查询（Byte0=0x00, Byte1=0x0E），不切换控制模式。"""
        self.send_cmd(motor_id, bytes([0x00, QUERY_STATUS]))

    def read_voltage(self, motor_id: int) -> None:
        """查母线电压（0x0D）和温度（0x0C）。"""
        self.send_cmd(motor_id, bytes([0x00, QUERY_VOLTAGE]))
        self.send_cmd(motor_id, bytes([0x00, QUERY_TEMP]))

    def set_timeout_ms(self, motor_id: int, timeout_ms: int) -> None:
        """映射为周期返回。timeout_ms<=0 或 <0.1ms 时关闭周期返回。"""
        if timeout_ms <= 0:
            period_us = 0
        else:
            period_us = max(100, int(timeout_ms) * 1000)
        self.send_cmd(
            motor_id,
            bytes([SYS_CMD, 0x00, SYS_PERIODIC, QUERY_STATUS])
            + struct.pack("<I", period_us & 0xFFFFFFFF),
            reply=False,
        )

    def set_zero(self, motor_id: int) -> None:
        """当前位置 → 机械零位（RAM）。"""
        self.send_cmd(motor_id, bytes([SYS_CMD, SYS_CMD, SYS_ZERO]), reply=True)

    def save_config(self, motor_id: int) -> None:
        """保存设置到 flash。"""
        self.send_cmd(motor_id, bytes([SYS_CMD, SYS_CMD, SYS_SAVE]), reply=True)

    def command_pos_vel_tqe(
        self,
        motor: Motor,
        position_rad: float,
        velocity_rad_s: float,
        torque_max_nm: float,
        reply: bool = False,
    ) -> None:
        """位置 + 最大速度 + 力矩上限（模式 0x1F）。

        reply 为假时查询码用 0x00，电机不回状态帧。
        50 Hz 下每台都回 8 字节会把发送队列堵死，摇杆指令就丢。
        """
        query = QUERY_STATUS if reply else 0x00
        data = (
            bytes([MODE_POS_VEL_TQE, query])
            + i16_le(pos_to_int(position_rad))
            + i16_le(vel_to_int(velocity_rad_s))
            + i16_le(tqe_to_int(torque_max_nm, motor.type))
        )
        self.send_cmd(motor.id, data, reply=reply)

    def pos_vel_maxtqe(
        self,
        motor: Motor,
        position_rad: float,
        velocity_rad_s: float,
        torque_max_nm: float,
    ) -> None:
        self.command_pos_vel_tqe(motor, position_rad, velocity_rad_s, torque_max_nm)

    def control_velocity(
        self,
        motor: Motor,
        velocity_rad_s: float,
        acc_rps2: float = 5.0,
    ) -> None:
        """速度 + 加速度（模式 0x1E）。"""
        _ = acc_rps2
        data = (
            bytes([MODE_VELOCITY, QUERY_STATUS])
            + i16_le(vel_to_int(velocity_rad_s))
        )
        self.send_cmd(motor.id, data, reply=False)

    def control_position(
        self, motor: Motor, position_rad: float, torque_max_nm: float
    ) -> None:
        self.command_pos_vel_tqe(motor, position_rad, 2.0, torque_max_nm)

    def stream_pos_vel_tqe(
        self,
        motor: Motor,
        position_rad: float,
        velocity_rad_s: float,
        torque_max_nm: float,
    ) -> None:
        self.command_pos_vel_tqe(motor, position_rad, velocity_rad_s, torque_max_nm)

    def send_stream90_plain(
        self,
        motor: Motor,
        position_rad: float,
        velocity_rad_s: float,
        torque_max_nm: float,
    ) -> None:
        self.command_pos_vel_tqe(motor, position_rad, velocity_rad_s, torque_max_nm)

    def collect(self, timeout: float = 0.05, include_errors: bool = False) -> List[CanFrame]:
        frames: List[CanFrame] = []
        deadline = time.monotonic() + timeout
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                break
            frame = self.can.recv(timeout=remain)
            if frame is None:
                continue
            if frame.error:
                if include_errors:
                    frames.append(frame)
                continue
            frames.append(frame)
            mid = motor_id_from_can_id(frame.arb_id)
            motor = self.motors.get(mid)
            if motor is not None:
                motor.update_from_status(frame)
        return frames

    def scan(self, ids: Sequence[int] = range(1, 13), timeout_each: float = 0.04) -> List[int]:
        """按 ID 对回包。上一台的迟到帧记到对应 ID，缺的再查一轮。"""
        want = [int(mid) for mid in ids]
        found: List[int] = []
        seen: set[int] = set()
        self.can.drain(0.02)

        def _note(frames: List[CanFrame]) -> None:
            for frame in frames:
                mid = motor_id_from_can_id(frame.arb_id)
                if mid not in want or mid in seen:
                    continue
                seen.add(mid)
                found.append(mid)
                if mid not in self.motors:
                    self.add_motor(Motor(mid))
                self.motors[mid].update_from_status(frame)

        for mid in want:
            self.ping(mid)
            _note(self.collect(timeout_each))
        missing = [mid for mid in want if mid not in seen]
        if missing:
            time.sleep(0.05)
            self.can.drain(0.01)
            for mid in missing:
                self.ping(mid)
                _note(self.collect(max(timeout_each, 0.08)))
        return found

    def poll_status(
        self, motor_ids: Optional[Iterable[int]] = None, timeout: float = 0.03
    ) -> None:
        ids = list(motor_ids) if motor_ids is not None else list(self.motors)
        for mid in ids:
            self.read_status(mid)
        self.collect(timeout)


def format_state(motor: Motor) -> str:
    s = motor.state
    mode = MODE_NAMES.get(s.mode, f"0x{s.mode:02X}")
    return (
        f"ID:{motor.id:2d} {motor.name:12s} type={motor.type:8s} "
        f"mode={mode:4s} fault=0x{s.fault:02X} "
        f"pos={s.position:+7.3f}rad vel={s.velocity:+7.3f} "
        f"tor={s.torque:+7.3f}Nm  raw=({s.raw_pos:+6d},{s.raw_vel:+6d},{s.raw_tqe:+6d}) "
        f"can=0x{s.can_id:08X} data={s.payload.hex(' ')}"
    )
