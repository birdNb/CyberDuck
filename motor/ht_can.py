#!/usr/bin/env python3
"""HighTorque motor control over SocketCAN (PCAN-USB).

原工程 hightorque_robot 走 USB-CDC 通讯板(0xF7 + MODE_*)。
本模块改为 PCAN / SocketCAN 直连电机：29-bit 扩展帧，波特率 1 Mbps。
协议按 Seeed HighTorque_Control / 电机寄存器命令实现。
"""

from __future__ import annotations

import math
import select
import socket
import struct
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_FLAG = getattr(socket, "CAN_EFF_FLAG", 0x80000000)
CAN_RTR_FLAG = getattr(socket, "CAN_RTR_FLAG", 0x40000000)
CAN_ERR_FLAG = getattr(socket, "CAN_ERR_FLAG", 0x20000000)
CAN_EFF_MASK = getattr(socket, "CAN_EFF_MASK", 0x1FFFFFFF)
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_ERR_FILTER = 2
CAN_RAW_LOOPBACK = 3
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
MAGIC_POS = -32768  # 0x8000, 原 SDK 未填充槽位的占位

# 与 include/hardware/motor.hpp 中 motor_tqe_adj 对齐
MOTOR_TQE_ADJ = {
    "3536_32": 0.4581,
    "4538_19": 0.4450,
    "5046_20": 0.5280,
    "5047_9": 0.5330,
    "5047_36": 0.4938,
    "5047_36_2": 0.8030,
    "4438_30": 0.5256,
    "4438_32": 0.5584,
    "6056_36": 0.6770,
    "7256_35": 0.6770,
    "60SG_35": 0.7942,
    "60BM_35": 0.7942,
    "5043_20": 0.9660,
    "General": 0.5000,
    "NONE": 1.0000,
    "HT4538": 0.4450,
    "HT5046": 0.5280,
    "HT5047_36": 0.8030,
    "HT5047_9": 0.5330,
}

PING_PAYLOAD = bytes([0x11, 0x00, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50])


def _clip_i16(value: int) -> int:
    if value >= 32700:
        return 32700
    if value <= -32700:
        return -32700
    return int(value)


def pos_to_int(rad: float) -> int:
    """rad -> 0.0001 圈，与原 SDK radian_2pi 一致。"""
    return _clip_i16(int(rad / TWO_PI * 10000.0))


def vel_to_int(rad_s: float) -> int:
    """rad/s -> 0.00025 转/秒。"""
    return _clip_i16(int(rad_s / TWO_PI * 4000.0))


def tqe_to_int(nm: float, motor_type: str) -> int:
    adj = MOTOR_TQE_ADJ.get(motor_type)
    if adj is None:
        raise KeyError(f"未知电机型号 {motor_type}，可选: {sorted(MOTOR_TQE_ADJ)}")
    return _clip_i16(int(nm / (adj * 0.01)))


def pos_from_int(raw: int) -> float:
    return raw * TWO_PI / 10000.0


def vel_from_int(raw: int) -> float:
    return raw * TWO_PI / 4000.0


def tqe_from_int(raw: int, motor_type: str) -> float:
    adj = MOTOR_TQE_ADJ.get(motor_type, 0.5)
    return raw * (adj * 0.01)


MODE_NAMES = {
    0x00: "停止",
    0x0A: "位置",
    0x0B: "速度",
    0x0C: "力矩",
}


def motor_can_id(motor_id: int, reply: bool = True) -> int:
    """寄存器命令 CAN ID：低 8 位电机 ID，bit15=需要回复。"""
    can_id = motor_id & 0xFF
    if reply:
        can_id |= 0x8000
    return can_id


def motor_id_from_can_id(can_id: int) -> int:
    """实机回复是标准帧 0xID00，例如 ID1 → 0x100。"""
    cid = can_id & CAN_EFF_MASK
    src = (cid >> 8) & 0x7F
    if 1 <= src <= 127:
        return src
    return cid & 0xFF


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
        try:
            n = self.sock.send(frame)
        except OSError as exc:
            if exc.errno != 105:
                raise
            for _ in range(2):
                self._drop_pending()
                time.sleep(0.001)
                try:
                    n = self.sock.send(frame)
                    break
                except OSError as retry_exc:
                    if retry_exc.errno != 105:
                        raise
                    exc = retry_exc
            else:
                raise OSError(
                    105,
                    "CAN 发送缓冲满。位置保持时刷新太密，或总线暂时发不出去。",
                ) from exc
        if n != CAN_FRAME_SIZE:
            raise OSError(f"CAN 发送不完整: {n}/{CAN_FRAME_SIZE}")

    def recv(self, timeout: Optional[float] = 0.05) -> Optional[CanFrame]:
        if timeout is not None:
            ready, _, _ = select.select([self.sock], [], [], timeout)
            if not ready:
                return None
        raw = self.sock.recv(CAN_FRAME_SIZE)
        if len(raw) < CAN_FRAME_SIZE:
            return None
        cid, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
        return CanFrame(
            can_id=cid & CAN_EFF_MASK,
            data=data[:dlc],
            extended=bool(cid & CAN_EFF_FLAG),
            error=bool(cid & CAN_ERR_FLAG),
        )

    def _drop_pending(self, limit: int = 64) -> None:
        """把已经到主机的帧读掉，避免接收积压后发送一直返回缓冲满。"""
        for _ in range(limit):
            if self.recv(timeout=0) is None:
                break

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
    def __init__(self, motor_id: int, motor_type: str = "4538_19", name: str = ""):
        self.id = int(motor_id)
        self.type = motor_type
        self.name = name or f"motor{self.id}"
        self.state = MotorState(motor_id=self.id)

    def update_from_status(self, frame: CanFrame) -> bool:
        data = frame.data
        self.state.motor_id = self.id
        self.state.can_id = frame.arb_id
        self.state.payload = data
        self.state.t_rx = time.monotonic()
        if not data:
            return False
        cmd = data[0]
        # 读 1 个 int16：0x15 0x0D → 回复 0x25 0x0D，母线电压，1 LSB = 0.1V
        if cmd == 0x25 and len(data) >= 4 and data[1] == 0x0D:
            self.state.voltage = unpack_i16_le(data, 2) * 0.1
            return True
        # 读模式回复: 21 00 0A → 位置模式
        if cmd in (0x21, 0x11) and len(data) >= 3 and data[1] == 0x00:
            self.state.mode = data[2]
            return True
        if len(data) < 8:
            if len(data) >= 2:
                self.state.mode = data[1]
            return False
        if cmd in (0x27, 0x17):
            # [0x27, 起始寄存器, pos, vel, tqe]，data[1] 不是模式
            raw_pos = unpack_i16_le(data, 2)
            raw_vel = unpack_i16_le(data, 4)
            raw_tqe = unpack_i16_le(data, 6)
        else:
            raw_pos = unpack_i16_le(data, 0)
            raw_vel = unpack_i16_le(data, 2)
            raw_tqe = unpack_i16_le(data, 4)
        self.state.raw_pos = raw_pos
        self.state.raw_vel = raw_vel
        self.state.raw_tqe = raw_tqe
        self.state.position = pos_from_int(raw_pos)
        self.state.velocity = vel_from_int(raw_vel)
        self.state.torque = tqe_from_int(raw_tqe, self.type)
        return True


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
        self.send_cmd(
            motor_id,
            bytes([0x01, 0x00, mode & 0xFF, 0x50, 0x50, 0x50, 0x50, 0x50]),
            reply=False,
        )

    def enable(self, motor_id: int) -> None:
        self.set_mode(motor_id, 0x0A)

    def disable(self, motor_id: int) -> None:
        self.set_mode(motor_id, 0x00)

    def set_torque_limit(self, motor_id: int, nm: float) -> None:
        self.send_cmd(
            motor_id,
            bytes([0x0D, 0x22]) + struct.pack("<f", float(nm)) + bytes([0x50, 0x50]),
            reply=False,
        )

    def stop(self, motor_id: int) -> None:
        self.disable(motor_id)

    def ping(self, motor_id: int) -> None:
        self.send_cmd(motor_id, PING_PAYLOAD)

    def read_status(self, motor_id: int) -> None:
        self.send_cmd(motor_id, bytes([0x17, 0x01]))

    def read_voltage(self, motor_id: int) -> None:
        """读母线电压，寄存器 0x0D，1 个 int16。"""
        self.send_cmd(motor_id, bytes([0x15, 0x0D]))

    def set_timeout_ms(self, motor_id: int, timeout_ms: int) -> None:
        # 0 表示禁用超时。原 SDK MODE_TIME_OUT。
        self.send_cmd(motor_id, bytes([0x05, 0xb4, 0x02, 0x00]) + struct.pack("<H", timeout_ms & 0xFFFF))

    def set_zero(self, motor_id: int) -> None:
        """把当前位置设为机械零位（只改 RAM）。官方帧 40 01 04 64 20 63 0A。"""
        self.send_cmd(motor_id, bytes([0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0A]))

    def save_config(self, motor_id: int) -> None:
        """把 RAM 配置写入 flash。官方帧 05 B3 02 00 00，建议之后给电机重新上电。"""
        self.send_cmd(motor_id, bytes([0x05, 0xB3, 0x02, 0x00, 0x00]))

    def command_pos_vel_tqe(
        self,
        motor: Motor,
        position_rad: float,
        velocity_rad_s: float,
        torque_max_nm: float,
        reply: bool = False,
    ) -> None:
        """按电机 ID 发位置/速度/力矩，避免 0x0090 广播带动整条总线。"""
        data = (
            bytes([0x07, 0x07])
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
        """对应原 SDK MODE_POS_VEL_TQE 0x90 流控。实机已验证 CAN ID=0x0090。"""
        self.send_stream90_plain(motor, position_rad, velocity_rad_s, torque_max_nm)

    def control_velocity(
        self,
        motor: Motor,
        velocity_rad_s: float,
        acc_rps2: float = 5.0,
    ) -> None:
        """速度模式：CAN ID 0x00AD，pos=MAGIC(-32768)，vel 单位 rad/s。"""
        vel_int = vel_to_int(velocity_rad_s)
        acc = 30.0 if abs(velocity_rad_s) < 1e-6 else abs(acc_rps2)
        acc_int = _clip_i16(int(acc * 1000.0))
        data = i16_le(MAGIC_POS) + i16_le(vel_int) + i16_le(acc_int) + bytes([0x50, 0x50])
        self.can.send(0x00AD, data, extended=True)

    def control_position(self, motor: Motor, position_rad: float, torque_max_nm: float) -> None:
        pos = pos_to_int(position_rad)
        tqe = tqe_to_int(torque_max_nm, motor.type)
        data = bytes([0x07, 0x07]) + i16_le(pos) + bytes([0x00, 0x00]) + i16_le(tqe)
        self.send_cmd(motor.id, data)

    def stream_pos_vel_tqe(
        self,
        motor: Motor,
        position_rad: float,
        velocity_rad_s: float,
        torque_max_nm: float,
    ) -> None:
        """HighTorque 流式 0x90 帧：CAN ID=0x0090，数据 pos/vel/tqe + 0x50 0x50。

        多电机时再叠加电机 ID 到低 8 位（0x0090 | id 无法区分，故用 0x90<<8 | id）。
        扫描阶段若电机只认 0x0090，可再用 send_stream90_plain。
        """
        pos = pos_to_int(position_rad)
        vel = vel_to_int(velocity_rad_s)
        tqe = tqe_to_int(torque_max_nm, motor.type)
        data = i16_le(pos) + i16_le(vel) + i16_le(tqe) + bytes([0x50, 0x50])
        self.can.send((0x90 << 8) | (motor.id & 0xFF), data, extended=True)

    def send_stream90_plain(
        self,
        motor: Motor,
        position_rad: float,
        velocity_rad_s: float,
        torque_max_nm: float,
    ) -> None:
        pos = pos_to_int(position_rad)
        vel = vel_to_int(velocity_rad_s)
        tqe = tqe_to_int(torque_max_nm, motor.type)
        data = i16_le(pos) + i16_le(vel) + i16_le(tqe) + bytes([0x50, 0x50])
        self.can.send(0x0090, data, extended=True)

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

    def scan(self, ids: Sequence[int] = range(1, 15), timeout_each: float = 0.04) -> List[int]:
        found: List[int] = []
        self.can.drain(0.02)
        for mid in ids:
            self.ping(mid)
            frames = self.collect(timeout_each)
            hits = [f for f in frames if motor_id_from_can_id(f.arb_id) == mid]
            if hits:
                found.append(mid)
                if mid not in self.motors:
                    self.add_motor(Motor(mid))
                self.motors[mid].update_from_status(hits[0])
        return found

    def poll_status(self, motor_ids: Optional[Iterable[int]] = None, timeout: float = 0.03) -> None:
        ids = list(motor_ids) if motor_ids is not None else list(self.motors)
        for mid in ids:
            self.read_status(mid)
        self.collect(timeout)


def format_state(motor: Motor) -> str:
    s = motor.state
    mode = MODE_NAMES.get(s.mode, f"0x{s.mode:02X}")
    return (
        f"ID:{motor.id:2d} {motor.name:12s} type={motor.type:8s} "
        f"mode={mode:4s} pos={s.position:+7.3f}rad vel={s.velocity:+7.3f} "
        f"tor={s.torque:+7.3f}Nm  raw=({s.raw_pos:+6d},{s.raw_vel:+6d},{s.raw_tqe:+6d}) "
        f"can=0x{s.can_id:08X} data={s.payload.hex(' ')}"
    )
