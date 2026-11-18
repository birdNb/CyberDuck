#!/usr/bin/env python3
"""FT-SCS half-duplex bus protocol used by FEETECH HD1910M servos."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Iterable, Optional

import serial


HEADER = b"\xff\xff"
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_ACTION = 0x05
INST_RESET = 0x06
INST_SYNC_WRITE = 0x83

ADDR_ID = 0x05
ADDR_BAUD = 0x06
ADDR_SECONDARY_ID = 0x07
ADDR_RETURN_LEVEL = 0x08
ADDR_TORQUE_ENABLE = 0x28
ADDR_ACCELERATION = 0x29
ADDR_GOAL_POSITION = 0x2A
ADDR_TARGET_CURRENT = 0x2C
ADDR_GOAL_SPEED = 0x2E
ADDR_TORQUE_LIMIT = 0x30
ADDR_KP = 0x32
ADDR_KD = 0x33
ADDR_KI = 0x34
ADDR_PRESENT_POSITION = 0x38
ADDR_PRESENT_SPEED = 0x3A
ADDR_PRESENT_LOAD = 0x3C
ADDR_PRESENT_VOLTAGE = 0x3E
ADDR_PRESENT_TEMPERATURE = 0x3F
ADDR_SERVO_STATUS = 0x41
ADDR_MOVING = 0x42
ADDR_GOAL_POSITION_FEEDBACK = 0x43
ADDR_PRESENT_CURRENT = 0x45
ADDR_CURRENT_BIAS = 0x49

HD1910M_POSITION_COUNTS = 4095
HD1910M_SPEED_RPM_PER_COUNT = 0.732
HD1910M_CURRENT_MA_PER_COUNT = 6.5
HD1910M_LOAD_SIGN_BIT = 0x0400
HD1910M_LOAD_MAGNITUDE_MASK = 0x03FF
HD1910M_FEEDBACK_SIZE = 19


class ServoProtocolError(RuntimeError):
    pass


class ServoTimeout(ServoProtocolError):
    pass


def checksum(packet_without_checksum: bytes) -> int:
    return (~sum(packet_without_checksum)) & 0xFF


def make_packet(servo_id: int, instruction: int, params: bytes = b"") -> bytes:
    if not 0 <= servo_id <= 253:
        raise ValueError(f"servo id out of range: {servo_id}")
    length = len(params) + 2
    if length > 255:
        raise ValueError("packet is too long")
    body = bytes((servo_id, length, instruction)) + bytes(params)
    return HEADER + body + bytes((checksum(body),))


@dataclass(frozen=True)
class StatusPacket:
    servo_id: int
    error: int
    params: bytes
    raw: bytes


def parse_status(packet: bytes) -> StatusPacket:
    if len(packet) < 6 or packet[:2] != HEADER:
        raise ServoProtocolError(f"invalid status packet: {packet.hex(' ')}")
    servo_id = packet[2]
    length = packet[3]
    expected = length + 4
    if len(packet) != expected:
        raise ServoProtocolError(
            f"invalid packet length: got {len(packet)}, expected {expected}"
        )
    if length < 2:
        raise ServoProtocolError("invalid status length")
    if checksum(packet[2:-1]) != packet[-1]:
        raise ServoProtocolError(f"checksum mismatch: {packet.hex(' ')}")
    return StatusPacket(servo_id, packet[4], packet[5:-1], packet)


def u16le(data: bytes, offset: int = 0) -> int:
    if offset + 2 > len(data):
        raise ServoProtocolError("response is shorter than expected")
    return data[offset] | (data[offset + 1] << 8)


def put_u16le(value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"u16 out of range: {value}")
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def signed_position(raw: int) -> int:
    """Decode HLS signed values that use bit 15 as the direction bit."""
    magnitude = raw & 0x7FFF
    return -magnitude if raw & 0x8000 else magnitude


def signed_load(raw: int) -> int:
    """Decode HD1910M load; bit 10 is the direction bit."""
    magnitude = raw & HD1910M_LOAD_MAGNITUDE_MASK
    return -magnitude if raw & HD1910M_LOAD_SIGN_BIT else magnitude


def signed_value(value: int) -> bytes:
    """Encode an HLS signed value using bit 15 and magnitude bits 0..14."""
    if not -0x7FFF <= value <= 0x7FFF:
        raise ValueError(f"signed HLS value out of range: {value}")
    raw = abs(value) | (0x8000 if value < 0 else 0)
    return put_u16le(raw)


@dataclass
class ServoState:
    servo_id: int
    position_raw: Optional[int] = None
    position: Optional[int] = None
    speed_raw: Optional[int] = None
    speed: Optional[int] = None
    load_raw: Optional[int] = None
    load: Optional[int] = None
    voltage_v: Optional[float] = None
    temperature_c: Optional[int] = None
    status: Optional[int] = None
    moving: Optional[bool] = None
    moving_flags: Optional[int] = None
    async_write: Optional[int] = None
    goal_position_raw: Optional[int] = None
    current_ma: Optional[float] = None
    current_bias_raw: Optional[int] = None


class FtScsBus:
    """Serial transport for a USB-TTL adapter connected to the HLS bus."""

    def __init__(
        self,
        port: str,
        baudrate: int = 1_000_000,
        timeout: float = 0.08,
        write_timeout: float = 0.08,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=write_timeout,
        )

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "FtScsBus":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _read_status(self, expected_id: int) -> StatusPacket:
        deadline = time.monotonic() + self.timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self.ser.read(max(1, min(64, int(self.ser.in_waiting) or 1)))
            if chunk:
                buf.extend(chunk)
            while len(buf) >= 2 and bytes(buf[:2]) != HEADER:
                del buf[0]
            if len(buf) < 4:
                continue
            length = buf[3]
            total = length + 4
            if length < 2 or total > 260:
                del buf[0]
                continue
            if len(buf) < total:
                continue
            packet = bytes(buf[:total])
            del buf[:total]
            status = parse_status(packet)
            if status.servo_id != expected_id:
                continue
            if status.error:
                raise ServoProtocolError(
                    f"servo {expected_id} returned error 0x{status.error:02x}"
                )
            return status
        raise ServoTimeout(f"timeout waiting for servo {expected_id}")

    def transact(
        self,
        servo_id: int,
        instruction: int,
        params: bytes = b"",
        expect_response: bool = True,
        flush: bool = True,
    ) -> Optional[StatusPacket]:
        self.ser.reset_input_buffer()
        packet = make_packet(servo_id, instruction, params)
        self.ser.write(packet)
        if flush:
            self.ser.flush()
        if not expect_response:
            return None
        return self._read_status(servo_id)

    def ping(self, servo_id: int) -> StatusPacket:
        response = self.transact(servo_id, INST_PING)
        assert response is not None
        return response

    def scan(self, ids: Iterable[int] = range(0, 254)) -> list[int]:
        found = []
        for servo_id in ids:
            try:
                self.ping(int(servo_id))
            except ServoProtocolError:
                continue
            found.append(int(servo_id))
        return found

    def read(self, servo_id: int, address: int, size: int) -> bytes:
        response = self.transact(servo_id, INST_READ, bytes((address, size)))
        assert response is not None
        if len(response.params) < size:
            raise ServoProtocolError(
                f"servo {servo_id} read 0x{address:02x}: "
                f"got {len(response.params)} bytes, expected {size}"
            )
        return response.params[:size]

    def write(
        self,
        servo_id: int,
        address: int,
        data: bytes,
        response: bool = False,
        flush: bool = True,
    ) -> Optional[StatusPacket]:
        return self.transact(
            servo_id,
            INST_WRITE,
            bytes((address,)) + bytes(data),
            expect_response=response,
            flush=flush,
        )

    def read_u8(self, servo_id: int, address: int) -> int:
        return self.read(servo_id, address, 1)[0]

    def read_u16(self, servo_id: int, address: int) -> int:
        return u16le(self.read(servo_id, address, 2))

    def write_u8(self, servo_id: int, address: int, value: int, response: bool = False) -> None:
        self.write(servo_id, address, bytes((value & 0xFF,)), response=response)

    def write_u16(
        self,
        servo_id: int,
        address: int,
        value: int,
        response: bool = False,
        flush: bool = True,
    ) -> None:
        self.write(servo_id, address, put_u16le(value), response=response, flush=flush)

    def read_state(self, servo_id: int) -> ServoState:
        state = ServoState(servo_id=servo_id)
        # 0x38..0x49 is the complete HD1910M feedback block.
        raw = self.read(servo_id, ADDR_PRESENT_POSITION, HD1910M_FEEDBACK_SIZE)
        state.position_raw = u16le(raw, 0)
        state.position = signed_position(state.position_raw)
        state.speed_raw = u16le(raw, 2)
        state.speed = signed_position(state.speed_raw)
        state.load_raw = u16le(raw, 4)
        state.load = signed_load(state.load_raw)
        state.voltage_v = raw[6] * 0.1
        state.temperature_c = raw[7]
        state.async_write = raw[8]
        state.status = raw[9]
        state.moving_flags = raw[10]
        state.moving = bool(raw[10] & 0x03)
        state.goal_position_raw = u16le(raw, 11)
        state.current_ma = signed_position(u16le(raw, 13)) * HD1910M_CURRENT_MA_PER_COUNT
        state.current_bias_raw = u16le(raw, 17)
        return state

