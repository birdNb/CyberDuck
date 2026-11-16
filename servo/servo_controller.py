#!/usr/bin/env python3
"""High-level angle control for FEETECH HD1910M servos."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Mapping

from bus_servo import (
    ADDR_ACCELERATION,
    ADDR_GOAL_POSITION,
    ADDR_GOAL_SPEED,
    ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    ADDR_TORQUE_LIMIT,
    FtScsBus,
    HD1910M_POSITION_COUNTS,
    signed_value,
)


# 一整圈按 0~4096 映射到 0~360°。
HLS_COUNTS_PER_REV = 4096.0


@dataclass(frozen=True)
class ServoConfig:
    servo_id: int
    name: str
    center_deg: float = 180.0
    min_deg: float = 0.0
    max_deg: float = 360.0
    direction: int = 1
    center_raw: int = 2048
    raw_min: int = 0
    raw_max: int = HD1910M_POSITION_COUNTS


class ServoController:
    def __init__(self, bus: FtScsBus, configs: Mapping[str, ServoConfig]) -> None:
        self.bus = bus
        self.configs = dict(configs)
        self.by_id = {cfg.servo_id: cfg for cfg in self.configs.values()}

    @staticmethod
    def angle_to_raw(angle_deg: float, cfg: ServoConfig) -> int:
        angle = max(cfg.min_deg, min(cfg.max_deg, angle_deg))
        relative = (angle - cfg.center_deg) * cfg.direction
        raw = int(round(cfg.center_raw + relative / 360.0 * HLS_COUNTS_PER_REV))
        return max(cfg.raw_min, min(cfg.raw_max, raw))

    @staticmethod
    def raw_to_angle(raw: int, cfg: ServoConfig) -> float:
        position = raw & 0x0FFF
        relative = (position - cfg.center_raw) * 360.0 / HLS_COUNTS_PER_REV
        return cfg.center_deg + cfg.direction * relative

    def set_angle(
        self,
        name: str,
        angle_deg: float,
        *,
        speed: int = 20,
        acceleration: int = 0,
        torque_limit: int | None = None,
    ) -> int:
        cfg = self.configs[name]
        raw = self.angle_to_raw(angle_deg, cfg)
        if torque_limit is not None:
            self.bus.write_u16(cfg.servo_id, ADDR_TORQUE_LIMIT, max(0, min(1000, torque_limit)))
        self.bus.write_u8(cfg.servo_id, ADDR_ACCELERATION, max(0, min(254, acceleration)))
        self.bus.write(cfg.servo_id, ADDR_GOAL_SPEED, signed_value(speed))
        self.bus.write_u16(cfg.servo_id, ADDR_GOAL_POSITION, raw)
        return raw

    def set_raw(
        self,
        name: str,
        raw: int,
        *,
        speed: int = 20,
        acceleration: int = 0,
        torque_limit: int | None = None,
    ) -> int:
        """Send a raw HD1910M position, useful for first mechanical calibration."""
        cfg = self.configs[name]
        raw = max(cfg.raw_min, min(cfg.raw_max, int(raw)))
        if torque_limit is not None:
            self.bus.write_u16(cfg.servo_id, ADDR_TORQUE_LIMIT, max(0, min(1000, torque_limit)))
        self.bus.write_u8(cfg.servo_id, ADDR_ACCELERATION, max(0, min(254, acceleration)))
        self.bus.write(cfg.servo_id, ADDR_GOAL_SPEED, signed_value(speed))
        self.bus.write_u16(cfg.servo_id, ADDR_GOAL_POSITION, raw)
        return raw

    def set_angles(self, angles_deg: Mapping[str, float], **kwargs) -> dict[str, int]:
        raw = {}
        for name, angle in angles_deg.items():
            raw[name] = self.set_angle(name, angle, **kwargs)
        return raw

    def torque(self, name: str, enabled: bool) -> None:
        self.bus.write_u8(self.configs[name].servo_id, ADDR_TORQUE_ENABLE, 1 if enabled else 0)

    def torque_all(self, enabled: bool) -> None:
        for name in self.configs:
            self.torque(name, enabled)

    def center(self, name: str, **kwargs) -> int:
        return self.set_angle(name, self.configs[name].center_deg, **kwargs)

    def center_all(self, **kwargs) -> dict[str, int]:
        return self.set_angles(
            {name: cfg.center_deg for name, cfg in self.configs.items()}, **kwargs
        )

    def read_angle(self, name: str) -> float:
        cfg = self.configs[name]
        raw = self.bus.read_u16(cfg.servo_id, ADDR_PRESENT_POSITION)
        return self.raw_to_angle(raw, cfg)

    def move_slowly(
        self,
        name: str,
        target_deg: float,
        *,
        duration_s: float = 1.0,
        steps: int = 50,
        speed: int = 20,
        acceleration: int = 0,
        torque_limit: int | None = None,
    ) -> None:
        cfg = self.configs[name]
        start = self.read_angle(name)
        target = max(cfg.min_deg, min(cfg.max_deg, target_deg))

        # Motion settings are constant for the whole interpolation. Avoid
        # sending the same configuration packet once per position step.
        if torque_limit is not None:
            self.bus.write_u16(cfg.servo_id, ADDR_TORQUE_LIMIT, max(0, min(1000, torque_limit)))
        self.bus.write_u8(cfg.servo_id, ADDR_ACCELERATION, max(0, min(254, acceleration)))
        self.bus.write(cfg.servo_id, ADDR_GOAL_SPEED, signed_value(speed))

        step_count = max(1, steps)
        for i in range(1, step_count + 1):
            u = i / step_count
            # Smoothstep avoids a sudden start and stop.
            u = u * u * (3.0 - 2.0 * u)
            raw = self.angle_to_raw(start + (target - start) * u, cfg)
            self.bus.write_u16(cfg.servo_id, ADDR_GOAL_POSITION, raw)
            time.sleep(max(0.0, duration_s / step_count))

