#!/usr/bin/env python3
"""High-level angle control for FEETECH STS3215 servos."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Mapping

from bus_servo import (
    ADDR_ACCELERATION,
    ADDR_GOAL_POSITION,
    ADDR_GOAL_SPEED,
    ADDR_TORQUE_ENABLE,
    ADDR_TORQUE_LIMIT,
    FtScsBus,
    signed_position,
    signed_value,
)


STS_COUNTS_PER_REV = 4095.0


@dataclass(frozen=True)
class ServoConfig:
    servo_id: int
    name: str
    center_deg: float = 180.0
    min_deg: float = 0.0
    max_deg: float = 360.0
    direction: int = 1


class ServoController:
    def __init__(self, bus: FtScsBus, configs: Mapping[str, ServoConfig]) -> None:
        self.bus = bus
        self.configs = dict(configs)
        self.by_id = {cfg.servo_id: cfg for cfg in self.configs.values()}

    @staticmethod
    def angle_to_raw(angle_deg: float, cfg: ServoConfig) -> int:
        angle = max(cfg.min_deg, min(cfg.max_deg, angle_deg))
        relative = (angle - cfg.center_deg) * cfg.direction
        raw = int(round(2048.0 + relative / 360.0 * STS_COUNTS_PER_REV))
        return max(0, min(4095, raw))

    @staticmethod
    def raw_to_angle(raw: int, cfg: ServoConfig) -> float:
        position = raw & 0x0FFF
        relative = (position - 2048.0) * 360.0 / STS_COUNTS_PER_REV
        return cfg.center_deg + cfg.direction * relative

    def set_angle(
        self,
        name: str,
        angle_deg: float,
        *,
        speed: int = 0,
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
        speed: int = 0,
        acceleration: int = 0,
        torque_limit: int | None = None,
    ) -> int:
        """Send a raw STS position, useful for first mechanical calibration."""
        cfg = self.configs[name]
        raw = max(0, min(4095, int(raw)))
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
        raw = self.bus.read_u16(cfg.servo_id, 0x38)
        return self.raw_to_angle(raw, cfg)

    def move_slowly(
        self,
        name: str,
        target_deg: float,
        *,
        duration_s: float = 1.0,
        steps: int = 50,
        speed: int = 0,
        acceleration: int = 0,
        torque_limit: int | None = None,
    ) -> None:
        start = self.read_angle(name)
        target = max(self.configs[name].min_deg, min(self.configs[name].max_deg, target_deg))
        for i in range(1, max(1, steps) + 1):
            u = i / max(1, steps)
            # Smoothstep avoids a sudden start and stop.
            u = u * u * (3.0 - 2.0 * u)
            self.set_angle(
                name,
                start + (target - start) * u,
                speed=speed,
                acceleration=acceleration,
                torque_limit=torque_limit,
            )
            time.sleep(max(0.0, duration_s / max(1, steps)))

