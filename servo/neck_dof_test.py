#!/usr/bin/env python3
"""脖子 roll / yaw 自由度测试。

一整圈按 0~4096 对应 0~360°。0° 在原始值 2048。
yaw 限位 ±45°（原始值 1536~2560）。roll 默认 ±25°，与现有关节限位一致。

默认只读当前位置并打印两端原始值。加 --move 才缓慢扫过两端，结束后回到起点。

用法（在 servo 目录）：
  python3 neck_dof_test.py
  python3 neck_dof_test.py --move
  python3 neck_dof_test.py --joint yaw --move
  python3 neck_dof_test.py --joint roll --roll 30 --move
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import yaml

from bus_servo import FtScsBus
from servo_controller import ServoConfig, ServoController


HERE = Path(__file__).resolve().parent
COUNTS = 4096.0
# 这批 HD1910M 速度 0 才是最快，非 0 会爬得很慢。轨迹用小步目标放慢。
SPEED = 0


def deg_to_raw(deg: float, center_raw: int, direction: int) -> int:
    raw = center_raw + direction * deg / 360.0 * COUNTS
    return int(round(max(0.0, min(COUNTS, raw))))


def load_joint(data: dict, name: str, fallback_id: int, span_deg: float) -> ServoConfig:
    spec = (data.get("servos") or {}).get(name) or {}
    center = float(spec.get("center_deg", 180.0))
    return ServoConfig(
        servo_id=int(spec.get("id", fallback_id)),
        name=name,
        center_deg=center,
        min_deg=center - span_deg,
        max_deg=center + span_deg,
        direction=1 if int(spec.get("direction", 1)) >= 0 else -1,
        center_raw=int(spec.get("center_raw", 2048)),
        raw_min=0,
        raw_max=int(COUNTS),
    )


def read_raw(bus: FtScsBus, servo_id: int) -> int:
    return bus.read_u16(servo_id, 0x38) & 0x0FFF


def sweep(ctrl: ServoController, cfg: ServoConfig, span_deg: float, rate_deg_s: float) -> None:
    """从当前角度走到 +span、回到 0、再到 -span，最后回起点。角度相对 center_deg。"""
    start = ctrl.read_angle(cfg.name)
    center = cfg.center_deg
    goals = (center + span_deg, center, center - span_deg, center, start)
    current = start
    ctrl.torque(cfg.name, True)
    print(f"{cfg.name} ID {cfg.servo_id}  起点 {start - center:+.1f}°  原始值 {deg_to_raw(start - center, cfg.center_raw, cfg.direction)}")
    for goal in goals:
        span = goal - current
        steps = max(1, int(abs(span) / max(rate_deg_s, 1.0) / 0.05))
        for i in range(1, steps + 1):
            angle = current + span * i / steps
            raw = ctrl.set_angle(cfg.name, angle, speed=SPEED, acceleration=0, torque_limit=400)
            if i == steps:
                print(f"  -> {angle - center:+.1f}°  原始值 {raw}")
            time.sleep(0.05)
        time.sleep(0.3)
        current = goal
    seen = ctrl.read_angle(cfg.name)
    print(f"  读回 {seen - center:+.1f}°  原始值 {read_raw(ctrl.bus, cfg.servo_id)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="脖子 roll / yaw 自由度测试，0~4096 对应 0~360°")
    parser.add_argument("--config", type=Path, default=HERE / "servo_config.yaml")
    parser.add_argument("--port")
    parser.add_argument("--joint", choices=("roll", "yaw", "all"), default="roll")
    parser.add_argument("--roll", type=float, default=25.0, help="roll 半幅，度。默认 ±25")
    parser.add_argument("--yaw", type=float, default=45.0, help="yaw 半幅，度。默认 ±45")
    parser.add_argument("--rate", type=float, default=20.0, help="扫描速度，度/秒")
    parser.add_argument("--move", action="store_true", help="确认允许转动脖子")
    args = parser.parse_args()
    if args.roll < 0 or args.yaw < 0 or args.yaw > 45.0:
        parser.error("yaw 不超过 ±45°，半幅不能为负")

    data = yaml.safe_load(args.config.read_text(encoding="utf-8")) or {}
    joints = {
        "roll": load_joint(data, "neck_roll", 14, args.roll),
        "yaw": load_joint(data, "neck_yaw", 13, args.yaw),
    }
    spans = {"roll": args.roll, "yaw": args.yaw}
    names = ("roll", "yaw") if args.joint == "all" else (args.joint,)
    port = args.port or str(data.get("port", "/dev/ttyMSM0"))
    baud = int(data.get("baudrate", 1_000_000))

    print(f"串口 {port}  {baud} bps。0~4096 = 0~360°，0° = 原始值 2048。")
    for name in names:
        cfg = joints[name]
        span = spans[name]
        lo = deg_to_raw(-span, cfg.center_raw, cfg.direction)
        hi = deg_to_raw(span, cfg.center_raw, cfg.direction)
        print(f"  {cfg.name} ID {cfg.servo_id}  ±{span:.0f}°  原始值 {lo} ~ {hi}")

    try:
        bus = FtScsBus(port, baudrate=baud, timeout=float(data.get("timeout_s", 0.08)))
    except Exception as exc:
        print(f"打开串口失败：{exc}")
        return 1

    try:
        ctrl = ServoController(bus, {cfg.name: cfg for cfg in joints.values()})
        for name in names:
            cfg = joints[name]
            try:
                bus.ping(cfg.servo_id)
                raw = read_raw(bus, cfg.servo_id)
            except Exception as exc:
                print(f"{cfg.name} ID {cfg.servo_id} 没有应答：{exc}")
                return 2
            offset = (raw - cfg.center_raw) * cfg.direction * 360.0 / COUNTS
            print(f"{cfg.name} 当前原始值 {raw}，相对 0° {offset:+.1f}°")
        if not args.move:
            print("未加 --move，脖子保持不动。")
            return 0
        for name in names:
            sweep(ctrl, joints[name], spans[name], args.rate)
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
