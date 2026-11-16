#!/usr/bin/env python3
"""Standalone HD1910M test tool. Real motion requires --move."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import serial
import yaml

from bus_servo import FtScsBus, ServoProtocolError
from servo_controller import ServoConfig, ServoController


HERE = Path(__file__).resolve().parent


def load_config(path: Path) -> tuple[dict, dict[str, ServoConfig]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    configs = {}
    for name, item in (data.get("servos") or {}).items():
        configs[name] = ServoConfig(
            servo_id=int(item["id"]),
            name=name,
            center_deg=float(item.get("center_deg", 180.0)),
            min_deg=float(item.get("min_deg", 0.0)),
            max_deg=float(item.get("max_deg", 360.0)),
            direction=1 if int(item.get("direction", 1)) >= 0 else -1,
            center_raw=int(item.get("center_raw", 2048)),
            raw_min=int(item.get("raw_min", 0)),
            raw_max=int(item.get("raw_max", 4095)),
        )
    return data, configs


def main() -> int:
    parser = argparse.ArgumentParser(description="飞特 HD1910M 独立测试工具")
    parser.add_argument("--config", type=Path, default=HERE / "servo_config.yaml")
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int)
    parser.add_argument("--scan", action="store_true", help="扫描 0..253，耗时较长")
    parser.add_argument("--id", type=int, help="直接操作的舵机 ID")
    parser.add_argument("--read", action="store_true", help="读取位置、速度、电压等状态")
    parser.add_argument("--center", action="store_true", help="移动指定舵机到中心")
    parser.add_argument("--all-center", action="store_true", help="三个配置舵机回中心")
    parser.add_argument("--angle", type=float, help="指定舵机目标角度")
    parser.add_argument("--raw", type=int, help="直接指定 HD1910M 目标位置 0..4095，用于首次校准")
    parser.add_argument("--speed", type=int, help="HD1910M 速度原始值，单位约 0.732 RPM；0 表示停止")
    parser.add_argument("--acceleration", type=int, help="HD1910M 加速度 0..254")
    parser.add_argument("--torque-limit", type=int, help="输出扭矩限制 0..1000，单位 0.1%%")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--move", action="store_true", help="确认允许真实动作")
    args = parser.parse_args()

    data, configs = load_config(args.config)
    port = args.port or data.get("port", "/dev/ttyUSB0")
    baudrate = args.baudrate or int(data.get("baudrate", 1_000_000))
    timeout = float(data.get("timeout_s", 0.08))
    speed = int(data.get("default_speed", 20) if args.speed is None else args.speed)
    acceleration = int(
        data.get("default_acceleration", 20)
        if args.acceleration is None
        else args.acceleration
    )
    torque_limit = args.torque_limit
    if torque_limit is None and data.get("torque_limit") is not None:
        torque_limit = int(data["torque_limit"])
    if args.scan and args.id is not None:
        parser.error("--scan cannot be combined with --id")
    if (args.center or args.all_center or args.angle is not None or args.raw is not None) and not args.move:
        parser.error("motion requires explicit --move")

    try:
        with FtScsBus(port, baudrate=baudrate, timeout=timeout) as bus:
            if args.scan:
                print(f"scanning {port} @ {baudrate} ...")
                print("found:", bus.scan(range(0, 254)))
                return 0

            controller = ServoController(bus, configs)
            if args.read:
                ids = [args.id] if args.id is not None else [c.servo_id for c in configs.values()]
                for servo_id in ids:
                    state = bus.read_state(servo_id)
                    print(state)
                return 0

            if args.all_center:
                print("centering configured servos")
                controller.torque_all(True)
                controller.center_all(
                    speed=speed,
                    acceleration=acceleration,
                    torque_limit=torque_limit,
                )
                return 0

            if args.raw is not None:
                if args.id is None:
                    parser.error("--raw requires --id")
                if args.id not in controller.by_id:
                    parser.error(f"id {args.id} is not present in servo_config.yaml")
                name = controller.by_id[args.id].name
                controller.torque(name, True)
                raw = controller.set_raw(
                    name,
                    args.raw,
                    speed=speed,
                    acceleration=acceleration,
                    torque_limit=torque_limit,
                )
                print(f"{name}: sent raw target {raw}")
                return 0

            if args.id is None and (args.center or args.angle is not None):
                parser.error("--center/--angle requires --id")
            if args.id is not None and args.id not in controller.by_id:
                parser.error(f"id {args.id} is not present in servo_config.yaml")
            cfg = controller.by_id[args.id] if args.id is not None else None
            if cfg is not None:
                if args.center:
                    target = cfg.center_deg
                elif args.angle is not None:
                    target = args.angle
                else:
                    parser.error("nothing to do")
                name = cfg.name
                controller.torque(name, True)
                controller.move_slowly(
                    name,
                    target,
                    duration_s=args.duration,
                    speed=speed,
                    acceleration=acceleration,
                    torque_limit=torque_limit,
                )
                print(f"{name}: moved to {target:.2f} deg")
                return 0

            parser.error("choose --scan, --read, --center, --all-center, or --angle")
    except (OSError, serial.SerialException, ServoProtocolError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

