#!/usr/bin/env python3
"""Drive FEETECH STS3215 over the board DEBUG_UART (/dev/ttyMSM0).

Hardware (expansion header):
  Pin 6  GND
  Pin 8  DEBUG_UART_TX_3V3  -> short with RX onto STS DATA
  Pin 10 DEBUG_UART_RX_3V3

ttyMSM0 is also the Linux console. This tool stops serial-getty while it
runs and restores it on exit unless --keep-console-off is passed.

Real motion requires --move.
"""

from __future__ import annotations

import argparse
import atexit
import os
import subprocess
import sys
import time
from pathlib import Path

import serial
import yaml

from bus_servo import FtScsBus, ServoProtocolError
from servo_controller import ServoConfig, ServoController

HERE = Path(__file__).resolve().parent
GETTY = "serial-getty@ttyMSM0.service"
DEFAULT_PORT = "/dev/ttyMSM0"


def load_config(path: Path) -> tuple[dict, dict[str, ServoConfig]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    configs: dict[str, ServoConfig] = {}
    for name, item in (data.get("servos") or {}).items():
        configs[name] = ServoConfig(
            servo_id=int(item["id"]),
            name=name,
            center_deg=float(item.get("center_deg", 180.0)),
            min_deg=float(item.get("min_deg", 0.0)),
            max_deg=float(item.get("max_deg", 360.0)),
            direction=1 if int(item.get("direction", 1)) >= 0 else -1,
        )
    return data, configs


def _systemctl(*args: str) -> None:
    cmd = ["sudo", "-n", "systemctl", *args]
    try:
        subprocess.run(cmd, check=False, capture_output=True, text=True)
    except FileNotFoundError:
        pass


def claim_debug_uart(port: str) -> None:
    if port != DEFAULT_PORT:
        return
    _systemctl("stop", GETTY)
    # Give the kernel a moment to release the line.
    time.sleep(0.3)


def restore_debug_uart(port: str, restore: bool) -> None:
    if port != DEFAULT_PORT or not restore:
        return
    _systemctl("start", GETTY)


def main() -> int:
    parser = argparse.ArgumentParser(description="STS3215 over DEBUG_UART (ttyMSM0)")
    parser.add_argument("--config", type=Path, default=HERE / "servo_config.yaml")
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int)
    parser.add_argument("--scan", action="store_true")
    parser.add_argument("--id", type=int)
    parser.add_argument("--read", action="store_true")
    parser.add_argument("--center", action="store_true")
    parser.add_argument("--all-center", action="store_true")
    parser.add_argument("--angle", type=float)
    parser.add_argument("--raw", type=int)
    parser.add_argument("--speed", type=int, default=None)
    parser.add_argument("--acceleration", type=int, default=None)
    parser.add_argument("--torque-limit", type=int, default=None)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--move", action="store_true", help="allow real motion")
    parser.add_argument(
        "--keep-console-off",
        action="store_true",
        help="do not restart serial-getty@ttyMSM0 on exit",
    )
    args = parser.parse_args()

    data, configs = load_config(args.config)
    port = args.port or data.get("port", DEFAULT_PORT)
    baudrate = args.baudrate or int(data.get("baudrate", 1_000_000))
    timeout = float(data.get("timeout_s", 0.08))
    speed = args.speed if args.speed is not None else int(data.get("default_speed", 150))
    acceleration = (
        args.acceleration
        if args.acceleration is not None
        else int(data.get("default_acceleration", 50))
    )
    torque_limit = (
        args.torque_limit
        if args.torque_limit is not None
        else int(data.get("torque_limit", 200))
    )

    if args.scan and args.id is not None:
        parser.error("--scan cannot be combined with --id")
    if (args.center or args.all_center or args.angle is not None or args.raw is not None) and not args.move:
        parser.error("motion requires explicit --move")

    restore = not args.keep_console_off
    claim_debug_uart(port)
    atexit.register(restore_debug_uart, port, restore)

    if not os.path.exists(port):
        print(f"error: missing serial port {port}", file=sys.stderr)
        return 2

    try:
        with FtScsBus(port, baudrate=baudrate, timeout=timeout) as bus:
            print(f"open {port} @ {baudrate} model={data.get('model', 'STS3215')}", flush=True)
            if args.scan:
                print("scanning ids 0..253 ...", flush=True)
                print("found:", bus.scan(range(0, 254)), flush=True)
                return 0

            controller = ServoController(bus, configs)

            if args.read:
                ids = [args.id] if args.id is not None else [c.servo_id for c in configs.values()]
                for servo_id in ids:
                    print(bus.read_state(servo_id), flush=True)
                return 0

            if args.all_center:
                controller.torque_all(True)
                controller.center_all(
                    speed=speed, acceleration=acceleration, torque_limit=torque_limit
                )
                print("centered all configured servos", flush=True)
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
                print(f"{name}: sent raw target {raw}", flush=True)
                return 0

            if args.id is None and (args.center or args.angle is not None):
                parser.error("--center/--angle requires --id")
            if args.id is not None and args.id not in controller.by_id:
                parser.error(f"id {args.id} is not present in servo_config.yaml")

            if args.id is not None:
                cfg = controller.by_id[args.id]
                if args.center:
                    target = cfg.center_deg
                elif args.angle is not None:
                    target = args.angle
                else:
                    parser.error("nothing to do")
                controller.torque(cfg.name, True)
                controller.move_slowly(
                    cfg.name,
                    target,
                    duration_s=args.duration,
                    speed=speed,
                    acceleration=acceleration,
                    torque_limit=torque_limit,
                )
                print(f"{cfg.name}: moved to {target:.2f} deg", flush=True)
                return 0

            parser.error("choose --scan, --read, --center, --all-center, --raw, or --angle")
    except (OSError, serial.SerialException, ServoProtocolError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
