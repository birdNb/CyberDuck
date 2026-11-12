#!/usr/bin/env python3
"""FEETECH STS3215 id=13 @ 1 Mbps on DEBUG_UART (/dev/ttyMSM0).

Wiring:
  Pin 6  GND
  Pin 8  DEBUG_UART_TX_3V3  --+-- STS DATA (half-duplex)
  Pin 10 DEBUG_UART_RX_3V3 --+

Examples:
  python3 servo13_debug_uart.py --read
  python3 servo13_debug_uart.py --center --move
  python3 servo13_debug_uart.py --angle 210 --duration 2 --move
  python3 servo13_debug_uart.py --raw 2048 --move
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

from bus_servo import (
    ADDR_TORQUE_ENABLE,
    ADDR_TORQUE_LIMIT,
    FtScsBus,
    ServoProtocolError,
)
from servo_controller import ServoConfig, ServoController

PORT = "/dev/ttyMSM0"
BAUD = 1_000_000
SID = 13
GETTY = "serial-getty@ttyMSM0.service"


def systemctl(*args: str) -> None:
    try:
        subprocess.run(["sudo", "-n", "systemctl", *args], check=False, capture_output=True)
    except FileNotFoundError:
        pass


def claim_uart(port: str = PORT) -> None:
    systemctl("stop", GETTY)
    time.sleep(0.3)
    # Console ownership is root:tty; open it for dialout users after getty stops.
    try:
        subprocess.run(["sudo", "-n", "chmod", "666", port], check=False, capture_output=True)
    except FileNotFoundError:
        pass


def restore_uart(restore: bool) -> None:
    if restore:
        systemctl("start", GETTY)


def main() -> int:
    parser = argparse.ArgumentParser(description="STS3215 id=13 on DEBUG_UART")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baudrate", type=int, default=BAUD)
    parser.add_argument("--read", action="store_true")
    parser.add_argument("--ping", action="store_true")
    parser.add_argument("--center", action="store_true")
    parser.add_argument("--angle", type=float)
    parser.add_argument("--raw", type=int)
    parser.add_argument("--speed", type=int, default=150)
    parser.add_argument("--acceleration", type=int, default=50)
    parser.add_argument("--torque-limit", type=int, default=200)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--move", action="store_true")
    parser.add_argument("--keep-console-off", action="store_true")
    args = parser.parse_args()

    if (args.center or args.angle is not None or args.raw is not None) and not args.move:
        parser.error("motion requires explicit --move")

    restore = not args.keep_console_off
    claim_uart(args.port)
    atexit.register(restore_uart, restore)

    if not os.path.exists(args.port):
        print(f"error: missing {args.port}", file=sys.stderr)
        return 2

    cfg = ServoConfig(servo_id=SID, name="servo13", center_deg=180.0, min_deg=0.0, max_deg=360.0)
    try:
        with FtScsBus(args.port, baudrate=args.baudrate, timeout=0.12) as bus:
            print(f"STS3215 id={SID} {args.port} @{args.baudrate}", flush=True)
            if args.ping or args.read or args.move:
                bus.ping(SID)
                print("ping ok", flush=True)

            if args.read or args.ping:
                print(bus.read_state(SID), flush=True)
                if not args.move:
                    return 0

            controller = ServoController(bus, {"servo13": cfg})
            controller.torque("servo13", True)
            bus.write_u16(SID, ADDR_TORQUE_LIMIT, max(0, min(1000, args.torque_limit)))

            if args.raw is not None:
                raw = controller.set_raw(
                    "servo13",
                    args.raw,
                    speed=args.speed,
                    acceleration=args.acceleration,
                    torque_limit=args.torque_limit,
                )
                print(f"sent raw {raw}", flush=True)
                return 0

            if args.center or args.angle is not None:
                target = cfg.center_deg if args.center else float(args.angle)
                controller.move_slowly(
                    "servo13",
                    target,
                    duration_s=args.duration,
                    speed=args.speed,
                    acceleration=args.acceleration,
                    torque_limit=args.torque_limit,
                )
                print(f"moved to {target:.2f} deg", flush=True)
                return 0

            if args.move and not (args.center or args.angle is not None or args.raw is not None):
                parser.error("with --move choose --center / --angle / --raw")
            parser.error("choose --ping, --read, --center, --angle, or --raw")
    except (OSError, serial.SerialException, ServoProtocolError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
