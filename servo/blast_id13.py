#!/usr/bin/env python3
"""Open-loop write to STS3215 id=13 (no response required). Watch the horn."""
from __future__ import annotations

import subprocess
import sys
import time

from bus_servo import (
    ADDR_ACCELERATION,
    ADDR_GOAL_POSITION,
    ADDR_GOAL_SPEED,
    ADDR_TORQUE_ENABLE,
    ADDR_TORQUE_LIMIT,
    FtScsBus,
    signed_value,
)

PORT = "/dev/ttyMSM0"
SID = 13


def main() -> int:
    subprocess.run(["sudo", "-n", "systemctl", "stop", "serial-getty@ttyMSM0.service"], check=False)
    time.sleep(0.2)
    subprocess.run(["sudo", "-n", "chmod", "666", PORT], check=False)

    targets = [2048, 2400, 1700, 2048]
    with FtScsBus(PORT, baudrate=1_000_000, timeout=0.05) as bus:
        print(f"open-loop blast id={SID} on {PORT}", flush=True)
        bus.write_u8(SID, ADDR_TORQUE_ENABLE, 1, response=False)
        bus.write_u16(SID, ADDR_TORQUE_LIMIT, 300, response=False)
        bus.write_u8(SID, ADDR_ACCELERATION, 50, response=False)
        bus.write(SID, ADDR_GOAL_SPEED, signed_value(200), response=False)
        for raw in targets:
            bus.write_u16(SID, ADDR_GOAL_POSITION, raw, response=False)
            print(f"sent goal {raw}", flush=True)
            time.sleep(1.2)
    print("done — did the servo move?", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
