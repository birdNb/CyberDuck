#!/usr/bin/env python3
"""Quick probe for STS3215 after rewiring."""
from __future__ import annotations

import os
import subprocess
import sys
import time

from bus_servo import FtScsBus, make_packet, INST_PING

PORTS = ["/dev/ttyMSM0", "/dev/ttyHS0", "/dev/ttyS3", "/dev/ttyS2", "/dev/ttyS1", "/dev/ttyS0"]
BAUDS = [1_000_000, 115_200, 500_000, 250_000, 2_000_000]
IDS = [13, 1, 2, 3]


def free_msm0():
    subprocess.run(["sudo", "-n", "systemctl", "stop", "serial-getty@ttyMSM0.service"], check=False)
    time.sleep(0.3)
    subprocess.run(["sudo", "-n", "chmod", "666", "/dev/ttyMSM0"], check=False)


def main() -> int:
    free_msm0()
    found = []
    for port in PORTS:
        if not os.path.exists(port):
            print(f"skip missing {port}", flush=True)
            continue
        for baud in BAUDS:
            try:
                bus = FtScsBus(port, baudrate=baud, timeout=0.12, write_timeout=0.2)
            except Exception as exc:
                print(f"open fail {port}@{baud}: {exc}", flush=True)
                continue
            try:
                # Flush and send a raw ping to id 13 first for visibility.
                pkt = make_packet(13, INST_PING)
                bus.ser.reset_input_buffer()
                n = bus.ser.write(pkt)
                bus.ser.flush()
                time.sleep(0.02)
                echo = bus.ser.read(bus.ser.in_waiting or 1)
                print(f"{port}@{baud}: wrote {n} ping13, rx={echo.hex(' ') if echo else '-'}", flush=True)
                for sid in IDS:
                    try:
                        bus.ping(sid)
                        st = bus.read_state(sid)
                        msg = f"FOUND {port}@{baud} id={sid} {st}"
                        print(msg, flush=True)
                        found.append(msg)
                    except Exception as exc:
                        print(f"  id={sid} {type(exc).__name__}", flush=True)
            finally:
                bus.close()
    print("DONE found=" + str(len(found)), flush=True)
    for line in found:
        print(line, flush=True)
    return 0 if found else 1


if __name__ == "__main__":
    raise SystemExit(main())
