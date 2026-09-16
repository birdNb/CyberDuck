#!/usr/bin/env python3
"""扫描 PCAN/SocketCAN 上的 HighTorque 电机 (ping 0x8000|id)。"""

from __future__ import annotations

import argparse
import sys
import time

from ht_can import MotorBus, format_state


def main() -> int:
    parser = argparse.ArgumentParser(description="扫描 HighTorque 电机")
    parser.add_argument("-i", "--iface", default="can0")
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=14)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--status", action="store_true", help="扫描后读一次状态")
    args = parser.parse_args()

    ids = list(range(args.start, args.end + 1))
    print(f"扫描 {args.iface} 电机 ID {ids[0]}..{ids[-1]} ...")
    try:
        bus = MotorBus(args.iface)
    except OSError as exc:
        print(f"打开 {args.iface} 失败: {exc}", file=sys.stderr)
        print("请先: sudo ./setup_can0.sh", file=sys.stderr)
        return 1

    try:
        found = bus.scan(ids, timeout_each=args.timeout)
        if not found:
            print("未发现电机。检查：电机供电、CANH/CANL、120Ω 终端、can0 是否 UP。")
            return 2
        print(f"发现 {len(found)} 个电机: {found}")
        if args.status:
            time.sleep(0.05)
            bus.poll_status(found, timeout=0.08)
            for mid in found:
                print(format_state(bus.motors[mid]))
        else:
            for mid in found:
                m = bus.motors[mid]
                payload = m.state.payload.hex(" ")
                print(f"  ID {mid:2d}  CAN=0x{m.state.can_id:08X}  data={payload}")
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
