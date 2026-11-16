#!/usr/bin/env python3
"""扫描 PCAN/SocketCAN 上的 HighTorque 电机 (ping 0x8000|id)。"""

from __future__ import annotations

import argparse
import os
import sys
import time

from ht_can import MotorBus, format_state

ENETDOWN = 100


def iface_down_reason(iface: str) -> str | None:
    sysfs = f"/sys/class/net/{iface}"
    if not os.path.exists(sysfs):
        return f"{iface} 不存在（PCAN-USB 未识别到）"
    try:
        with open(f"{sysfs}/flags", encoding="utf-8") as f:
            flags = int(f.read().strip(), 16)
    except OSError:
        return f"无法读取 {iface} 状态"
    if not (flags & 0x1):
        return f"{iface} 处于 DOWN（Network is down）"
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="扫描 HighTorque 电机")
    parser.add_argument("-i", "--iface", default="can0")
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=12)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--status", action="store_true", help="扫描后读一次状态")
    args = parser.parse_args()

    ids = list(range(args.start, args.end + 1))
    down = iface_down_reason(args.iface)
    if down:
        print(down, file=sys.stderr)
        print("请先: sudo ./setup_can0.sh", file=sys.stderr)
        return 1
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
            print("未发现电机。检查：电机供电、CANH/CANL 是否接反、120Ω 终端、can0 是否 UP。")
            print("建议再跑: python3 probe_can.py --end 15")
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
        missing = [i for i in ids if i not in found]
        if missing:
            print(f"未应答 ID: {missing}")
        return 0
    except OSError as exc:
        if getattr(exc, "errno", None) == ENETDOWN:
            print(f"{args.iface} 中途掉线: {exc}", file=sys.stderr)
            print("请先: sudo ./setup_can0.sh", file=sys.stderr)
            return 1
        print(f"扫描失败: {exc}", file=sys.stderr)
        return 1
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
