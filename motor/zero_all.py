#!/usr/bin/env python3
"""一键把当前在线电机的当前位置设为零位。

官方顺序：rezero（RAM）→ 等 1s → conf write（写入 flash）。
默认会写入 flash，断电后仍有效；彻底生效建议给电机重新上电一次。

用法：
  python3 zero_all.py
  python3 zero_all.py --no-save          # 只改 RAM，不写 flash
  python3 zero_all.py --start 1 --end 15
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

from ht_can import MotorBus, format_state

DEG = 3.141592653589793 / 180.0
ZERO_OK_DEG = 3.0


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
        return f"{iface} 处于 DOWN"
    return None


def ensure_can(iface: str) -> None:
    if iface_down_reason(iface) is None:
        return
    setup = Path(__file__).with_name("setup_can0.sh")
    print(f"{iface} 未启动，尝试 {setup} ...")
    try:
        subprocess.check_call([str(setup), iface])
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"抬起 {iface} 失败: {exc}", file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description="一键把在线电机当前位置设为零位")
    parser.add_argument("-i", "--iface", default="can0")
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=12)
    parser.add_argument("--timeout", type=float, default=0.06)
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="只改 RAM，不写入 flash（掉电丢失）",
    )
    args = parser.parse_args()

    ids = list(range(args.start, args.end + 1))
    ensure_can(args.iface)
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
        missing = [i for i in ids if i not in found]
        if not found:
            print("未发现电机。检查供电、CANH/CANL、120Ω、can0 是否 UP。")
            return 2
        print(f"在线 {len(found)} 个: {found}")
        if missing:
            print(f"未应答，跳过: {missing}")

        print("失能全部在线电机 ...")
        for mid in found:
            bus.disable(mid)
        time.sleep(0.08)
        bus.poll_status(found, timeout=0.10)
        print("设零前：")
        for mid in found:
            print("  " + format_state(bus.motors[mid]))

        print("发送零位命令（当前位置 → 0）...")
        for mid in found:
            bus.set_zero(mid)
            time.sleep(0.015)
        print("等待 1s ...")
        time.sleep(1.0)

        if args.no_save:
            print("未写入 flash（--no-save），本次通电有效。")
        else:
            print("写入 flash ...")
            for mid in found:
                bus.save_config(mid)
                time.sleep(0.015)
            time.sleep(0.25)
            print("已请求保存。彻底生效建议给电机重新上电一次。")

        bus.poll_status(found, timeout=0.12)
        print("设零后：")
        bad: list[int] = []
        for mid in found:
            m = bus.motors[mid]
            print("  " + format_state(m))
            if abs(m.state.position) > ZERO_OK_DEG * DEG:
                bad.append(mid)

        if bad:
            print(f"这些 ID 读回仍偏离 0 超过 {ZERO_OK_DEG:.0f}°: {bad}", file=sys.stderr)
            return 3
        print(f"完成：{len(found)} 个电机已以当前姿态调零。")
        return 0
    finally:
        try:
            for mid in bus.motors:
                bus.disable(mid)
        except OSError:
            pass
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
