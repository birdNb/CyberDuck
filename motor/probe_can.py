#!/usr/bin/env python3
"""只探测通讯，不发运动指令。判断 ACK / 错误帧 / 电机回复。"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

from ht_can import MotorBus, describe_error_frame, motor_id_from_can_id


def can_link_dump(iface: str) -> str:
    try:
        out = subprocess.check_output(
            ["ip", "-s", "-d", "link", "show", iface],
            text=True,
            stderr=subprocess.STDOUT,
        )
        return out.strip()
    except Exception as exc:
        return f"(无法读取 {iface}: {exc})"


def fmt_frame(frame) -> str:
    kind = "ERR" if frame.error else ("EXT" if frame.extended else "STD")
    extra = f" {describe_error_frame(frame)}" if frame.error else ""
    return f"{kind} id=0x{frame.arb_id:08X} dlc={len(frame.data)} data={frame.data.hex(' ')}{extra}"


def main() -> int:
    parser = argparse.ArgumentParser(description="HighTorque CAN 通讯探测")
    parser.add_argument("-i", "--iface", default="can0")
    parser.add_argument("--listen", type=float, default=2.0, help="先静听秒数")
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=14)
    parser.add_argument("--timeout", type=float, default=0.08)
    args = parser.parse_args()

    print("==== 接口状态 ====")
    print(can_link_dump(args.iface))
    print()

    try:
        bus = MotorBus(args.iface)
    except OSError as exc:
        print(f"打开 {args.iface} 失败: {exc}", file=sys.stderr)
        return 1

    found = []
    errors = []
    try:
        print(f"==== 静听 {args.listen:.1f}s（请转一下电机轴，看有无自发反馈）====")
        t0 = time.monotonic()
        n_listen = 0
        while time.monotonic() - t0 < args.listen:
            frame = bus.can.recv(timeout=0.1)
            if frame is None:
                continue
            n_listen += 1
            print("  RX", fmt_frame(frame))
            if frame.error:
                errors.append(frame)
        if n_listen == 0:
            print("  （静听无任何帧）")

        print()
        print(f"==== Ping ID {args.start}..{args.end}（扩展帧 0x8000|id）====")
        bus.can.drain(0.02)
        for mid in range(args.start, args.end + 1):
            try:
                bus.ping(mid)
            except OSError as exc:
                print(f"  ID {mid:2d} 发送失败: {exc}")
                print(can_link_dump(args.iface))
                return 3
            frames = bus.collect(args.timeout, include_errors=True)
            data_frames = [
                f for f in frames
                if not f.error and motor_id_from_can_id(f.arb_id) == mid
            ]
            err_frames = [f for f in frames if f.error]
            errors.extend(err_frames)
            if err_frames and not data_frames:
                print(f"  ID {mid:2d} 无数据回复，错误: {describe_error_frame(err_frames[0])}")
                continue
            if not data_frames:
                print(f"  ID {mid:2d} 无响应")
                continue
            found.append(mid)
            print(f"  ID {mid:2d} 有回复:")
            for f in data_frames:
                print("    ", fmt_frame(f))

        print()
        print("==== 探测后接口状态 ====")
        print(can_link_dump(args.iface))
    finally:
        bus.close()

    print()
    if found:
        print(f"通讯成功，应答 ID: {found}")
        return 0

    ack_miss = any(f.can_id & 0x20 for f in errors)
    print("没有电机应答。")
    if ack_miss:
        print("出现 ACK 缺失：更像没接上或 CANH/CANL 反了。请对调 H/L 后再跑本脚本。")
    else:
        print(
            "控制器错误计数若仍接近 0，物理层多半通（有节点在 ACK），"
            "H/L 不一定反了；更像电机 ID/协议不对，或电机未进入 CAN 工作状态。"
        )
        print("建议：确认电机已上电、120Ω 终端、对调 H/L 试一次、再跑 python3 probe_can.py")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
