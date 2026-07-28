#!/usr/bin/env python3
"""测试 can0 读取电机状态的频率上限（默认 15 个电机）。

指标：
- 循环频率 Hz：每轮发送 15 次 read_status + 收包 的完整频率
- 回复帧频率 Hz：收到的有效状态帧总数 / 时间
- 单电机频率 Hz：每个 ID 实际收到状态帧的频率
"""

from __future__ import annotations

import argparse
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path

from ht_can import MotorBus, motor_id_from_can_id


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


def percentile(sorted_vals: list[float], p: float) -> float:
    if not sorted_vals:
        return 0.0
    idx = int((len(sorted_vals) - 1) * p)
    return sorted_vals[max(0, min(len(sorted_vals) - 1, idx))]


def main() -> int:
    parser = argparse.ArgumentParser(description="基准测试读取 15 个电机状态的频率")
    parser.add_argument("-i", "--iface", default="can0")
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=15)
    parser.add_argument("--duration", type=float, default=10.0, help="测试时长秒")
    parser.add_argument("--scan-timeout", type=float, default=0.06, help="扫描单 ID 超时秒")
    parser.add_argument(
        "--collect-timeout",
        type=float,
        default=0.006,
        help="每轮收包窗口秒（越小循环越快）",
    )
    parser.add_argument("--quiet", action="store_true", help="不打印每秒进度")
    args = parser.parse_args()

    if args.end < args.start:
        print("参数错误：--end 不能小于 --start", file=sys.stderr)
        return 2
    if args.duration <= 0:
        print("参数错误：--duration 需 > 0", file=sys.stderr)
        return 2

    ids = list(range(args.start, args.end + 1))
    ensure_can(args.iface)
    down = iface_down_reason(args.iface)
    if down:
        print(down, file=sys.stderr)
        print("请先: sudo ./setup_can0.sh", file=sys.stderr)
        return 1

    try:
        bus = MotorBus(args.iface)
    except OSError as exc:
        print(f"打开 {args.iface} 失败: {exc}", file=sys.stderr)
        print("请先: sudo ./setup_can0.sh", file=sys.stderr)
        return 1

    counts = {mid: 0 for mid in ids}
    total_frames = 0
    loops = 0
    loop_cost_s: list[float] = []
    one_sec_tick = 0.0

    try:
        found = bus.scan(ids, timeout_each=args.scan_timeout)
        if not found:
            print("未发现电机，无法测试。先检查供电/接线。", file=sys.stderr)
            return 3
        missing = [mid for mid in ids if mid not in found]
        test_ids = found
        print(f"在线电机: {test_ids}")
        if missing:
            print(f"未应答，跳过: {missing}")

        # 清掉历史缓冲，避免把旧帧算进吞吐。
        bus.can.drain(0.03)

        t0 = time.monotonic()
        one_sec_tick = t0 + 1.0
        deadline = t0 + args.duration
        while True:
            now = time.monotonic()
            if now >= deadline:
                break

            l0 = now
            for mid in test_ids:
                bus.read_status(mid)
            frames = bus.collect(timeout=args.collect_timeout, include_errors=False)
            loops += 1
            cost = time.monotonic() - l0
            loop_cost_s.append(cost)

            for f in frames:
                mid = motor_id_from_can_id(f.arb_id)
                if mid in counts:
                    counts[mid] += 1
                    total_frames += 1

            if (not args.quiet) and now >= one_sec_tick:
                elapsed = now - t0
                loop_hz = loops / elapsed if elapsed > 0 else 0.0
                rx_hz = total_frames / elapsed if elapsed > 0 else 0.0
                print(
                    f"[{elapsed:5.1f}s] loop={loop_hz:7.1f}Hz  "
                    f"rx={rx_hz:7.1f}fps  total_frames={total_frames}",
                    flush=True,
                )
                one_sec_tick += 1.0

        t1 = time.monotonic()
    finally:
        bus.close()

    elapsed = max(1e-9, t1 - t0)
    loop_hz = loops / elapsed
    rx_hz = total_frames / elapsed
    sorted_cost_ms = sorted(x * 1000.0 for x in loop_cost_s)
    avg_ms = statistics.mean(sorted_cost_ms) if sorted_cost_ms else 0.0
    p50_ms = percentile(sorted_cost_ms, 0.50)
    p95_ms = percentile(sorted_cost_ms, 0.95)
    p99_ms = percentile(sorted_cost_ms, 0.99)

    print("\n=== 结果 ===")
    print(f"测试电机数: {len(test_ids)}")
    print(f"测试时长:   {elapsed:.3f}s")
    print(f"循环频率:   {loop_hz:.2f} Hz")
    print(f"回复帧率:   {rx_hz:.2f} frame/s")
    print(f"循环耗时:   avg={avg_ms:.3f}ms p50={p50_ms:.3f}ms p95={p95_ms:.3f}ms p99={p99_ms:.3f}ms")
    print("\n每个电机状态读取频率：")
    for mid in test_ids:
        hz = counts[mid] / elapsed
        print(f"  ID {mid:2d}: {hz:7.2f} Hz  ({counts[mid]} frames)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
