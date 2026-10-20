#!/usr/bin/env python3
"""左腿对角升降测试：髋 pitch（8 号）和膝（7 号）同步变化，机身垂直移动。

两个关节按站立角度的同一比例缩放（关节空间里走一条过原点的对角线），
脚的朝向不变、只有腿长变化。流程：

  站立（比例 1.0）→ 收缩限位（比例 scale，最短）→ 0 位（比例 0，两关节
  归零、腿完全伸直，最长）→ 回到站立 → 卸力

只动左腿 7、8 号。其余关节保持站立角，两个轮电机（1、6 号）在整个
过程中保持失能。单腿伸缩会让机身侧倾，测试时请扶稳或吊住机器人。

用法（在 motor 目录）：
  python3 wheel_run/leg_lift_test.py
  python3 wheel_run/leg_lift_test.py --scale 1.4 --seconds 10
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "pose_contrl"))

import pose

KNEE = 7
HIP_PITCH = 8
WHEELS = (1, 6)
JOINT_LIMIT_DEG = 88.0


def ratio_point(stand_deg: dict[int, float], factor: float) -> dict[int, float]:
    """对角线上的点：两关节角度同时乘 factor。factor 0 即两关节归零、腿伸直。"""
    return {KNEE: stand_deg[KNEE] * factor, HIP_PITCH: stand_deg[HIP_PITCH] * factor}


def main() -> int:
    parser = argparse.ArgumentParser(description="左腿髋膝对角升降测试")
    parser.add_argument("--iface", default="can0")
    parser.add_argument("--pose", default="stand")
    parser.add_argument("--scale", type=float, default=1.6, help="收缩限位的比例，1 是站立点")
    parser.add_argument("--seconds", type=float, default=8.0, help="每一段运动的秒数")
    parser.add_argument("--hold", type=float, default=1.0, help="在最短和最长处停留秒数")
    parser.add_argument("--tqe", type=float, default=None, help="关节力矩上限 Nm，默认用 pose_config 的值")
    args = parser.parse_args()

    cfg = pose.load_config(pose.CONFIG_PATH)
    cfg["interface"] = args.iface
    spec = pose.pose_spec(cfg, args.pose)
    joints_cfg = spec.get("joints") or {}
    stand_deg = {int(k): float(v) for k, v in joints_cfg.items()}
    if KNEE not in stand_deg or HIP_PITCH not in stand_deg:
        print(f"姿态 {args.pose} 里缺少 7 号或 8 号角度。", file=sys.stderr)
        return 2

    knee0 = stand_deg[KNEE]
    hip0 = stand_deg[HIP_PITCH]
    knee_max = knee0 * args.scale
    hip_max = hip0 * args.scale
    if abs(knee_max) > JOINT_LIMIT_DEG or abs(hip_max) > JOINT_LIMIT_DEG:
        print(
            f"收缩限位超出关节范围：膝 {knee_max:.1f}°、髋 {hip_max:.1f}°，"
            f"上限 {JOINT_LIMIT_DEG:.0f}°。请减小 --scale（当前 {args.scale}）。",
            file=sys.stderr,
        )
        return 2
    ratio = knee0 / hip0 if hip0 else 0.0
    print(
        f"左腿对角线：膝/髋 = {ratio:.3f}\n"
        f"  站立点   膝 {knee0:+.2f}°  髋 {hip0:+.2f}°\n"
        f"  收缩限位 膝 {knee_max:+.2f}°  髋 {hip_max:+.2f}°（最短，比例 {args.scale}）\n"
        f"  0 位     膝 +0.00°  髋 +0.00°（最长，腿完全伸直）"
    )

    bus, ids = pose.open_bus(cfg)
    if KNEE not in ids or HIP_PITCH not in ids:
        print("7 号或 8 号没有应答，取消测试。", file=sys.stderr)
        bus.close()
        return 2
    missing = [mid for mid in stand_deg if mid not in ids and mid not in WHEELS]
    if missing:
        print(f"站立关节 {missing} 没有应答。为防摔倒，取消测试。", file=sys.stderr)
        bus.close()
        return 2

    torque_nm = float(args.tqe) if args.tqe is not None else abs(float(cfg.get("torque_nm", 1.2)))
    hz = max(20.0, float(cfg.get("hz", 50.0)))
    period = 1.0 / hz
    hold_ids = [mid for mid in ids if mid not in WHEELS and mid not in (KNEE, HIP_PITCH)]
    leg_ids = (KNEE, HIP_PITCH)
    running = True

    def _stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    def send_all(leg_deg: dict[int, float]) -> None:
        cmd = {mid: stand_deg[mid] for mid in hold_ids}
        pose.command_position(bus, hold_ids, cmd, torque_nm)
        pose.command_position(bus, list(leg_ids), leg_deg, torque_nm)

    def sweep(f_from: float, f_to: float, seconds: float, label: str) -> bool:
        print(f"{label}：比例 {f_from:.2f} → {f_to:.2f}，{seconds:.1f}s")
        t0 = time.monotonic()
        next_print = t0
        while running:
            now = time.monotonic()
            u = (now - t0) / max(0.2, seconds)
            s = pose.smootherstep(u)
            factor = f_from + (f_to - f_from) * s
            leg = ratio_point(stand_deg, factor)
            send_all(leg)
            bus.collect(timeout=min(0.004, period * 0.3))
            if now >= next_print:
                kk = bus.motors[KNEE]
                hh = bus.motors[HIP_PITCH]
                print(
                    f"  比例 {factor:4.2f}  指令 膝 {leg[KNEE]:+6.2f}° 髋 {leg[HIP_PITCH]:+6.2f}°  "
                    f"反馈 膝 {kk.state.position / pose.DEG:+6.2f}° 髋 {hh.state.position / pose.DEG:+6.2f}°  "
                    f"力矩 膝 {kk.state.torque:+5.2f} 髋 {hh.state.torque:+5.2f} Nm",
                    flush=True,
                )
                next_print = now + 0.5
            if u >= 1.0:
                return True
            spent = time.monotonic() - now
            if spent < period:
                time.sleep(period - spent)
        return False

    def stay(seconds: float, factor: float) -> bool:
        t0 = time.monotonic()
        while running and time.monotonic() - t0 < seconds:
            send_all(ratio_point(stand_deg, factor))
            time.sleep(period)
        return running

    try:
        for mid in hold_ids + list(leg_ids):
            bus.enable(mid)
            bus.set_torque_limit(mid, torque_nm)
        for wid in WHEELS:
            if wid in ids:
                bus.disable(wid)
        time.sleep(0.05)

        # 先柔性进入站立姿态（左腿也一起到站立点）。
        start_deg = pose.read_degrees(bus, hold_ids + list(leg_ids))
        all_hold = hold_ids + list(leg_ids)
        duration = max(0.5, float(cfg.get("transition_s", 3.0)))
        print(f"进入站立姿态 {args.pose}，过渡 {duration:.1f}s ...")
        t0 = time.monotonic()
        while running:
            now = time.monotonic()
            u = (now - t0) / duration
            s = pose.smootherstep(u)
            cmd = {
                mid: start_deg[mid] + (stand_deg[mid] - start_deg[mid]) * s for mid in all_hold
            }
            pose.command_position(bus, all_hold, cmd, torque_nm)
            if u >= 1.0:
                break
            spent = time.monotonic() - now
            if spent < period:
                time.sleep(period - spent)
        if not running:
            print("站立过渡被打断，停在当前位置。")
            return 0
        print("已站稳，开始左腿升降。单腿伸缩会让机身侧倾，请扶稳。")

        ok = sweep(1.0, args.scale, args.seconds, "收缩到最短")
        if ok:
            ok = stay(args.hold, args.scale)
        if ok:
            ok = sweep(args.scale, 0.0, args.seconds, "伸长到 0 位（最长）")
        if ok:
            ok = stay(args.hold, 0.0)
        if ok:
            ok = sweep(0.0, 1.0, args.seconds, "回到站立点")
        if not ok:
            print("被打断，停在当前位置保持。")
            return 0

        print("测试完成，回到站立保持 2 秒后卸力。请扶稳机器人。")
        stay(2.0, 1.0)
        for mid in ids:
            bus.disable(mid)
        time.sleep(0.05)
        print("已卸力。")
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
