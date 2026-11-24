#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单关节正负步进 + 左右对称符号检查。

用途：
  1) 确认仿真角 +Δ 时，反馈仿真角也往同方向走（SIGN 是否反了）
  2) 左右腿同名关节是否都满足同一套世界方向约定
  3) 踝关节是否真有响应（翻滚日志里实机踝几乎无响应）

安全：
  - 默认只打印清单，不加 --move 不会发运动
  - --move 时力矩默认 0.8 N·m，步进默认 ±0.05 rad
  - 一次只动一个关节，其余锁在采集站立角
  - 机器人必须吊起/趴稳，周围清空

用法（在 locomotion/ 下，实机 Linux）:
  python3 scripts/check_joint_signs.py
  python3 scripts/check_joint_signs.py --move
  python3 scripts/check_joint_signs.py --move --joints l_hip_yaw,l_hip_pitch,r_hip_yaw,r_hip_pitch,l_ankle,r_ankle
  python3 scripts/check_joint_signs.py --score logs/sign_check_xxx.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "motor"))
sys.path.insert(0, str(REPO / "motor" / "pose_contrl"))

# 与 watch_policy.py 保持一致：仿真角 <-> 电机角只在边界乘一次 SIGN。
# 下标顺序 = JOINTS 顺序。
SIGN = (-1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, -1)
JOINTS = (
    # name, can_id, home_rad, scale, lo, hi, is_leg
    ("l_hip_yaw", 10, 0.0, 0.25, -0.436, 0.436, True),
    ("l_hip_roll", 9, 0.0, 0.10, -0.384, 0.384, True),
    ("l_hip_pitch", 8, 0.68, 0.80, -1.571, 1.571, True),
    ("l_calf", 7, -0.88, 1.00, -1.571, 1.571, True),
    ("l_ankle", 6, 0.21, 0.70, -1.571, 1.571, True),
    ("neck_pitch", 12, 0.52, 0.40, -1.571, 1.571, False),
    ("head_pitch", 11, -0.50, 0.40, -1.571, 1.571, False),
    ("head_yaw", 14, 0.0, 0.50, -2.967, 2.967, False),
    ("head_roll", 13, 0.0, 0.20, -0.436, 0.436, False),
    ("r_hip_yaw", 5, 0.0, 0.25, -0.436, 0.436, True),
    ("r_hip_roll", 4, 0.0, 0.10, -0.384, 0.384, True),
    ("r_hip_pitch", 3, 0.68, 0.80, -1.571, 1.571, True),
    ("r_calf", 2, -0.88, 1.00, -1.571, 1.571, True),
    ("r_ankle", 1, 0.21, 0.70, -1.571, 1.571, True),
)

# 翻滚日志优先怀疑的关节；默认先查这些。
DEFAULT_JOINTS = (
    "l_hip_yaw",
    "r_hip_yaw",
    "l_hip_pitch",
    "r_hip_pitch",
    "l_hip_roll",
    "r_hip_roll",
    "l_ankle",
    "r_ankle",
    "head_pitch",
)

# 仿真角 +Δ 时，人眼应看到的世界方向（左右应是同一列现象）。
EXPECT = {
    "hip_yaw": "从上往下看，该腿绕竖直轴逆时针（左右同世界方向，不是一顺一反）",
    "hip_roll": "绕朝前轴按右手定则；左腿正方向很快顶外侧小限位，右腿正方向是向内大行程",
    "hip_pitch": "绕指向左侧的轴；两条大腿朝同一世界方向转（站立 +0.68 基础上继续）",
    "calf": "绕指向左侧的轴；两条小腿朝同一世界方向转（站立 -0.88，更负才更深蹲）",
    "ankle": "绕指向左侧的轴；两只脚朝同一世界方向转",
    "neck_pitch": "头颈俯仰：正方向按 URDF/仿真约定检查",
    "head_pitch": "头俯仰：正方向按 URDF/仿真约定检查",
    "head_yaw": "头左右转：正方向按 URDF/仿真约定检查",
    "head_roll": "头侧倾：正方向按 URDF/仿真约定检查",
}

PAIR_ROOTS = ("hip_yaw", "hip_roll", "hip_pitch", "calf", "ankle")


@dataclass
class StepResult:
    joint: str
    side: str
    delta_cmd: float
    dq_sim: float
    dq_motor: float
    track_ratio: float
    same_sign: bool
    jump_flag: bool
    ok: bool
    note: str


def joint_index(name: str) -> int:
    for i, row in enumerate(JOINTS):
        if row[0] == name:
            return i
    raise KeyError(name)


def family_of(name: str) -> str:
    for key in (
        "hip_yaw",
        "hip_roll",
        "hip_pitch",
        "calf",
        "ankle",
        "neck_pitch",
        "head_pitch",
        "head_yaw",
        "head_roll",
    ):
        if name.endswith(key) or name == key:
            return key
    return name


def side_of(name: str) -> str:
    if name.startswith("l_"):
        return "L"
    if name.startswith("r_"):
        return "R"
    return "C"


def print_checklist(names: list[str]) -> None:
    print("=" * 72)
    print("单关节符号检查清单（不运动，仅说明）")
    print("=" * 72)
    print("准备：吊起或趴稳；确认 SIGN 与 watch_policy.py 一致；站立角已 capture。")
    print("判定：仿真目标 +Δ 后，反馈仿真角也应 +；|Δq|/|Δcmd| 建议 ≥ 0.35。")
    print("左右：同名关节都应满足同一世界方向描述；若右腿反了，只改该右腿 SIGN。")
    print("-" * 72)
    for name in names:
        i = joint_index(name)
        _n, can, home, _s, lo, hi, _leg = JOINTS[i]
        fam = family_of(name)
        print(f"[{name}] motor={can} SIGN={SIGN[i]:+d} HOME={home:+.3f} limit=[{lo:.3f},{hi:.3f}]")
        print(f"  期望(+0.05)：{EXPECT.get(fam, '按仿真右手定则检查')}")
        print(f"  步骤：站立保持 → +Δ 保持 1s → 回中 → -Δ 保持 1s → 回中")
    print("-" * 72)
    print("优先顺序（来自翻滚实机日志）：左 hip_yaw/pitch → 左右 ankle → 右对称核对 → head_pitch")
    print("跑实机：python3 scripts/check_joint_signs.py --move")
    print("=" * 72)


def load_stand_deg(pose_cfg: Path) -> dict[int, float]:
    import yaml

    data = yaml.safe_load(pose_cfg.read_text(encoding="utf-8")) or {}
    joints = ((data.get("poses") or {}).get("stand") or {}).get("joints") or {}
    out = {int(k): float(v) for k, v in joints.items()}
    # 头 yaw/roll 舵机常不在 CAN pose 里；缺省当 0°，只影响未测舵机关节。
    missing = [can for _n, can, *_ in JOINTS if can not in out]
    for can in missing:
        if can in (13, 14, 15):
            out[can] = 0.0
        else:
            raise SystemExit(f"pose_config stand 缺少电机: {[c for c in missing if c not in (13, 14, 15)]}")
    return out


def q_sim_from_motor(i: int, motor_rad: float, stand_deg: dict[int, float]) -> float:
    _name, can, home, *_ = JOINTS[i]
    return home + SIGN[i] * (motor_rad - math.radians(stand_deg[can]))


def motor_cmd_from_sim(i: int, q_sim: float, stand_deg: dict[int, float]) -> float:
    _name, can, home, _scale, lo, hi, _leg = JOINTS[i]
    limited = min(hi, max(lo, q_sim))
    return math.radians(stand_deg[can]) + SIGN[i] * (limited - home)


def hold_pose(
    bus,
    motors_by_id: dict,
    cmd_motor_rad: list[float],
    tau: list[float],
    vel_lim: float,
    hz: float,
    seconds: float,
    running_flag: list[bool],
) -> None:
    period = 1.0 / max(hz, 1.0)
    t_end = time.monotonic() + seconds
    while running_flag[0] and time.monotonic() < t_end:
        t0 = time.monotonic()
        for i, (_name, can, *_rest) in enumerate(JOINTS):
            bus.pos_vel_maxtqe(motors_by_id[can], cmd_motor_rad[i], vel_lim, tau[i])
        bus.poll_status([can for _n, can, *_ in JOINTS], timeout=0.02)
        dt = time.monotonic() - t0
        if dt < period:
            time.sleep(period - dt)


def measure_mean(bus, stand_deg: dict[int, float], samples: int = 5) -> tuple[list[float], list[float]]:
    cans = [can for _n, can, *_ in JOINTS]
    acc_m = [0.0] * len(JOINTS)
    for _ in range(samples):
        bus.poll_status(cans, timeout=0.03)
        for i, (_n, can, *_r) in enumerate(JOINTS):
            acc_m[i] += bus.motors[can].state.position
        time.sleep(0.02)
    motor = [v / samples for v in acc_m]
    sim = [q_sim_from_motor(i, motor[i], stand_deg) for i in range(len(JOINTS))]
    return sim, motor


def run_move(args: argparse.Namespace) -> int:
    from ht_can import Motor, MotorBus

    import pose

    pose_cfg = Path(args.pose_config)
    stand_deg = load_stand_deg(pose_cfg)
    names = [n.strip() for n in args.joints.split(",") if n.strip()]
    for n in names:
        joint_index(n)

    print_checklist(names)
    print()
    print("将在 3 秒后开始运动。Ctrl+C 可中止。")
    time.sleep(3.0)

    iface = args.iface
    pose.ensure_can(iface)
    bus = MotorBus(iface)
    motors_by_id: dict[int, Motor] = {}
    for name, can, *_rest in JOINTS:
        m = Motor(can, args.motor_type, name)
        bus.add_motor(m)
        motors_by_id[can] = m
        bus.enable(can)
    time.sleep(0.1)
    bus.poll_status([can for _n, can, *_ in JOINTS], timeout=0.08)

    running = [True]

    def _stop(*_):
        running[0] = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    # 力矩：腿/颈/头 pitch 用 args.tqe；头 yaw/roll 额定更小
    tau = []
    for name, *_r in JOINTS:
        if name in ("head_yaw", "head_roll"):
            tau.append(min(args.tqe, 0.25))
        else:
            tau.append(args.tqe)

    # 先锁站立
    home_sim = [row[2] for row in JOINTS]
    cmd_motor = [motor_cmd_from_sim(i, home_sim[i], stand_deg) for i in range(len(JOINTS))]
    print("锁定站立...")
    hold_pose(bus, motors_by_id, cmd_motor, tau, args.vel, args.hz, 1.5, running)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"sign_check_{stamp}.csv"
    results: list[StepResult] = []

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "joint",
                "side",
                "sign",
                "delta_cmd",
                "q0_sim",
                "q1_sim",
                "dq_sim",
                "q0_motor",
                "q1_motor",
                "dq_motor",
                "track_ratio",
                "same_sign",
                "jump_flag",
                "ok",
                "note",
            ]
        )

        for name in names:
            if not running[0]:
                break
            i = joint_index(name)
            _n, can, home, _scale, lo, hi, _leg = JOINTS[i]
            print(f"\n>>> {name}  motor={can} SIGN={SIGN[i]:+d}")
            for delta in (args.delta, -args.delta):
                if not running[0]:
                    break
                # 回站立
                cmd_motor = [motor_cmd_from_sim(j, home_sim[j], stand_deg) for j in range(len(JOINTS))]
                hold_pose(bus, motors_by_id, cmd_motor, tau, args.vel, args.hz, 0.6, running)
                q0_sim, q0_motor = measure_mean(bus, stand_deg)

                target = min(hi, max(lo, home + delta))
                actual_delta = target - home
                if abs(actual_delta) < 0.5 * abs(delta):
                    note = "步进被限位吃掉，结果不可靠"
                    print(f"  delta={delta:+.3f}: {note}")
                    results.append(
                        StepResult(name, side_of(name), delta, 0.0, 0.0, 0.0, False, False, False, note)
                    )
                    writer.writerow(
                        [name, side_of(name), SIGN[i], delta, "", "", "", "", "", "", "", "", "", False, note]
                    )
                    continue

                cmd_sim = list(home_sim)
                cmd_sim[i] = target
                cmd_motor = [motor_cmd_from_sim(j, cmd_sim[j], stand_deg) for j in range(len(JOINTS))]
                print(f"  指令仿真 {home:+.3f} -> {target:+.3f} (Δ={actual_delta:+.3f})")
                hold_pose(bus, motors_by_id, cmd_motor, tau, args.vel, args.hz, args.hold, running)
                q1_sim, q1_motor = measure_mean(bus, stand_deg)

                dq_sim = q1_sim[i] - q0_sim[i]
                dq_motor = q1_motor[i] - q0_motor[i]
                # 其它关节是否异常大跳
                jump = False
                for j in range(len(JOINTS)):
                    if j == i:
                        continue
                    if abs(q1_sim[j] - q0_sim[j]) > 0.25:
                        jump = True
                track = abs(dq_sim) / max(abs(actual_delta), 1e-6)
                same = math.copysign(1.0, dq_sim) == math.copysign(1.0, actual_delta) if abs(dq_sim) > 1e-4 else False
                ok = same and track >= args.min_track and not jump
                note = "PASS" if ok else (
                    "SIGN可疑/方向反了" if abs(dq_sim) > 1e-4 and not same else
                    "跟不上/几乎不动" if track < args.min_track else
                    "其它关节异常跳变" if jump else "FAIL"
                )
                print(
                    f"  反馈 Δq_sim={dq_sim:+.4f}  Δq_motor={dq_motor:+.4f}  "
                    f"track={track:.2f}  => {note}"
                )
                fam = family_of(name)
                print(f"  目视核对：{EXPECT.get(fam, '')}")

                row = StepResult(
                    name, side_of(name), actual_delta, dq_sim, dq_motor, track, same, jump, ok, note
                )
                results.append(row)
                writer.writerow(
                    [
                        name,
                        side_of(name),
                        SIGN[i],
                        f"{actual_delta:.6f}",
                        f"{q0_sim[i]:.6f}",
                        f"{q1_sim[i]:.6f}",
                        f"{dq_sim:.6f}",
                        f"{q0_motor[i]:.6f}",
                        f"{q1_motor[i]:.6f}",
                        f"{dq_motor:.6f}",
                        f"{track:.4f}",
                        int(same),
                        int(jump),
                        int(ok),
                        note,
                    ]
                )

            # 回站立
            cmd_motor = [motor_cmd_from_sim(j, home_sim[j], stand_deg) for j in range(len(JOINTS))]
            hold_pose(bus, motors_by_id, cmd_motor, tau, args.vel, args.hz, 0.8, running)

    print("\n回到站立并保持出力。日志:", csv_path)
    summarize(results)
    score_pairs(results)
    return 0 if results and all(r.ok for r in results) else 2


def summarize(results: list[StepResult]) -> None:
    print("\n" + "=" * 72)
    print("逐关节结果")
    print("=" * 72)
    if not results:
        print("无结果")
        return
    for r in results:
        flag = "OK" if r.ok else "FAIL"
        print(
            f"[{flag}] {r.joint:12s} Δcmd={r.delta_cmd:+.3f} Δq={r.dq_sim:+.4f} "
            f"track={r.track_ratio:.2f}  {r.note}"
        )


def score_pairs(results: list[StepResult]) -> None:
    print("\n左右对称核对（同名关节正步进是否都同号响应）")
    by = {}
    for r in results:
        if r.delta_cmd > 0:
            by.setdefault(r.joint, r)
    for root in PAIR_ROOTS:
        lname, rname = f"l_{root}", f"r_{root}"
        if lname not in by or rname not in by:
            continue
        l, r = by[lname], by[rname]
        both_ok = l.same_sign and r.same_sign
        # 同向世界约定：两侧 +sim 都应得到 +dq_sim
        print(
            f"  {root}: L same={l.same_sign} track={l.track_ratio:.2f} | "
            f"R same={r.same_sign} track={r.track_ratio:.2f} | "
            f"{'对称通过' if both_ok else '有一侧方向/跟踪异常 → 先改 FAIL 侧 SIGN'}"
        )


def score_csv(path: Path, min_track: float) -> int:
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    results: list[StepResult] = []
    for row in rows:
        if not row.get("joint"):
            continue
        dq = float(row["dq_sim"]) if row.get("dq_sim") not in ("", None) else 0.0
        dc = float(row["delta_cmd"]) if row.get("delta_cmd") not in ("", None) else 0.0
        tr = float(row["track_ratio"]) if row.get("track_ratio") not in ("", None) else 0.0
        same = bool(int(float(row["same_sign"]))) if row.get("same_sign") not in ("", None) else False
        jump = bool(int(float(row["jump_flag"]))) if row.get("jump_flag") not in ("", None) else False
        ok = bool(int(float(row["ok"]))) if row.get("ok") not in ("", None) else False
        results.append(
            StepResult(
                row["joint"],
                row.get("side", ""),
                dc,
                dq,
                float(row["dq_motor"]) if row.get("dq_motor") not in ("", None) else 0.0,
                tr,
                same,
                jump,
                ok,
                row.get("note", ""),
            )
        )
    # 重新按阈值判定一遍，方便事后调 min_track
    for r in results:
        r.ok = r.same_sign and r.track_ratio >= min_track and not r.jump_flag
        if r.ok:
            r.note = "PASS"
        elif not r.same_sign and abs(r.dq_sim) > 1e-4:
            r.note = "SIGN可疑/方向反了"
        elif r.track_ratio < min_track:
            r.note = "跟不上/几乎不动"
        elif r.jump_flag:
            r.note = "其它关节异常跳变"
    summarize(results)
    score_pairs(results)
    suspects = [r.joint for r in results if (not r.same_sign) and abs(r.dq_sim) > 1e-4]
    weak = [r.joint for r in results if r.track_ratio < min_track]
    print("\n建议：")
    if suspects:
        print("  优先取反 SIGN：", ", ".join(sorted(set(suspects))))
    if weak:
        print("  优先查驱动/力矩/模式（跟踪弱）：", ", ".join(sorted(set(weak))))
    if not suspects and not weak:
        print("  符号与跟踪看起来正常，回去查观测跳变与控制频率。")
    return 0 if results and all(r.ok for r in results) else 2


def main() -> int:
    parser = argparse.ArgumentParser(description="单关节正负步进符号检查")
    parser.add_argument("--move", action="store_true", help="真正发运动（默认只打印清单）")
    parser.add_argument("--score", type=str, default="", help="事后对 CSV 评分")
    parser.add_argument(
        "--joints",
        type=str,
        default=",".join(DEFAULT_JOINTS),
        help="逗号分隔关节名",
    )
    parser.add_argument("--delta", type=float, default=0.05, help="仿真角步进 rad")
    parser.add_argument("--hold", type=float, default=1.0, help="每步保持秒数")
    parser.add_argument("--tqe", type=float, default=0.8, help="最大力矩 N·m")
    parser.add_argument("--vel", type=float, default=0.6, help="位置模式速度限制 rad/s")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--min-track", type=float, default=0.35, help="|Δq|/|Δcmd| 下限")
    parser.add_argument("--iface", default="can0")
    parser.add_argument("--motor-type", default="4538_19")
    parser.add_argument(
        "--pose-config",
        default=str(REPO / "motor" / "pose_contrl" / "pose_config.yaml"),
    )
    parser.add_argument(
        "--out-dir",
        default=str(REPO / "locomotion" / "logs" / "sign_check"),
    )
    args = parser.parse_args()

    if args.score:
        return score_csv(Path(args.score), args.min_track)

    names = [n.strip() for n in args.joints.split(",") if n.strip()]
    if not args.move:
        print_checklist(names)
        return 0
    return run_move(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n已中断")
        sys.exit(130)
