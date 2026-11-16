#!/usr/bin/env python3
"""在预设姿态之间柔性切换。15 号电机不参与。

位置姿态 stand / squat / fold 从当前角度平滑过渡，默认 3 秒。
damp：保持当前姿态、低力矩跟随，方便用手掰。
release：失能 1..14，不再出力。

用法：
  python3 pose.py
  python3 pose.py go stand
  python3 pose.py capture stand
  python3 pose.py read
  python3 pose.py list
  python3 teach.py
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ht_can import Motor, MotorBus, QUERY_STATUS

try:
    import yaml
except ImportError:
    yaml = None

DEG = math.pi / 180.0
CONFIG_PATH = Path(__file__).with_name("pose_config.yaml")
POSE_ORDER = ("stand", "squat", "fold", "damp", "release")
MOTOR_DIR = Path(__file__).resolve().parents[1]


def smootherstep(u: float) -> float:
    u = max(0.0, min(1.0, u))
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


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
    setup = MOTOR_DIR / "setup_can0.sh"
    print(f"{iface} 未启动，尝试 {setup} ...")
    try:
        subprocess.check_call([str(setup), iface])
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"抬起 {iface} 失败: {exc}", file=sys.stderr)


def load_config(path: Path) -> dict:
    if yaml is None:
        raise SystemExit("缺少 PyYAML，请先: pip install pyyaml")
    if not path.exists():
        raise SystemExit(f"找不到配置: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    data.setdefault("poses", {})
    return data


def save_config(path: Path, cfg: dict) -> None:
    text = yaml.safe_dump(
        cfg,
        allow_unicode=True,
        sort_keys=False,
        default_flow_style=False,
    )
    header = (
        "# Joint poses in degrees. Motor 15 is not included.\n"
        "# Capture: python3 pose.py capture stand\n"
        "# Switch:  python3 pose.py go stand\n"
    )
    path.write_text(header + text, encoding="utf-8")


def joint_ids(cfg: dict) -> list[int]:
    start = int(cfg.get("start_id", 1))
    end = int(cfg.get("end_id", 14))
    skip = {int(x) for x in cfg.get("skip_ids") or [15]}
    return [i for i in range(start, end + 1) if i not in skip]


def pose_spec(cfg: dict, name: str) -> dict:
    poses = cfg.get("poses") or {}
    if name not in poses:
        known = ", ".join(poses) or "(empty)"
        raise SystemExit(f"没有姿态 {name!r}。可选: {known}")
    spec = poses[name] or {}
    spec.setdefault("mode", "position")
    spec.setdefault("joints", {})
    return spec


def open_bus(cfg: dict) -> tuple[MotorBus, list[int]]:
    iface = str(cfg.get("interface", "can0"))
    ensure_can(iface)
    down = iface_down_reason(iface)
    if down:
        raise SystemExit(f"{down}\n请先: sudo ../setup_can0.sh")
    ids = joint_ids(cfg)
    motor_type = str(cfg.get("motor_type", "3532_0"))
    bus = MotorBus(iface)
    for mid in ids:
        bus.add_motor(Motor(mid, motor_type, f"J{mid}"))
    found = bus.scan(ids, timeout_each=0.06)
    missing = [i for i in ids if i not in found]
    if missing:
        time.sleep(0.2)
        found = sorted(set(found) | set(bus.scan(missing, timeout_each=0.1)))
        missing = [i for i in ids if i not in found]
    if missing:
        print(f"未应答，本次跳过: {missing}")
    if not found:
        bus.close()
        raise SystemExit("没有在线关节，取消动作。")
    # 关闭周期主动上报，状态靠控制/查询帧回传。
    for mid in found:
        bus.set_timeout_ms(mid, 0)
    # 进入总线默认卸力，避免扫完就在零位顶着力矩。
    bus.disable_all(found)
    # 扫完后再读一次 0x0E 状态，保证 position 有效。
    bus.poll_status(found, timeout=0.10)
    return bus, found


def read_degrees(bus: MotorBus, ids: list[int]) -> dict[int, float]:
    bus.poll_status(ids, timeout=0.08)
    out: dict[int, float] = {}
    stale: list[int] = []
    for mid in ids:
        motor = bus.motors[mid]
        payload = motor.state.payload
        if len(payload) < 8 or payload[0] != QUERY_STATUS:
            stale.append(mid)
            continue
        out[mid] = motor.state.position / DEG
    if stale:
        raise RuntimeError(f"这些关节没有读到位置: {stale}")
    return out


def print_degrees(degrees: dict[int, float]) -> None:
    for mid in sorted(degrees):
        print(f"  ID {mid:2d}  {degrees[mid]:+8.2f}°")


def poll_one(bus: MotorBus, mid: int) -> None:
    bus.read_status(mid)
    bus.collect(timeout=0.008)
    bus.read_voltage(mid)
    bus.collect(timeout=0.008)


def format_joint_table(bus: MotorBus, ids: list[int], cmd: dict[int, float]) -> list[str]:
    lines = [f"{'ID':>4}  {'cmd':>8}  {'pos':>8}  {'torque':>9}  {'voltage':>8}"]
    for mid in ids:
        state = bus.motors[mid].state
        volt = f"{state.voltage:6.1f}V" if state.voltage > 0.5 else "    --"
        lines.append(
            f"{mid:4d}  {cmd[mid]:+7.2f}°  {state.position / DEG:+7.2f}°  "
            f"{state.torque:+7.3f}Nm  {volt}"
        )
    return lines


class LiveTable:
    def __init__(self) -> None:
        self.lines = 0

    def show(self, lines: list[str]) -> None:
        if self.lines:
            sys.stdout.write(f"\033[{self.lines}A")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()
        self.lines = len(lines)


def capture_pose(cfg: dict, path: Path, name: str) -> int:
    spec = pose_spec(cfg, name)
    if spec.get("mode") not in ("position", "damping", "release"):
        raise SystemExit(f"姿态 {name} 的 mode 无法识别: {spec.get('mode')}")
    bus, ids = open_bus(cfg)
    try:
        degrees = read_degrees(bus, ids)
    finally:
        bus.close()
    joints = {str(mid): round(degrees[mid], 2) for mid in sorted(degrees)}
    cfg["poses"][name]["joints"] = joints
    if "mode" not in cfg["poses"][name]:
        cfg["poses"][name]["mode"] = spec.get("mode", "position")
    save_config(path, cfg)
    print(f"已把当前角度写入 {path.name} → {name}")
    print_degrees(degrees)
    return 0


def command_position(
    bus: MotorBus,
    ids: list[int],
    degrees: dict[int, float],
    torque: float,
    vel_limit: float = 2.0,
) -> None:
    """位置+最大速度+力矩上限（经典 CAN 模式 0x1F）。"""
    for mid in ids:
        motor = bus.motors[mid]
        bus.command_pos_vel_tqe(
            motor,
            degrees[mid] * DEG,
            abs(vel_limit),
            torque,
            reply=False,
        )


def transition(bus: MotorBus, cfg: dict, ids: list[int], target: dict[int, float], running) -> None:
    start = read_degrees(bus, ids)
    missing = [mid for mid in ids if mid not in target]
    if missing:
        raise RuntimeError(f"姿态里缺少关节角度: {missing}。先 capture。")

    duration = max(0.5, float(cfg.get("transition_s", 3.0)))
    hz = max(20.0, float(cfg.get("hz", 50.0)))
    period = 1.0 / hz
    vel_cap = abs(float(cfg.get("vel_limit", 1.0)))
    torque = abs(float(cfg.get("torque_nm", 1.2)))

    deltas = [abs(target[mid] - start[mid]) for mid in ids]
    peak = max(deltas) if deltas else 0.0
    need = (1.875 * peak * DEG) / vel_cap if vel_cap > 1e-6 else duration
    if need > duration:
        print(f"最大关节行程 {peak:.1f}°，3s 会过快，过渡延长到 {need:.1f}s")
        duration = need

    print(f"柔性过渡 {duration:.1f}s，力矩上限 {torque:.2f} Nm")
    print("起点:")
    print_degrees(start)
    print("目标:")
    print_degrees({mid: target[mid] for mid in ids})

    for mid in ids:
        bus.enable(mid)
        bus.set_torque_limit(mid, torque)
    time.sleep(0.05)

    t0 = time.monotonic()
    last_cmd = dict(start)
    while running():
        now = time.monotonic()
        u = (now - t0) / duration
        s = smootherstep(u)
        cmd = {mid: start[mid] + (target[mid] - start[mid]) * s for mid in ids}
        command_position(bus, ids, cmd, torque)
        last_cmd = cmd
        bus.collect(timeout=min(0.004, period * 0.3))
        if u >= 1.0:
            break
        spent = time.monotonic() - now
        if spent < period:
            time.sleep(period - spent)

    hold = target if running() else last_cmd
    if running():
        print("已到达目标姿态，位置锁定。下面实时刷新，Ctrl+C 退出后仍停在该角度。")
    else:
        print("过渡被打断，锁定在当前位置。")
    table = LiveTable()
    poll_i = 0
    next_ui = 0.0
    while running():
        t0 = time.monotonic()
        command_position(bus, ids, hold, torque)
        if ids and t0 >= next_ui:
            poll_one(bus, ids[poll_i % len(ids)])
            poll_i += 1
            table.show(format_joint_table(bus, ids, hold))
            next_ui = t0 + 0.05
        spent = time.monotonic() - t0
        if spent < period:
            time.sleep(period - spent)
    t_end = time.monotonic()
    while time.monotonic() - t_end < 0.2:
        command_position(bus, ids, hold, torque)
        time.sleep(period)


def enter_damping(bus: MotorBus, cfg: dict, ids: list[int], running) -> None:
    torque = abs(float(cfg.get("damping_torque_nm", 0.35)))
    hz = max(20.0, float(cfg.get("hz", 50.0)))
    period = 1.0 / hz
    print(f"进入 damp：低力矩跟随当前角度（{torque:.2f} Nm）。Ctrl+C 停在当前位置。")
    hold = read_degrees(bus, ids)
    for mid in ids:
        bus.enable(mid)
        bus.set_torque_limit(mid, torque)
    table = LiveTable()
    poll_i = 0
    next_ui = 0.0
    while running():
        t0 = time.monotonic()
        try:
            measured = read_degrees(bus, ids)
        except RuntimeError:
            measured = hold
        for mid in ids:
            hold[mid] = hold[mid] * 0.85 + measured[mid] * 0.15
        command_position(bus, ids, hold, torque)
        if ids and t0 >= next_ui:
            poll_one(bus, ids[poll_i % len(ids)])
            poll_i += 1
            table.show(format_joint_table(bus, ids, hold))
            next_ui = t0 + 0.05
        spent = time.monotonic() - t0
        if spent < period:
            time.sleep(period - spent)
    print("damp 结束，电机停在当前位置。")


def enter_release(bus: MotorBus, ids: list[int]) -> None:
    print("release：失能关节 1..14（15 号不动）。")
    for mid in ids:
        bus.disable(mid)
    time.sleep(0.05)
    print("已卸力。")


def go_pose(cfg: dict, name: str) -> int:
    spec = pose_spec(cfg, name)
    mode = str(spec.get("mode", "position"))
    bus, ids = open_bus(cfg)
    running = True

    def _stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        if mode == "release":
            enter_release(bus, ids)
            return 0
        if mode == "damping":
            enter_damping(bus, cfg, ids, lambda: running)
            return 0
        if mode != "position":
            raise SystemExit(f"未知 mode: {mode}")
        joints = spec.get("joints") or {}
        if not joints:
            raise SystemExit(f"姿态 {name} 还没有角度。先摆好再执行: python3 pose.py capture {name}")
        target = {int(k): float(v) for k, v in joints.items()}
        transition(bus, cfg, ids, target, lambda: running)
        return 0
    finally:
        bus.close()


def print_menu(cfg: dict) -> None:
    print("姿态：")
    for i, name in enumerate(POSE_ORDER, start=1):
        spec = (cfg.get("poses") or {}).get(name) or {}
        mode = spec.get("mode", "position")
        n = len(spec.get("joints") or {})
        extra = f"{n} 个关节" if mode == "position" else mode
        print(f"  {i}. {name}  ({extra})")
    print("  c. 采集当前角度到某个姿态")
    print("  r. 只读当前角度")
    print("  t. 示教：30 Hz 记录动作序列")
    print("  q. 退出")


def interactive(cfg: dict, path: Path) -> int:
    while True:
        print_menu(cfg)
        choice = input("> ").strip()
        if choice in ("q", "Q", ""):
            return 0
        if choice in ("r", "R"):
            bus, ids = open_bus(cfg)
            try:
                print_degrees(read_degrees(bus, ids))
            finally:
                bus.close()
            continue
        if choice in ("c", "C"):
            name = input("写入哪个姿态（stand/squat/fold/damp/release）: ").strip()
            capture_pose(cfg, path, name)
            cfg = load_config(path)
            continue
        if choice in ("t", "T"):
            from teach import main as teach_main

            teach_main()
            continue
        if choice.isdigit() and 1 <= int(choice) <= len(POSE_ORDER):
            go_pose(cfg, POSE_ORDER[int(choice) - 1])
            continue
        if choice in (cfg.get("poses") or {}):
            go_pose(cfg, choice)
            continue
        print("无法识别，输入 1-5、c、r 或 q。")


def main() -> int:
    parser = argparse.ArgumentParser(description="柔性切换预设姿态（跳过 15 号电机）")
    parser.add_argument("action", nargs="?", choices=("go", "capture", "read", "list"))
    parser.add_argument("name", nargs="?", help="姿态名，例如 stand")
    parser.add_argument("-c", "--config", default=str(CONFIG_PATH))
    parser.add_argument("--seconds", type=float, default=None, help="覆盖过渡时间，默认用 yaml 的 3s")
    args = parser.parse_args()

    path = Path(args.config)
    cfg = load_config(path)
    if args.seconds is not None:
        cfg["transition_s"] = args.seconds

    if args.action is None:
        return interactive(cfg, path)
    if args.action == "list":
        print_menu(cfg)
        return 0
    if args.action == "read":
        bus, ids = open_bus(cfg)
        try:
            print_degrees(read_degrees(bus, ids))
        finally:
            bus.close()
        return 0
    if not args.name:
        raise SystemExit("请给出姿态名，例如: python3 pose.py go stand")
    if args.action == "capture":
        return capture_pose(cfg, path, args.name)
    return go_pose(cfg, args.name)


if __name__ == "__main__":
    raise SystemExit(main())
