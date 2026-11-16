#!/usr/bin/env python3
"""微调轮行站立姿态（pose_config.yaml 里的 drive）。

只动腿：2 3 4 5 和 7 8 9 10。轮子 1、6 保持卸力，不追位置。
11、12 按现有 drive 角度扶住，不写入这次修改。

按键：
  ← →   切换关节
  ↑ ↓   当前关节 ±0.5°
  回车  把这 8 个关节的目标角写进 drive，作为站立姿态
  q     卸力退出

用法（先停驾驶，避免两路抢 CAN）：
  sudo systemctl stop wheel-drive
  cd /home/aidlux/cyberDuck/motor
  python3 pose_contrl/tune_stance.py
  sudo systemctl start wheel-drive
"""
from __future__ import annotations

import os
import re
import select
import signal
import subprocess
import sys
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pose import command_position, load_config, move_to_pose, open_bus

CONFIG = Path(__file__).with_name("pose_config.yaml")
IDS = (2, 3, 4, 5, 7, 8, 9, 10)
NAMES = {
    2: "右膝",
    3: "右髋pitch",
    4: "右髋roll",
    5: "右髋yaw",
    7: "左膝",
    8: "左髋pitch",
    9: "左髋roll",
    10: "左髋yaw",
}
STEP = 0.5
HOLD = (11, 12)


def drive_busy() -> bool:
    try:
        out = subprocess.check_output(["pgrep", "-af", "wheel_run/wheel_drive.py"], text=True)
    except (OSError, subprocess.CalledProcessError):
        return False
    return "wheel_drive.py" in out


def update_drive_joints(path: Path, updates: dict[int, float]) -> None:
    """只改 drive 段里这几个关节的数字，保留注释和其余姿态。"""
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    in_drive = False
    in_joints = False
    seen: set[int] = set()
    for index, line in enumerate(lines):
        if re.match(r"^  drive:\s*$", line):
            in_drive = True
            in_joints = False
            continue
        if in_drive and re.match(r"^  [A-Za-z0-9_]+\s*:", line):
            in_drive = False
            in_joints = False
        if in_drive and re.match(r"^    joints:\s*$", line):
            in_joints = True
            continue
        if in_joints and re.match(r"^  \S", line):
            in_joints = False
            in_drive = False
        if not in_joints:
            continue
        match = re.match(r"^(\s*)'(\d+)':\s*[-+]?\d+(?:\.\d+)?(.*)$", line.rstrip("\n"))
        if match is None:
            continue
        mid = int(match.group(2))
        if mid not in updates:
            continue
        tail = match.group(3)
        newline = "\n" if line.endswith("\n") else ""
        lines[index] = f"{match.group(1)}'{mid}': {updates[mid]:.2f}{tail}{newline}"
        seen.add(mid)
    missing = [mid for mid in updates if mid not in seen]
    if missing:
        raise SystemExit(f"drive 姿态里没找到关节 {missing}，没有写入。")
    path.write_text("".join(lines), encoding="utf-8")


class Keys:
    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd) if sys.stdin.isatty() else None
        if self.old is not None:
            tty.setcbreak(self.fd)

    def poll(self) -> str:
        if self.old is None or not select.select([sys.stdin], [], [], 0)[0]:
            return ""
        raw = os.read(self.fd, 1)
        if raw == b"\x1b":
            if select.select([sys.stdin], [], [], 0.03)[0]:
                rest = os.read(self.fd, 8)
                return {
                    b"[D": "left",
                    b"[C": "right",
                    b"[A": "up",
                    b"[B": "down",
                }.get(rest, "")
            return ""
        if raw in (b"\r", b"\n"):
            return "enter"
        if raw in (b"q", b"Q"):
            return "quit"
        return ""

    def close(self) -> None:
        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            self.old = None


def show(cmd: dict[int, float], index: int) -> None:
    bits = []
    for i, mid in enumerate(IDS):
        mark = ">" if i == index else " "
        bits.append(f"{mark}{mid}{NAMES[mid]} {cmd[mid]:+7.2f}°")
    print("  ".join(bits), flush=True)


def main() -> int:
    if not sys.stdin.isatty():
        print("需要在终端里运行，才能读方向键。", file=sys.stderr)
        return 2
    if drive_busy():
        print(
            "驾驶进程还占着 CAN。先执行: sudo systemctl stop wheel-drive",
            file=sys.stderr,
        )
        return 2
    cfg = load_config(CONFIG)
    spec = cfg.get("poses", {}).get("drive") or {}
    joints = {int(k): float(v) for k, v in (spec.get("joints") or {}).items()}
    missing = [mid for mid in IDS + HOLD if mid not in joints]
    if missing:
        print(f"drive 姿态缺关节 {missing}", file=sys.stderr)
        return 2
    cmd = {mid: joints[mid] for mid in IDS + HOLD}
    torque = abs(float(cfg.get("torque_nm", 1.0)))
    bus, found = open_bus(cfg)
    running = True

    def _stop(*_args) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    keys = Keys()
    index = 0
    try:
        body = [mid for mid in IDS + HOLD if mid in found]
        print("先走到当前 drive 站姿，再微调。轮子保持卸力。", flush=True)
        move_to_pose(bus, cfg, body, cmd, lambda: running, label="站立姿态", vel_limit=2.0)
        if not running:
            return 0
        print(
            f"左右键切换关节，上下键 ±{STEP:.1f}°，回车保存到 drive，q 卸力退出。",
            flush=True,
        )
        show(cmd, index)
        period = 1.0 / max(20.0, float(cfg.get("hz", 50.0)))
        while running:
            now = time.monotonic()
            key = keys.poll()
            if key == "left":
                index = (index - 1) % len(IDS)
                show(cmd, index)
            elif key == "right":
                index = (index + 1) % len(IDS)
                show(cmd, index)
            elif key == "up":
                mid = IDS[index]
                cmd[mid] = round(cmd[mid] + STEP, 2)
                show(cmd, index)
            elif key == "down":
                mid = IDS[index]
                cmd[mid] = round(cmd[mid] - STEP, 2)
                show(cmd, index)
            elif key == "enter":
                saved = {mid: cmd[mid] for mid in IDS}
                update_drive_joints(CONFIG, saved)
                print("已写入 drive 站立姿态：", flush=True)
                for mid in IDS:
                    print(f"  {mid} {NAMES[mid]} {saved[mid]:+.2f}°", flush=True)
            elif key == "quit":
                break
            command_position(bus, body, cmd, torque, vel_limit=2.0)
            spent = time.monotonic() - now
            if spent < period:
                time.sleep(period - spent)
    finally:
        keys.close()
        try:
            bus.disable_all(found)
            print("已卸力。", flush=True)
        except OSError as exc:
            print(f"卸力失败：{exc}", file=sys.stderr)
        bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
