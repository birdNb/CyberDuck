#!/usr/bin/env python3
"""示教：低力矩跟人掰，30 Hz 记录关节角，点与点之间平滑回放。

用法（在 pose_contrl 目录）：
  python3 teach.py
  python3 teach.py list
  python3 teach.py play 名字
  python3 teach.py mirror 源名字 新名字    左腿动作镜像到右腿

记录中按键：
  r  开始记录（阻尼跟人掰）/ 停止并位置保持最后一帧
  c  检查：2 秒走到起点，空格开始或暂停
  n  命名并保存
  q  退出并卸力
"""
from __future__ import annotations

import csv
import re
import select
import sys
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pose import (
    DEG,
    command_position,
    load_config,
    open_bus,
    smootherstep,
)

HZ = 30.0
PLAY_HZ = 100.0
DIR = Path(__file__).with_name("teachings")
NAME_OK = re.compile(r"^[\w\-\u4e00-\u9fff]+$", re.UNICODE)


class Keys:
    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd) if sys.stdin.isatty() else None
        if self.old is not None:
            tty.setcbreak(self.fd)

    def poll(self) -> str:
        if self.old is None or not select.select([sys.stdin], [], [], 0)[0]:
            return ""
        return sys.stdin.read(1)

    def close(self) -> None:
        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            self.old = None


def sample(bus, ids: list[int], last: dict[int, float]) -> dict[int, float]:
    bus.poll_status(ids, timeout=0.02)
    out = dict(last)
    for mid in ids:
        payload = bus.motors[mid].state.payload
        if len(payload) >= 8 and payload[0] in (0x17, 0x27):
            out[mid] = bus.motors[mid].state.position / DEG
    return out


def summary(frames: list[dict[int, float]], ids: list[int]) -> None:
    if not frames:
        print("还没有记录。")
        return
    print(f"共 {len(frames)} 帧，{len(frames) / HZ:.1f} s，{HZ:.0f} Hz")
    print(f"{'ID':>4}  {'起点':>8}  {'终点':>8}  {'最小':>8}  {'最大':>8}")
    for mid in ids:
        col = [frame[mid] for frame in frames]
        print(
            f"{mid:4d}  {col[0]:+7.2f}°  {col[-1]:+7.2f}°  "
            f"{min(col):+7.2f}°  {max(col):+7.2f}°"
        )


APPROACH_S = 2.0


def blend_to(bus, ids, origin, goal, seconds, torque, keys: Keys) -> bool:
    """缓缓走到目标。返回 False 表示按了 q。"""
    t0 = time.monotonic()
    while True:
        u = min(1.0, (time.monotonic() - t0) / seconds)
        scale = smootherstep(u)
        cmd = {mid: origin[mid] + (goal[mid] - origin[mid]) * scale for mid in ids}
        command_position(bus, ids, cmd, torque)
        if keys.poll() in ("q", "Q"):
            return False
        if u >= 1.0:
            return True
        time.sleep(0.02)


def segment_pose(frames: list[dict[int, float]], ids: list[int], index: int, u: float) -> dict[int, float]:
    """相邻记录点用三次插值过渡，经过点位时速度连续，避免阶跃过冲。"""
    u = max(0.0, min(1.0, u))
    n = len(frames)
    i0 = min(index, n - 1)
    i1 = min(i0 + 1, n - 1)
    prev_i = max(i0 - 1, 0)
    next_i = min(i1 + 1, n - 1)
    u2 = u * u
    u3 = u2 * u
    h00 = 2 * u3 - 3 * u2 + 1
    h10 = u3 - 2 * u2 + u
    h01 = -2 * u3 + 3 * u2
    h11 = u3 - u2
    out = {}
    for mid in ids:
        p0 = frames[i0][mid]
        p1 = frames[i1][mid]
        m0 = 0.5 * (p1 - frames[prev_i][mid])
        m1 = 0.5 * (frames[next_i][mid] - p0)
        out[mid] = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1
    return out


def playback(
    bus,
    cfg: dict,
    ids: list[int],
    frames: list[dict[int, float]],
    keys: Keys,
    times: list[float] | None = None,
) -> dict[int, float] | None:
    if not frames:
        print("还没有记录。")
        return None
    torque = abs(float(cfg.get("torque_nm", 1.2)))
    for mid in ids:
        bus.enable(mid)
        bus.set_torque_limit(mid, torque)
    origin = sample(bus, ids, frames[0])
    print(f"{APPROACH_S:.0f} 秒走到起始姿态。按 q 取消。")
    if not blend_to(bus, ids, origin, frames[0], APPROACH_S, torque, keys):
        print("已取消，停在当前位置。")
        return origin
    print("已到起点。空格开始或暂停，q 结束检查。")
    if times is None or len(times) != len(frames):
        times = [i / HZ for i in range(len(frames))]
    t0 = times[0]
    times = [t - t0 for t in times]
    duration = times[-1] if times else 0.0
    playing = False
    finished = False
    progress = 0.0
    seg_t0 = time.monotonic()
    hold = dict(frames[0])
    play_period = 1.0 / PLAY_HZ
    while True:
        key = keys.poll()
        if key in ("q", "Q"):
            print("检查结束，位置保持当前帧。")
            return hold
        if key == " ":
            if finished:
                finished = False
                print(f"{APPROACH_S:.0f} 秒回到起始姿态。按 q 取消。", flush=True)
                origin = sample(bus, ids, hold)
                if not blend_to(bus, ids, origin, frames[0], APPROACH_S, torque, keys):
                    print("检查结束，停在当前位置。")
                    return sample(bus, ids, hold)
                hold = dict(frames[0])
            playing = not playing
            if playing:
                seg_t0 = time.monotonic() - progress
            else:
                progress = min(duration, time.monotonic() - seg_t0)
            print("播放" if playing else "暂停", flush=True)
        if playing:
            elapsed = time.monotonic() - seg_t0
            if elapsed >= duration:
                playing = False
                finished = True
                progress = 0.0
                hold = dict(frames[-1])
                print(f"播放结束，停在最后一帧。空格 {APPROACH_S:.0f} 秒回到起点后再播。", flush=True)
            else:
                index = 0
                while index + 1 < len(times) and times[index + 1] < elapsed:
                    index += 1
                span = times[index + 1] - times[index] if index + 1 < len(times) else 1.0 / HZ
                u = 0.0 if span <= 1e-4 else (elapsed - times[index]) / span
                hold = segment_pose(frames, ids, index, u)
        command_position(bus, ids, hold, torque)
        time.sleep(play_period)


def save_clip(name: str, ids: list[int], frames: list[dict[int, float]]) -> Path:
    DIR.mkdir(parents=True, exist_ok=True)
    path = DIR / f"{name}.csv"
    if path.exists():
        raise FileExistsError(path)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t_s", *[str(mid) for mid in ids]])
        for i, frame in enumerate(frames):
            writer.writerow([f"{i / HZ:.3f}", *[f"{frame[mid]:.2f}" for mid in ids]])
    return path


def load_clip(path: Path) -> tuple[list[int], list[dict[int, float]], list[float]]:
    with path.open(encoding="utf-8", newline="") as fh:
        rows = list(csv.reader(fh))
    if len(rows) < 2:
        raise SystemExit(f"{path} 是空的")
    ids = [int(col) for col in rows[0][1:]]
    frames = []
    times = []
    for row in rows[1:]:
        times.append(float(row[0]))
        frames.append({mid: float(row[i + 1]) for i, mid in enumerate(ids)})
    return ids, frames, times


# 左腿电机 → 右腿电机：踝、小腿、髋 pitch、髋 roll、髋 yaw。
# 两腿电机镜像安装，对称姿态下电机角左右反号（stand、fold、坐姿起点都如此）。
LEFT_TO_RIGHT = {6: 1, 7: 2, 8: 3, 9: 4, 10: 5}
MIRROR_SIGN = -1.0


def mirror_clip(src: str, dst: str) -> int:
    path = DIR / f"{src}.csv"
    if not path.exists():
        raise SystemExit(f"没有 {path}")
    if not NAME_OK.match(dst):
        raise SystemExit("名字只能用字母、数字、下划线、减号或中文。")
    out = DIR / f"{dst}.csv"
    if out.exists():
        raise SystemExit(f"已经有同名文件: {out}")
    ids, frames, times = load_clip(path)
    missing = [mid for pair in LEFT_TO_RIGHT.items() for mid in pair if mid not in ids]
    if missing:
        raise SystemExit(f"示教里缺少关节: {missing}")
    for frame in frames:
        for left, right in LEFT_TO_RIGHT.items():
            frame[right] = MIRROR_SIGN * frame[left]
    with out.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t_s", *[str(mid) for mid in ids]])
        for t, frame in zip(times, frames):
            writer.writerow([f"{t:.3f}", *[f"{frame[mid]:.2f}" for mid in ids]])
    print(f"左腿 6..10 镜像到右腿 1..5，已保存 {out}")
    summary(frames, ids)
    return 0


def list_clips() -> int:
    files = sorted(DIR.glob("*.csv")) if DIR.exists() else []
    if not files:
        print(f"{DIR} 里还没有示教。")
        return 0
    for path in files:
        ids, frames, times = load_clip(path)
        dur = times[-1] - times[0] if len(times) > 1 else 0.0
        print(f"{path.stem}  {dur:.1f} s  {len(frames)} 帧  关节 {ids[0]}..{ids[-1]}")
    return 0


def play_saved(name: str) -> int:
    path = DIR / f"{name}.csv"
    if not path.exists():
        raise SystemExit(f"没有 {path}")
    ids, frames, times = load_clip(path)
    summary(frames, ids)
    cfg = load_config(Path(__file__).with_name("pose_config.yaml"))
    bus, online = open_bus(cfg)
    missing = [mid for mid in ids if mid not in online]
    if missing:
        bus.close()
        raise SystemExit(f"这些关节不在线，不能回放: {missing}")
    keys = Keys()
    try:
        playback(bus, cfg, ids, frames, keys, times)
    finally:
        keys.close()
        for mid in ids:
            try:
                bus.disable(mid)
            except OSError:
                pass
        bus.close()
    return 0


def session() -> int:
    cfg = load_config(Path(__file__).with_name("pose_config.yaml"))
    bus, ids = open_bus(cfg)
    damp_nm = 0.08
    hold_nm = abs(float(cfg.get("torque_nm", 1.2)))
    keys = Keys()
    frames: list[dict[int, float]] = []
    recording = False
    holding = False
    next_sample = time.monotonic()
    hold: dict[int, float] = {}
    follow: dict[int, float] = {}
    print("示教。进入后关节不出力。")
    print("  r  开始记录（阻尼）/ 停止并位置保持最后一帧")
    print("  c  检查：2 秒走到起点，空格开始或暂停")
    print("  n  命名并保存")
    print("  q  退出")
    try:
        while True:
            key = keys.poll()
            if key in ("q", "Q"):
                break
            if key in ("r", "R"):
                recording = not recording
                if recording:
                    holding = False
                    frames = []
                    follow = sample(bus, ids, {mid: 0.0 for mid in ids})
                    for mid in ids:
                        bus.enable(mid)
                        bus.set_torque_limit(mid, damp_nm)
                    next_sample = time.monotonic()
                    print(f"开始记录，阻尼 {damp_nm:.2f} Nm。再按 r 停止。", flush=True)
                else:
                    recording = False
                    holding = True
                    if frames:
                        hold = dict(frames[-1])
                    elif follow:
                        hold = dict(follow)
                    for mid in ids:
                        bus.enable(mid)
                        bus.set_torque_limit(mid, hold_nm)
                    print(
                        f"停止。录了 {len(frames)} 帧，{len(frames) / HZ:.1f} s。"
                        f"位置保持最终姿态，力矩 {hold_nm:.2f} Nm。",
                        flush=True,
                    )
            elif key in ("c", "C"):
                recording = False
                summary(frames, ids)
                if frames:
                    stayed = playback(bus, cfg, ids, frames, keys)
                    if stayed:
                        hold = stayed
                        holding = True
                    for mid in ids:
                        bus.set_torque_limit(mid, hold_nm)
            elif key in ("n", "N"):
                if not frames:
                    print("没有可保存的记录。")
                else:
                    keys.close()
                    name = input("名字: ").strip()
                    keys = Keys()
                    if not NAME_OK.match(name):
                        print("名字只能用字母、数字、下划线、减号或中文。")
                    else:
                        try:
                            path = save_clip(name, ids, frames)
                        except FileExistsError as exc:
                            print(f"已经有同名文件: {exc}")
                        else:
                            print(f"已保存 {path}")
            now = time.monotonic()
            if recording:
                if now >= next_sample:
                    measured = sample(bus, ids, follow)
                    follow = dict(measured)
                    frames.append(dict(measured))
                    if len(frames) % int(HZ) == 0:
                        print(f"  记录中 {len(frames) / HZ:.0f} s", flush=True)
                    next_sample += 1.0 / HZ
                    if next_sample < now:
                        next_sample = now
                command_position(bus, ids, follow, damp_nm)
            elif holding and hold:
                command_position(bus, ids, hold, hold_nm)
            time.sleep(0.01)
    finally:
        keys.close()
        for mid in ids:
            try:
                bus.disable(mid)
            except OSError:
                pass
        bus.close()
        print("已卸力。")
    return 0


def main() -> int:
    if len(sys.argv) == 1:
        return session()
    cmd = sys.argv[1]
    if cmd == "list":
        return list_clips()
    if cmd == "play" and len(sys.argv) == 3:
        return play_saved(sys.argv[2])
    if cmd == "mirror" and len(sys.argv) == 4:
        return mirror_clip(sys.argv[2], sys.argv[3])
    raise SystemExit("用法: python3 teach.py [list | play 名字 | mirror 源名字 新名字]")


if __name__ == "__main__":
    raise SystemExit(main())
