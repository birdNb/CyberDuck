#!/usr/bin/env python3
"""头部随机张望。张嘴闭嘴跟这一声鸭叫同时开始、同时结束。

参与：11 头俯仰、12 脖子俯仰（CAN），13 yaw、14 roll、15 嘴（舵机）。
进来先把 11、12、13、14 收到 0°（伸直），嘴保持闭合。
Ctrl+C 时回到伸直、闭嘴，再卸力。

用法（在 servo 目录）：
  python3 head_play.py
"""

from __future__ import annotations

import errno
import math
import termios
import random
import signal
import sys
import threading
import time
from pathlib import Path

import yaml

from bus_servo import ADDR_GOAL_POSITION, ADDR_TORQUE_ENABLE, FtScsBus, put_u16le, signed_value
from servo_controller import ServoConfig, ServoController


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "motor"))
sys.path.insert(0, str(ROOT / "speaker"))

import numpy as np

from duck import Speaker, render, variant_count  # noqa: E402
from ht_can import Motor, MotorBus  # noqa: E402
from play import play_pcm  # noqa: E402
from synth import SR  # noqa: E402


COUNTS = 4096.0
SHORT_TAGS = ("peck", "chirp", "greet", "coo", "inquire")
POSE_YAML = ROOT / "sport_ctrl" / "pose_contrl" / "pose_config.yaml"
SERVO_YAML = Path(__file__).resolve().parent / "servo_config.yaml"


class Quack:
    def __init__(self) -> None:
        self.speaker = Speaker()
        self._playing = False
        self._lock = threading.Lock()

    def start(self) -> float:
        """开始播放，返回这一声的秒数。忙的时候返回 0。"""
        with self._lock:
            if self._playing:
                return 0.0
            self._playing = True
        tag = random.choice(SHORT_TAGS)
        try:
            variant = random.randrange(variant_count(tag))
            buf = render(tag, self.speaker.personality, variant)
        except Exception as exc:
            print(f"鸭叫没生成：{exc}", flush=True)
            with self._lock:
                self._playing = False
            return 0.0
        vol = max(0.0, min(1.0, float(self.speaker.volume)))
        if vol < 0.999:
            buf = np.clip(buf * vol, -1.0, 1.0)
        duration = float(np.asarray(buf).shape[0]) / SR
        print(f"鸭叫：{tag} {duration:.2f}s", flush=True)

        def _run() -> None:
            try:
                used = play_pcm(buf, self.speaker.device)
                self.speaker.device = self.speaker.device or used
            except Exception as exc:
                print(f"鸭叫没播出来：{exc}", flush=True)
            finally:
                with self._lock:
                    self._playing = False

        threading.Thread(target=_run, daemon=True).start()
        return duration


def approach(cur: float, goal: float, rate: float, dt: float) -> float:
    step = abs(rate) * dt
    delta = goal - cur
    if abs(delta) <= step:
        return goal
    return cur + math.copysign(step, delta)


def servo_raw(offset_deg: float, center_raw: int, direction: int) -> int:
    raw = center_raw + direction * offset_deg / 360.0 * COUNTS
    return int(round(max(0.0, min(COUNTS, raw))))


def sample(lo: float, hi: float) -> float:
    """多数时候在中间轻轻动，偶尔才靠近限位。"""
    if random.random() < 0.75:
        mid = (lo + hi) * 0.5
        half = (hi - lo) * 0.28
        return random.uniform(mid - half, mid + half)
    return random.uniform(lo, hi)


def load_yaml() -> tuple[dict, dict]:
    neck = (yaml.safe_load(POSE_YAML.read_text(encoding="utf-8")) or {}).get("neck") or {}
    servo = yaml.safe_load(SERVO_YAML.read_text(encoding="utf-8")) or {}
    for key in ("pitch", "neck", "yaw", "roll", "mouth"):
        if key not in neck:
            raise SystemExit(f"pose_config.yaml 的 neck 缺少 {key}")
    return neck, servo


def main() -> int:
    neck, servo_cfg = load_yaml()
    pitch = neck["pitch"]
    neck_pitch = neck["neck"]
    yaw = neck["yaw"]
    roll = neck["roll"]
    mouth = neck["mouth"]
    rate = float(neck.get("rate_deg_s", 40.0))
    mouth_rate = float(neck.get("mouth_rate_deg_s", 200.0))
    torque = float(neck.get("torque_nm", 0.8))
    port = str(servo_cfg.get("port", "/dev/ttyMSM0"))
    baud = int(servo_cfg.get("baudrate", 1_000_000))

    pitch_id = int(pitch["id"])
    neck_id = int(neck_pitch["id"])
    yaw_id = int(yaw["id"])
    roll_id = int(roll["id"])
    mouth_id = int(mouth["id"])
    can_axes = {"pitch": pitch, "neck": neck_pitch}
    axes = {
        "pitch": pitch,
        "neck": neck_pitch,
        "yaw": yaw,
        "roll": roll,
    }

    running = True

    def _stop(*_args) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    print(
        f"头俯仰 {pitch_id}、脖子俯仰 {neck_id}（CAN）  yaw {yaw_id}  roll {roll_id}  嘴 {mouth_id}。"
        f"头 {rate:.0f}°/s，嘴 {mouth_rate:.0f}°/s。先收到 0° 伸直。"
    )
    try:
        can = MotorBus("can0")
    except OSError as exc:
        print(f"打开 can0 失败：{exc}")
        return 1
    motors = {
        "pitch": can.add_motor(Motor(pitch_id, "3532_0", "pitch")),
        "neck": can.add_motor(Motor(neck_id, "3532_0", "neck")),
    }
    for mid in (pitch_id, neck_id):
        can.read_status(mid)
    can.collect(0.25)
    missing = [motors[name].id for name in can_axes if motors[name].state.t_pos <= 0]
    if missing:
        print(f"这些关节没有应答，取消：{missing}")
        can.close()
        return 2
    pose = {name: math.degrees(motors[name].state.position) for name in can_axes}

    try:
        bus = FtScsBus(port, baudrate=baud, timeout=0.05)
    except Exception as exc:
        print(f"打开舵机串口失败：{exc}")
        can.close()
        return 1

    cfgs = {}
    for name, spec, sid in (
        ("yaw", yaw, yaw_id),
        ("roll", roll, roll_id),
        ("mouth", mouth, mouth_id),
    ):
        center = 180.0
        cfgs[name] = ServoConfig(
            servo_id=sid,
            name=name,
            center_deg=center,
            min_deg=center - 90.0,
            max_deg=center + 90.0,
            direction=1 if int(spec.get("direction", 1)) >= 0 else -1,
            center_raw=int(spec.get("center_raw", 2048)),
            raw_min=0,
            raw_max=int(COUNTS),
        )
    ctrl = ServoController(bus, cfgs)
    try:
        for name, sid in (("yaw", yaw_id), ("roll", roll_id), ("mouth", mouth_id)):
            bus.ping(sid)
            raw = bus.read_u16(sid, 0x38) & 0x0FFF
            cfg = cfgs[name]
            pose[name] = (raw - cfg.center_raw) * cfg.direction * 360.0 / COUNTS
            print(f"{name} ID {sid}  现在 {pose[name]:+.1f}°  原始值 {raw}")
    except Exception as exc:
        print(f"舵机没有应答：{exc}")
        bus.close()
        can.close()
        return 2

    for name in ("yaw", "roll", "mouth"):
        cfg = cfgs[name]
        bus.write_u8(cfg.servo_id, ADDR_TORQUE_ENABLE, 1)
        bus.write_u16(cfg.servo_id, 0x30, 700 if name == "mouth" else 300)
        bus.write_u8(cfg.servo_id, 0x29, 0)
        bus.write(cfg.servo_id, 0x2E, signed_value(0))

    quack = Quack()
    vel_cap = max(0.15, math.radians(rate) * 1.4)

    def send() -> None:
        try:
            for name, motor in motors.items():
                can.command_pos_vel_tqe(
                    motor, math.radians(pose[name]), vel_cap, torque, reply=False
                )
            for name in ("yaw", "roll", "mouth"):
                cfg = cfgs[name]
                raw = servo_raw(pose[name], cfg.center_raw, cfg.direction)
                bus.write(cfg.servo_id, ADDR_GOAL_POSITION, put_u16le(raw))
        except (InterruptedError, OSError, termios.error) as exc:
            if getattr(exc, "errno", errno.EINTR) not in (errno.EINTR, 4):
                raise
            nonlocal running
            running = False

    def goto(goals: dict[str, float], label: str, move_rate: float | None = None, force: bool = False) -> None:
        print(label, flush=True)
        move_rate = rate if move_rate is None else move_rate
        while force or running:
            dt = 0.02
            done = True
            for name, goal in goals.items():
                pose[name] = approach(pose[name], goal, move_rate, dt)
                if abs(pose[name] - goal) > 0.4:
                    done = False
            try:
                send()
            except (InterruptedError, OSError, termios.error):
                if not force:
                    break
            if done:
                break
            try:
                time.sleep(dt)
            except InterruptedError:
                if not force:
                    break
        try:
            send()
        except (InterruptedError, OSError, termios.error):
            pass

    pose["mouth"] = min(pose.get("mouth", 0.0), float(mouth.get("open_deg", 30.0)))
    home = {"pitch": 0.0, "neck": 0.0, "yaw": 0.0, "roll": 0.0, "mouth": 0.0}
    released = False

    def release() -> None:
        nonlocal running, released
        if released:
            return
        released = True
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        running = True
        print("回到伸直并卸力。", flush=True)
        try:
            goto(home, "收回 0°。", force=True)
        except Exception as exc:
            print(f"回零时出错：{exc}", flush=True)
        for _ in range(4):
            try:
                can.disable(pitch_id)
                can.disable(neck_id)
                for cfg in cfgs.values():
                    bus.write_u8(cfg.servo_id, ADDR_TORQUE_ENABLE, 0)
                break
            except (InterruptedError, OSError, termios.error):
                time.sleep(0.05)
        print("已卸力。", flush=True)

    try:
        goto(home, "脖子收到 0°（伸直），嘴闭合。")
        if running:
            print("开始随机张望。Ctrl+C 回到伸直并卸力。", flush=True)
        open_deg = float(mouth.get("open_deg", 30.0))
        while running:
            goals = {name: sample(float(spec["min_deg"]), float(spec["max_deg"])) for name, spec in axes.items()}
            goals["mouth"] = 0.0
            want_mouth = random.random() < 0.55
            print(
                "张望  "
                f"头 {goals['pitch']:+.0f}°  脖子 {goals['neck']:+.0f}°  "
                f"yaw {goals['yaw']:+.0f}°  roll {goals['roll']:+.0f}°"
                + ("  张嘴" if want_mouth else ""),
                flush=True,
            )
            while running:
                dt = 0.02
                done = True
                for name in ("pitch", "neck", "yaw", "roll"):
                    pose[name] = approach(pose[name], goals[name], rate, dt)
                    if abs(pose[name] - goals[name]) > 0.4:
                        done = False
                send()
                if done:
                    break
                time.sleep(dt)
            if not running:
                break
            if want_mouth:
                opened = random.uniform(0.75, 1.0) * open_deg
                sound_s = quack.start()
                if sound_s < 0.08:
                    sound_s = 0.28
                open_s = min(0.14, max(0.06, sound_s * 0.32))
                close_s = min(0.12, max(0.05, sound_s * 0.28))
                if open_s + close_s > sound_s:
                    scale = sound_s / (open_s + close_s)
                    open_s *= scale
                    close_s *= scale
                hold_s = max(0.0, sound_s - open_s - close_s)
                print(f"张嘴 {opened:.0f}°，和叫声一起 {sound_s:.2f}s", flush=True)
                phases = (
                    (opened, opened / open_s, open_s),
                    (opened, mouth_rate, hold_s),
                    (0.0, opened / close_s, close_s),
                )
                for goal, move_rate, seconds in phases:
                    deadline = time.monotonic() + seconds
                    while running and time.monotonic() < deadline:
                        pose["mouth"] = approach(pose["mouth"], goal, move_rate, 0.02)
                        send()
                        time.sleep(0.02)
                if running and abs(pose["mouth"]) > 0.4:
                    pose["mouth"] = 0.0
                    send()
            else:
                hold_until = time.monotonic() + random.uniform(0.6, 1.6)
                while running and time.monotonic() < hold_until:
                    send()
                    time.sleep(0.02)

    finally:
        try:
            release()
        finally:
            bus.close()
            can.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
