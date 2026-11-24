#!/usr/bin/env python3
"""C++ 轮行的伴随程序：头部、嘴、灯环、音量、起身、翻滚。

不打开 CAN。头俯仰和脖子的角度，以及起身/翻滚的关节角，写回 C++，
由 C++ 的 CAN 线程发出。舵机、喇叭和灯环仍用原来的 Python 实现。
"""

from __future__ import annotations

import math
import socket
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

RUN = Path(__file__).resolve().parents[1]
REPO = RUN.parents[1]
sys.path.insert(0, str(RUN))
sys.path.insert(0, str(REPO / "sport_ctrl" / "pose_contrl"))

import pose  # noqa: E402
from roulade_policy import RouladePolicy  # noqa: E402
from wheel_drive import (  # noqa: E402
    HEAD_PITCH_ID,
    NECK_PITCH_ID,
    DuckVolume,
    Gamepad,
    HeadLook,
    LedRing,
    Mouth,
)

NECK_TQ = 1.0
SKILL_TQ = 2.0
SKILL_VEL = 6.0


class _Motor:
    def __init__(self) -> None:
        self.state = SimpleNamespace(position=0.0, velocity=0.0, torque=0.0, t_pos=0.0)


class _Bus:
    def __init__(self) -> None:
        self.motors = {i: _Motor() for i in range(1, 16)}


class _Link:
    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.buf = bytearray()

    def read(self) -> list[str] | None:
        while True:
            if self.buf.startswith(b".\n"):
                self.buf = self.buf[2:]
                return []
            dot = self.buf.find(b"\n.\n")
            if dot >= 0:
                text = self.buf[:dot].decode("utf-8", errors="replace")
                self.buf = self.buf[dot + 3 :]
                return [line for line in text.splitlines() if line]
            chunk = self.sock.recv(4096)
            if not chunk:
                return None
            self.buf.extend(chunk)


def _stand_deg(config: str) -> dict[int, float]:
    cfg = pose.load_config(Path(config))
    spec = pose.pose_spec(cfg, "drive")
    return {int(k): float(v) for k, v in (spec.get("joints") or {}).items()}


def _meas_deg(bus: _Bus, can: int, fallback: float) -> float:
    state = bus.motors[can].state
    if state.t_pos <= 0.0 or time.monotonic() - state.t_pos > 0.4:
        return fallback
    return math.degrees(state.position)


def main() -> int:
    if len(sys.argv) < 3:
        print("用法: show_bridge.py <socket> <pose_config.yaml>", flush=True)
        return 2
    sock_path, config = sys.argv[1], sys.argv[2]
    stand = _stand_deg(config)
    bus = _Bus()
    mouth = Mouth()
    led = LedRing()
    threading.Thread(target=led.start, name="led-boot", daemon=True).start()
    volume = DuckVolume(mouth.speaker)
    roulade = RouladePolicy()
    print(roulade.status_line(), flush=True)
    print("倒地起身的推理在轮行程序里。倒地后按 LB 才执行。", flush=True)
    pad = Gamepad()
    head: HeadLook | None = None
    head_lock = threading.Lock()
    phase = "boot"
    drive_ready = False
    homing = False
    cancel_home = False
    lb_prev = False
    imu_warned = False
    mouth_was_down = False
    mouth_off_at = 0.0
    rt_down_at = 0.0
    long_quack_sent = False
    was_skill = False
    g_on = False
    g_yaw = 0.0
    g_roll = 0.0
    gravity = (0.0, 0.0, -1.0)
    gyro = (0.0, 0.0, 0.0)
    imu_ok = False
    prev = time.monotonic()

    raw = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    deadline = time.monotonic() + 15.0
    while True:
        try:
            raw.connect(sock_path)
            break
        except OSError:
            if time.monotonic() > deadline:
                print("接不上轮行程序。", flush=True)
                return 1
            time.sleep(0.1)
    link = _Link(raw)
    print("已接上轮行程序。卸力时可以按 RB 做头部表演。", flush=True)

    def drop_head() -> None:
        nonlocal head, drive_ready, homing
        with head_lock:
            current = head
            head = None
            drive_ready = False
            homing = False
        if current is not None:
            try:
                current.release()
            except Exception as exc:
                print(f"头部卸力失败：{exc}", flush=True)

    def begin_drive_head() -> None:
        nonlocal homing

        def _run() -> None:
            nonlocal drive_ready, homing, head
            with head_lock:
                current = head
            if current is None:
                homing = False
                return
            try:
                current.hold_current(
                    _meas_deg(bus, HEAD_PITCH_ID, stand.get(HEAD_PITCH_ID, 0.0)),
                    _meas_deg(bus, NECK_PITCH_ID, stand.get(NECK_PITCH_ID, 0.0)),
                )
            except Exception as exc:
                print(f"头部回到默认位失败：{exc}", flush=True)
            if cancel_home:
                try:
                    current.release()
                except Exception as exc:
                    print(f"头部卸力失败：{exc}", flush=True)
                with head_lock:
                    if head is current:
                        head = None
                    drive_ready = False
                    homing = False
                return
            with head_lock:
                if head is not current:
                    homing = False
                    return
                drive_ready = True
                homing = False

        homing = True
        threading.Thread(target=_run, name="head-home", daemon=True).start()

    try:
        while True:
            lines = link.read()
            if lines is None:
                break
            now = time.monotonic()
            dt = max(0.001, min(0.05, now - prev))
            prev = now
            for line in lines:
                parts = line.split()
                if not parts:
                    continue
                if parts[0] == "P" and len(parts) >= 2:
                    phase = parts[1]
                elif parts[0] == "I" and len(parts) >= 2:
                    imu_ok = parts[1] == "1" and len(parts) >= 8
                    if imu_ok:
                        gyro = (float(parts[2]), float(parts[3]), float(parts[4]))
                        gravity = (float(parts[5]), float(parts[6]), float(parts[7]))
                elif parts[0] == "G" and len(parts) >= 2:
                    g_on = parts[1] == "1"
                    if g_on and len(parts) >= 4:
                        g_yaw = float(parts[2])
                        g_roll = float(parts[3])
                elif parts[0] == "M" and len(parts) >= 6:
                    can = int(parts[1])
                    if can in bus.motors:
                        state = bus.motors[can].state
                        state.position = float(parts[2])
                        state.velocity = float(parts[3])
                        state.torque = float(parts[4])
                        age = int(float(parts[5]))
                        state.t_pos = 0.0 if age < 0 or age > 400 else now - age / 1000.0

            if phase == "wait":
                if homing:
                    cancel_home = True
                elif drive_ready or head is None:
                    drop_head()
                    with head_lock:
                        head = HeadLook(mouth, stand)
                        cancel_home = False
            elif phase == "drive":
                if not homing and not drive_ready:
                    if head is None:
                        with head_lock:
                            head = HeadLook(mouth, stand)
                            cancel_home = False
                    begin_drive_head()
            elif phase in ("stop", "exit"):
                if homing:
                    cancel_home = True
                elif head is not None:
                    drop_head()

            fwd, turn, spin, _boost, _height, lt_on, rt_on = pad.poll()
            bogus = Gamepad.sticks_look_bogus(fwd, turn, spin)
            if bogus:
                pad.button_a = False
                pad.button_x = False
                pad.button_rb = False
            hat_y = pad.axis[7] if len(pad.axis) > 7 else 0.0
            if not bogus and not lt_on and volume.apply_hat(hat_y, now):
                print(f"音量 {volume.percent():.0f}%", flush=True)
            if pad.button_x and not bogus:
                pad.button_x = False
                print(f"灯效：{led.cycle()}", flush=True)
            led.drain()

            with head_lock:
                current = head
            if current is not None and pad.button_rb and not bogus and not homing and phase in ("wait", "drive"):
                pad.button_rb = False
                if phase == "wait":
                    print("腿保持卸力，只动头部。", flush=True)
                current.toggle()
            elif pad.button_rb:
                pad.button_rb = False

            looking = current is not None and current.active()
            rt = 0.0 if bogus or phase != "drive" or looking else max(pad.rt_level(), 1.0 if rt_on else 0.0)
            if looking:
                mouth_was_down = False
            elif rt >= 0.35:
                if not mouth_was_down:
                    mouth_off_at = 0.0
                    mouth.set_level(1.0)
                    mouth.quack("short")
                    mouth_was_down = True
                    rt_down_at = now
                    long_quack_sent = False
                    print("右扳机：张嘴，短叫。", flush=True)
                elif not long_quack_sent and now - rt_down_at >= 3.0:
                    if mouth.quack("long"):
                        long_quack_sent = True
                        print("右扳机按住 3 秒：长鸣。", flush=True)
            elif rt < 0.2 and mouth_was_down:
                mouth.set_level(0.0)
                mouth_was_down = False
                mouth_off_at = now + 0.35
            if mouth_off_at > 0.0 and now >= mouth_off_at and not mouth_was_down and not looking:
                mouth.torque(False)
                mouth_off_at = 0.0

            a_rise = False
            if pad.button_a:
                pad.button_a = False
                a_rise = not bogus and phase == "drive" and not homing
            lb_now = pad.lb_held() and not bogus and phase == "drive" and not homing
            lb_rise = lb_now and not lb_prev
            lb_prev = pad.lb_held() and phase == "drive"

            skill = None
            if phase in ("stop", "exit") or g_on:
                if roulade.active:
                    roulade.update(dt, gravity, gyro, bus, {"yaw": 0.0, "roll": 0.0}, arm=True)
            elif not imu_ok:
                if a_rise and not imu_warned:
                    print("没有 IMU，前左翻滚不能跑。", flush=True)
                    imu_warned = True
            elif current is not None and not homing and phase == "drive" and (a_rise or roulade.active):
                if a_rise and not roulade.ready:
                    print(roulade.status_line(), flush=True)
                else:
                    servo_now = current.read_yaw_roll()
                    skill = roulade.update(dt, gravity, gyro, bus, servo_now, arm=a_rise)

            skill_on = roulade.active
            if current is not None and not homing:
                if g_on:
                    current.track_yaw_roll(g_yaw, g_roll)
                elif roulade.active:
                    current.track_yaw_roll(roulade.servo_deg["yaw"], roulade.servo_deg["roll"])
                elif phase in ("wait", "drive"):
                    current.tick(dt)
            if was_skill and not skill_on and current is not None and not homing:
                current.return_to_drive()
            was_skill = skill_on

            reply = ["W 0"]
            if current is not None:
                reply.append(
                    f"F {float(current.fb.get('yaw', 0.0)):.3f} {float(current.fb.get('roll', 0.0)):.3f}"
                )
            if skill:
                for can, deg in skill.items():
                    tq = NECK_TQ if can in (HEAD_PITCH_ID, NECK_PITCH_ID) else SKILL_TQ
                    reply.append(f"J {can:.0f} {float(deg):.3f} {SKILL_VEL:.2f} {tq:.2f}")
                reply.append("N 0")
            elif current is not None and not homing and (
                (phase == "drive" and (drive_ready or current.active()))
                or (phase == "wait" and current.active())
            ):
                neck = current.can_deg()
                vel = max(1.2, math.radians(current.rate) * 1.4)
                reply.append(
                    "N 1 "
                    f"{neck.get(HEAD_PITCH_ID, 0.0):.3f} "
                    f"{neck.get(NECK_PITCH_ID, 0.0):.3f} "
                    f"{vel:.2f} {NECK_TQ:.2f}"
                )
            else:
                reply.append("N 0")

            torques = [abs(bus.motors[i].state.torque) for i in (1, 3, 6, 8)]
            led.set_torque_level(max(torques) / max(SKILL_TQ, 3.0))
            raw.sendall(("\n".join(reply) + "\n.\n").encode())
    finally:
        drop_head()
        try:
            mouth.close()
        except Exception:
            pass
        try:
            led.close()
        except Exception:
            pass
        raw.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
