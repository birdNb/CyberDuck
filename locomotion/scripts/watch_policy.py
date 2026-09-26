#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""站立附近看策略，空格才把策略角度写进电机。

腿跟随限位并限步后的策略目标。颈和头锁在站立采集角。
再按空格停在当前角度并保持力矩，不卸力。Ctrl+C 退出时失能。

用法（在 locomotion/ 下，用带 onnxruntime 的解释器）:
  python3 scripts/watch_policy.py
"""
from __future__ import annotations

import argparse
import math
import os
import select
import signal
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "motor"))
sys.path.insert(0, str(REPO / "motor" / "pose_contrl"))
sys.path.insert(0, str(REPO / "IMU"))

import pose  # noqa: E402
from ht_can import i16_le, pos_to_int, tqe_to_int  # noqa: E402
from lsm6dsv16x import I2CGpioError, LSM6DSV16X, WHO_AM_I_VAL, ADDRS, load_lib, open_bus, scan_bus  # noqa: E402

# 与 htdk_sim2real.md 2.4 一致。头 yaw/roll 力矩不超过 0.3。
KP = (60, 60, 60, 60, 60, 20, 20, 8, 8, 60, 60, 60, 60, 60)
TAU = (0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.3, 0.3, 0.5, 0.5, 0.5, 0.5, 0.5)

# 策略下标，电机号，HOME，SCALE，URDF 下限/上限 rad，是否腿。
# 限位与 htdk_sim2real.md 第 2.3 节一致。腿用 HOME+scale*action 后再裁切，颈和头目标固定 0。
JOINTS = (
    ("l_hip_yaw", 10, 0.0, 0.25, -0.436, 0.176, True),
    ("l_hip_roll", 9, 0.0, 0.10, -0.384, 0.080, True),
    ("l_hip_pitch", 8, 0.175, 0.50, -1.571, 1.571, True),
    ("l_calf", 7, 0.0, 0.50, -1.571, 1.571, True),
    ("l_ankle", 6, -0.175, 0.40, -1.571, 1.571, True),
    ("neck_pitch", 12, 0.0, 0.40, -1.200, 1.571, False),
    ("head_pitch", 11, 0.0, 0.40, -1.240, 1.571, False),
    ("head_yaw", 14, 0.0, 0.50, -2.967, 2.967, False),
    ("head_roll", 13, 0.0, 0.20, -0.436, 0.436, False),
    ("r_hip_yaw", 5, 0.0, 0.25, -0.176, 0.436, True),
    ("r_hip_roll", 4, 0.0, 0.10, -0.080, 0.384, True),
    ("r_hip_pitch", 3, 0.175, 0.50, -1.571, 1.571, True),
    ("r_calf", 2, 0.0, 0.50, -1.571, 1.571, True),
    ("r_ankle", 1, -0.175, 0.40, -1.571, 1.571, True),
)

DEFAULT_ONNX = (
    REPO
    / "locomotion"
    / "logs"
    / "rsl_rl"
    / "velocity"
    / "2026-09-27_01-50-18_velocity"
    / "2026-09-27_01-50-18_velocity.onnx"
)


class Keys:
    """终端里读单个按键，不用回车。"""

    def __init__(self) -> None:
        self.ok = sys.stdin.isatty()
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd) if self.ok else None
        if self.ok:
            tty.setcbreak(self.fd)

    def poll(self) -> str:
        if not self.ok or not select.select([sys.stdin], [], [], 0)[0]:
            return ""
        return sys.stdin.read(1)

    def close(self) -> None:
        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            self.old = None


def open_imu():
    import fcntl

    lock_fd = os.open("/tmp/microduck-imu-i2c6.lock", os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(lock_fd)
        raise I2CGpioError("IMU 被占用")
    os.write(lock_fd, f"{os.getpid()}\n".encode())
    lib = load_lib()
    open_bus(lib, pullup=False)
    found = scan_bus(lib)
    addrs = [a for a in found if a in ADDRS] or list(ADDRS)
    imu = None
    for addr in addrs:
        candidate = LSM6DSV16X(lib, addr)
        try:
            who = candidate.who_am_i()
        except I2CGpioError:
            continue
        if who == WHO_AM_I_VAL:
            candidate.init()
            imu = candidate
            break
    if imu is None:
        lib.i2c_gpio_close()
        os.close(lock_fd)
        raise I2CGpioError("未找到 LSM6DSV16X")
    return lib, lock_fd, imu


def encoder_command_deg(action: np.ndarray, stand_deg: dict[int, float]) -> tuple[list[float], list[bool]]:
    """策略角先在 URDF 里限位，再换回编码器角度。

    站立采集角当作站立姿势。电机指令 = 站立采集角 + (q_des - HOME)。
    颈和头的 q_des 锁在 0，HOME 也是 0，所以指令停在站立采集角，不会收到编码器 0。
    """
    cmd = []
    clipped = []
    for i, (_name, can, home, scale, lo, hi, leg) in enumerate(JOINTS):
        raw = home + scale * float(action[i]) if leg else 0.0
        limited = min(hi, max(lo, raw))
        clipped.append(raw < lo - 1e-6 or raw > hi + 1e-6)
        cmd.append(stand_deg[can] + math.degrees(limited - home))
    return cmd, clipped


def build_obs(gyro, gravity, q_rad, q_dot_prev, last_action, stand_deg: dict[int, float]) -> np.ndarray:
    obs = np.zeros(61, dtype=np.float32)
    obs[0:3] = gyro
    obs[3:6] = gravity
    for i, (_name, can, _home, _scale, _lo, _hi, _leg) in enumerate(JOINTS):
        obs[6 + i] = q_rad[i] - math.radians(stand_deg[can])
    obs[20:34] = q_dot_prev
    obs[34:48] = last_action
    return obs


def yellow_nm(nm: float) -> str:
    return f"\033[33m{nm:+6.3f}\033[0m"


def red_diff(deg: float) -> str:
    text = f"{deg:+7.2f}°"
    if abs(deg) >= 10.0:
        return f"\033[1;31m{text}\033[0m"
    if abs(deg) >= 3.0:
        return f"\033[31m{text}\033[0m"
    return text


def torque_of(can: int) -> float:
    return 0.3 if can in (13, 14) else 0.5


def step_limited(cmd_deg: list[float], now_deg: dict[int, float]) -> list[float]:
    """本拍下发不超过 tau/kp，避免一拍顶满。"""
    out = []
    for i, (_name, can, _home, _scale, _lo, _hi, _leg) in enumerate(JOINTS):
        cap = math.degrees(TAU[i] / KP[i])
        cur = now_deg[can]
        out.append(min(cur + cap, max(cur - cap, cmd_deg[i])))
    return out


def send_degrees(bus, ids: list[int], degrees: dict[int, float]) -> None:
    for mid in ids:
        motor = bus.motors[mid]
        data = (
            bytes([0x07, 0x07])
            + i16_le(pos_to_int(degrees[mid] * pose.DEG))
            + bytes([0x00, 0x00])
            + i16_le(tqe_to_int(torque_of(mid), motor.type))
        )
        bus.send_cmd(motor.id, data, reply=False)


def gravity_tilt_deg(gravity) -> float:
    dot = max(-1.0, min(1.0, -float(gravity[2])))
    return math.degrees(math.acos(dot))


def refresh_motors(bus, ids: list[int], volt_id: int) -> None:
    """状态查询限在几毫秒内，避免把 50 Hz 堵成 10 Hz。电压每拍只问一个电机。"""
    for mid in ids:
        bus.read_status(mid)
    bus.collect(timeout=0.004)
    bus.read_voltage(volt_id)
    bus.collect(timeout=0.001)


def main() -> int:
    parser = argparse.ArgumentParser(description="空格执行策略输出，再按一次停在当前角度")
    parser.add_argument("--onnx", default=str(DEFAULT_ONNX))
    parser.add_argument("--print-every", type=float, default=1.0)
    args = parser.parse_args()

    import onnxruntime as ort

    cfg = pose.load_config(pose.CONFIG_PATH)
    spec = pose.pose_spec(cfg, "stand")
    joints = spec.get("joints") or {}
    if not joints:
        print("stand 还没有角度", file=sys.stderr)
        return 1
    stand = {int(k): float(v) for k, v in joints.items()}
    ids = [i for i in range(1, 15)]
    missing = [mid for mid in ids if mid not in stand]
    if missing:
        print(f"stand 缺少电机 {missing}", file=sys.stderr)
        return 1

    bus, ids = pose.open_bus(cfg)
    session = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    in_name = session.get_inputs()[0].name
    print(f"策略 {args.onnx}")
    print(f"输入 {in_name} {session.get_inputs()[0].shape}  输出 {[o.name for o in session.get_outputs()]}")
    print("刚启动不出力。按空格执行策略，再按一次停在当前角度并保持力矩，不卸力。")
    print("腿力矩 0.50 N·m，头 yaw/roll 0.30 N·m。颈和头不跟策略，停在站立采集角。")

    lib = lock_fd = imu = None
    keys = None
    motors_on = False
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        keys = Keys()
        if not keys.ok:
            print("标准输入不是终端，读不到空格。", file=sys.stderr)
        lib, lock_fd, imu = open_imu()
        pose.read_degrees(bus, ids)
        period = 1.0 / 50.0
        phase = "idle"
        hold_deg = {mid: stand[mid] for mid in ids}
        last_space = 0.0
        space_edge = False

        print("目标角和电机角都是编码器角度：站立采集角 + (限位后的 URDF 目标 - HOME)。")
        print("空格下发的是限步后的这个角度。红字是策略还想再走的角度。")
        print(
            f"{'idx':>3} {'关节':<12} {'电机':>4} {'动作':>8} {'策略角':>8} {'电机角':>8} "
            f"{'力矩Nm':>8} {'角度差':>10}  备注"
        )
        last_action = np.zeros(14, dtype=np.float32)
        q_dot_prev = np.zeros(14, dtype=np.float32)
        window_t = time.monotonic()
        next_print = window_t + args.print_every
        loops = 0
        infer_us = 0.0
        volt_i = 0
        while running:
            tick = time.monotonic()
            if keys.poll() == " " and tick - last_space > 0.3:
                last_space = tick
                space_edge = True
            refresh_motors(bus, ids, ids[volt_i % len(ids)])
            volt_i += 1
            now_deg = {mid: bus.motors[mid].state.position / pose.DEG for mid in ids}
            q_rad = np.zeros(14, dtype=np.float64)
            q_dot = np.zeros(14, dtype=np.float32)
            for i, (_name, can, _home, _scale, _lo, _hi, _leg) in enumerate(JOINTS):
                q_rad[i] = now_deg[can] * pose.DEG
                q_dot[i] = bus.motors[can].state.velocity
            sample = imu.read_raw().to_policy()
            obs = build_obs(sample.gyro, sample.gravity, q_rad, q_dot_prev, last_action, stand)
            t_infer = time.monotonic()
            action = session.run(None, {in_name: obs.reshape(1, 61)})[0][0]
            infer_us = (time.monotonic() - t_infer) * 1e6
            loops += 1
            last_action = np.asarray(action, dtype=np.float32).copy()
            q_dot_prev = q_dot
            q_cmd_deg, clipped = encoder_command_deg(action, stand)
            if space_edge:
                space_edge = False
                if phase == "track":
                    hold_deg = dict(now_deg)
                    phase = "hold"
                    print("空格：停止跟策略，停在当前角度，力矩保持", flush=True)
                else:
                    tilt = gravity_tilt_deg(sample.gravity)
                    stand_err = max(abs(now_deg[mid] - stand[mid]) for mid in ids)
                    if tilt > 8.0 or stand_err > math.degrees(0.35):
                        print(
                            f"空格未执行：重力偏 {tilt:.1f}°，离站立 {stand_err:.1f}°。"
                            f"先摆回站立附近再按。",
                            flush=True,
                        )
                    else:
                        for mid in ids:
                            bus.enable(mid)
                            bus.set_torque_limit(mid, torque_of(mid))
                        motors_on = True
                        phase = "track"
                        print("空格：开始执行策略输出。再按一次停在当前角度。", flush=True)
            if phase == "track":
                limited = step_limited(q_cmd_deg, now_deg)
                send_degrees(bus, ids, {can: limited[i] for i, (_n, can, *_r) in enumerate(JOINTS)})
            elif phase == "hold":
                send_degrees(bus, ids, hold_deg)
            if tick >= next_print:
                elapsed = max(tick - window_t, 1e-6)
                loop_hz = loops / elapsed
                window_t = tick
                loops = 0
                hold_err = max(abs(now_deg[mid] - stand[mid]) for mid in ids)
                volts = {mid: bus.motors[mid].state.voltage for mid in ids}
                known = [v for v in volts.values() if v > 0.5]
                vmin = min(known) if known else 0.0
                vmax = max(known) if known else 0.0
                volt_txt = " ".join(f"{mid}:{volts[mid]:.1f}V" for mid in ids)
                state = {
                    "idle": "未出力，按空格执行策略",
                    "track": "正在执行策略，按空格停住",
                    "hold": "停在当前角度，按空格继续执行策略",
                }[phase]
                print(
                    f"电机执行: {state}  循环 {loop_hz:.1f} Hz  推理 {infer_us:.0f} us  "
                    f"腿力矩 0.50 N·m  电压 {vmin:.1f}–{vmax:.1f} V  "
                    f"stand偏差 {hold_err:.2f}°"
                )
                print(f"电压 {volt_txt}")
                print(
                    f"gravity {sample.gravity[0]:+.3f} {sample.gravity[1]:+.3f} {sample.gravity[2]:+.3f}  "
                    f"gyro {sample.gyro[0]:+.3f} {sample.gyro[1]:+.3f} {sample.gyro[2]:+.3f} rad/s"
                )
                for i, (name, can, _home, _scale, lo, hi, leg) in enumerate(JOINTS):
                    pol = q_cmd_deg[i]
                    cur = now_deg[can]
                    note = ""
                    if clipped[i]:
                        note = f"已裁到 {math.degrees(lo):+.1f}°~{math.degrees(hi):+.1f}°"
                    elif not leg:
                        note = "锁 0，不跟策略"
                    print(
                        f"{i:3d} {name:<12} {can:4d} {float(action[i]):+8.3f} {pol:+8.2f} {cur:+8.2f} "
                        f"{yellow_nm(bus.motors[can].state.torque)} {red_diff(pol - cur)}  {note}"
                    )
                print(flush=True)
                next_print = tick + args.print_every
            spent = time.monotonic() - tick
            if spent < period:
                time.sleep(period - spent)
        return 0
    finally:
        if motors_on:
            for mid in ids:
                bus.disable(mid)
        if keys is not None:
            keys.close()
        if lib is not None:
            lib.i2c_gpio_close()
        if lock_fd is not None:
            os.close(lock_fd)
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
