#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""进程序先站到采集的站立角，空格才执行策略。

再按空格停止策略并回到站立。Ctrl+C 也先回到站立再退出，电机保持出力。
腿和颈、头 pitch 的力矩保护是 1.2 N·m。头 yaw/roll 额定 0.3，不加。

用法（在 locomotion/ 下，用带 onnxruntime 的解释器）:
  python3 scripts/watch_policy.py
"""
from __future__ import annotations

import argparse
import math
import os
import select
import signal
import struct
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

# kp 与文档 2.4 一致。力矩保护：腿、颈、头 pitch 为 1.2 N·m；头 yaw/roll 额定 0.3。
# 同一拍采样 IMU 和关节。指令再经 0.06 s 平滑，去掉高频。
# 腿的动作 1.2 倍。仍相对上一拍限速，不跟编码器跑，力矩顶住 1.2 N·m。
# 站立前倾过多。两侧髋 pitch 只留 0.02 rad 的前移，观测里减掉同样的量。
KP = (60, 60, 60, 60, 60, 20, 20, 8, 8, 60, 60, 60, 60, 60)
TAU = (1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 0.3, 0.3, 1.2, 1.2, 1.2, 1.2, 1.2)
STEP_SCALE = 2.0
LEG_GAIN = 1.2
CMD_TAU = 0.06
HIP_PITCH_BIAS = 0.02
# 下蹲时左腿 roll/pitch/小腿的编码器与右腿反向，URDF 左右却同向。观测和指令一起取反。
# 右踝原来和左踝相反：前进时左脚往下踩、右脚往上提。右踝改成和左踝同一个符号。
# 髋 yaw 不翻。
ENC_SIGN = (1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, -1)
# 满杆 ±0.5 直接写入步频太高；0.6 倍大约 ±0.30。再放到 0.8 倍，满杆约 ±0.40。
POLICY_VX_SCALE = 0.8

# 策略下标，电机号，HOME，SCALE，URDF 下限/上限 rad，是否腿。
# HOME 与 htdk_policy_joint_order copy.md 的训练站立零位一致。
# 腿用 HOME+scale*action 后再裁切。颈和头锁在 HOME，不跟动作。
JOINTS = (
    ("l_hip_yaw", 10, 0.0, 0.25, -0.436, 0.176, True),
    ("l_hip_roll", 9, 0.0, 0.10, -0.384, 0.080, True),
    ("l_hip_pitch", 8, 0.68, 0.50, -1.571, 1.571, True),
    ("l_calf", 7, -0.88, 0.50, -1.571, 1.571, True),
    ("l_ankle", 6, 0.21, 0.40, -1.571, 1.571, True),
    ("neck_pitch", 12, 0.52, 0.40, -1.200, 1.571, False),
    ("head_pitch", 11, -0.50, 0.40, -1.240, 1.571, False),
    ("head_yaw", 14, 0.0, 0.50, -2.967, 2.967, False),
    ("head_roll", 13, 0.0, 0.20, -0.436, 0.436, False),
    ("r_hip_yaw", 5, 0.0, 0.25, -0.176, 0.436, True),
    ("r_hip_roll", 4, 0.0, 0.10, -0.080, 0.384, True),
    ("r_hip_pitch", 3, 0.68, 0.50, -1.571, 1.571, True),
    ("r_calf", 2, -0.88, 0.50, -1.571, 1.571, True),
    ("r_ankle", 1, 0.21, 0.40, -1.571, 1.571, True),
)

DEFAULT_ONNX = (
    REPO
    / "locomotion"
    / "logs"
    / "rsl_rl"
    / "velocity"
    / "2026-09-28_02-51-17_velocity"
    / "model_3500.onnx"
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

    站立采集角当作训练 HOME 对应的实机姿势。
    电机指令 = 站立采集角 + 符号 × (q_des - HOME)。颈和头的 q_des 锁在 HOME，所以停在采集角。
    """
    cmd = []
    clipped = []
    for i, (_name, can, home, scale, lo, hi, leg) in enumerate(JOINTS):
        raw = home + LEG_GAIN * scale * float(action[i]) if leg else home
        if i in (2, 11):
            raw += HIP_PITCH_BIAS
        limited = min(hi, max(lo, raw))
        clipped.append(raw < lo - 1e-6 or raw > hi + 1e-6)
        cmd.append(stand_deg[can] + ENC_SIGN[i] * math.degrees(limited - home))
    return cmd, clipped


def build_obs(gyro, gravity, q_rad, q_dot_prev, last_action, stand_deg: dict[int, float], vx: float) -> np.ndarray:
    obs = np.zeros(61, dtype=np.float32)
    obs[0:3] = gyro
    obs[3:6] = gravity
    # 采集站立角对应训练 HOME。joint_pos = 符号 × (编码器角 - 采集角)。
    for i, (_name, can, _home, _scale, _lo, _hi, _leg) in enumerate(JOINTS):
        pos = ENC_SIGN[i] * (q_rad[i] - math.radians(stand_deg[can]))
        if i in (2, 11):
            pos -= HIP_PITCH_BIAS
        obs[6 + i] = pos
    obs[20:34] = q_dot_prev
    obs[34:48] = last_action
    # 摇杆向前时实机往后退，写入策略前取反，再乘 0.8。满杆约 ±0.40。
    # 打印仍是摇杆方向和满杆 ±0.5。
    obs[48] = -vx * POLICY_VX_SCALE
    return obs


class Stick:
    """左摇杆前后 → 机头方向速度，满杆 ±0.5 m/s。死区 0.12，约 0.1 s 低通。"""

    PATH = "/dev/input/js0"
    DEADZONE = 0.12
    VX_MAX = 0.5
    TAU = 0.1

    def __init__(self) -> None:
        self.axis = [0.0] * 8
        self.vx = 0.0
        self.fd = -1
        try:
            self.fd = os.open(self.PATH, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            print(f"没有打开 {self.PATH}：{exc}。前后指令保持 0。", file=sys.stderr)

    def poll(self, dt: float, enabled: bool) -> float:
        if self.fd >= 0:
            while True:
                try:
                    raw = os.read(self.fd, 8)
                except BlockingIOError:
                    break
                if len(raw) < 8:
                    break
                _time, value, kind, number = struct.unpack("IhBB", raw)
                if (kind & ~0x80) == 0x02 and number < 8:
                    self.axis[number] = max(-1.0, min(1.0, value / 32767.0))
        ly = -self.axis[1]
        if abs(ly) < self.DEADZONE:
            ly = 0.0
        target = ly * self.VX_MAX if enabled else 0.0
        target = max(-self.VX_MAX, min(self.VX_MAX, target))
        alpha = dt / (self.TAU + dt) if dt > 0 else 1.0
        self.vx += alpha * (target - self.vx)
        if abs(self.vx) < 1e-3:
            self.vx = 0.0
        return self.vx

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


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
    return 0.3 if can in (13, 14) else 1.2


class Gravity:
    """陀螺每拍积分。加速度只在接近 1 g 时用 0.5 s 修正，避免震动把目标打回来。"""

    TAU = 0.5

    def __init__(self) -> None:
        self.g: tuple[float, float, float] | None = None

    def update(self, measured, gyro, acc_norm: float, dt: float):
        if self.g is None or dt <= 0 or dt > 0.2:
            self.g = measured
            return self.g
        gx, gy, gz = self.g
        wx, wy, wz = gyro
        nx = gx - dt * (wy * gz - wz * gy)
        ny = gy - dt * (wz * gx - wx * gz)
        nz = gz - dt * (wx * gy - wy * gx)
        n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        nx, ny, nz = nx / n, ny / n, nz / n
        if 7.0 < acc_norm < 13.0:
            a = 1.0 - math.exp(-dt / self.TAU)
            nx = (1.0 - a) * nx + a * measured[0]
            ny = (1.0 - a) * ny + a * measured[1]
            nz = (1.0 - a) * nz + a * measured[2]
            n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            nx, ny, nz = nx / n, ny / n, nz / n
        self.g = (nx, ny, nz)
        return self.g


def smooth_command(target: list[float], prev: list[float], dt: float) -> list[float]:
    """去掉策略目标里的高频。平滑后的角度仍由上一拍指令限速，不跟实测角跑。"""
    alpha = 1.0 if dt <= 0.0 or dt > 0.2 else dt / (CMD_TAU + dt)
    return [p + alpha * (g - p) for p, g in zip(prev, target)]


def step_limited(cmd_deg: list[float], prev_deg: list[float]) -> list[float]:
    """本拍指令相对上一拍指令移动，不超过 STEP_SCALE * tau/kp。

    不能相对实测角度限幅。抖动时实测角一跳，目标就跟着跑，电机几乎没有位置误差，力矩掉下去。
    """
    out = []
    for i, (_name, _can, _home, _scale, _lo, _hi, _leg) in enumerate(JOINTS):
        cap = math.degrees(TAU[i] / KP[i]) * STEP_SCALE
        cur = prev_deg[i]
        out.append(min(cur + cap, max(cur - cap, cmd_deg[i])))
    return out


def send_degrees(bus, ids: list[int], degrees: dict[int, float]) -> int:
    """发出位置指令。缓冲满时先排空再试，仍失败就跳过该电机，不让控制环退出。"""
    dropped = 0
    for n, mid in enumerate(ids):
        if n and n % 4 == 0:
            bus.can._drop_pending()
        motor = bus.motors[mid]
        data = (
            bytes([0x07, 0x07])
            + i16_le(pos_to_int(degrees[mid] * pose.DEG))
            + bytes([0x00, 0x00])
            + i16_le(tqe_to_int(torque_of(mid), motor.type))
        )
        try:
            bus.send_cmd(motor.id, data, reply=False)
        except OSError as exc:
            if not can_send_failed(exc):
                raise
            bus.can._drop_pending()
            try:
                bus.send_cmd(motor.id, data, reply=False)
            except OSError as again:
                if not can_send_failed(again):
                    raise
                dropped += 1
    return dropped


def can_send_failed(exc: OSError) -> bool:
    """缓冲满、总线掉线都不当成致命错误，避免抖动丢帧时把控制环打掉。"""
    return exc.errno in (19, 100, 105) or "缓冲" in str(exc)


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
    parser = argparse.ArgumentParser(description="先站立，空格开始或停止策略")
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
    print("进程序先站立，力矩保护 1.20 N·m。站稳后按空格执行策略，再按一次回到站立。")
    print("摇杆向前为前进。指令平滑 0.06 s。腿部动作 1.2 倍。")
    print("执行策略时左摇杆前后仍是 ±0.50 m/s。写入策略为 0.8 倍，满杆约 ±0.40。未执行时指令为 0。")
    print("Ctrl+C 先回到站立再退出，电机保持出力。头 yaw/roll 仍是 0.30 N·m。")

    lib = lock_fd = imu = None
    keys = None
    stick = None
    motors_on = False
    running = True
    leaving = False

    def stop(*_):
        nonlocal running, leaving
        if leaving:
            running = False
        else:
            leaving = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        keys = Keys()
        if not keys.ok:
            print("标准输入不是终端，读不到空格。", file=sys.stderr)
        lib, lock_fd, imu = open_imu()
        pose.read_degrees(bus, ids)
        for mid in ids:
            bus.enable(mid)
            bus.set_torque_limit(mid, torque_of(mid))
        motors_on = True
        period = 1.0 / 50.0
        phase = "stand"
        stand_cmd = [stand[can] for (_name, can, _home, _scale, _lo, _hi, _leg) in JOINTS]
        held: list[float] | None = None
        shaped: list[float] | None = None
        last_space = 0.0
        space_edge = False
        leave_t0 = 0.0
        print("电机已使能，正在走向站立。", flush=True)

        print("目标角和电机角都是编码器角度：站立采集角 + (限位后的 URDF 目标 - HOME)。")
        print("空格下发的是限步后的这个角度。红字是电机还没跟上的角度。")
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
        can_drop = 0
        stick = Stick()
        gravity_filt = Gravity()
        prev_tick = time.monotonic()
        while running:
            tick = time.monotonic()
            if leaving and leave_t0 == 0.0:
                leave_t0 = tick
                phase = "stand"
                print("Ctrl+C：回到站立后退出，电机保持出力。再按一次立即退出。", flush=True)
            if not leaving and keys.poll() == " " and tick - last_space > 0.3:
                last_space = tick
                space_edge = True
            bus.can._drop_pending()
            try:
                refresh_motors(bus, ids, ids[0])
            except OSError as exc:
                if not can_send_failed(exc):
                    raise
                can_drop += 1
                bus.can._drop_pending()
            now_deg = {mid: bus.motors[mid].state.position / pose.DEG for mid in ids}
            raw_imu = imu.read_raw()
            t_sample = time.monotonic()
            dt = max(0.0, t_sample - prev_tick)
            prev_tick = t_sample
            q_rad = np.zeros(14, dtype=np.float64)
            q_dot = np.zeros(14, dtype=np.float32)
            for i, (_name, can, _home, _scale, _lo, _hi, _leg) in enumerate(JOINTS):
                q_rad[i] = now_deg[can] * pose.DEG
                q_dot[i] = ENC_SIGN[i] * bus.motors[can].state.velocity
            sample = raw_imu.to_policy()
            acc_n = 9.81 * math.sqrt(sum(c * c for c in raw_imu.acc_g))
            gravity = gravity_filt.update(sample.gravity, sample.gyro, acc_n, dt)
            vx = stick.poll(dt, phase == "track")
            obs = build_obs(sample.gyro, gravity, q_rad, q_dot_prev, last_action, stand, vx)
            t_infer = time.monotonic()
            action = session.run(None, {in_name: obs.reshape(1, 61)})[0][0]
            infer_us = (time.monotonic() - t_infer) * 1e6
            loops += 1
            last_action = np.asarray(action, dtype=np.float32).copy()
            q_dot_prev = q_dot
            q_cmd_deg, clipped = encoder_command_deg(action, stand)
            stand_err_now = max(abs(now_deg[mid] - stand[mid]) for mid in ids)
            if leaving:
                phase = "stand"
                space_edge = False
            if space_edge:
                space_edge = False
                if phase == "track":
                    phase = "stand"
                    print("空格：停止策略，回到站立，力矩保持", flush=True)
                else:
                    tilt = gravity_tilt_deg(gravity)
                    if stand_err_now > math.degrees(0.35) or tilt > 8.0:
                        print(
                            f"空格未执行：还在站立，离目标 {stand_err_now:.1f}°，重力偏 {tilt:.1f}°。",
                            flush=True,
                        )
                    else:
                        phase = "track"
                        print("空格：开始执行策略。再按一次回到站立。", flush=True)
            target = q_cmd_deg if phase == "track" else stand_cmd
            if held is None:
                held = [now_deg[can] for (_name, can, _home, _scale, _lo, _hi, _leg) in JOINTS]
                shaped = list(held)
            shaped = smooth_command(target, shaped, dt)
            held = step_limited(shaped, held)
            limited = held
            can_drop += send_degrees(bus, ids, {can: limited[i] for i, (_n, can, *_r) in enumerate(JOINTS)})
            if leaving and (stand_err_now < 4.0 or tick - leave_t0 > 8.0):
                if stand_err_now < 4.0:
                    for _ in range(8):
                        send_degrees(bus, ids, stand)
                        time.sleep(period)
                print(f"已退出，停在站立方向，偏差 {stand_err_now:.1f}°，电机继续保持出力。", flush=True)
                break
            if tick >= next_print:
                elapsed = max(tick - window_t, 1e-6)
                loop_hz = loops / elapsed
                window_t = tick
                loops = 0
                hold_err = max(abs(now_deg[mid] - stand[mid]) for mid in ids)
                bus_v = bus.motors[ids[0]].state.voltage
                if phase == "track":
                    state = "正在执行策略，按空格回到站立"
                elif hold_err > 5.0:
                    state = "正在站立，站稳后再按空格"
                else:
                    state = "已站立，按空格执行策略"
                print(
                    f"电机执行: {state}  循环 {loop_hz:.1f} Hz  推理 {infer_us:.0f} us  "
                    f"力矩保护 1.20 N·m  电压 {bus_v:.1f} V  "
                    f"stand偏差 {hold_err:.2f}°"
                    + (f"  CAN丢帧 {can_drop}" if can_drop else "")
                )
                can_drop = 0
                print(
                    f"gravity {gravity[0]:+.3f} {gravity[1]:+.3f} {gravity[2]:+.3f}  "
                    f"gyro {sample.gyro[0]:+.3f} {sample.gyro[1]:+.3f} {sample.gyro[2]:+.3f} rad/s  "
                    f"指令前后 {vx:+.2f} m/s"
                )
                for i, (name, can, _home, _scale, lo, hi, leg) in enumerate(JOINTS):
                    pol = q_cmd_deg[i]
                    cur = now_deg[can]
                    note = ""
                    if clipped[i]:
                        note = f"已裁到 {math.degrees(lo):+.1f}°~{math.degrees(hi):+.1f}°"
                    elif not leg:
                        note = "锁 HOME，不跟策略"
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
        if stick is not None:
            stick.close()
        if keys is not None:
            keys.close()
        if lib is not None:
            lib.i2c_gpio_close()
        if lock_fd is not None:
            os.close(lock_fd)
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
