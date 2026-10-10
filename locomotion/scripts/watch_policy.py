#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""进程序先走到 pose_config.yaml 里的站立角，空格才执行策略。

再按空格停止策略并回到站立。Ctrl+C 也先回到站立再退出，电机保持出力。
按 q 立刻卸力并退出。
腿和颈、头 pitch 的力矩保护是 1.2 N·m。头 yaw/roll 额定 0.3，不加。

用法（在 locomotion/ 下，用带 onnxruntime 的解释器）:
  python3 scripts/watch_policy.py
"""
from __future__ import annotations

import argparse
import csv
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

try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except (AttributeError, OSError):
    pass


def say(text: str = "") -> None:
    """直接写到终端。写不进去就丢掉这一行，不能让打印堵住 50 Hz。"""
    data = (text + "\n").encode()
    fd = sys.stdout.fileno()
    try:
        import fcntl

        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        if not flags & os.O_NONBLOCK:
            fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    except OSError:
        pass
    try:
        os.write(fd, data)
    except BlockingIOError:
        return
    except OSError:
        try:
            sys.stdout.write(text + "\n")
            sys.stdout.flush()
        except OSError:
            return

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "motor"))
sys.path.insert(0, str(REPO / "motor" / "pose_contrl"))
sys.path.insert(0, str(REPO / "IMU"))

import pose  # noqa: E402
from ht_can import i16_le, pos_to_int, tqe_to_int  # noqa: E402
from lsm6dsv16x import I2CGpioError, LSM6DSV16X, WHO_AM_I_VAL, ADDRS, load_lib, open_bus, scan_bus  # noqa: E402

# 力矩：腿、颈、头 pitch 为 1.2 N·m；头 yaw/roll 额定 0.3。
# 仿真角和电机角只在读写边界乘一次 SIGN。左腿和头颈为 +1，右腿五个关节全部为 -1。
# 缩放是当前训练值：髋 pitch 0.80，小腿 1.00，踝 0.70。
KP = (60, 60, 60, 60, 60, 20, 20, 8, 8, 60, 60, 60, 60, 60)
TAU = (1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 0.3, 0.3, 1.2, 1.2, 1.2, 1.2, 1.2)
STEP_SCALE = 2.0
SIGN = (1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1)
# 摇杆满杆先映射到训练范围 ±0.4 m/s，再写入策略 ±0.25 m/s。
CMD_SCALE = 0.25 / 0.4
# 必须是 1。小腿 SCALE 已是 1，再乘 1.25 时，位置环增益超过 1：
# 腿弯一点，策略就把目标送到更弯的地方，平放时会自己越蹲越深。
LEG_GAIN = 1.0
# 电机目标低通。约 0.035 s 去掉高频抖动，走路的步频还能通过。
CMD_TAU = 0.035

# 策略下标，电机号，仿真 HOME，SCALE，仿真下限/上限 rad，是否腿。
# 腿用 HOME+SCALE*action 后再裁切。颈和头锁在 HOME，不跟动作。
JOINTS = (
    ("l_hip_yaw", 10, 0.0, 0.25, -0.436, 0.176, True),
    ("l_hip_roll", 9, 0.0, 0.10, -0.384, 0.080, True),
    ("l_hip_pitch", 8, 0.68, 0.80, -1.571, 1.571, True),
    ("l_calf", 7, -0.88, 1.00, -1.571, 1.571, True),
    ("l_ankle", 6, 0.21, 0.70, -1.571, 1.571, True),
    ("neck_pitch", 12, 0.52, 0.40, -1.200, 1.571, False),
    ("head_pitch", 11, -0.50, 0.40, -1.240, 1.571, False),
    ("head_yaw", 14, 0.0, 0.50, -2.967, 2.967, False),
    ("head_roll", 13, 0.0, 0.20, -0.436, 0.436, False),
    ("r_hip_yaw", 5, 0.0, 0.25, -0.176, 0.436, True),
    ("r_hip_roll", 4, 0.0, 0.10, -0.080, 0.384, True),
    ("r_hip_pitch", 3, 0.68, 0.80, -1.571, 1.571, True),
    ("r_calf", 2, -0.88, 1.00, -1.571, 1.571, True),
    ("r_ankle", 1, 0.21, 0.70, -1.571, 1.571, True),
)

DEFAULT_ONNX = (
    REPO
    / "locomotion"
    / "logs"
    / "rsl_rl"
    / "velocity"
    / "2026-09-28_02-51-17_velocity"
    / "model_30700.onnx"
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


def motor_command_deg(action: np.ndarray, stand_deg: dict[int, float]) -> tuple[list[float], list[bool]]:
    """仿真目标先限位。电机角 = 采集站立角 + SIGN × (仿真目标 − HOME)。

    动作是 0、脖子锁在 HOME 时，电机就停在采集的站立角。右腿 SIGN 为 -1。
    """
    cmd = []
    clipped = []
    for i, (_name, can, home, scale, lo, hi, leg) in enumerate(JOINTS):
        raw = home + scale * LEG_GAIN * float(action[i]) if leg else home
        limited = min(hi, max(lo, raw))
        clipped.append(raw < lo - 1e-6 or raw > hi + 1e-6)
        cmd.append(stand_deg[can] + math.degrees(SIGN[i] * (limited - home)))
    return cmd, clipped


def build_obs(gyro, gravity, q_motor_rad, q_dot_prev, last_action, stand_deg: dict[int, float], vx: float) -> np.ndarray:
    obs = np.zeros(61, dtype=np.float32)
    obs[0:3] = gyro
    obs[3:6] = gravity
    # 前倾时实测 gx>0，且低头时 gyro_y>0，与训练约定一致。不要再把 gx 取反，
    # 否则策略把前倾看成后仰，左右髋 yaw、小腿会往相反方向踹。
    for i, (_name, can, home, _scale, _lo, _hi, _leg) in enumerate(JOINTS):
        q_sim = home + SIGN[i] * (q_motor_rad[i] - math.radians(stand_deg[can]))
        obs[6 + i] = q_sim - home
    obs[20:34] = q_dot_prev
    obs[34:48] = last_action
    obs[48] = max(-0.4, min(0.4, vx)) * CMD_SCALE
    obs[49] = 0.0
    obs[50] = 0.0
    return obs


class Stick:
    """只读左摇杆前后。每拍把积压事件读完，只用最新位置，不再做低通。"""

    PATH = "/dev/input/js0"
    DEADZONE = 0.12
    VX_MAX = 0.4
    FORWARD_AXIS = 1
    BTN_A = 0
    BTN_B = 1

    def __init__(self) -> None:
        self.axis = [0.0] * 8
        self.vx = 0.0
        self.cmd = 0.0
        self.raw = 0.0
        self.button = ""
        self.name = ""
        self.fd = -1
        self.warned = False
        self._next_open = 0.0
        self.try_open()

    def try_open(self) -> None:
        if self.fd >= 0:
            return
        now = time.monotonic()
        if now < self._next_open:
            return
        self._next_open = now + 1.0
        try:
            self.fd = os.open(self.PATH, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            if not self.warned:
                say(f"手柄未打开 {self.PATH}：{exc}。插上北通接收器后会自动重试。")
                self.warned = True
            return
        buf = bytearray(128)
        req = (2 << 30) | (128 << 16) | (ord("j") << 8) | 0x13
        try:
            import fcntl

            fcntl.ioctl(self.fd, req, buf, True)
            self.name = buf.split(b"\x00", 1)[0].decode(errors="replace")
        except OSError:
            self.name = self.PATH
        self.warned = False
        say(f"手柄已接入 {self.name}（{self.PATH}）。左摇杆或十字键前后，向前为正。A 启停策略，B 卸力。")

    def drop(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1
        self.axis = [0.0] * 8
        self._next_open = time.monotonic() + 0.5

    def poll(self, dt: float, enabled: bool) -> float:
        del dt
        self.try_open()
        if self.fd >= 0:
            while True:
                try:
                    blob = os.read(self.fd, 8 * 64)
                except BlockingIOError:
                    break
                except OSError:
                    self.drop()
                    break
                if not blob:
                    # 接收器重新枚举后，旧句柄会一直读到空，摇杆停在 0。
                    self.drop()
                    break
                if len(blob) < 8:
                    break
                for off in range(0, len(blob) - 7, 8):
                    _time, value, kind, number = struct.unpack_from("IhBB", blob, off)
                    init = bool(kind & 0x80)
                    kind &= ~0x80
                    if kind == 0x02 and number < 8:
                        self.axis[number] = max(-1.0, min(1.0, value / 32767.0))
                    elif kind == 0x01 and not init and value == 1:
                        if number == self.BTN_B:
                            self.button = "B"
                        elif number == self.BTN_A and self.button != "B":
                            self.button = "A"
        ly = -self.axis[self.FORWARD_AXIS]
        if abs(ly) < self.DEADZONE:
            ly = -self.axis[7]
        if abs(ly) < self.DEADZONE:
            ly = 0.0
        self.raw = ly
        self.cmd = ly * self.VX_MAX
        self.vx = self.cmd if enabled else 0.0
        return self.vx

    def take_button(self) -> str:
        name = self.button
        self.button = ""
        return name

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


class CsvLog:
    """每拍一行，50 Hz。缓冲写入，不跟着终端打印堵住控制环。"""

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.file = path.open("w", newline="", encoding="utf-8")
        names = [j[0] for j in JOINTS]
        cols = ["t_s", "unix_s", "phase", "dt_s", "infer_us"]
        cols += [f"gyro_{a}" for a in "xyz"]
        cols += [f"gravity_{a}" for a in "xyz"]
        cols += [f"meas_gravity_{a}" for a in "xyz"]
        cols += [f"acc_g_{a}" for a in "xyz"]
        cols += ["acc_norm", "roll_deg", "pitch_deg", "tilt_deg"]
        cols += ["stick_raw", "stick_mps", "obs_vx", "obs_vy", "obs_wz"]
        cols += [f"obs_q_{n}" for n in names]
        cols += [f"obs_dq_{n}" for n in names]
        cols += [f"dq_{n}" for n in names]
        cols += [f"last_action_{n}" for n in names]
        cols += [f"action_{n}" for n in names]
        cols += [f"policy_deg_{n}" for n in names]
        cols += [f"cmd_deg_{n}" for n in names]
        cols += [f"motor_deg_{n}" for n in names]
        cols += [f"torque_{n}" for n in names]
        cols += ["voltage_v", "can_drop"]
        self.writer = csv.writer(self.file)
        self.writer.writerow(cols)
        self.rows = 0

    def write(self, values: list[float | str]) -> None:
        self.writer.writerow(values)
        self.rows += 1
        if self.rows % 25 == 0:
            self.file.flush()

    def close(self) -> None:
        self.file.flush()
        self.file.close()


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
    """陀螺每拍积分。加速度只在接近 1 g 时修正。

    0.04 s 会把站立时的加速度噪声写进倾角，策略跟着抖，形成自激。
    0.10 s 仍快于早先的 0.15 s。冲击仍被 7–13 m/s² 的门挡住。
    """

    TAU = 0.10

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


def blend_stand(origin: list[float], goal: list[float], elapsed: float, duration: float) -> list[float]:
    """从当前指令平滑到站立角。duration 秒内走完，两端速度为零。"""
    if duration <= 0.0 or elapsed >= duration:
        return list(goal)
    scale = pose.smootherstep(elapsed / duration)
    return [a + (b - a) * scale for a, b in zip(origin, goal)]


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


def tilt_axes_deg(gravity) -> tuple[float, float, float]:
    """由机身重力向量得到倾斜角。直立时重力约 (0, 0, -1)。

    绕 X 正值是左侧抬起，绕 Y 正值是机头抬起。离竖直是机身朝上轴偏离世界竖直的角度。
    """
    gx, gy, gz = (float(gravity[0]), float(gravity[1]), float(gravity[2]))
    roll = math.degrees(math.atan2(-gy, -gz))
    pitch = math.degrees(math.atan2(-gx, math.hypot(gy, gz)))
    tilt = math.degrees(math.atan2(math.hypot(gx, gy), max(-gz, 0.0)))
    return roll, pitch, tilt


def refresh_motors(bus, ids: list[int], volt_id: int) -> None:
    """状态查询限在几毫秒内，避免把 50 Hz 堵成 10 Hz。电压每拍只问一个电机。"""
    for mid in ids:
        bus.read_status(mid)
    bus.collect(timeout=0.004)
    bus.read_voltage(volt_id)
    bus.collect(timeout=0.001)


def main() -> int:
    say("watch_policy 启动")
    parser = argparse.ArgumentParser(description="先站立，空格开始或停止策略")
    parser.add_argument("--onnx", default=str(DEFAULT_ONNX))
    parser.add_argument("--print-every", type=float, default=1.0)
    args = parser.parse_args()

    import onnxruntime as ort

    cfg = pose.load_config(pose.CONFIG_PATH)
    stand_s = float(cfg.get("transition_s", 3.0))
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
    say("正在打开 CAN ...")
    bus, ids = pose.open_bus(cfg)
    say(f"CAN 在线电机 {ids}")
    say("正在加载策略 ...")
    session = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    in_name = session.get_inputs()[0].name
    say(f"策略 {args.onnx}")
    say(f"输入 {in_name} {session.get_inputs()[0].shape}  输出 {[o.name for o in session.get_outputs()]}")
    say(f"进程序用 {stand_s:.0f} 秒渐变到 pose_config.yaml 的站立角。右腿动作相对这个姿势取反。")
    say("操作：A 或空格 启动/停止策略 | B 或 Q 立刻卸力 | Ctrl+C 回到站立并保持出力")
    say("站稳后再按 A 或空格。力矩保护 1.20 N·m。头 yaw/roll 仍是 0.30 N·m。")
    say("控制频率目标 50 Hz。手柄只取前后：满杆写入策略 ±0.25 m/s。未执行时指令为 0。")

    lib = lock_fd = imu = None
    keys = None
    stick = None
    csv_log = None
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
        stand_hold = {can: stand[can] for (_name, can, _home, _scale, _lo, _hi, _leg) in JOINTS}
        held: list[float] | None = None
        cmd_filt: list[float] | None = None
        stand_origin: list[float] | None = None
        stand_t0 = 0.0
        last_space = 0.0
        space_edge = False
        leave_t0 = 0.0
        say(f"电机已使能，{stand_s:.0f} 秒渐变到站立。")
        say("电机角 = 采集站立角 + SIGN × (仿真目标 − HOME)。红字是还没跟上的角度。")
        say("操作：A 或空格 启停策略 | B 或 Q 卸力 | Ctrl+C 回站立保持")
        last_action = np.zeros(14, dtype=np.float32)
        q_dot_prev = np.zeros(14, dtype=np.float32)
        window_t = time.monotonic()
        next_print = window_t
        loops = 0
        infer_us = 0.0
        can_drop = 0
        stick = Stick()
        gravity_filt = Gravity()
        prev_tick = time.monotonic()
        log_t0 = prev_tick
        log_path = REPO / "locomotion" / "logs" / "watch" / time.strftime("watch_%Y%m%d_%H%M%S.csv")
        csv_log = CsvLog(log_path)
        say(f"50 Hz 记录 {log_path}")
        while running:
            tick = time.monotonic()
            if leaving and leave_t0 == 0.0:
                leave_t0 = tick
                phase = "stand"
                if held is not None:
                    stand_origin = list(held)
                    stand_t0 = tick
                print(f"Ctrl+C：{stand_s:.0f} 秒回到站立后退出，电机保持出力。再按一次立即退出。", flush=True)
            key = keys.poll()
            if key in ("q", "Q"):
                print("卸力。", flush=True)
                for mid in ids:
                    try:
                        bus.disable(mid)
                    except OSError as exc:
                        if not can_send_failed(exc):
                            raise
                motors_on = False
                print("已卸力。", flush=True)
                break
            if not leaving and key == " " and tick - last_space > 0.3:
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
                q_dot[i] = SIGN[i] * bus.motors[can].state.velocity
            sample = raw_imu.to_policy()
            acc_n = 9.81 * math.sqrt(sum(c * c for c in raw_imu.acc_g))
            gravity = gravity_filt.update(sample.gravity, sample.gyro, acc_n, dt)
            vx = stick.poll(dt, phase == "track")
            btn = stick.take_button()
            if btn == "B":
                print("B：电机卸力。", flush=True)
                for mid in ids:
                    try:
                        bus.disable(mid)
                    except OSError as exc:
                        if not can_send_failed(exc):
                            raise
                motors_on = False
                print("已卸力。", flush=True)
                break
            if btn == "A" and not leaving and tick - last_space > 0.3:
                last_space = tick
                space_edge = True
            obs = build_obs(sample.gyro, gravity, q_rad, q_dot_prev, last_action, stand, vx)
            t_infer = time.monotonic()
            action = session.run(None, {in_name: obs.reshape(1, 61)})[0][0]
            infer_us = (time.monotonic() - t_infer) * 1e6
            loops += 1
            last_action = np.asarray(action, dtype=np.float32).copy()
            q_dot_prev = q_dot
            q_cmd_deg, clipped = motor_command_deg(action, stand)
            stand_err_now = max(abs(now_deg[JOINTS[i][1]] - stand_cmd[i]) for i in range(len(JOINTS)))
            if leaving:
                phase = "stand"
                space_edge = False
            if space_edge:
                space_edge = False
                if phase == "track":
                    phase = "stand"
                    stand_origin = list(held) if held is not None else None
                    stand_t0 = tick
                    print(f"停止策略，{stand_s:.0f} 秒回到站立，力矩保持", flush=True)
                else:
                    tilt = gravity_tilt_deg(gravity)
                    if stand_err_now > math.degrees(0.35) or tilt > 8.0:
                        print(
                            f"未执行：还在站立，离目标 {stand_err_now:.1f}°，重力偏 {tilt:.1f}°。",
                            flush=True,
                        )
                    else:
                        phase = "track"
                        print("开始执行策略。再按 A 或空格回到站立。", flush=True)
            if held is None:
                held = [now_deg[can] for (_name, can, _home, _scale, _lo, _hi, _leg) in JOINTS]
            if phase == "stand":
                if stand_origin is None:
                    stand_origin = list(held)
                    stand_t0 = tick
                target = blend_stand(stand_origin, stand_cmd, tick - stand_t0, stand_s)
            else:
                stand_origin = None
                if cmd_filt is None:
                    cmd_filt = list(held)
                alpha = dt / (CMD_TAU + dt) if dt > 0.0 else 1.0
                cmd_filt = [a + alpha * (b - a) for a, b in zip(cmd_filt, q_cmd_deg)]
                target = cmd_filt
            if phase != "track":
                cmd_filt = None
            held = step_limited(target, held)
            limited = held
            dropped = send_degrees(bus, ids, {can: limited[i] for i, (_n, can, *_r) in enumerate(JOINTS)})
            can_drop += dropped
            roll_deg, pitch_deg, tilt_deg = tilt_axes_deg(gravity)
            obs_vx = float(max(-0.4, min(0.4, vx)) * CMD_SCALE)
            csv_log.write(
                [
                    f"{tick - log_t0:.4f}",
                    f"{time.time():.3f}",
                    phase,
                    f"{dt:.5f}",
                    f"{infer_us:.1f}",
                    *[f"{float(v):.5f}" for v in sample.gyro],
                    *[f"{float(v):.5f}" for v in gravity],
                    *[f"{float(v):.5f}" for v in sample.gravity],
                    *[f"{float(v):.5f}" for v in raw_imu.acc_g],
                    f"{acc_n:.4f}",
                    f"{roll_deg:.3f}",
                    f"{pitch_deg:.3f}",
                    f"{tilt_deg:.3f}",
                    f"{stick.raw:.4f}",
                    f"{stick.cmd:.4f}",
                    f"{obs_vx:.4f}",
                    "0",
                    "0",
                    *[f"{float(obs[6 + i]):.5f}" for i in range(14)],
                    *[f"{float(obs[20 + i]):.5f}" for i in range(14)],
                    *[f"{float(q_dot[i]):.5f}" for i in range(14)],
                    *[f"{float(obs[34 + i]):.5f}" for i in range(14)],
                    *[f"{float(action[i]):.5f}" for i in range(14)],
                    *[f"{q_cmd_deg[i]:.3f}" for i in range(14)],
                    *[f"{limited[i]:.3f}" for i in range(14)],
                    *[f"{now_deg[JOINTS[i][1]]:.3f}" for i in range(14)],
                    *[f"{bus.motors[JOINTS[i][1]].state.torque:.4f}" for i in range(14)],
                    f"{bus.motors[ids[0]].state.voltage:.2f}",
                    str(dropped),
                ]
            )
            if leaving and (stand_err_now < 4.0 or tick - leave_t0 > stand_s + 5.0):
                if stand_err_now < 4.0:
                    for _ in range(8):
                        send_degrees(bus, ids, stand_hold)
                        time.sleep(period)
                print(f"已退出，停在站立方向，偏差 {stand_err_now:.1f}°，电机继续保持出力。", flush=True)
                break
            if tick >= next_print:
                elapsed = max(tick - window_t, period)
                loop_hz = loops / elapsed
                window_t = tick
                loops = 0
                hold_err = max(abs(now_deg[JOINTS[i][1]] - stand_cmd[i]) for i in range(len(JOINTS)))
                bus_v = bus.motors[ids[0]].state.voltage
                ramp_left = 0.0
                if phase == "stand" and stand_origin is not None:
                    ramp_left = max(0.0, stand_s - (tick - stand_t0))
                if phase == "track":
                    state = "正在执行策略，按 A 或空格回到站立"
                elif ramp_left > 0.05:
                    state = f"正在站立，还剩 {ramp_left:.1f} 秒"
                elif hold_err > 5.0:
                    state = "正在站立，站稳后再按 A 或空格"
                else:
                    state = "已站立，按 A 或空格执行策略"
                written = max(-0.4, min(0.4, vx)) * CMD_SCALE
                say(
                    f"控制频率 {loop_hz:.1f} Hz（目标 50）  {state}  推理 {infer_us:.0f} us  "
                    f"电压 {bus_v:.1f} V  stand偏差 {hold_err:.2f}°  "
                    f"手柄 {stick.raw:+.2f}  摇杆速度 {stick.cmd:+.2f} m/s  本拍写入 {written:+.2f}"
                    + (f"  CAN丢帧 {can_drop}" if can_drop else "")
                )
                say("操作：A 或空格 启停策略 | B 或 Q 卸力 | Ctrl+C 回站立保持")
                can_drop = 0
                say(
                    f"{'idx':>3} {'关节':<12} {'电机':>4} {'动作':>8} {'策略角':>8} {'电机角':>8} "
                    f"{'力矩Nm':>8} {'角度差':>10}  备注"
                )
                say(
                    f"gravity {gravity[0]:+.3f} {gravity[1]:+.3f} {gravity[2]:+.3f}  "
                    f"gyro {sample.gyro[0]:+.3f} {sample.gyro[1]:+.3f} {sample.gyro[2]:+.3f} rad/s  "
                    f"倾斜 绕X {roll_deg:+.1f}°  绕Y {pitch_deg:+.1f}°  离竖直 {tilt_deg:.1f}°"
                )
                for i, (name, can, _home, _scale, lo, hi, leg) in enumerate(JOINTS):
                    pol = limited[i]
                    cur = now_deg[can]
                    note = ""
                    if clipped[i]:
                        note = f"已裁到 {math.degrees(lo):+.1f}°~{math.degrees(hi):+.1f}°"
                    elif not leg:
                        note = "锁站立，不跟策略"
                    say(
                        f"{i:3d} {name:<12} {can:4d} {float(action[i]):+8.3f} {pol:+8.2f} {cur:+8.2f} "
                        f"{yellow_nm(bus.motors[can].state.torque)} {red_diff(pol - cur)}  {note}"
                    )
                say()
                next_print = tick + args.print_every
            spent = time.monotonic() - tick
            if spent < period:
                time.sleep(period - spent)
        return 0
    finally:
        if csv_log is not None:
            csv_log.close()
            say(f"记录已写入 {csv_log.path}  共 {csv_log.rows} 行")
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
