#!/usr/bin/env python3
"""站立姿态下的轮行驾驶：手柄控制 6 号（左轮）和 1 号（右轮）。

流程：扫描全部关节 → 柔性进入 stand 站立姿态 →
1 号和 6 号改为速度控制，其余关节保持站立角 → 监听手柄开车。
上电后轮子不转，不做自检和倒计时，启动直接进入站立。

手柄（北通，/dev/input/js0，和 watch_policy.py 同一套轴号）：
  左摇杆上下：前进 / 后退
  左摇杆左右：行进中转弯（差速）
  右摇杆左右：原地自转
  右摇杆上下：只升降双腿。髋 pitch 和膝（2、3、7、8 号）在收缩点
    （drive 姿态）和伸直点（extended 姿态，腿物理伸直时的实测读数）之间
    逐关节一一对应插值：推上升高，拉下降低。零位不准，所以不拟合到 0。
    脖子不跟右摇杆。自平衡只改两侧髋 pitch。
  A 键：原地起跳。髋和膝从收缩快速伸直，再立刻收回收缩。
  B 键：停车并退出
  X 键：切换灯效（和 watch_policy.py 同一套灯环）
  Y 键：关闭 / 打开实时平衡（默认关闭，只用两侧髋）
  RB：开关头部随机张望（默认关闭）。打开后头、脖子、yaw、roll 按 head_play 张望，嘴和短叫声一起张合
  扳机：嘴舵机张开（上限 30°）。短按短叫，按住满 3 秒才长鸣
  进入驾驶后前 3 秒不发轮速；摇杆同时卡在 ±0.5 视为错误模式，指令作废
  方向键上下：喇叭音量，一格 5%，松开保持
  方向键左右：左右侧倾。目标在 pose_config.yaml 的 lean_left / lean_right。
    抬起侧的膝和髋 pitch 伸向 0，压低侧保持收缩，髋 roll 同时偏。
    默认 50%，按住 LT 到 100%。侧出和回中都是 12°/s，避免腿突然伸直把车身弹起来

全程力矩保护默认 1 N·m：站立保持、升降和轮电机共用该上限。

启动后先扫描（默认卸力）、等空格或长按 Y，再柔性进入轮行姿态 drive。
1 / 6 号为轮电机，姿态到位后切速度控制。

结束流程：轮速归零（轮子不追位置）→ 腿和脖子走到卸力姿态并确认到位 →
再逐台回读到停止模式后才关总线退出。退出前请扶稳机器人。

用法（在 motor 目录）：
  python3 wheel_run/wheel_drive.py
  python3 wheel_run/wheel_drive.py --vmax 6
  python3 wheel_run/wheel_drive.py --swap-wheels
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import random
import select
import signal
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "IMU"))

ROOT = Path(__file__).resolve().parents[1]
RUN_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(RUN_DIR))
sys.path.insert(0, str(ROOT / "pose_contrl"))

import pose
from ht_can import MODE_POS_VEL_TQE, MODE_STOP, QUERY_TEMP, Motor, MotorBus
from wheel_speed import (
    MODE_VELOCITY,
    battery_percent,
    battery_text,
    send_speed,
    yellow_nm,
)

# 实机安装：6 号是左轮，1 号是右轮。
WHEEL_LEFT = 6
WHEEL_RIGHT = 1
WHEELS = (WHEEL_LEFT, WHEEL_RIGHT)


def confirm_all_released(bus: MotorBus, ids, wheel_ids, abort) -> list:
    """逐台发停止，并用 0x0C 回读。连续两次读到停止模式才算卸力。

    轮子只在开头写一次 0 速。确认循环里再写速度会把轮子拉回速度模式，
    停止帧刚生效又被冲掉，1 号就会一直停在模式 0x05。
    轮子速度已经接近 0、停止帧也发过数次，就不再死等模式字节。
    abort() 为真（再按一次 Ctrl+C）时返回还没确认的 ID。
    """
    wheels = {int(mid) for mid in wheel_ids}
    for _ in range(4):
        for mid in wheels:
            motor = bus.motors.get(mid)
            if motor is None:
                continue
            try:
                send_speed(bus, motor, 0.0, 0.0, reply=False)
            except OSError:
                time.sleep(0.02)
        time.sleep(0.02)
    pending = [int(mid) for mid in ids if mid in bus.motors]
    streak = {mid: 0 for mid in pending}
    stop_sent = {mid: 0 for mid in pending}
    last_print = 0.0
    bus.can.flush_rx(64)
    if pending:
        print("确认卸力中。再按一次 Ctrl+C 可放弃等待。", flush=True)
    while pending:
        if abort():
            print(f"再次中断，这些关节还没确认卸力: {pending}", flush=True)
            return pending
        still = []
        for mid in pending:
            motor = bus.motors[mid]
            try:
                bus.can.flush_rx(8)
                t_send = time.monotonic()
                motor.state.mode = 0
                motor.state.t_pos = 0.0
                bus.send_cmd(mid, bytes([MODE_STOP, QUERY_TEMP]), reply=False)
                bus.collect(0.04)
                if mid in wheels:
                    bus.read_status(mid)
                    bus.collect(0.03)
            except OSError:
                time.sleep(0.03)
                streak[mid] = 0
                still.append(mid)
                continue
            stop_sent[mid] += 1
            mode_ok = motor.state.t_rx >= t_send and motor.state.mode == MODE_STOP
            wheel_idle = (
                mid in wheels
                and stop_sent[mid] >= 4
                and motor.state.t_pos >= t_send
                and abs(motor.state.velocity) < 0.4
            )
            if mode_ok or wheel_idle:
                streak[mid] += 1
            else:
                streak[mid] = 0
            if streak[mid] < 2:
                still.append(mid)
        pending = still
        now = time.monotonic()
        if pending and now - last_print >= 0.8:
            bits = []
            for mid in pending:
                motor = bus.motors[mid]
                bits.append(f"{mid}:模式0x{motor.state.mode:02X}")
            print(f"等待卸力确认: {', '.join(bits)}", flush=True)
            last_print = now
    print("全部关节已确认卸力。", flush=True)
    return []
# 头颈组件（12 关节机：11/12；旧 13/14 若不在线会自动跳过）。
NECK_IDS = (11, 12, 13, 14)
# 升降组件：双腿髋 pitch 和膝。左腿髋 8 膝 7，右腿髋 3 膝 2。
LIFT_IDS = (2, 3, 7, 8)
# 髋 roll：右 4、左 9。中位 0；右肩向下为正（和 IMU wx>0 一致）。
# 侧倾同时动膝、髋 pitch：抬起侧伸直，压低侧保持收缩。目标在 yaml。
ROLL_IDS = (4, 9)
LEAN_LEG_IDS = (2, 3, 7, 8)
# 侧出和回中同一速率。抬起侧的腿在伸直，太快会把车身弹离地面。
ROLL_RATE_DEG_S = 12.0
LOG_DIR = RUN_DIR / "logs"
DEADZONE = 0.12
# 进入驾驶后这么久内不发轮速，给手柄模式确认和扳机归零留时间。
SPEED_ARM_S = 3.0
# 嘴最大张角（相对闭嘴 center）。
MOUTH_OPEN_MAX_DEG = 30.0


def load_lean_targets(cfg: dict) -> tuple[dict[int, float], dict[int, float]]:
    """读 lean_left / lean_right：髋 roll，以及膝、髋 pitch。缺省一侧伸直、一侧收缩。"""
    defaults = {
        "lean_right": {2: -100.51, 3: -98.39, 4: 20.0, 7: 0.0, 8: 0.0, 9: 20.0},
        "lean_left": {2: 0.0, 3: 0.0, 4: -20.0, 7: 100.51, 8: 98.39, 9: -20.0},
    }
    keep = set(ROLL_IDS) | set(LEAN_LEG_IDS)
    out: dict[str, dict[int, float]] = {}
    for name, fallback in defaults.items():
        try:
            spec = pose.pose_spec(cfg, name)
        except SystemExit:
            out[name] = dict(fallback)
            continue
        joints = {
            int(k): float(v)
            for k, v in (spec.get("joints") or {}).items()
            if int(k) in keep
        }
        if not joints:
            joints = dict(fallback)
        for mid, val in fallback.items():
            joints.setdefault(mid, val)
        out[name] = joints
    return out["lean_left"], out["lean_right"]


def wheel_sign(motor_id: int) -> float:
    """1 号电机装反：给它的速度取反，反馈也乘回来，显示以机器人前进为正。"""
    return -1.0 if motor_id == 1 else 1.0


def deadzone(v: float) -> float:
    v = max(-1.0, min(1.0, v))
    if abs(v) < DEADZONE:
        return 0.0
    # 死区外重新拉满，手感线性。
    return (abs(v) - DEADZONE) / (1.0 - DEADZONE) * (1.0 if v > 0 else -1.0)


# 启动确认：手柄 Y 需按住这么久才算长按，避免误触。
START_Y_HOLD_S = 1.0


def wait_for_space(running) -> bool:
    """阻塞等空格或手柄长按 Y。返回 False 表示被 Ctrl+C / 取消。"""
    import termios
    import tty

    tty_ok = sys.stdin.isatty()
    stdin_fd = sys.stdin.fileno() if tty_ok else -1
    old = None
    pad = Gamepad()
    y_down_at: float | None = None
    try:
        if tty_ok:
            old = termios.tcgetattr(stdin_fd)
            tty.setcbreak(stdin_fd)
        else:
            print(
                "标准输入不是终端，键盘空格不可用；可长按手柄 Y 启动，"
                "或改用 --no-wait-space。",
                file=sys.stderr,
            )
        while running():
            pad.poll()
            y_held = pad.fd >= 0 and bool(pad.buttons[4])
            # 边沿标志清掉，避免进入驾驶后立刻切一次自平衡。
            pad.button_y = False
            now = time.monotonic()
            if y_held:
                if y_down_at is None:
                    y_down_at = now
                elif now - y_down_at >= START_Y_HOLD_S:
                    print(f"收到长按 Y（{START_Y_HOLD_S:.0f}s），开始进入轮行姿态。")
                    return True
            else:
                y_down_at = None

            if tty_ok:
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if ready:
                    ch = sys.stdin.read(1)
                    if ch == " ":
                        print("收到空格，开始进入轮行姿态。")
                        return True
                    if ch in ("\x03", "q", "Q"):
                        return False
            else:
                time.sleep(0.05)
    except (termios.error, OSError) as exc:
        print(f"读键盘失败: {exc}", file=sys.stderr)
        return False
    finally:
        if old is not None and stdin_fd >= 0:
            try:
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old)
            except (termios.error, OSError):
                pass
        pad.close()
    return False


def wheel_targets(
    fwd_stick: float,
    turn_stick: float,
    spin_stick: float,
    vmax: float,
    turn_max: float,
    spin_max: float,
) -> tuple[float, float]:
    """由摇杆量算左右轮目标速度 rad/s。左转（正）时左轮慢、右轮快。

    左摇杆纯横向打杆（无前后、无自转）时走单轮枢轴：转弯内侧轮保持
    静止，只有外侧轮动。前进或后退中的转弯仍是左右差速。
    """
    fwd = deadzone(fwd_stick) * vmax
    turn = deadzone(turn_stick) * turn_max
    spin = deadzone(spin_stick) * spin_max
    if abs(fwd) < 1e-9 and spin == 0.0 and turn != 0.0:
        if turn > 0:
            return 0.0, turn
        return -turn, 0.0
    diff = turn + spin
    left = fwd - diff
    right = fwd + diff
    cap = max(vmax, spin_max)
    left = max(-cap, min(cap, left))
    right = max(-cap, min(cap, right))
    return left, right


def ramp(cur: float, goal: float, accel: float, dt: float) -> float:
    dv = abs(accel) * dt
    if cur < goal:
        return min(goal, cur + dv)
    if cur > goal:
        return max(goal, cur - dv)
    return goal


class LedRing:
    """灯环在单独进程里刷新，避免 WS2812 位bang 卡住控制环。手柄 X 轮询切换。

    和 locomotion/scripts/watch_policy.py 的 LedRing 同一套 fifo 协议。
    """

    ORDER = (
        ("torque", "力矩亮度"),
        ("breathe-fast", "快闪呼吸"),
        ("off", "熄灭"),
        ("flow", "流水灯"),
        ("ironman", "钢铁侠"),
        ("breathe", "呼吸灯"),
        ("white", "100%白光"),
        ("white50", "50%白光"),
        ("duck", "鸭黄心跳"),
        ("comet", "彗星"),
    )

    def __init__(self) -> None:
        self.proc: subprocess.Popen[str] | None = None
        self.index = -1
        self.label = "未连接"
        self.fifo = Path("/tmp/cyberduck-led.fifo")
        self.fifo_fd = -1
        self._bright = -1

    def _open_fifo(self) -> None:
        if self.fifo.exists():
            self.fifo.unlink()
        os.mkfifo(self.fifo, 0o666)
        os.chmod(self.fifo, 0o666)

    def start(self) -> None:
        script = REPO / "LED_Ring" / "white_flow.py"
        try:
            self._open_fifo()
        except OSError as exc:
            print(f"灯环命令管道没建起来：{exc}")
            return
        # 命令走 fifo。sudo 默认 use_pty 时，写给子进程的 stdin 到不了灯环。
        cmd = [
            "/usr/bin/python3", "-u", str(script),
            "--remote", "--fifo", str(self.fifo), "--effect", "torque",
        ]
        if os.geteuid() != 0:
            cmd = ["sudo", "-E", *cmd]
            print("灯环要 root。若出现密码提示，请在站立前输入。")
        try:
            self.proc = subprocess.Popen(
                cmd,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            print(f"灯环没启动：{exc}")
            self.proc = None
            return
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                err = ""
                if self.proc.stdout is not None:
                    err = self.proc.stdout.read() or ""
                print(f"灯环退出了。{err.strip()}")
                self.proc = None
                return
            if self.proc.stdout is None:
                break
            ready, _, _ = select.select([self.proc.stdout], [], [], 0.2)
            if not ready:
                continue
            line = self.proc.stdout.readline().strip()
            if line == "READY":
                try:
                    self.fifo_fd = os.open(self.fifo, os.O_WRONLY | os.O_NONBLOCK)
                except OSError as exc:
                    print(f"灯环命令管道打不开：{exc}")
                self.index = next(
                    i for i, (key, _label) in enumerate(self.ORDER) if key == "torque"
                )
                self.label = self.ORDER[self.index][1]
                print("灯环已接通，当前力矩亮度，最低 5%。手柄 X 下一档是快闪呼吸。")
                return
            if line:
                print(line)
        print("灯环等待超时，按 X 不会切灯。")
        self.close()

    def drain(self) -> None:
        proc = self.proc
        if proc is None or proc.stdout is None or proc.poll() is not None:
            return
        while True:
            ready, _, _ = select.select([proc.stdout], [], [], 0)
            if not ready:
                return
            if not proc.stdout.readline():
                return

    def cycle(self) -> str:
        self.drain()
        self.index = (self.index + 1) % len(self.ORDER)
        key, label = self.ORDER[self.index]
        self.label = label
        if self.proc is None or self.proc.poll() is not None or self.fifo_fd < 0:
            return f"{label}（灯环没接通）"
        try:
            if key == "torque":
                self._bright = -1
            else:
                os.write(self.fifo_fd, b"b 64\n")
                self._bright = 64
            os.write(self.fifo_fd, (key + "\n").encode())
        except OSError:
            return f"{label}（灯环写失败）"
        return label

    def set_torque_level(self, ratio: float) -> None:
        """默认灯效：亮度 = 5% + 95% × 最大力矩 / 力矩上限。"""
        if self.fifo_fd < 0 or self.index < 0 or self.ORDER[self.index][0] != "torque":
            return
        ratio = max(0.0, min(1.0, ratio))
        level = int(round(255 * (0.05 + 0.95 * ratio)))
        if abs(level - self._bright) < 4:
            return
        self._bright = level
        try:
            os.write(self.fifo_fd, f"b {level}\n".encode())
        except OSError:
            self.fifo_fd = -1

    def close(self) -> None:
        proc = self.proc
        self.proc = None
        if self.fifo_fd >= 0:
            try:
                os.write(self.fifo_fd, b"quit\n")
            except OSError:
                pass
            os.close(self.fifo_fd)
            self.fifo_fd = -1
        if proc is not None:
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        try:
            if self.fifo.exists():
                self.fifo.unlink()
        except OSError:
            pass


class BodyImu:
    """LSM6DSV16X 直连读取，和 watch_policy.py 的 open_imu 同一套总线。"""

    def __init__(self) -> None:
        self.lib = None
        self.lock_fd = -1
        self.dev = None
        try:
            import fcntl

            from lsm6dsv16x import (
                ADDRS,
                LSM6DSV16X,
                WHO_AM_I_VAL,
                I2CGpioError,
                load_lib,
                open_bus,
                scan_bus,
            )

            self.lock_fd = os.open("/tmp/microduck-imu-i2c6.lock", os.O_CREAT | os.O_RDWR, 0o644)
            try:
                fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise I2CGpioError("IMU 被占用，可能有另一个程序在读")
            os.write(self.lock_fd, f"{os.getpid()}\n".encode())
            self.lib = load_lib()
            open_bus(self.lib, pullup=False)
            found = scan_bus(self.lib)
            for addr in [a for a in found if a in ADDRS] or list(ADDRS):
                candidate = LSM6DSV16X(self.lib, addr)
                try:
                    who = candidate.who_am_i()
                except I2CGpioError:
                    continue
                if who == WHO_AM_I_VAL:
                    candidate.init()
                    self.dev = candidate
                    break
            if self.dev is None:
                raise I2CGpioError("未找到 LSM6DSV16X")
            print("IMU 已打开，自平衡启用。")
        except Exception as exc:
            print(f"IMU 未打开（{exc}），自平衡禁用，其余驾驶功能正常。")
            self.close()

    @property
    def ok(self) -> bool:
        return self.dev is not None

    def read(self):
        """返回 (gyro rad/s, gravity 单位向量)。直立时 gravity 约 (0, 0, -1)，前倾时 gx > 0。"""
        sample = self.dev.read_raw().to_policy()
        return sample.gyro, sample.gravity

    def close(self) -> None:
        if self.lib is not None:
            try:
                self.lib.i2c_gpio_close()
            except Exception:
                pass
            self.lib = None
        if self.lock_fd >= 0:
            try:
                os.close(self.lock_fd)
            except OSError:
                pass
            self.lock_fd = -1
        self.dev = None


class Balance:
    """机身俯仰负反馈。前倾为正，补偿方向已按实测翻正。

    倾角不用重力低通。重力低通会把观测拖后一两百毫秒，补偿按过时角度去追就会震荡。
    这里用互补滤波：陀螺积分给出几乎无延迟的倾角，加速度只以较慢的时间常数
    把漂移拉回来。阻尼按角速度的「度/秒」计算，和比例项同一量纲。
    """

    ACC_TAU_S = 0.8
    # 比例只看倾角的慢变化（坡度）。4 秒左右的来回晃不进比例环，避免髋追着自己晃。
    HOLD_TAU_S = 0.7
    RATE_DEG_S = 50.0
    DEAD_DEG = 1.5
    RATE_LPF_S = 0.08
    RATE_DEAD_DPS = 4.0

    def __init__(self, imu: BodyImu, kp: float, kd: float, max_deg: float, neck_ratio: float, flip: bool) -> None:
        self.imu = imu
        self.kp = kp
        self.kd = kd
        self.max_deg = max_deg
        self.neck_ratio = neck_ratio
        self.flip = -1.0 if flip else 1.0
        self.lean_deg: float | None = None
        self.lean_hold: float | None = None
        self.rate_f = 0.0
        self.delta = 0.0

    @staticmethod
    def _accel_lean_deg(gravity: tuple[float, float, float]) -> float:
        gx, gy, gz = gravity
        return math.degrees(math.atan2(gx, math.hypot(gy, gz)))

    def update(self, dt: float, active: bool = True) -> tuple[float, float, float]:
        """返回（前倾角°，髋补偿°，颈补偿°）。关闭时补偿按速率平滑回零，倾角估计不停。"""
        if not self.imu.ok:
            return 0.0, 0.0, 0.0
        try:
            gyro, gravity = self.imu.read()
        except Exception:
            lean = 0.0 if self.lean_deg is None else self.lean_deg
            return lean, self.delta, self.delta * self.neck_ratio
        dt = max(0.001, min(0.05, dt))
        meas = self._accel_lean_deg(gravity)
        rate_deg_s = math.degrees(float(gyro[1]))
        if self.lean_deg is None:
            self.lean_deg = meas
        else:
            # 倾角仍用这一拍陀螺积分，保持坡上的响应。阻尼另走低通。
            self.lean_deg += rate_deg_s * dt
            self.lean_deg += (meas - self.lean_deg) * (dt / self.ACC_TAU_S)
        a = dt / (self.RATE_LPF_S + dt)
        self.rate_f += a * (rate_deg_s - self.rate_f)
        rate_d = 0.0 if abs(self.rate_f) < self.RATE_DEAD_DPS else self.rate_f
        lean = self.lean_deg
        if self.lean_hold is None:
            self.lean_hold = lean
        else:
            b = dt / (self.HOLD_TAU_S + dt)
            self.lean_hold += b * (lean - self.lean_hold)
        hold = self.lean_hold
        hold_eff = hold - math.copysign(self.DEAD_DEG, hold) if abs(hold) > self.DEAD_DEG else 0.0
        raw = (self.kp * hold_eff + self.kd * rate_d) * self.flip
        raw = max(-self.max_deg, min(self.max_deg, raw))
        if not active:
            raw = 0.0
        step = self.RATE_DEG_S * dt
        self.delta += max(-step, min(step, raw - self.delta))
        return lean, self.delta, self.delta * self.neck_ratio


# 髋 pitch 两侧电机符号：左 8 号为正向、右 3 号镜像为负向，补偿按机身方向施加时先乘这个。
HIP_SIGN = {8: 1.0, 3: -1.0}
NECK_PITCH_ID = 12
HEAD_PITCH_ID = 11
JUMP_HALF_S = 0.12


class Mouth:
    """扳机张开嘴舵机，按下沿同时叫一声。舵机不在时驾驶照常进行。"""

    def __init__(self) -> None:
        self.ok = False
        self.ctrl = None
        self.bus = None
        self.speaker = None
        self.closed_deg = 180.0
        self.open_deg = 210.0
        self._playing = False
        self._lock = threading.Lock()
        self._warned = False
        try:
            sys.path.insert(0, str(REPO / "servo"))
            sys.path.insert(0, str(REPO / "speaker"))
            import yaml
            from bus_servo import FtScsBus
            from duck import Speaker
            from servo_controller import ServoConfig, ServoController

            raw = yaml.safe_load((REPO / "servo" / "servo_config.yaml").read_text(encoding="utf-8"))
            spec = (raw.get("servos") or {}).get("mouth") or {}
            servo_id = int(spec.get("id", 15))
            self.closed_deg = float(spec.get("center_deg", 180.0))
            # 最大张嘴限制 30°，即使 yaml 写更大也裁掉。
            yaml_open = float(spec.get("max_deg", self.closed_deg + MOUTH_OPEN_MAX_DEG))
            self.open_deg = min(yaml_open, self.closed_deg + MOUTH_OPEN_MAX_DEG)
            self.speaker = Speaker()
            self.bus = FtScsBus(
                str(raw.get("port", "/dev/ttyMSM0")),
                baudrate=int(raw.get("baudrate", 1000000)),
                timeout=0.02,
            )
            cfg = ServoConfig(
                servo_id=servo_id,
                name="mouth",
                center_deg=self.closed_deg,
                min_deg=float(spec.get("min_deg", self.closed_deg)),
                max_deg=self.open_deg,
                direction=int(spec.get("direction", 1)),
                center_raw=int(spec.get("center_raw", 2048)),
            )
            self.ctrl = ServoController(self.bus, {"mouth": cfg})
            self.ok = True
            try:
                self.bus.ping(servo_id)
                print(
                    f"嘴舵机 ID {servo_id} 已应答。"
                    f"张嘴上限 {self.open_deg - self.closed_deg:.0f}°。右扳机按下张嘴并鸭叫。"
                )
            except Exception as exc:
                print(f"嘴舵机 ID {servo_id} 没有回包（{exc}）。右扳机仍会发位置。")
            # 启动先闭嘴，避免默认轴值把嘴拉到最大。
            self.set_level(0.0)
        except Exception as exc:
            print(f"嘴舵机串口打不开（{exc}）。右扳机仍会鸭叫，嘴巴先不动。")
            if self.bus is not None:
                try:
                    self.bus.close()
                except OSError:
                    pass
                self.bus = None

    def set_level(self, level: float) -> None:
        if not self.ok or self.ctrl is None:
            return
        level = max(0.0, min(1.0, level))
        span = min(self.open_deg - self.closed_deg, MOUTH_OPEN_MAX_DEG)
        angle = self.closed_deg + span * level
        try:
            self.ctrl.torque("mouth", True)
            # 这批 HD1910M 速度 0 才是最快。写成几十反而几乎不动，张嘴幅度看起来很小。
            self.ctrl.set_angle("mouth", angle, speed=0, acceleration=0, torque_limit=800)
        except Exception as exc:
            if not self._warned:
                print(f"嘴舵机写入失败：{exc}")
                self._warned = True
            self.ok = False

    def quack(self, kind: str = "short") -> bool:
        if self.speaker is None:
            return False
        with self._lock:
            if self._playing:
                return False
            self._playing = True
        tag = "wheee" if kind == "long" else random.choice(("peck", "chirp", "greet", "coo", "inquire"))

        def _run() -> None:
            try:
                print("鸭叫：长鸣" if kind == "long" else f"鸭叫：{tag}", flush=True)
                self.speaker.quack(tag)
            except Exception as exc:
                if not self._warned:
                    print(f"鸭叫没播出来：{exc}")
                    self._warned = True
            finally:
                with self._lock:
                    self._playing = False

        threading.Thread(target=_run, daemon=True).start()
        return True

    def start_timed(self, tags: tuple[str, ...]) -> float:
        """开始一声短叫，返回这一声的秒数。正在叫时返回 0。"""
        if self.speaker is None:
            return 0.0
        with self._lock:
            if self._playing:
                return 0.0
            self._playing = True
        try:
            import numpy as np
            from duck import render, variant_count
            from play import play_pcm
            from synth import SR

            tag = random.choice(tags)
            variant = random.randrange(variant_count(tag))
            buf = render(tag, self.speaker.personality, variant)
            vol = max(0.0, min(1.0, float(self.speaker.volume)))
            if vol < 0.999:
                buf = np.clip(buf * vol, -1.0, 1.0)
            duration = float(np.asarray(buf).shape[0]) / SR
        except Exception as exc:
            print(f"鸭叫没生成：{exc}", flush=True)
            with self._lock:
                self._playing = False
            return 0.0
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

    def release(self) -> None:
        """回到闭嘴零位，再卸力。退出程序时调用。"""
        if not self.ok or self.ctrl is None:
            return
        try:
            print("嘴舵机：回零位并卸力 ...", flush=True)
            self.set_level(0.0)
            time.sleep(0.35)
            self.ctrl.torque("mouth", False)
            print("嘴舵机已回零并卸力。", flush=True)
        except Exception as exc:
            if not self._warned:
                print(f"嘴舵机回零/卸力失败：{exc}", flush=True)
                self._warned = True

    def close(self) -> None:
        self.release()
        if self.bus is not None:
            try:
                self.bus.close()
            except OSError:
                pass
            self.bus = None
        self.ok = False
        self.ctrl = None


class HeadLook:
    """把 head_play 的随机张望嵌进驾驶循环。RB 开关，默认关。

    11 头俯仰、12 脖子俯仰走 CAN，13 yaw、14 roll、15 嘴走已打开的舵机总线。
    yaw、roll 收到 0° 再随机看。12 号脖子只到零位往后 30°，头俯仰同向 15°，让头保持相对水平。
    张嘴和这一声短叫同时开始、同时结束。关闭后 CAN 回到驾驶姿态，舵机回中。
    """

    AXES = ("pitch", "neck", "yaw", "roll")
    SHORT_TAGS = ("peck", "chirp", "greet", "coo", "inquire")

    def __init__(self, mouth: Mouth, drive_deg: dict[int, float]) -> None:
        self.mouth = mouth
        self.drive = {
            "pitch": float(drive_deg.get(HEAD_PITCH_ID, 0.0)),
            "neck": float(drive_deg.get(NECK_PITCH_ID, 0.0)),
        }
        self.phase = "off"
        self.pose = {
            "pitch": self.drive["pitch"],
            "neck": self.drive["neck"],
            "yaw": 0.0,
            "roll": 0.0,
            "mouth": 0.0,
        }
        self.goals = dict(self.pose)
        self.want_mouth = False
        self.deadline = 0.0
        self.mouth_rate_now = 40.0
        self.servos: dict = {}
        self.armed = False
        self.rate = 40.0
        self.mouth_rate = 200.0
        self.open_deg = MOUTH_OPEN_MAX_DEG
        self.neck_back = self.drive["neck"]
        self.head_level = 0.0
        self._mouth_phases: list[tuple[float, float, float]] = []
        self._mouth_i = 0
        self._load()

    def _load(self) -> None:
        try:
            import yaml

            raw = yaml.safe_load(
                (ROOT / "pose_contrl" / "pose_config.yaml").read_text(encoding="utf-8")
            ) or {}
            neck = raw.get("neck") or {}
        except Exception as exc:
            print(f"头部限位没读到（{exc}），张望用默认范围。")
            neck = {}
        self.rate = float(neck.get("rate_deg_s", 40.0))
        self.mouth_rate = float(neck.get("mouth_rate_deg_s", 200.0))
        self.mouth_rate_now = self.rate
        defaults = {
            "pitch": {"id": 11, "min_deg": -20.0, "max_deg": 20.0},
            "neck": {"id": 12, "min_deg": -25.0, "max_deg": 25.0},
            "yaw": {"id": 13, "min_deg": -45.0, "max_deg": 45.0, "center_raw": 2048, "direction": 1},
            "roll": {"id": 14, "min_deg": -25.0, "max_deg": 25.0, "center_raw": 2048, "direction": 1},
            "mouth": {"id": 15, "open_deg": 30.0, "center_raw": 1001, "direction": -1},
        }
        self.specs = {}
        for name, fallback in defaults.items():
            spec = dict(fallback)
            spec.update(neck.get(name) or {})
            self.specs[name] = spec
        self.open_deg = min(MOUTH_OPEN_MAX_DEG, float(self.specs["mouth"].get("open_deg", 30.0)))
        # 张望时脖子只比零位再往后 30°。默认姿态太靠后，会压到电池。
        # 头俯仰同向再偏 15°，和脖子形成同位角，头保持相对水平。
        drive_n = self.drive["neck"]
        sign = 1.0 if drive_n >= 0.0 else -1.0
        self.neck_back = sign * 30.0
        self.head_level = sign * 15.0

    def active(self) -> bool:
        return self.phase != "off"

    def toggle(self) -> None:
        if self.phase == "off":
            self._arm()
            self.goals = {
                "pitch": self.head_level,
                "neck": self.neck_back,
                "yaw": 0.0,
                "roll": 0.0,
                "mouth": 0.0,
            }
            self.phase = "home"
            self.mouth_rate_now = self.rate
            print("RB：头部张望开。脖子到零位后 30°，头俯仰同向 15°。", flush=True)
        else:
            self.goals = {
                "pitch": self.drive["pitch"],
                "neck": self.drive["neck"],
                "yaw": 0.0,
                "roll": 0.0,
                "mouth": 0.0,
            }
            self.phase = "park"
            self.mouth_rate_now = self.rate
            print("RB：头部张望关。脖子回到驾驶姿态。", flush=True)

    def can_deg(self) -> dict[int, float]:
        return {HEAD_PITCH_ID: self.pose["pitch"], NECK_PITCH_ID: self.pose["neck"]}

    def tick(self, dt: float) -> None:
        if self.phase == "off":
            return
        for name in self.AXES + ("mouth",):
            rate = self.mouth_rate_now if name == "mouth" else self.rate
            self.pose[name] = ramp(self.pose[name], self.goals[name], rate, dt)
        self._write_servos()
        if self.phase == "home":
            if self._arrived(self.AXES + ("mouth",)):
                print("头已到水平位，开始张望。", flush=True)
                self._begin_look()
        elif self.phase == "look":
            if self._arrived(self.AXES):
                if self.want_mouth:
                    self._begin_mouth()
                else:
                    self.phase = "hold"
                    self.deadline = time.monotonic() + random.uniform(0.6, 1.6)
        elif self.phase == "hold":
            if time.monotonic() >= self.deadline:
                self._begin_look()
        elif self.phase == "mouth":
            if time.monotonic() >= self.deadline:
                self._advance_mouth()
        elif self.phase == "park" and self._arrived(self.AXES + ("mouth",)):
            self.phase = "off"
            print("头部已回到驾驶姿态。", flush=True)

    def release(self) -> None:
        """退出时把 yaw、roll 回中并卸力。嘴由 Mouth.close 负责。"""
        if not self.servos or self.mouth.bus is None:
            return
        for name in ("yaw", "roll", "mouth"):
            self.pose[name] = 0.0
            self.goals[name] = 0.0
        self._write_servos()
        time.sleep(0.25)
        try:
            from bus_servo import ADDR_TORQUE_ENABLE
        except Exception:
            return
        for name in ("yaw", "roll"):
            cfg = self.servos.get(name)
            if cfg is None:
                continue
            try:
                self.mouth.bus.write_u8(cfg.servo_id, ADDR_TORQUE_ENABLE, 0)
            except OSError:
                pass

    def _sample(self, lo: float, hi: float) -> float:
        if random.random() < 0.75:
            mid = (lo + hi) * 0.5
            half = (hi - lo) * 0.28
            return random.uniform(mid - half, mid + half)
        return random.uniform(lo, hi)

    def _arrived(self, names: tuple[str, ...]) -> bool:
        return all(abs(self.pose[name] - self.goals[name]) <= 0.4 for name in names)

    def _sample_neck(self) -> float:
        """张望时脖子固定在零位往后 30°。"""
        return self.neck_back

    def _begin_look(self) -> None:
        self.goals = {
            name: self._sample(float(self.specs[name]["min_deg"]), float(self.specs[name]["max_deg"]))
            for name in ("yaw", "roll")
        }
        self.goals["pitch"] = self.head_level
        self.goals["neck"] = self._sample_neck()
        self.goals["mouth"] = 0.0
        self.want_mouth = random.random() < 0.55 and "mouth" in self.servos
        self.mouth_rate_now = self.rate
        self.phase = "look"
        print(
            "张望  "
            f"头 {self.goals['pitch']:+.0f}°  脖子 {self.goals['neck']:+.0f}°  "
            f"yaw {self.goals['yaw']:+.0f}°  roll {self.goals['roll']:+.0f}°"
            + ("  张嘴" if self.want_mouth else ""),
            flush=True,
        )

    def _begin_mouth(self) -> None:
        opened = random.uniform(0.75, 1.0) * self.open_deg
        sound_s = self.mouth.start_timed(self.SHORT_TAGS)
        if sound_s < 0.08:
            sound_s = 0.28
        open_s = min(0.14, max(0.06, sound_s * 0.32))
        close_s = min(0.12, max(0.05, sound_s * 0.28))
        if open_s + close_s > sound_s:
            scale = sound_s / (open_s + close_s)
            open_s *= scale
            close_s *= scale
        hold_s = max(0.0, sound_s - open_s - close_s)
        self._mouth_phases = [
            (opened, opened / open_s, open_s),
            (opened, self.mouth_rate, hold_s),
            (0.0, opened / close_s, close_s),
        ]
        self._mouth_i = 0
        self.phase = "mouth"
        self._apply_mouth_phase()
        print(f"张嘴 {opened:.0f}°，和叫声一起 {sound_s:.2f}s", flush=True)

    def _apply_mouth_phase(self) -> None:
        goal, rate, seconds = self._mouth_phases[self._mouth_i]
        self.goals["mouth"] = goal
        self.mouth_rate_now = max(1.0, rate)
        self.deadline = time.monotonic() + seconds

    def _advance_mouth(self) -> None:
        self._mouth_i += 1
        if self._mouth_i >= len(self._mouth_phases):
            self.goals["mouth"] = 0.0
            self.pose["mouth"] = 0.0
            self._write_servos()
            self._begin_look()
            return
        self._apply_mouth_phase()

    def _arm(self) -> None:
        if self.armed:
            return
        self.armed = True
        bus = self.mouth.bus
        ctrl = self.mouth.ctrl
        if bus is None or ctrl is None:
            print("舵机串口不可用，张望只动头和脖子。", flush=True)
            return
        try:
            from bus_servo import ADDR_TORQUE_ENABLE, signed_value
            from servo_controller import ServoConfig
        except Exception as exc:
            print(f"舵机模块没装上（{exc}），张望只动头和脖子。", flush=True)
            return
        center = 180.0
        for name in ("yaw", "roll"):
            spec = self.specs[name]
            cfg = ServoConfig(
                servo_id=int(spec["id"]),
                name=name,
                center_deg=center,
                min_deg=center + float(spec["min_deg"]),
                max_deg=center + float(spec["max_deg"]),
                direction=1 if int(spec.get("direction", 1)) >= 0 else -1,
                center_raw=int(spec.get("center_raw", 2048)),
                raw_min=0,
                raw_max=4096,
            )
            ctrl.configs[name] = cfg
            ctrl.by_id[cfg.servo_id] = cfg
            self.servos[name] = cfg
        mouth_cfg = ctrl.configs.get("mouth")
        if mouth_cfg is not None:
            self.servos["mouth"] = mouth_cfg
        for name, cfg in list(self.servos.items()):
            try:
                bus.ping(cfg.servo_id)
                raw = bus.read_u16(cfg.servo_id, 0x38) & 0x0FFF
                self.pose[name] = (raw - cfg.center_raw) * cfg.direction * 360.0 / 4096.0
                bus.write_u8(cfg.servo_id, ADDR_TORQUE_ENABLE, 1)
                bus.write_u16(cfg.servo_id, 0x30, 700 if name == "mouth" else 300)
                bus.write_u8(cfg.servo_id, 0x29, 0)
                bus.write(cfg.servo_id, 0x2E, signed_value(0))
                print(f"{name} ID {cfg.servo_id}  现在 {self.pose[name]:+.1f}°", flush=True)
            except Exception as exc:
                print(f"{name} 舵机没有应答（{exc}），这路先不动。", flush=True)
                self.servos.pop(name, None)

    def _write_servos(self) -> None:
        ctrl = self.mouth.ctrl
        bus = self.mouth.bus
        if ctrl is None or bus is None or not self.servos:
            return
        try:
            from bus_servo import ADDR_GOAL_POSITION
            from servo_controller import ServoController
        except Exception:
            return
        for name, cfg in list(self.servos.items()):
            if name == "mouth":
                angle = self.mouth.closed_deg + max(0.0, min(self.open_deg, self.pose["mouth"]))
            else:
                angle = 180.0 + self.pose[name]
            try:
                raw = ServoController.angle_to_raw(angle, cfg)
                bus.write_u16(cfg.servo_id, ADDR_GOAL_POSITION, raw)
            except Exception as exc:
                print(f"{name} 舵机写入失败：{exc}", flush=True)
                self.servos.pop(name, None)
                return


class DuckVolume:
    """方向键上下调节音量。软件增益给鸭叫，系统喇叭同步到同一档。"""

    STEP = 0.05

    def __init__(self, speaker) -> None:
        self.speaker = speaker
        self.level = self._read_hw()
        self._held = 0
        self._repeat_at = 0.0
        self._push_soft()
        print(f"音量 {self.percent():.0f}%。方向键上加大，下减小。")

    def percent(self) -> float:
        return self.level * 100.0

    def _read_hw(self) -> float:
        try:
            out = subprocess.check_output(
                ["wpctl", "get-volume", "@DEFAULT_AUDIO_SINK@"],
                text=True,
                stderr=subprocess.DEVNULL,
                timeout=1.0,
            )
        except (OSError, subprocess.SubprocessError):
            return 0.75
        for tok in out.replace("[MUTED]", " ").split():
            try:
                return max(0.0, min(1.0, float(tok)))
            except ValueError:
                continue
        return 0.75

    def _push_soft(self) -> None:
        if self.speaker is not None:
            self.speaker.volume = self.level

    def _push_hw(self) -> None:
        level = f"{self.level:.2f}"

        def _run() -> None:
            try:
                subprocess.run(
                    ["wpctl", "set-volume", "@DEFAULT_AUDIO_SINK@", level],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=1.0,
                    check=False,
                )
            except (OSError, subprocess.SubprocessError):
                pass

        threading.Thread(target=_run, daemon=True).start()

    def apply_hat(self, hat_y: float, now: float) -> bool:
        """十字键纵向。北通上推是负。返回这一拍音量是否变了。"""
        direction = 1 if hat_y <= -0.5 else -1 if hat_y >= 0.5 else 0
        if direction == 0:
            self._held = 0
            return False
        if direction != self._held:
            self._held = direction
            self._repeat_at = now + 0.35
            return self._step(direction)
        if now < self._repeat_at:
            return False
        self._repeat_at = now + 0.16
        return self._step(direction)

    def _step(self, direction: int) -> bool:
        nxt = max(0.0, min(1.0, round(self.level + self.STEP * direction, 2)))
        if abs(nxt - self.level) < 1e-4:
            return False
        self.level = nxt
        self._push_soft()
        self._push_hw()
        return True


class Gamepad:
    """读 /dev/input/js* 的最新摇杆位置。轴号和 watch_policy.py 一致。"""

    PATHS = ("/dev/input/js0", "/dev/input/js1", "/dev/input/js2", "/dev/input/js3")
    # 模拟扳机轴。数字键按北通常见布局：左侧 6/4，右侧 7/5。
    LT_AXIS = 5

    def __init__(self) -> None:
        self.axis = [0.0] * 8
        # 扳机静止是 -1；默认 0 会被当成半按，一上来嘴就开。
        self.axis[4] = -1.0
        self.axis[5] = -1.0
        self.axis_seen = [False] * 8
        self.buttons = [False] * 16
        self.button_a = False
        self.button_b = False
        self.button_x = False
        self.button_y = False
        self.button_rb = False
        self.fd = -1
        self.path = ""
        self._next_open = 0.0
        self._next_warn = 0.0
        self._bogus_warned = False
        self.try_open()

    def try_open(self) -> None:
        if self.fd >= 0:
            return
        now = time.monotonic()
        if now < self._next_open:
            return
        self._next_open = now + 1.0
        last_exc: OSError | None = None
        for path in self.PATHS:
            try:
                self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as exc:
                last_exc = exc
                continue
            self.path = path
            self._next_warn = 0.0
            self._bogus_warned = False
            self.axis = [0.0] * 8
            self.axis[4] = -1.0
            self.axis[5] = -1.0
            self.axis_seen = [False] * 8
            print(
                f"手柄已接入 {path}。请确认是 PC/XInput 模式（摇杆回中应为 0）。"
                "左摇杆前进后退和转弯，右摇杆左右自转，B 键退出。"
            )
            return
        if now >= self._next_warn:
            print(
                f"手柄未连接：在 {', '.join(self.PATHS)} 都没找到（{last_exc}）。"
                "检查接收器和手柄电源，轮子保持停止，插好后会自动重连。"
            )
            self._next_warn = now + 5.0

    def drop(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1
        self.axis = [0.0] * 8
        self.axis[4] = -1.0
        self.axis[5] = -1.0
        self.axis_seen = [False] * 8
        self.buttons = [False] * 16
        self.button_rb = False
        self._next_open = time.monotonic() + 0.5

    @staticmethod
    def sticks_look_bogus(fwd: float, turn: float, spin: float) -> bool:
        """多轴同时卡在 ±0.5 附近 = 常见错误手柄模式，指令不可信。"""
        halfish = sum(1 for v in (fwd, turn, spin) if abs(abs(v) - 0.5) < 0.06)
        return halfish >= 2

    def poll(self) -> tuple[float, float, float, int, float, bool, bool]:
        """返回（前进，左转，左自转，档位 0/1/2，升降，LT按下，RT按下）。摇杆量推左/推前/推上为正。"""
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
                if not blob or len(blob) < 8:
                    self.drop()
                    break
                for off in range(0, len(blob) - 7, 8):
                    _t, value, kind, number = struct.unpack_from("IhBB", blob, off)
                    init = bool(kind & 0x80)
                    kind &= ~0x80
                    if kind == 0x02 and number < 8:
                        self.axis[number] = max(-1.0, min(1.0, value / 32767.0))
                        self.axis_seen[number] = True
                    elif kind == 0x01 and number < 16:
                        self.buttons[number] = bool(value)
                        if not init and value == 1 and number == 0:
                            self.button_a = True
                        if not init and value == 1 and number == 1:
                            self.button_b = True
                        # 北通丝印 X 在 js 2，内核 BTN_X 在 js 3，两个都算。
                        if not init and value == 1 and number in (2, 3):
                            self.button_x = True
                        # Y 键切换自平衡（js 4）。不要把 4 当成 LT。
                        if not init and value == 1 and number == 4:
                            self.button_y = True
                        # 这只北通 js 7 是 RB（BTN_TR），不是右扳机。
                        if not init and value == 1 and number == 7:
                            self.button_rb = True
        fwd = -self.axis[1]
        turn = -self.axis[0]
        # 北通右摇杆横向是 axis 2、纵向是 axis 3，推上为正（升高）。
        spin = -self.axis[2]
        height = -self.axis[3]
        # 档位：0 普通，1 按 LT/LB 高速。不用 button 4（Y）。
        lt = self.buttons[6] or (
            self.axis_seen[self.LT_AXIS] and self.axis[self.LT_AXIS] > 0.3
        )
        # 右扳机数字全按在 js 9。js 7 是 RB，不能算进扳机。
        rt = self.buttons[9] or self.buttons[5]
        gear = 1 if lt else 0
        if self.fd < 0:
            return 0.0, 0.0, 0.0, 0, 0.0, False, False
        return fwd, turn, spin, gear, height, lt, rt

    @staticmethod
    def _axis_as_trigger(raw: float) -> float:
        """扳机静止是 -1，踩满是 +1。"""
        return max(0.0, min(1.0, (raw + 1.0) * 0.5))

    def rt_level(self) -> float:
        """右扳机。模拟量在轴 4（静止 -32767），数字键在 7 或 5。未见轴事件前当松开。"""
        if self.buttons[9] or self.buttons[5]:
            return 1.0
        if not self.axis_seen[4]:
            return 0.0
        return self._axis_as_trigger(self.axis[4])

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


def main() -> int:
    parser = argparse.ArgumentParser(description="站立姿态下手柄轮行驾驶")
    parser.add_argument("--iface", default="can0")
    parser.add_argument("--pose", default="drive", help="初始姿态名，用 pose_config.yaml 里的 drive")
    parser.add_argument("--vmax", type=float, default=10.0, help="普通档前进后退最大轮速 rad/s")
    parser.add_argument("--vmax-fast", type=float, default=20.0, help="按住 LT 后前进后退最大轮速 rad/s，实测已是电机上限")
    parser.add_argument("--turn", type=float, default=6.0, help="左摇杆左右转弯的差速上限 rad/s")
    parser.add_argument("--spin", type=float, default=4.0, help="右摇杆原地自转的差速上限 rad/s")
    parser.add_argument("--accel", type=float, default=18.0, help="轮速变化率 rad/s^2，跟手又平滑")
    parser.add_argument("--tqe", type=float, default=1.0, help="关节力矩保护上限 Nm，全程生效")
    parser.add_argument("--wheel-tqe", type=float, default=1.0, help="轮电机力矩保护上限 Nm")
    parser.add_argument("--hip-tqe", type=float, default=1.0, help="自平衡开启时髋 pitch 力矩上限 Nm")
    parser.add_argument(
        "--no-wait-space",
        action="store_true",
        help="不等空格/长按Y，扫描后立刻进入姿态过渡",
    )
    parser.add_argument("--height-rate", type=float, default=0.15, help="升降全行程速率 1/s，右摇杆全推约 7 秒走完")
    parser.add_argument("--swap-wheels", action="store_true", help="交换左右轮（改回 1 号当左轮、6 号当右轮）")
    parser.add_argument("--neck-seconds", type=float, default=1.5, help="结束时脖子回 0 位的过渡秒数")
    parser.add_argument("--no-led", action="store_true", help="不接灯环，手柄 X 无效")
    parser.add_argument("--no-balance", action="store_true", help="关闭 IMU 自平衡负反馈")
    parser.add_argument("--balance-kp", type=float, default=1.0, help="平衡 P：倾角每 1° 出 1° 髋补偿，用来在坡上拉平")
    parser.add_argument("--balance-kd", type=float, default=0.1, help="平衡 D：只抑制真实转动，不跟着晃动加码")
    parser.add_argument("--balance-max", type=float, default=40.0, help="平衡补偿限幅（度）")
    parser.add_argument("--balance-neck", type=float, default=0.45, help="neck pitch 随髋平衡补偿的倍率")
    parser.add_argument(
        "--neck-forward",
        type=float,
        default=110.0,
        help="完全收缩时脖子前伸角度，不会超过站立姿态的脖子角度",
    )
    parser.add_argument("--balance-flip", action="store_true", help="平衡补偿方向反了时整体翻转")
    args = parser.parse_args()

    cfg = pose.load_config(pose.CONFIG_PATH)
    cfg["interface"] = args.iface

    spec = pose.pose_spec(cfg, args.pose)
    joints_cfg = spec.get("joints") or {}
    if not joints_cfg:
        print(f"姿态 {args.pose} 还没有角度。先执行: python3 pose_contrl/pose.py capture {args.pose}", file=sys.stderr)
        return 2
    stand_deg = {int(k): float(v) for k, v in joints_cfg.items()}

    # 升降的伸直点：extended 与 stand 一致，膝 / 髋 pitch 伸直到 0。
    extended_deg: dict[int, float] = {}
    try:
        extended_spec = pose.pose_spec(cfg, "extended")
        extended_deg = {
            int(k): float(v) for k, v in (extended_spec.get("joints") or {}).items()
        }
    except SystemExit:
        print("没有 extended 姿态，升降锁定在收缩位。先把腿摆到物理伸直再采集 extended。")

    lean_left_deg, lean_right_deg = load_lean_targets(cfg)
    print(
        "侧倾目标（yaml）："
        f"左 {[f'{m}:{lean_left_deg[m]:+.1f}°' for m in ROLL_IDS]}  "
        f"右 {[f'{m}:{lean_right_deg[m]:+.1f}°' for m in ROLL_IDS]}"
    )

    bus, ids = pose.open_bus(cfg)
    left_id, right_id = (WHEEL_RIGHT, WHEEL_LEFT) if args.swap_wheels else (WHEEL_LEFT, WHEEL_RIGHT)
    missing_wheels = [w for w in WHEELS if w not in ids]
    if missing_wheels:
        print(f"轮电机 {missing_wheels} 没有应答，取消驾驶。", file=sys.stderr)
        bus.close()
        return 2
    missing_stand = [mid for mid in stand_deg if mid not in ids]
    if missing_stand:
        print(f"站立关节 {missing_stand} 没有应答。为防摔倒，取消驾驶。", file=sys.stderr)
        bus.close()
        return 2
    print(f"  关节在线 {sorted(ids)}，左轮 {left_id} 号，右轮 {right_id} 号")
    # 扫描刚应答完，先记下角度。等空格之后总线容易被停止帧堵住，读不全时用这份补。
    boot_deg = pose.read_degrees(bus, ids, retries=2, strict=False)
    if len(boot_deg) < len(ids):
        missed = [mid for mid in ids if mid not in boot_deg]
        print(f"开机位置未齐，缺 {missed}，进姿态前会再读一次。")

    torque_nm = abs(float(args.tqe))
    hz = max(20.0, float(cfg.get("hz", 50.0)))
    period = 1.0 / hz
    running = True
    stop_hits = 0
    released = False

    def _stop(*_):
        nonlocal running, stop_hits
        stop_hits += 1
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    hold_ids = [mid for mid in ids if mid not in WHEELS]
    cmd_left = 0.0
    cmd_right = 0.0
    led = None
    imu = None
    balance = None
    mouth = None
    head = None
    try:
        # 默认卸力：等空格或长按 Y 前不给任何关节发位置/速度力矩。
        bus.disable_all(ids)
        print("当前为卸力状态（停止模式），可手动摆姿。")

        led = LedRing()
        if not args.no_led:
            led.start()
        if not args.no_balance:
            imu = BodyImu()
            if imu.ok:
                balance = Balance(
                    imu,
                    args.balance_kp,
                    args.balance_kd,
                    args.balance_max,
                    args.balance_neck,
                    args.balance_flip,
                )
                print(
                    f"平衡估计：陀螺积分 + 加速度 {Balance.ACC_TAU_S:.1f}s 修正，"
                    f"kp={args.balance_kp:.2f} kd={args.balance_kd:.2f} °/(°/s) 限幅 ±{args.balance_max:.0f}°。"
                    "默认关闭，按 Y 立即进入实时平衡，再按 Y 退出。"
                )
        # 默认关闭自平衡。按 Y 才打开，再按 Y 关掉。
        balance_on = False

        if not args.no_wait_space:
            print(
                f"已扫到 {len(ids)} 个关节。力矩保护 {torque_nm:.1f} N·m。"
                f"扶稳后按空格，或长按手柄 Y {START_Y_HOLD_S:.0f} 秒，才上力矩进入轮行姿态；"
                "Ctrl+C 取消。"
            )
            if not wait_for_space(lambda: running):
                print("已取消，保持卸力退出。")
                bus.disable_all(ids)
                return 0
            if not running:
                print("已取消，保持卸力退出。")
                bus.disable_all(ids)
                return 0

        # 先逐台读位置，再停轮。先停轮会连发停止帧，把腿的状态回包冲掉。
        fresh = pose.read_degrees(bus, hold_ids, retries=3, strict=False)
        start_deg = {mid: boot_deg[mid] for mid in hold_ids if mid in boot_deg}
        start_deg.update(fresh)
        missing_pos = [mid for mid in hold_ids if mid not in start_deg]
        if missing_pos:
            print(f"读当前位置失败: {missing_pos}", file=sys.stderr)
            print("保持卸力退出。请检查电源/CAN 后重试。", file=sys.stderr)
            bus.disable_all(ids)
            return 1
        if len(fresh) < len(hold_ids):
            used = [mid for mid in hold_ids if mid not in fresh]
            print(f"关节 {used} 用开机时的角度作为过渡起点。")

        # 进入姿态：只给非轮关节发位置指令。轮电机保持失能，禁止状态下不转。
        for mid in hold_ids:
            bus.set_torque_limit(mid, torque_nm)
        for wid in WHEELS:
            bus.disable(wid)
        print("姿态阶段轮电机（1、6 号）保持失能，不发指令。")
        time.sleep(0.02)
        duration = max(0.5, float(cfg.get("transition_s", 3.0)))
        print(f"进入轮行姿态 {args.pose}，过渡 {duration:.1f}s，力矩上限 {torque_nm:.1f} N·m ...")
        print("脖子停在驾驶姿态，右摇杆只升降腿。")

        t0 = time.monotonic()
        while running:
            now = time.monotonic()
            u = (now - t0) / duration
            s = pose.smootherstep(u)
            cmd_deg = {
                mid: start_deg[mid] + (stand_deg[mid] - start_deg[mid]) * s for mid in hold_ids
            }
            if balance is not None:
                # 站立过渡阶段静止，不输出平衡补偿，避免原地来回抢。
                _lean, d_hip, _d_neck = balance.update(period, False)
                for hid, sign in HIP_SIGN.items():
                    if hid in cmd_deg:
                        cmd_deg[hid] += sign * d_hip
            pose.command_position(bus, hold_ids, cmd_deg, torque_nm)
            bus.collect(timeout=min(0.004, period * 0.3))
            if u >= 1.0:
                break
            spent = time.monotonic() - now
            if spent < period:
                time.sleep(period - spent)
        if not running:
            print("站立过渡被打断，保持当前位置退出。")
            return 0
        print("已站稳。下面 1 号和 6 号切速度控制，其余关节保持站立角。")

        # 必须先切到速度模式（0x0B）。在位置模式下发速度帧，
        # 位置字 0x8000 会被当成 -20.6 rad 的位置目标，轮子会自己全速跑。
        # 轮电机通讯超时 300 ms：程序退出或断连后轮子自己停。
        for wid in WHEELS:
            bus.set_mode(wid, MODE_VELOCITY)
            bus.set_timeout_ms(wid, 300)
        time.sleep(0.05)

        # 接管检查：连续发 0 速 0.6 秒，轮子必须停住，否则不进驾驶。
        pad = None
        takeover_ok = False
        check_t0 = time.monotonic()
        while running and time.monotonic() - check_t0 < 0.6:
            pose.command_position(bus, hold_ids, stand_deg, torque_nm)
            for wid in WHEELS:
                send_speed(bus, bus.motors[wid], 0.0, args.wheel_tqe)
            bus.read_status(left_id)
            bus.read_status(right_id)
            bus.collect(timeout=0.002)
            time.sleep(period)
        fb_l = bus.motors[left_id].state.velocity * wheel_sign(left_id)
        fb_r = bus.motors[right_id].state.velocity * wheel_sign(right_id)
        if abs(fb_l) <= 1.0 and abs(fb_r) <= 1.0:
            takeover_ok = True
            print(f"轮子已停住（反馈 左{fb_l:+.2f} 右{fb_r:+.2f} rad/s），进入驾驶。")
        else:
            print(
                f"轮子没有停住：反馈 左{fb_l:+.2f} 右{fb_r:+.2f} rad/s。"
                "取消驾驶，走结束流程。",
                file=sys.stderr,
            )

        pad = Gamepad()
        mouth = Mouth()
        volume = DuckVolume(mouth.speaker)
        mouth_level = 0.0
        mouth_was_down = False
        rt_down_at = 0.0
        long_quack_sent = False
        roll_u = 0.0
        roll_held = 0
        roll_scale_prev = 0.5
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        log_path = LOG_DIR / f"drive_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        log_f = log_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(log_f)
        writer.writerow(
            [
                "t_s", "fwd", "turn", "spin", "gear", "height_stick", "height_u", "height_cap",
                "cmd_left_rad_s", "cmd_right_rad_s",
                "fb_left_rad_s", "fb_right_rad_s", "torque_left_nm", "torque_right_nm",
                "voltage_v", "battery_pct", "lean_deg", "balance_hip_deg", "balance_on",
            ]
        )
        pose_ids = [
            mid for mid in hold_ids if mid not in LIFT_IDS and mid not in NECK_IDS
        ]
        lift_ids = [mid for mid in LIFT_IDS if mid in ids]
        neck_ids = [mid for mid in (HEAD_PITCH_ID, NECK_PITCH_ID) if mid in ids and mid in stand_deg]
        head = HeadLook(mouth, stand_deg)
        # 升降进度 u：0 是 drive 收缩位（最低），1 是 extended 伸直位（最高）。
        # 升降关节在两组实测角度之间逐关节插值，不拟合数学 0 位。
        lift_ok = all(mid in extended_deg for mid in lift_ids)
        height_u = 0.0
        print(
            "开始驾驶。左摇杆上下前进后退，左摇杆左右转弯，右摇杆左右自转，右摇杆上下只升降腿。"
            f"前 {SPEED_ARM_S:.0f}s 不发轮速。"
            f"普通档 ±{args.vmax:.0f} rad/s，按住 LT 切高速档 ±{args.vmax_fast:.0f} rad/s。"
            f"扳机张嘴（上限 {MOUTH_OPEN_MAX_DEG:.0f}°）并鸭叫。RB 开关头部张望（默认关）。"
            "方向键上下调音量，左右侧倾（默认半幅，LT 满幅）。"
            f"平时力矩 {torque_nm:.1f} N·m，自平衡时髋 {args.hip_tqe:.1f} N·m。B 键或 Ctrl+C 停车退出。"
        )
        drive_t0 = time.monotonic()
        speed_armed = False
        next_print = drive_t0
        next_status = drive_t0
        prev_t = drive_t0
        boost_prev = 0
        hip_tqe_sent = torque_nm
        hip_hold = 0.0
        prev_lean = 0.0
        hip_frozen = False
        jump_t0 = None
        lift_deg = {mid: stand_deg[mid] for mid in lift_ids}
        gear_names = ("普通", "高速")
        gear_caps = (args.vmax, args.vmax_fast)
        try:
            while running and takeover_ok:
                loop_t = time.monotonic()
                # 用真实循环间隔算斜坡，循环被拖慢时指令也不会滞后。
                dt_real = max(0.001, min(0.1, loop_t - prev_t))
                prev_t = loop_t
                fwd_s, turn_s, spin_s, boost, height_s, lt_on, rt_on = pad.poll()
                # 错误手柄模式：多轴同时卡在 ±0.5，整帧指令作废。
                pad_bogus = Gamepad.sticks_look_bogus(fwd_s, turn_s, spin_s)
                if pad_bogus:
                    if not pad._bogus_warned:
                        print(
                            "手柄模式异常（摇杆同时卡在 ±0.5），本帧指令不执行。"
                            "请切到 PC/XInput 模式并回中摇杆。",
                            flush=True,
                        )
                        pad._bogus_warned = True
                    fwd_s = turn_s = spin_s = height_s = 0.0
                    boost = 0
                    lt_on = False
                    rt_on = False
                elif pad._bogus_warned:
                    print("手柄摇杆已恢复正常。", flush=True)
                    pad._bogus_warned = False

                speed_ready = (loop_t - drive_t0) >= SPEED_ARM_S
                if speed_ready and not speed_armed:
                    speed_armed = True
                    print(f"轮速已解锁（启动已满 {SPEED_ARM_S:.0f}s）。", flush=True)
                if not speed_ready or pad_bogus:
                    fwd_s = turn_s = spin_s = 0.0

                hat_x = pad.axis[6] if len(pad.axis) > 6 else 0.0
                hat_y = pad.axis[7] if len(pad.axis) > 7 else 0.0
                if not pad_bogus and volume.apply_hat(hat_y, loop_t):
                    print(f"音量 {volume.percent():.0f}%", flush=True)
                roll_dir = 0 if pad_bogus else (1 if hat_x >= 0.5 else -1 if hat_x <= -0.5 else 0)
                # 默认半幅侧倾；按住 LT 才到 yaml 满幅。
                roll_scale = 1.0 if lt_on else 0.5
                roll_target = float(roll_dir) * roll_scale
                lean_level = "满幅" if lt_on else "半幅"
                if roll_dir != roll_held:
                    roll_held = roll_dir
                    if roll_dir == 0:
                        print("侧倾回中。", flush=True)
                    else:
                        side = "向右" if roll_dir > 0 else "向左"
                        print(f"侧倾：{side}（{lean_level}）。", flush=True)
                elif roll_dir != 0 and roll_scale != roll_scale_prev:
                    print(f"侧倾改为{lean_level}。", flush=True)
                roll_scale_prev = roll_scale
                # roll_u∈[-1,1]：-1 左倾、+1 右倾；速率按侧倾目标角峰值换算成 °/s。
                lean_span = max(
                    abs(lean_right_deg.get(mid, 20.0) - stand_deg.get(mid, 0.0))
                    for mid in ROLL_IDS
                )
                lean_span = max(lean_span, 1.0)
                roll_u = ramp(roll_u, roll_target, ROLL_RATE_DEG_S / lean_span, dt_real)
                if pad.button_rb and head is not None and not pad_bogus:
                    pad.button_rb = False
                    head.toggle()
                rt = 0.0 if pad_bogus else max(pad.rt_level(), 1.0 if rt_on else 0.0)
                # 张望自己管嘴。这段时间右扳机不再改嘴，避免两路抢舵机。
                if head is not None and head.active():
                    mouth_was_down = False
                elif rt >= 0.35:
                    if not mouth_was_down:
                        mouth.set_level(1.0)
                        mouth.quack("short")
                        mouth_level = 1.0
                        mouth_was_down = True
                        rt_down_at = loop_t
                        long_quack_sent = False
                        print("右扳机：张嘴，短叫。", flush=True)
                    elif not long_quack_sent and loop_t - rt_down_at >= 3.0:
                        if mouth.quack("long"):
                            long_quack_sent = True
                            print("右扳机按住 3 秒：长鸣。", flush=True)
                elif rt < 0.2 and mouth_was_down:
                    mouth.set_level(0.0)
                    mouth_level = 0.0
                    mouth_was_down = False
                gear_now = int(boost)
                if pad.button_b:
                    print("B 键停车，退出驾驶。")
                    break
                if pad_bogus:
                    # 错误模式边沿一律丢掉，只保留 B 紧急停车。
                    pad.button_a = False
                    pad.button_x = False
                    pad.button_y = False
                    pad.button_rb = False
                if pad.button_x:
                    pad.button_x = False
                    print(f"灯效：{led.cycle()}", flush=True)
                led.drain()
                if pad.button_a:
                    pad.button_a = False
                    if jump_t0 is None and lift_ok:
                        jump_t0 = loop_t
                        print("A：原地起跳，髋和膝伸直后立刻收回。", flush=True)
                if pad.button_y:
                    pad.button_y = False
                    if balance is not None:
                        balance_on = not balance_on
                        print(
                            "Y：进入自平衡。" if balance_on else "Y：退出自平衡，补偿平滑回零。",
                            flush=True,
                        )
                if gear_now != boost_prev:
                    print(
                        f"{gear_names[gear_now]}档：速度上限 ±{gear_caps[gear_now]:.0f} rad/s",
                        flush=True,
                    )
                    boost_prev = gear_now
                vmax_now = gear_caps[gear_now]
                tgt_left, tgt_right = wheel_targets(
                    fwd_s, turn_s, spin_s, vmax_now, args.turn, args.spin
                )
                cmd_left = ramp(cmd_left, tgt_left, args.accel, dt_real)
                cmd_right = ramp(cmd_right, tgt_right, args.accel, dt_real)
                jump_u = None
                if jump_t0 is not None:
                    jumped = loop_t - jump_t0
                    if jumped < JUMP_HALF_S:
                        jump_u = jumped / JUMP_HALF_S
                    elif jumped < 2.0 * JUMP_HALF_S:
                        jump_u = 1.0 - (jumped - JUMP_HALF_S) / JUMP_HALF_S
                    else:
                        jump_t0 = None
                        height_u = 0.0
                        print("起跳结束，腿回到收缩。", flush=True)
                    cmd_left = 0.0
                    cmd_right = 0.0

                # 右摇杆上推升高（u 增大、走向伸直点），下拉降低（走向收缩点）。
                if lift_ok and jump_u is None:
                    height_u = max(
                        0.0,
                        min(1.0, height_u + deadzone(height_s) * args.height_rate * dt_real),
                    )
                # 高度只跟右摇杆。扳机留给嘴舵机，不再把身子压下去。
                height_cap = 1.0

                # 按 Y 后立即实时平衡，静止、坡上、行驶都出力。再按 Y 才平滑回零。
                balance_active = balance_on
                hip_tqe_now = args.hip_tqe if balance_active else torque_nm
                if hip_tqe_now != hip_tqe_sent:
                    for hid in HIP_SIGN:
                        if hid in bus.motors:
                            bus.set_torque_limit(hid, hip_tqe_now)
                    hip_tqe_sent = hip_tqe_now
                    print(f"髋力矩上限 {hip_tqe_now:.1f} N·m", flush=True)
                if balance is not None:
                    lean_deg, d_hip, d_neck = balance.update(dt_real, balance_active)
                else:
                    lean_deg, d_hip, d_neck = 0.0, 0.0, 0.0
                hip_torque = 0.0
                for hid in HIP_SIGN:
                    if hid in bus.motors:
                        hip_torque = max(hip_torque, abs(bus.motors[hid].state.torque))
                # 力矩已经顶住、倾角却没减小：停住补偿角，避免继续加角度造成晃动。
                hip_sat = balance_active and hip_torque >= 0.85 * hip_tqe_now
                lean_growing = abs(lean_deg) >= abs(prev_lean) - 0.3
                if hip_sat and lean_growing and abs(d_hip) > hip_hold:
                    d_hip = math.copysign(hip_hold, d_hip)
                    if balance is not None:
                        balance.delta = d_hip
                    if not hip_frozen:
                        print(
                            f"髋力矩 {hip_torque:.2f} N·m 已顶住，倾角 {lean_deg:+.1f}° 未减小，补偿停在 {d_hip:+.1f}°",
                            flush=True,
                        )
                        hip_frozen = True
                else:
                    hip_hold = abs(d_hip)
                    hip_frozen = False
                prev_lean = lean_deg

                pose_cmd = stand_deg
                lean_tgt: dict[int, float] = {}
                lean_a = abs(roll_u)
                if lean_a > 0.02:
                    pose_cmd = dict(stand_deg)
                    lean_tgt = lean_right_deg if roll_u >= 0.0 else lean_left_deg
                    for mid in ROLL_IDS:
                        if mid not in pose_cmd:
                            continue
                        base = stand_deg.get(mid, 0.0)
                        pose_cmd[mid] = base + (lean_tgt.get(mid, base) - base) * lean_a
                pose.command_position(bus, pose_ids, pose_cmd, torque_nm, vel_limit=6.0)
                leg_u = jump_u if jump_u is not None else height_u
                lift_deg = {
                    mid: stand_deg[mid] + (extended_deg[mid] - stand_deg[mid]) * leg_u
                    for mid in lift_ids
                }
                # 侧倾时抬起侧的膝和髋 pitch 走向 yaml 里的伸直角，不再只动 roll。
                if lean_tgt and jump_u is None:
                    for mid in lift_ids:
                        if mid not in lean_tgt:
                            continue
                        lift_deg[mid] = lift_deg[mid] + (lean_tgt[mid] - lift_deg[mid]) * lean_a
                for hid, sign in HIP_SIGN.items():
                    if hid in lift_deg:
                        lift_deg[hid] += sign * d_hip
                knee_ids = [mid for mid in lift_ids if mid not in HIP_SIGN]
                hip_ids = [mid for mid in lift_ids if mid in HIP_SIGN]
                knee_tqe = args.hip_tqe if jump_u is not None else torque_nm
                pose.command_position(bus, knee_ids, lift_deg, knee_tqe, vel_limit=6.0)
                pose.command_position(
                    bus,
                    hip_ids,
                    lift_deg,
                    hip_tqe_now if balance_active or jump_u is not None else torque_nm,
                    vel_limit=6.0,
                )
                if head is not None:
                    head.tick(dt_real)
                    neck_cmd = head.can_deg()
                else:
                    neck_cmd = {mid: float(stand_deg.get(mid, 0.0)) for mid in neck_ids}
                if neck_ids:
                    pose.command_position(
                        bus,
                        neck_ids,
                        neck_cmd,
                        torque_nm,
                        vel_limit=max(1.2, math.radians(head.rate) * 1.4) if head is not None else 2.0,
                    )
                send_speed(bus, bus.motors[left_id], cmd_left * wheel_sign(left_id), args.wheel_tqe)
                send_speed(bus, bus.motors[right_id], cmd_right * wheel_sign(right_id), args.wheel_tqe)

                # 状态 5 Hz。只在这一拍收两台轮子的回包，其余拍不 flush，避免把回包清掉。
                if loop_t >= next_status:
                    bus.read_status(left_id)
                    bus.read_status(right_id)
                    bus.collect(timeout=0.012)
                    next_status = loop_t + 0.2

                lm = bus.motors[left_id]
                rm = bus.motors[right_id]
                fb_left = lm.state.velocity * wheel_sign(left_id)
                fb_right = rm.state.velocity * wheel_sign(right_id)
                tor_left = lm.state.torque * wheel_sign(left_id)
                tor_right = rm.state.torque * wheel_sign(right_id)
                hip_now = 0.0
                for hid in HIP_SIGN:
                    if hid in bus.motors:
                        hip_now = max(hip_now, abs(bus.motors[hid].state.torque))
                torque_space = max(args.hip_tqe, args.wheel_tqe, 0.2)
                torque_ratio = max(abs(tor_left), abs(tor_right), hip_now) / torque_space
                if led is not None:
                    led.set_torque_level(torque_ratio)
                volts = bus.motors[WHEEL_LEFT].state.voltage
                elapsed = loop_t - drive_t0
                writer.writerow(
                    [
                        f"{elapsed:.3f}", f"{fwd_s:.3f}", f"{turn_s:.3f}", f"{spin_s:.3f}",
                        str(gear_now),
                        f"{height_s:.3f}", f"{height_u:.4f}", f"{height_cap:.4f}",
                        f"{cmd_left:.3f}", f"{cmd_right:.3f}",
                        f"{fb_left:.3f}", f"{fb_right:.3f}",
                        f"{tor_left:.4f}", f"{tor_right:.4f}",
                        f"{volts:.2f}", f"{battery_percent(volts):.1f}",
                        f"{lean_deg:.3f}", f"{d_hip:.3f}", "1" if balance_active else "0",
                    ]
                )

                if loop_t >= next_print:
                    left_st = bus.motors[WHEEL_LEFT].state
                    volts = left_st.voltage
                    temp_c = left_st.temperature
                    volt_txt = (
                        f"电压 {volts:.1f} V 电量 {battery_text(volts)}"
                        if volts >= 5.0
                        else "电压 --"
                    )
                    temp_txt = f"温度 {temp_c:.1f}°C" if temp_c > 1.0 else "温度 --"
                    print(
                        f"t={elapsed:5.1f}s  杆 前{fwd_s:+.2f} 转{turn_s:+.2f} 自{spin_s:+.2f}  "
                        f"档{gear_names[gear_now]}  升降 {height_u:4.2f}/≤{height_cap:4.2f}  "
                        f"前倾 {lean_deg:+5.1f}° 补偿 {d_hip:+5.2f}° 侧倾 {roll_u:+4.2f} "
                        f"音量 {volume.percent():.0f}% 髋力矩 {hip_torque:4.2f}/{hip_tqe_now:.1f} "
                        f"{'髋力矩顶住' if hip_frozen else ''} "
                        f"颈 {neck_cmd.get(NECK_PITCH_ID, 0.0):+5.1f}° 头 {neck_cmd.get(HEAD_PITCH_ID, 0.0):+5.1f}° "
                        f"张望{'开' if head is not None and head.active() else '关'} "
                        f"自平衡{'开' if balance_active else '关'}  "
                        f"轮指令 左{cmd_left:+6.2f} 右{cmd_right:+6.2f}  "
                        f"反馈 左{fb_left:+6.2f} 右{fb_right:+6.2f}  "
                        f"{volt_txt}  {temp_txt}  灯 {led.label}  "
                        f"力矩 左{yellow_nm(tor_left)} 右{yellow_nm(tor_right)}",
                        flush=True,
                    )
                    next_print = loop_t + 0.5

                spent = time.monotonic() - loop_t
                if spent < period:
                    time.sleep(period - spent)
        finally:
            log_f.flush()
            log_f.close()
            print(f"驾驶日志 {log_path}")

        # 轮子只停转、卸力，绝不切回位置模式。
        # 速度模式下编码器会累到上千度，再按卸力姿态去追位置就会空转好多圈。
        print("停车：轮电机速度归零并卸力，不发位置指令。", flush=True)
        for _ in range(6):
            for wid in WHEELS:
                if wid in bus.motors:
                    send_speed(bus, bus.motors[wid], 0.0, 0.0)
            bus.collect(timeout=0.002)
            time.sleep(0.02)
        for wid in WHEELS:
            if wid in bus.motors:
                bus.disable(wid)
        body_ids = [mid for mid in ids if mid not in WHEELS]

        unload_deg = pose.unload_target(cfg, body_ids)
        arrived = False
        keep_going = lambda: stop_hits < 2
        if unload_deg is not None:
            print("卸力：腿和脖子先到卸力姿态，确认到位后才卸力。轮子保持停止。", flush=True)
            try:
                for mid in body_ids:
                    bus.set_mode(mid, MODE_POS_VEL_TQE)
                time.sleep(0.05)
                roll_ids = [mid for mid in ROLL_IDS if mid in body_ids and mid in unload_deg]
                roll_now = pose.read_degrees(bus, roll_ids, retries=2, strict=False) if roll_ids else {}
                roll_far = [
                    mid
                    for mid in roll_ids
                    if mid not in roll_now or abs(roll_now[mid] - unload_deg[mid]) > 2.0
                ]
                if roll_far:
                    peak = max(abs(roll_now.get(mid, unload_deg[mid]) - unload_deg[mid]) for mid in roll_ids)
                    print(
                        f"先收髋 roll {roll_far} 到卸力角（最大还差 {peak:.1f}°），再收腿。"
                        "一起收时 roll 会被腿压住，到不了位。",
                        flush=True,
                    )
                    pose.move_to_pose(
                        bus,
                        cfg,
                        roll_ids,
                        {mid: unload_deg[mid] for mid in roll_ids},
                        keep_going,
                        label="卸力髋 roll",
                        vel_limit=2.5,
                        duration_s=max(0.6, peak / 40.0),
                    )
                    pose.hold_until_reached(
                        bus,
                        roll_ids,
                        {mid: unload_deg[mid] for mid in roll_ids},
                        keep_going,
                        torque=abs(float(cfg.get("torque_nm", torque_nm))),
                        vel_limit=2.5,
                        tol_deg=2.0,
                        label="卸力髋 roll",
                    )
                pose.move_to_pose(
                    bus,
                    cfg,
                    body_ids,
                    unload_deg,
                    keep_going,
                    label="卸力姿态",
                    vel_limit=2.0,
                )
            except (RuntimeError, OSError) as exc:
                print(f"过渡读数失败（{exc}），改为直接保持卸力目标角。", flush=True)
            arrived = pose.hold_until_reached(
                bus,
                body_ids,
                unload_deg,
                keep_going,
                torque=abs(float(cfg.get("torque_nm", torque_nm))),
                vel_limit=max(1.5, abs(float(cfg.get("vel_limit", 1.0)))),
                tol_deg=2.0,
                label="卸力姿态",
            )
        else:
            print(
                "还没有卸力姿态。可先摆好再: python3 pose_contrl/pose.py capture unload",
                flush=True,
            )

        if unload_deg is not None and not arrived:
            print("已放弃等待卸力姿态，改为直接卸力。", flush=True)
        print("卸力：全部关节切停止模式，逐台回读，确认后才退出。", flush=True)
        left = confirm_all_released(bus, ids, WHEELS, lambda: stop_hits >= 2)
        released = not left
        if pad is not None:
            pad.close()
        return 0
    finally:
        if not released:
            try:
                left = confirm_all_released(bus, ids, WHEELS, lambda: stop_hits >= 2)
                released = not left
                if left:
                    print(f"退出时仍有关节未卸力: {left}", flush=True)
            except OSError as exc:
                print(f"卸力确认时总线异常: {exc}", flush=True)
        if imu is not None:
            imu.close()
        if led is not None:
            led.close()
        if head is not None:
            head.release()
        if mouth is not None:
            mouth.close()
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
