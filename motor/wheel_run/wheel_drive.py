#!/usr/bin/env python3
"""站立姿态下的轮行驾驶：手柄控制 6 号（左轮）和 1 号（右轮）。

流程：扫描全部关节 → 柔性进入 stand 站立姿态 →
1 号和 6 号改为速度控制，其余关节保持站立角 → 监听手柄开车。
上电后轮子不转，不做自检和倒计时，启动直接进入站立。

手柄（北通，/dev/input/js0，和 watch_policy.py 同一套轴号）：
  左摇杆上下：前进 / 后退
  左摇杆左右：行进中转弯（差速）
  右摇杆左右：原地自转
  右摇杆上下：机身升降。双腿髋 pitch 和膝（2、3、7、8 号）在收缩点
    （drive 姿态）和伸直点（extended 姿态，腿物理伸直时的实测读数）之间
    逐关节一一对应插值：推上升高，拉下降低。零位不准，所以不拟合到 0。
    右摇杆同时带动脖子：向下是收缩（drive 姿态，默认），向上伸向 stand。
    自平衡只改两侧髋 pitch，不用脖子调重心。
  A 键：原地起跳。髋和膝从收缩快速伸直，再立刻收回收缩。
  B 键：停车并退出
  X 键：切换灯效（和 watch_policy.py 同一套灯环）
  Y 键：关闭 / 打开实时平衡（默认关闭，只用两侧髋）
  扳机：嘴舵机张开。短按短叫，按住满 3 秒才长鸣
  方向键上下：喇叭音量，一格 5%，松开保持
  方向键左右：左右侧倾。右 4、左 9 髋 roll 同向偏，松开回到姿态角

全程力矩保护默认 1 N·m：站立保持、升降和轮电机共用该上限。

启动后先扫描（默认卸力）、等空格，再柔性进入轮行姿态 drive。
1 / 6 号为轮电机，姿态到位后切速度控制。

结束流程：轮速归零后反复发停止卸力，不再继续位置保持。
退出前请扶稳机器人。

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
from ht_can import Motor, MotorBus
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
# 头颈组件（12 关节机：11/12；旧 13/14 若不在线会自动跳过）。
NECK_IDS = (11, 12, 13, 14)
# 升降组件：双腿髋 pitch 和膝。左腿髋 8 膝 7，右腿髋 3 膝 2。
LIFT_IDS = (2, 3, 7, 8)
# 髋 roll：右 4、左 9。两条腿正方向都是绕朝前轴的右手定则，同一角度是同一侧倾。
# 右肩向下为正，和 IMU 的 wx>0 一致。方向键右为正。限位来自策略关节范围。
ROLL_LIMIT_DEG = {4: (-4.6, 22.0), 9: (-22.0, 4.6)}
ROLL_MAX_DEG = 8.0
ROLL_RATE_DEG_S = 24.0
LOG_DIR = RUN_DIR / "logs"
DEADZONE = 0.12


def wheel_sign(motor_id: int) -> float:
    """1 号电机装反：给它的速度取反，反馈也乘回来，显示以机器人前进为正。"""
    return -1.0 if motor_id == 1 else 1.0


def deadzone(v: float) -> float:
    v = max(-1.0, min(1.0, v))
    if abs(v) < DEADZONE:
        return 0.0
    # 死区外重新拉满，手感线性。
    return (abs(v) - DEADZONE) / (1.0 - DEADZONE) * (1.0 if v > 0 else -1.0)


def wait_for_space(running) -> bool:
    """阻塞等空格。返回 False 表示被 Ctrl+C / 取消。"""
    import termios
    import tty

    fd = sys.stdin.fileno()
    if not sys.stdin.isatty():
        print("标准输入不是终端，无法等空格；改用 --no-wait-space 或接上 TTY。", file=sys.stderr)
        return False
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while running():
            ready, _, _ = select.select([sys.stdin], [], [], 0.2)
            if not ready:
                continue
            ch = sys.stdin.read(1)
            if ch == " ":
                print("收到空格，开始进入轮行姿态。")
                return True
            if ch in ("\x03", "q", "Q"):
                return False
    except (termios.error, OSError) as exc:
        print(f"读键盘失败: {exc}", file=sys.stderr)
        return False
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
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


def step_limit(cur: float, goal: float, step: float = 5.0) -> float:
    """朝目标最多走 step 度，不会越过目标。"""
    err = goal - cur
    if abs(err) <= step:
        return goal
    return cur + math.copysign(step, err)


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
# 脖子每次最多走 5°，剩余不足 5° 时停在目标上，不冲过去。
NECK_STEP_DEG = 5.0
NECK_STEP_S = 0.12
HEAD_SLEW_DEG_S = 20.0
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
            self.open_deg = float(spec.get("max_deg", 210.0))
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
                min_deg=float(spec.get("min_deg", 160.0)),
                max_deg=self.open_deg,
                direction=int(spec.get("direction", 1)),
                center_raw=int(spec.get("center_raw", 2048)),
            )
            self.ctrl = ServoController(self.bus, {"mouth": cfg})
            self.ok = True
            try:
                self.bus.ping(servo_id)
                print(f"嘴舵机 ID {servo_id} 已应答。右扳机按下张嘴并鸭叫。")
            except Exception as exc:
                print(f"嘴舵机 ID {servo_id} 没有回包（{exc}）。右扳机仍会发位置。")
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
        angle = self.closed_deg + (self.open_deg - self.closed_deg) * level
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

    def close(self) -> None:
        if self.bus is not None:
            try:
                self.bus.close()
            except OSError:
                pass
            self.bus = None
        self.ok = False


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
        self.buttons = [False] * 16
        self.button_a = False
        self.button_b = False
        self.button_x = False
        self.button_y = False
        self.fd = -1
        self.path = ""
        self._next_open = 0.0
        self._next_warn = 0.0
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
            print(f"手柄已接入 {path}。左摇杆前进后退和转弯，右摇杆左右自转，B 键退出。")
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
        self.buttons = [False] * 16
        self._next_open = time.monotonic() + 0.5

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
                    elif kind == 0x01 and number < 16:
                        self.buttons[number] = bool(value)
                        if not init and value == 1 and number == 0:
                            self.button_a = True
                        if not init and value == 1 and number == 1:
                            self.button_b = True
                        # 北通丝印 X 在 js 2，内核 BTN_X 在 js 3，两个都算。
                        if not init and value == 1 and number in (2, 3):
                            self.button_x = True
                        # Y 键切换自平衡（js 4）。
                        if not init and value == 1 and number == 4:
                            self.button_y = True
        fwd = -self.axis[1]
        turn = -self.axis[0]
        # 北通右摇杆横向是 axis 2、纵向是 axis 3，推上为正（升高）。
        spin = -self.axis[2]
        height = -self.axis[3]
        # 档位：0 普通，1 按 LT 高速（实测 20 已到电机上限，不设更高档）。
        # 高度联动：左侧键（LT/LB）随速度把高度压到 1/3 以内，右侧键（RT/RB）压到最低。
        lt = self.buttons[6] or self.buttons[4] or self.axis[self.LT_AXIS] > 0.3
        rt = self.buttons[7] or self.buttons[5]
        gear = 1 if lt else 0
        if self.fd < 0:
            return 0.0, 0.0, 0.0, 0, 0.0, False, False
        return fwd, turn, spin, gear, height, lt, rt

    @staticmethod
    def _axis_as_trigger(raw: float) -> float:
        """扳机静止是 -1，踩满是 +1。"""
        return max(0.0, min(1.0, (raw + 1.0) * 0.5))

    def rt_level(self) -> float:
        """右扳机。模拟量在轴 4（静止 -32767），数字键在 7 或 5。"""
        level = self._axis_as_trigger(self.axis[4]) if len(self.axis) > 4 else 0.0
        if self.buttons[7] or self.buttons[5]:
            level = 1.0
        return level

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
    parser.add_argument("--spin", type=float, default=8.0, help="右摇杆自转的差速上限 rad/s")
    parser.add_argument("--accel", type=float, default=18.0, help="轮速变化率 rad/s^2，跟手又平滑")
    parser.add_argument("--tqe", type=float, default=1.0, help="关节力矩保护上限 Nm，全程生效")
    parser.add_argument("--wheel-tqe", type=float, default=1.0, help="轮电机力矩保护上限 Nm")
    parser.add_argument("--hip-tqe", type=float, default=1.0, help="自平衡开启时髋 pitch 力矩上限 Nm")
    parser.add_argument(
        "--no-wait-space",
        action="store_true",
        help="不等空格，扫描后立刻进入姿态过渡",
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

    # 脖子联动的目标：stand 姿态的头颈角度。读不到就不联动。
    upright_deg: dict[int, float] = {}
    try:
        upright_spec = pose.pose_spec(cfg, "stand")
        upright_deg = {
            int(k): float(v) for k, v in (upright_spec.get("joints") or {}).items()
        }
    except SystemExit:
        print("没有 stand 姿态，升降时脖子不联动。")

    # 升降的伸直点：extended 姿态是腿物理伸直时的实测角度，逐关节对应。
    extended_deg: dict[int, float] = {}
    try:
        extended_spec = pose.pose_spec(cfg, "extended")
        extended_deg = {
            int(k): float(v) for k, v in (extended_spec.get("joints") or {}).items()
        }
    except SystemExit:
        print("没有 extended 姿态，升降锁定在收缩位。先把腿摆到物理伸直再采集 extended。")

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

    torque_nm = abs(float(args.tqe))
    hz = max(20.0, float(cfg.get("hz", 50.0)))
    period = 1.0 / hz
    running = True

    def _stop(*_):
        nonlocal running
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
    try:
        # 默认卸力：等空格前不给任何关节发位置/速度力矩。
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
                "扶稳后按空格才上力矩进入轮行姿态；Ctrl+C 取消。"
            )
            if not wait_for_space(lambda: running):
                print("已取消，保持卸力退出。")
                bus.disable_all(ids)
                return 0
            if not running:
                print("已取消，保持卸力退出。")
                bus.disable_all(ids)
                return 0

        # 进入姿态：只给非轮关节发位置指令。轮电机保持失能，禁止状态下不转。
        for mid in hold_ids:
            bus.set_torque_limit(mid, torque_nm)
        for wid in WHEELS:
            bus.disable(wid)
        print("姿态阶段轮电机（1、6 号）保持失能，不发指令。")
        time.sleep(0.05)
        start_deg = pose.read_degrees(bus, hold_ids)
        duration = max(0.5, float(cfg.get("transition_s", 3.0)))
        print(f"进入轮行姿态 {args.pose}，过渡 {duration:.1f}s，力矩上限 {torque_nm:.1f} N·m ...")
        print("脖子默认停在收缩姿态，右摇杆上下伸缩，每次最多 5°。")
        neck_cmd = {mid: float(stand_deg.get(mid, 0.0)) for mid in NECK_IDS}
        neck_step_at = 0.0

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
        roll_deg = 0.0
        roll_held = 0
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
        neck_ids = [
            mid
            for mid in NECK_IDS
            if mid in ids and mid in stand_deg and mid in upright_deg
        ]
        if neck_ids:
            print(f"脖子 {neck_ids} 随升降联动：升高时从 drive 方向滑向 stand 方向。")
        # 升降进度 u：0 是 drive 收缩位（最低），1 是 extended 伸直位（最高）。
        # 升降关节在两组实测角度之间逐关节插值，不拟合数学 0 位。
        lift_ok = all(mid in extended_deg for mid in lift_ids)
        height_u = 0.0
        print(
            "开始驾驶。左摇杆上下前进后退，左摇杆左右转弯，右摇杆左右自转，右摇杆上下升降。"
            f"普通档 ±{args.vmax:.0f} rad/s，按住 LT 切高速档 ±{args.vmax_fast:.0f} rad/s。"
            "扳机张开嘴并鸭叫。方向键上下调音量，左右侧倾。"
            f"平时力矩 {torque_nm:.1f} N·m，自平衡时髋 {args.hip_tqe:.1f} N·m。B 键或 Ctrl+C 停车退出。"
        )
        drive_t0 = time.monotonic()
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
                # 先清掉上一拍的电机回包，发送队列才不会被撑满后丢掉摇杆指令。
                bus.can.flush_rx()
                # 用真实循环间隔算斜坡，循环被拖慢时指令也不会滞后。
                dt_real = max(0.001, min(0.1, loop_t - prev_t))
                prev_t = loop_t
                fwd_s, turn_s, spin_s, boost, height_s, lt_on, rt_on = pad.poll()
                hat_x = pad.axis[6] if len(pad.axis) > 6 else 0.0
                hat_y = pad.axis[7] if len(pad.axis) > 7 else 0.0
                if volume.apply_hat(hat_y, loop_t):
                    print(f"音量 {volume.percent():.0f}%", flush=True)
                roll_dir = 1 if hat_x >= 0.5 else -1 if hat_x <= -0.5 else 0
                if roll_dir != roll_held:
                    roll_held = roll_dir
                    if roll_dir == 0:
                        print("侧倾回中。", flush=True)
                    else:
                        print("侧倾：向右。" if roll_dir > 0 else "侧倾：向左。", flush=True)
                roll_target = ROLL_MAX_DEG * roll_dir
                roll_deg = ramp(roll_deg, roll_target, ROLL_RATE_DEG_S, dt_real)
                rt = max(pad.rt_level(), 1.0 if rt_on else 0.0)
                if rt >= 0.35:
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
                if abs(roll_deg) > 0.05:
                    pose_cmd = dict(stand_deg)
                    for mid, (lo, hi) in ROLL_LIMIT_DEG.items():
                        if mid in pose_cmd:
                            pose_cmd[mid] = max(lo, min(hi, stand_deg[mid] + roll_deg))
                pose.command_position(bus, pose_ids, pose_cmd, torque_nm)
                leg_u = jump_u if jump_u is not None else height_u
                lift_deg = {
                    mid: stand_deg[mid] + (extended_deg[mid] - stand_deg[mid]) * leg_u
                    for mid in lift_ids
                }
                for hid, sign in HIP_SIGN.items():
                    if hid in lift_deg:
                        lift_deg[hid] += sign * d_hip
                knee_ids = [mid for mid in lift_ids if mid not in HIP_SIGN]
                hip_ids = [mid for mid in lift_ids if mid in HIP_SIGN]
                knee_tqe = args.hip_tqe if jump_u is not None else torque_nm
                pose.command_position(bus, knee_ids, lift_deg, knee_tqe)
                pose.command_position(bus, hip_ids, lift_deg, hip_tqe_now if balance_active or jump_u is not None else torque_nm)
                if neck_ids and loop_t >= neck_step_at:
                    neck_step_at = loop_t + NECK_STEP_S
                    # 右摇杆升降量 height_u：0 是收缩（drive），1 是伸直（stand）。
                    for mid in neck_ids:
                        if mid == 11 or mid not in stand_deg or mid not in upright_deg:
                            continue
                        goal = stand_deg[mid] + (upright_deg[mid] - stand_deg[mid]) * height_u
                        neck_cmd[mid] = step_limit(neck_cmd.get(mid, goal), goal)
                if neck_ids:
                    stand_neck = upright_deg.get(NECK_PITCH_ID)
                    if 11 in upright_deg and stand_neck is not None:
                        # 头不跟脖子的 5° 台阶跳，按 20°/s 连续跟上，保持水平。
                        head_goal = upright_deg[11] - stand_neck + neck_cmd.get(NECK_PITCH_ID, 0.0)
                        neck_cmd[11] = ramp(neck_cmd.get(11, 0.0), head_goal, HEAD_SLEW_DEG_S, dt_real)
                    pose.command_position(bus, neck_ids, neck_cmd, torque_nm)
                send_speed(bus, bus.motors[left_id], cmd_left * wheel_sign(left_id), args.wheel_tqe)
                send_speed(bus, bus.motors[right_id], cmd_right * wheel_sign(right_id), args.wheel_tqe)

                # 状态 10 Hz 轮询，不在每个控制拍上都要回包。
                if loop_t >= next_status:
                    bus.read_status(left_id)
                    bus.read_status(right_id)
                    bus.collect(timeout=0.001)
                    next_status = loop_t + 0.1

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
                    bus.read_voltage(WHEEL_LEFT)
                    bus.collect(timeout=0.01)
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
                        f"前倾 {lean_deg:+5.1f}° 补偿 {d_hip:+5.2f}° 侧倾 {roll_deg:+4.1f}° "
                        f"音量 {volume.percent():.0f}% 髋力矩 {hip_torque:4.2f}/{hip_tqe_now:.1f} "
                        f"{'髋力矩顶住' if hip_frozen else ''} "
                        f"颈 {neck_cmd.get(NECK_PITCH_ID, 0.0):+5.1f}° 头 {neck_cmd.get(11, 0.0):+5.1f}° "
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

        print("卸力：全部关节切停止模式，并回读确认 ...")
        still = bus.disable_all(ids, rounds=8)
        if still:
            print(f"这些关节仍未停在停止模式，再发一轮: {still}", flush=True)
            still = bus.disable_all(still, rounds=8)
        if still:
            print(f"警告：仍未卸力: {still}", flush=True)
        else:
            print("已卸力（模式 0x01）。")
        pad.close()
        return 0
    finally:
        try:
            left = bus.disable_all(ids, rounds=4)
            if left:
                print(f"退出时仍有关节未卸力: {left}", flush=True)
        except OSError:
            pass
        if imu is not None:
            imu.close()
        if led is not None:
            led.close()
        if mouth is not None:
            mouth.close()
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
