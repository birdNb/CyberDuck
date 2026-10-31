#!/usr/bin/env python3
"""站立姿态下的轮行驾驶：手柄控制 6 号（左轮）和 1 号（右轮）。

流程：开机灯效结束 → 长按 Y 才进入 drive 姿态 →
1 号和 6 号改为速度控制，其余关节保持姿态 → 监听手柄开车。
上电后轮子不转，不做自检和倒计时。没长按 Y 之前不站立、不驾驶。

手柄（北通，/dev/input/js0，和 watch_policy.py 同一套轴号）：
  左摇杆上下：前进 / 后退
  左摇杆左右：行进中转弯（差速）
  右摇杆左右：原地自转
  右摇杆上下：机身升降。双腿髋 pitch 和膝（2、3、7、8 号）在收缩点
    （drive 姿态）和伸直点（extended 姿态，腿物理伸直时的实测读数）之间
    逐关节一一对应插值：推上升高，拉下降低。零位不准，所以不拟合到 0。
    右摇杆同时带动脖子：向下是收缩（drive 姿态，默认），向上伸向 stand。
    自平衡：髋 pitch 抵消前后倾，左右腿屈伸抵消侧倾。不用脖子调重心。
  A 键：原地起跳。髋和膝从收缩快速伸直，再立刻收回收缩。
  B 键：停车并卸力，退出驾驶。再长按 Y 恢复站立和驾驶
  X 键：切换灯效（和 watch_policy.py 同一套灯环）
  长按 Y：进入或恢复站立和驾驶。短按 Y：关闭 / 打开实时平衡（默认开启）
  十字键左右：机身向左或向右倾斜，幅度为屈伸全程的 50%
  十字键上：启动或关闭 TOF。障碍近于 15 cm 后退，手在前面则保持 15 cm
  右扳机：嘴巴开合，松开 0°，按到底 60°。同时仍会把车身高度压到最低

全程力矩保护 1 N·m：站立保持、升降和轮电机的力矩上限都限制在 1 N·m。

结束流程：轮子停住后直接卸力，脖子停在当前角度。
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
import select
import signal
import struct
import subprocess
import sys
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
# 头颈组件：neck_pitch 12、head_pitch 11、head_yaw 14、head_roll 13。
NECK_IDS = (11, 12, 13, 14)
# 升降组件：双腿髋 pitch 和膝。左腿髋 8 膝 7，右腿髋 3 膝 2。
LIFT_IDS = (2, 3, 7, 8)
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


def command_mouth(bus, level: float, closed_deg: float | None) -> None:
    """松开停在进入时的角度，按到底在此基础上张开 60°。"""
    if closed_deg is None or MOUTH_ID not in bus.motors:
        return
    deg = closed_deg + MOUTH_OPEN_DEG * max(0.0, min(1.0, level))
    command_position_seek(bus, [MOUTH_ID], {MOUTH_ID: deg}, MOUTH_TQE_NM, MOUTH_VEL_RAD_S)


def keep_can_up(iface: str = "can0") -> None:
    """can0 掉线时重新拉起。接口是 DOWN 时，长按 Y 也发不出电机指令。"""
    if pose.iface_down_reason(iface) is None:
        return
    print(f"{iface} 掉线了，正在重新打开。在这之前电机不会动。", flush=True)
    pose.ensure_can(iface)


def publish_battery(bus, led, motor_id: int) -> None:
    """读到真实母线电压才刷新电量灯。0 V 是还没读到，不能当成空电。"""
    if led is None or motor_id not in bus.motors:
        return
    try:
        bus.read_voltage(motor_id)
        bus.collect(timeout=0.002)
    except OSError:
        return
    volts = bus.motors[motor_id].state.voltage
    if volts < 8.0:
        return
    led.set_battery(battery_percent(volts))


def command_position_seek(bus, ids, degrees: dict[int, float], torque: float, vel_rad_s: float) -> None:
    """位置目标配上速度上限，让关节能尽快走到目标，而不是被速度字 0 卡住。"""
    for mid in ids:
        if mid not in degrees or mid not in bus.motors:
            continue
        bus.command_pos_vel_tqe(
            bus.motors[mid],
            degrees[mid] * pose.DEG,
            vel_rad_s,
            torque,
        )


def head_goal_for(neck_deg: float, drive_deg: dict[int, float], stand_deg: dict[int, float]) -> float | None:
    """头 pitch 跟着脖子在两组实测姿态之间走。脖子在 drive 时头就是 drive 的头角。"""
    n0 = drive_deg.get(NECK_PITCH_ID)
    h0 = drive_deg.get(11)
    n1 = stand_deg.get(NECK_PITCH_ID)
    h1 = stand_deg.get(11)
    if h0 is None:
        return None
    if n0 is None or n1 is None or h1 is None or abs(n1 - n0) < 1e-3:
        return h0
    t = (neck_deg - n0) / (n1 - n0)
    return h0 + (h1 - h0) * t


FOLLOW_M = 0.15
FOLLOW_FAR_M = 0.50
FOLLOW_DEAD_M = 0.015
FOLLOW_KP = 16.0
FOLLOW_REVERSE = 2.5
FOLLOW_FORWARD = 2.0


def follow_speed(mm: int) -> float:
    """比 15 cm 近就后退，手在 15~50 cm 内就靠近并停在 15 cm。更远或没有目标则停。"""
    if mm < 20:
        return 0.0
    dist = mm / 1000.0
    if dist > FOLLOW_FAR_M:
        return 0.0
    err = dist - FOLLOW_M
    if abs(err) < FOLLOW_DEAD_M:
        return 0.0
    if err < 0.0:
        return max(-FOLLOW_REVERSE, err * FOLLOW_KP)
    return min(FOLLOW_FORWARD, err * FOLLOW_KP)


class TofFront:
    """VL53L5CX 最近距离。和 IMU 共用 I2C6，启动前必须先关掉 IMU。"""

    def __init__(self) -> None:
        self.proc: subprocess.Popen[str] | None = None
        self.mm = -1
        self.t = 0.0
        self._buf = ""
        self._err = ""

    def _paths(self) -> tuple[Path, Path, Path]:
        tof = REPO / "TOF"
        imu = REPO / "IMU"
        uld = tof / "VL53L5CX_ULD_driver"
        return tof, imu, uld

    def ensure_built(self) -> bool:
        tof, imu, uld = self._paths()
        binary = tof / "test_tof"
        sources = [
            tof / "test_tof.c",
            uld / "Platform" / "platform.c",
            uld / "VL53L5CX_ULD_API" / "src" / "vl53l5cx_api.c",
            imu / "i2c_gpio.c",
        ]
        if binary.exists() and all(binary.stat().st_mtime >= src.stat().st_mtime for src in sources):
            return True
        print("编译 TOF …", flush=True)
        cmd = [
            "gcc", "-O2", "-o", str(binary),
            f"-I{uld / 'VL53L5CX_ULD_API' / 'inc'}",
            f"-I{uld / 'Platform'}",
            f"-I{imu}",
            *[str(src) for src in sources],
        ]
        try:
            subprocess.check_call(cmd)
        except (OSError, subprocess.CalledProcessError) as exc:
            print(f"TOF 编译失败：{exc}", flush=True)
            return False
        return True

    def start(self) -> bool:
        self.stop()
        if not self.ensure_built():
            return False
        binary = self._paths()[0] / "test_tof"
        try:
            self.proc = subprocess.Popen(
                [str(binary), "-m"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except OSError as exc:
            print(f"TOF 没启动：{exc}", flush=True)
            self.proc = None
            return False
        for pipe in (self.proc.stdout, self.proc.stderr):
            if pipe is not None:
                os.set_blocking(pipe.fileno(), False)
        print("TOF 正在下载固件，第一次可能要几十秒。这段时间摇杆仍可驾驶。", flush=True)
        return True

    def _read_pipe(self, pipe) -> str:
        if pipe is None:
            return ""
        parts: list[str] = []
        while True:
            try:
                chunk = os.read(pipe.fileno(), 4096)
            except BlockingIOError:
                break
            except OSError:
                break
            if not chunk:
                break
            parts.append(chunk.decode(errors="replace"))
        return "".join(parts)

    def fresh(self) -> bool:
        return self.t > 0.0 and time.monotonic() - self.t <= 0.8

    def poll(self) -> int:
        proc = self.proc
        if proc is None:
            return -1
        err = self._err + self._read_pipe(proc.stderr)
        self._err = ""
        while "\n" in err:
            line, err = err.split("\n", 1)
            line = line.strip()
            if line:
                print(line, flush=True)
        self._err = err
        text = self._buf + self._read_pipe(proc.stdout)
        self._buf = ""
        while "\n" in text:
            line, text = text.split("\n", 1)
            line = line.strip()
            if not line.startswith("D "):
                continue
            try:
                self.mm = int(line.split()[1])
                self.t = time.monotonic()
            except ValueError:
                pass
        self._buf = text
        if proc.poll() is not None or not self.fresh():
            return -1
        return self.mm

    def stop(self) -> None:
        proc = self.proc
        self.proc = None
        self.mm = -1
        self.t = 0.0
        self._buf = ""
        self._err = ""
        if proc is None:
            return
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=0.2)


class LedRing:
    """灯环在单独进程里刷新，避免 WS2812 位bang 卡住控制环。手柄 X 轮询切换。

    和 locomotion/scripts/watch_policy.py 的 LedRing 同一套 fifo 协议。
    """

    ORDER = (
        ("battery", "电量"),
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
        self._soc = -1
        self._flash_t0: float | None = None
        self._flash_phase = ""

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
            "--remote", "--fifo", str(self.fifo), "--effect", "battery",
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
                    i for i, (key, _label) in enumerate(self.ORDER) if key == "battery"
                )
                self.label = self.ORDER[self.index][1]
                print("灯环已接通，当前电量。第一颗纯绿，逐颗变红，最后一颗纯红；电量下降从绿色那头熄灭。")
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
            elif key == "battery":
                self._soc = -1
            else:
                os.write(self.fifo_fd, b"b 64\n")
                self._bright = 64
            os.write(self.fifo_fd, (key + "\n").encode())
        except OSError:
            return f"{label}（灯环写失败）"
        return label

    def set_battery(self, pct: float) -> None:
        """默认灯效：电量 0~100 点亮对应比例的灯，颜色从绿到红。"""
        if self._flash_t0 is not None:
            return
        if self.fifo_fd < 0 or self.index < 0 or self.ORDER[self.index][0] != "battery":
            return
        soc = int(round(max(0.0, min(100.0, pct))))
        if soc == self._soc:
            return
        self._soc = soc
        try:
            os.write(self.fifo_fd, f"soc {soc}\n".encode())
        except OSError:
            self.fifo_fd = -1

    def begin_flash(self) -> None:
        """白光闪 3 次，周期 0.5 秒，然后回到电量灯。"""
        self._flash_t0 = time.monotonic()
        self._flash_phase = ""

    def poll_flash(self) -> None:
        if self._flash_t0 is None or self.fifo_fd < 0:
            return
        elapsed = time.monotonic() - self._flash_t0
        if elapsed >= 1.5:
            self._flash_t0 = None
            self._flash_phase = ""
            self._soc = -1
            try:
                os.write(self.fifo_fd, b"battery\n")
            except OSError:
                self.fifo_fd = -1
            return
        phase = "white" if (elapsed % 0.5) < 0.25 else "off"
        if phase == self._flash_phase:
            return
        self._flash_phase = phase
        try:
            os.write(self.fifo_fd, (phase + "\n").encode())
        except OSError:
            self.fifo_fd = -1

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
    """机身俯仰和侧倾负反馈。前倾为正，左侧抬起的 roll 为正。

    倾角不用重力低通。重力低通会把观测拖后一两百毫秒，补偿按过时角度去追就会震荡。
    这里用互补滤波：陀螺积分给出几乎无延迟的倾角，加速度只以较慢的时间常数
    把漂移拉回来。阻尼按角速度的「度/秒」计算，和比例项同一量纲。
    侧倾不改脖子，只输出左右腿屈伸差：正值表示左腿多屈、右腿多伸。
    """

    ACC_TAU_S = 0.8
    # 比例只看倾角的慢变化（坡度）。4 秒左右的来回晃不进比例环，避免髋追着自己晃。
    HOLD_TAU_S = 0.7
    RATE_DEG_S = 50.0
    DEAD_DEG = 1.5
    RATE_LPF_S = 0.08
    RATE_DEAD_DPS = 4.0
    # 侧倾仍可用整段屈伸。只看倾角、不加角速度，避免腿跟着抖。
    # 持续大约 18° 打满。输出最快 0.7/s，大约 1.4 秒能从平路走到一边最低、一边伸直。
    ROLL_HOLD_TAU_S = 0.5
    ROLL_DEAD_DEG = 2.0
    # 小于这个角度就停住。要再超过 ROLL_RESUME_DEG 才重新动，避免在水平附近自己激振。
    ROLL_SETTLE_DEG = 1.5
    ROLL_RESUME_DEG = 5.0
    ROLL_KP_U = 0.055
    ROLL_MAX_U = 1.0
    ROLL_RATE_U_S = 0.4

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
        self.roll_deg: float | None = None
        self.roll_hold: float | None = None
        self.roll_rate_f = 0.0
        self.roll_u = 0.0
        self.roll_settled = False

    @staticmethod
    def _accel_lean_deg(gravity: tuple[float, float, float]) -> float:
        gx, gy, gz = gravity
        return math.degrees(math.atan2(gx, math.hypot(gy, gz)))

    @staticmethod
    def _accel_roll_deg(gravity: tuple[float, float, float]) -> float:
        """绕 X。正值是左侧抬起，和 watch_policy 的倾斜定义一致。"""
        _gx, gy, gz = gravity
        return math.degrees(math.atan2(-gy, -gz))

    def update(self, dt: float, active: bool = True) -> tuple[float, float, float, float]:
        """返回（前倾角°，髋补偿°，颈补偿°，侧倾屈伸差）。关闭时补偿平滑回零。"""
        if not self.imu.ok:
            return 0.0, 0.0, 0.0, 0.0
        try:
            gyro, gravity = self.imu.read()
        except Exception:
            lean = 0.0 if self.lean_deg is None else self.lean_deg
            return lean, self.delta, self.delta * self.neck_ratio, self.roll_u
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

        roll_meas = self._accel_roll_deg(gravity)
        roll_rate = math.degrees(float(gyro[0]))
        if self.roll_deg is None:
            self.roll_deg = roll_meas
        else:
            self.roll_deg += roll_rate * dt
            self.roll_deg += (roll_meas - self.roll_deg) * (dt / self.ACC_TAU_S)
        if self.roll_hold is None:
            self.roll_hold = roll_meas
        else:
            rb = dt / (self.ROLL_HOLD_TAU_S + dt)
            self.roll_hold += rb * (roll_meas - self.roll_hold)
        self.roll_rate_f += a * (roll_rate - self.roll_rate_f)
        roll_mag = abs(self.roll_hold)
        # 倾角正在往回摆时不要再加屈伸，否则会冲过水平，形成左右自激。
        returning = self.roll_hold * self.roll_rate_f < 0.0 and abs(self.roll_rate_f) > 6.0
        if not active:
            self.roll_settled = False
        elif self.roll_settled:
            if roll_mag > self.ROLL_RESUME_DEG:
                self.roll_settled = False
        elif roll_mag < self.ROLL_SETTLE_DEG:
            self.roll_settled = True
        if self.roll_settled or returning:
            # 已经放平，或正在回正：停在现在的左右腿屈伸上。
            roll_raw = self.roll_u
        else:
            roll_eff = self.roll_hold
            if abs(roll_eff) > self.ROLL_DEAD_DEG:
                roll_eff -= math.copysign(self.ROLL_DEAD_DEG, roll_eff)
            else:
                roll_eff = 0.0
            # 左侧抬起时左腿多屈、右腿多伸。只用慢倾角，不加角速度进比例。
            roll_raw = self.ROLL_KP_U * roll_eff * self.flip
            roll_raw = max(-self.ROLL_MAX_U, min(self.ROLL_MAX_U, roll_raw))
            if not active:
                roll_raw = 0.0
        roll_step = self.ROLL_RATE_U_S * dt
        self.roll_u += max(-roll_step, min(roll_step, roll_raw - self.roll_u))
        return lean, self.delta, self.delta * self.neck_ratio, self.roll_u


# 髋 pitch 两侧电机符号：左 8 号为正向、右 3 号镜像为负向，补偿按机身方向施加时先乘这个。
HIP_SIGN = {8: 1.0, 3: -1.0}
# 左腿膝 7、髋 8，右腿膝 2、髋 3。
LEFT_LIFT = (7, 8)
RIGHT_LIFT = (2, 3)
NECK_PITCH_ID = 12
# 脖子每次最多走 5°，剩余不足 5° 时停在目标上，不冲过去。
NECK_STEP_DEG = 5.0
NECK_STEP_S = 0.12
HEAD_SLEW_DEG_S = 20.0
# 到位后收紧速度上限，只留修正误差的余量。进站立本身按过渡时间平滑走。
NECK_HOLD_RAD_S = 0.5
# 15 号是嘴。右扳机 0~1 映射到 0~60°。力矩按小电机额定，速度不要一下打到底。
MOUTH_ID = 15
MOUTH_OPEN_DEG = 60.0
MOUTH_TQE_NM = 0.3
MOUTH_VEL_RAD_S = 1.5
JUMP_HALF_S = 0.12
# 十字键左右：屈伸差 0.3，一侧多屈、另一侧多伸。正值是左侧降低。
DANCE_U = 0.50
DANCE_RATE_U_S = 1.2
DANCE_PAUSE_S = 3.0
Y_HOLD_S = 1.0


class Gamepad:
    """读 /dev/input/js* 的最新摇杆位置。轴号和 watch_policy.py 一致。"""

    PATHS = ("/dev/input/js0", "/dev/input/js1", "/dev/input/js2", "/dev/input/js3")
    # 模拟扳机轴。数字键按北通常见布局：左侧 6/4，右侧 7/5。
    LT_AXIS = 5

    def __init__(self) -> None:
        self.axis = [0.0] * 8
        self.axis_rest = [None] * 8
        self.buttons = [False] * 16
        self.button_a = False
        self.button_b = False
        self.button_x = False
        self.button_y = False
        self.y_since: float | None = None
        self.y_long = False
        self.y_armed = True
        self.rt_held = False
        # 北通：X 是 2 和 3，Y 是 4。Xbox 360：X 是 2，Y 是 3。
        self.x_buttons = (2, 3)
        self.y_button = 4
        # 北通：右摇杆是轴 2/3，LT 是轴 5。Xbox 360 的轴序不同，接入后再改。
        self.spin_axis = 2
        self.height_axis = 3
        self.lt_axis = 5
        self.rt_axis = 4
        self.rt_buttons = (5, 7)
        self.axis_fixed = [False] * 8
        self.axis_mid = [0] * 8
        self.fd = -1
        self.path = ""
        self.name = ""
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
            self.name = self._device_name()
            self._apply_button_map()
            self._next_warn = 0.0
            print(
                f"手柄已接入 {path}（{self.name}）。"
                f"Y 是 {self.y_button} 号键。长按 Y 进入站立，短按 Y 开关自平衡，B 键退出驾驶。"
            )
            return
        if now >= self._next_warn:
            print(
                f"手柄未连接：在 {', '.join(self.PATHS)} 都没找到（{last_exc}）。"
                "检查接收器和手柄电源，轮子保持停止，插好后会自动重连。"
            )
            self._next_warn = now + 5.0

    def _device_name(self) -> str:
        if self.fd < 0:
            return ""
        buf = bytearray(128)
        req = (2 << 30) | (128 << 16) | (ord("j") << 8) | 0x13
        try:
            import fcntl

            fcntl.ioctl(self.fd, req, buf, True)
        except OSError:
            return ""
        return bytes(buf).split(b"\x00", 1)[0].decode(errors="replace")

    def _apply_button_map(self) -> None:
        low = self.name.lower()
        if "x-box" in low or "xbox" in low or "360" in low:
            # 轴序按 ABS 编号：2 是 LT，3/4 是右摇杆，5 是 RT。2 号轴松开停在端点。
            self.x_buttons = (2,)
            self.y_button = 3
            self.spin_axis = 3
            self.height_axis = 4
            self.lt_axis = 2
            self.rt_axis = 5
            self.rt_buttons = ()
        else:
            self.x_buttons = (2, 3)
            self.y_button = 4
            self.spin_axis = 2
            self.height_axis = 3
            self.lt_axis = 5
            self.rt_axis = 4
            self.rt_buttons = (5, 7)

    def drop(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1
        self.axis = [0.0] * 8
        self.axis_rest = [None] * 8
        self.axis_fixed = [False] * 8
        self.axis_mid = [0] * 8
        self.buttons = [False] * 16
        self.y_since = None
        self.y_long = False
        self.y_armed = True
        self.rt_held = False
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
                        self._note_axis(number)
                    elif kind == 0x01 and number < 16:
                        self.buttons[number] = bool(value)
                        if not init and number in self.rt_buttons:
                            self.rt_held = bool(value)
                        if not init and value == 1 and number == 0:
                            self.button_a = True
                        if not init and value == 1 and number == 1:
                            self.button_b = True
                        # 北通 X 是 2 和 3；Xbox 360 的 X 只是 2，3 是脸上的 Y。
                        if not init and value == 1 and number in self.x_buttons:
                            self.button_x = True
                        # 按住约 1 秒是长按，松开前不到 1 秒算短按。
                        if not init and number == self.y_button:
                            if value == 1:
                                if self.y_armed and self.y_since is None:
                                    self.y_since = time.monotonic()
                            else:
                                if (
                                    self.y_since is not None
                                    and not self.y_long
                                    and time.monotonic() - self.y_since < Y_HOLD_S
                                ):
                                    self.button_y = True
                                self.y_since = None
                                self.y_armed = True
                                self.y_long = False
        fwd = -self._from_rest(1)
        turn = -self._from_rest(0)
        # 右摇杆上电若停在端点，先扣掉静止值，否则一进驾驶就满速自转。
        spin = -self._from_rest(self.spin_axis)
        height = -self._from_rest(self.height_axis)
        # 档位：0 普通，1 按 LT 高速（实测 20 已到电机上限，不设更高档）。
        # 高度联动：左侧键（LT/LB）随速度把高度压到 1/3 以内，右侧键（RT/RB）压到最低。
        lt = self.buttons[6] or self.buttons[4] or self._trigger(self.lt_axis) > 0.3
        rt = any(self.buttons[n] for n in self.rt_buttons) or self._trigger(self.rt_axis) > 0.3
        gear = 1 if lt else 0
        if (
            self.y_armed
            and self.y_since is not None
            and not self.y_long
            and time.monotonic() - self.y_since >= Y_HOLD_S
        ):
            self.y_long = True
            self.y_armed = False
            self.y_since = None
        if self.fd < 0:
            return 0.0, 0.0, 0.0, 0, 0.0, False, False
        return fwd, turn, spin, gear, height, lt, rt

    def take_long_y(self) -> bool:
        """长按 Y 满 1 秒触发一次。短按仍走 button_y。"""
        if not self.y_long:
            return False
        self.y_long = False
        self.button_y = False
        return True

    def clear_edges(self) -> None:
        self.button_a = False
        self.button_b = False
        self.button_x = False
        self.button_y = False
        self.y_long = False

    def rt_level(self) -> float:
        """右扳机开度 0~1。静止初值不算按下，必须离开静止位置才张开。"""
        analog = self._trigger(self.rt_axis)
        if self.rt_held:
            analog = 1.0
        return analog

    def _note_axis(self, number: int) -> None:
        """摇杆若能停在中间，中间就是零位。一直停在端点，端点就是零位，拨开才算有指令。"""
        raw = self.axis[number]
        rest = self.axis_rest[number]
        if rest is None:
            self.axis_rest[number] = raw
            return
        if self.axis_fixed[number]:
            return
        if abs(raw) <= 0.25 and abs(rest) < 0.85:
            self.axis_mid[number] += 1
            if self.axis_mid[number] >= 5:
                self.axis_rest[number] = raw
                self.axis_fixed[number] = True
            return
        self.axis_mid[number] = 0
        if abs(raw) >= 0.85:
            self.axis_rest[number] = raw

    def _trigger(self, number: int) -> float:
        """扳机开度 0~1。松开停在 +1 或 -1 都不算按下。"""
        raw = self.axis[number]
        rest = self.axis_rest[number]
        if rest is None:
            return 0.0
        analog = 0.0
        if abs(rest) >= 0.5:
            travel = -2.0 * rest
            delta = raw - rest
            if travel < 0.0 and delta < 0.0:
                analog = delta / travel
            elif travel > 0.0 and delta > 0.0:
                analog = delta / travel
        elif abs(rest) <= 0.35 and raw - rest > 0.0:
            analog = raw - rest
        analog = max(0.0, min(1.0, analog))
        if analog < 0.08:
            analog = 0.0
        return analog

    def dpad_x(self) -> float:
        """十字键左右。向左为 -1，向右为 +1。轴 6 是帽键，按键 13/14 作备用。"""
        raw = self.axis[6]
        rest = self.axis_rest[6]
        if rest is not None and abs(rest) >= 0.85:
            raw = max(-1.0, min(1.0, raw - rest))
        x = 0.0
        if raw <= -0.5:
            x = -1.0
        elif raw >= 0.5:
            x = 1.0
        if self.buttons[13]:
            x = -1.0
        elif self.buttons[14]:
            x = 1.0
        return x

    def dpad_up(self) -> bool:
        """十字键上。帽键轴 7 向上为负，按键 11 作备用。"""
        raw = self.axis[7]
        rest = self.axis_rest[7]
        if rest is not None and abs(rest) >= 0.85:
            raw = max(-1.0, min(1.0, raw - rest))
        if raw <= -0.5 or self.buttons[11]:
            return True
        return False

    def _from_rest(self, number: int) -> float:
        raw = self.axis[number]
        rest = self.axis_rest[number]
        if rest is None:
            return 0.0
        # 还没确认过中间零位、人又没拨开：停在端点不算打杆。
        if not self.axis_fixed[number] and abs(raw) >= 0.85:
            return 0.0
        if abs(rest) >= 0.85:
            return max(-1.0, min(1.0, raw - rest))
        return raw

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


def boot_led_active() -> bool:
    """开机灯效服务还在占 GPIO 时为真。没装这个服务就当作已经结束。"""
    try:
        result = subprocess.run(
            ["systemctl", "is-active", "--quiet", "led-boot.service"],
            timeout=1,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


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
    parser.add_argument("--hip-tqe", type=float, default=3.0, help="自平衡开启时髋 pitch 力矩上限 Nm")
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

    bus = None
    ids: list[int] = []
    for attempt in range(1, 16):
        try:
            bus, ids = pose.open_bus(cfg)
            break
        except SystemExit as exc:
            print(f"关节还没应答（{attempt}/15）：{exc}。2 秒后重试，请确认电机已上电、CAN 线接好。", flush=True)
            time.sleep(2.0)
    if bus is None:
        return 2
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
    mouth_type = str(cfg.get("motor_type", "4538_19"))
    bus.add_motor(Motor(MOUTH_ID, mouth_type, "mouth"))
    mouth_on = MOUTH_ID in bus.scan([MOUTH_ID], timeout_each=0.08)
    if mouth_on:
        bus.set_timeout_ms(MOUTH_ID, 0)
        print("15 号嘴部在线。右扳机松开 0°，按到底 60°。")
    else:
        print("15 号嘴部无应答，右扳机不控嘴。")

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
    pad = None
    tof: TofFront | None = None
    try:
        if boot_led_active():
            print("等待开机灯效结束。灯效没完之前不站立、不驾驶。")
            while running and boot_led_active():
                time.sleep(0.2)
        if not running:
            return 0
        print("开机灯效已结束。长按 Y 约 1 秒进入站立，之后才能驾驶。")
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
                    "前后倾动髋，侧倾用左右腿屈伸。默认开启，按 Y 退出。"
                )
        pad = Gamepad()
        tof = TofFront()
        tof_on = False
        mouth_closed: float | None = None
        next_batt = 0.0
        next_can = 0.0
        while running:
            print("电机保持卸力。长按 Y 约 1 秒进入站立，之后才能驾驶。")
            pad.clear_edges()
            while running:
                now_wait = time.monotonic()
                pad.poll()
                pad.button_a = False
                pad.button_b = False
                pad.button_x = False
                pad.button_y = False
                if now_wait >= next_batt:
                    publish_battery(bus, led, ids[0])
                    next_batt = now_wait + 0.5
                if now_wait >= next_can:
                    keep_can_up(str(cfg.get("interface", "can0")))
                    next_can = now_wait + 2.0
                if pad.take_long_y():
                    keep_can_up(str(cfg.get("interface", "can0")))
                    print("长按 Y：进入站立。")
                    break
                time.sleep(period)
            if not running:
                break
            # 默认进入自平衡。短按 Y 可以关掉。
            balance_on = balance is not None
            # 进入站立：只给非轮关节发站立指令。轮电机保持失能，禁止状态下不转。
            for mid in hold_ids:
                bus.enable(mid)
                bus.set_torque_limit(mid, torque_nm)
            for wid in WHEELS:
                bus.disable(wid)
            mouth_closed = None
            if mouth_on:
                bus.read_status(MOUTH_ID)
                bus.collect(timeout=0.01)
                if bus.motors[MOUTH_ID].state.t_pos > 0.0:
                    mouth_closed = bus.motors[MOUTH_ID].state.position / pose.DEG
                    bus.enable(MOUTH_ID)
                    bus.set_torque_limit(MOUTH_ID, MOUTH_TQE_NM)
                    command_mouth(bus, 0.0, mouth_closed)
                    print(f"嘴部保持当前 {mouth_closed:+.1f}°，右扳机在此基础上张开最多 60°。")
                else:
                    print("嘴部没有读到当前位置，本次不控嘴。")
            print("站立阶段轮电机（1、6 号）保持失能，不发指令。")
            time.sleep(0.05)
            start_deg = pose.read_degrees(bus, hold_ids)
            duration = max(0.5, float(cfg.get("transition_s", 3.0)))
            print(f"进入站立姿态 {args.pose}，{duration:.1f}s 平滑到位 ...")
            body_ids = [mid for mid in hold_ids if mid not in NECK_IDS]
            neck_hold = [mid for mid in NECK_IDS if mid in hold_ids and mid in stand_deg]
            move_ids = body_ids + neck_hold
            # smootherstep 两端慢、中段最快（系数 1.875）。速度上限只取这段峰值，3 秒内到，不再加码。
            peak_deg = max((abs(stand_deg[mid] - start_deg[mid]) for mid in move_ids), default=0.0)
            move_vel = 1.875 * math.radians(peak_deg) / duration if duration > 0 else NECK_HOLD_RAD_S
            move_vel = max(NECK_HOLD_RAD_S, move_vel)
            print(f"最大行程 {peak_deg:.0f}°，速度上限 {move_vel:.2f} rad/s。")
            neck_cmd = {mid: float(start_deg.get(mid, stand_deg[mid])) for mid in neck_hold}
            neck_step_at = 0.0

            t0 = time.monotonic()
            while running:
                now = time.monotonic()
                u = (now - t0) / duration
                s = pose.smootherstep(u)
                cmd_deg = {
                    mid: start_deg[mid] + (stand_deg[mid] - start_deg[mid]) * s for mid in move_ids
                }
                for mid in neck_hold:
                    neck_cmd[mid] = cmd_deg[mid]
                if balance is not None:
                    # 站立过渡阶段静止，不输出平衡补偿，避免原地来回抢。
                    _lean, d_hip, _d_neck, _roll_u = balance.update(period, False)
                    for hid, sign in HIP_SIGN.items():
                        if hid in cmd_deg:
                            cmd_deg[hid] += sign * d_hip
                command_position_seek(bus, move_ids, cmd_deg, torque_nm, move_vel)
                if mouth_on:
                    pad.poll()
                    pad.button_a = False
                    pad.button_b = False
                    pad.button_x = False
                    pad.button_y = False
                    command_mouth(bus, pad.rt_level(), mouth_closed)
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
                bus.set_torque_limit(wid, args.wheel_tqe)
            time.sleep(0.05)

            # 接管检查：连续发 0 速 0.6 秒，轮子必须停住，否则不进驾驶。
            takeover_ok = False
            check_t0 = time.monotonic()
            while running and time.monotonic() - check_t0 < 0.6:
                command_position_seek(bus, body_ids, stand_deg, torque_nm, NECK_HOLD_RAD_S)
                if neck_hold:
                    command_position_seek(bus, neck_hold, neck_cmd, torque_nm, NECK_HOLD_RAD_S)
                if mouth_on:
                    pad.poll()
                    command_mouth(bus, pad.rt_level(), mouth_closed)
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
                    "roll_deg", "roll_u",
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
                f"普通档 ±{args.vmax:.0f} rad/s，按住 LT 切高速档 ±{args.vmax_fast:.0f} rad/s。LT 时高度随速度压到 1/3 以内，RT 时压到最低，同时嘴巴从 0° 开到 60°。"
                f"平时力矩 {torque_nm:.1f} N·m，自平衡时髋 {args.hip_tqe:.1f} N·m。"
                "测距跟随默认关闭，按十字键上才打开。"
                "B 键卸力退出驾驶，再长按 Y 恢复。Ctrl+C 结束程序。"
            )
            drive_t0 = time.monotonic()
            next_print = drive_t0
            prev_t = drive_t0
            boost_prev = 0
            trig_prev = 0
            hip_tqe_sent = torque_nm
            hip_hold = 0.0
            prev_lean = 0.0
            hip_frozen = False
            jump_t0 = None
            dance_u = 0.0
            roll_pause_until = 0.0
            dpad_up_prev = False
            dpad_up_arm = False
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
                    pad.take_long_y()
                    gear_now = int(boost)
                    if pad.button_b:
                        print("B 键停车，卸力退出驾驶。长按 Y 可再进入。")
                        break
                    if pad.button_x:
                        pad.button_x = False
                        print(f"灯效：{led.cycle()}", flush=True)
                    led.drain()
                    led.poll_flash()
                    dpad_up = pad.dpad_up()
                    # 上电时帽键若停在“上”，不当成一次按下。松手之后再按才打开跟随。
                    if not dpad_up_arm:
                        if not dpad_up:
                            dpad_up_arm = True
                    elif dpad_up and not dpad_up_prev and jump_t0 is None:
                        if not tof_on:
                            print("十字键上：打开测距。障碍近于 15 cm 后退，手在前面保持 15 cm。", flush=True)
                            led.begin_flash()
                            led.poll_flash()
                            if imu is not None:
                                imu.close()
                            if tof is not None and tof.start():
                                tof_on = True
                            else:
                                print("测距没打开，继续普通驾驶。", flush=True)
                                imu = BodyImu()
                                if balance is not None:
                                    balance.imu = imu
                        else:
                            print("十字键上：关闭测距，恢复自平衡。", flush=True)
                            if tof is not None:
                                tof.stop()
                            tof_on = False
                            imu = BodyImu()
                            if balance is not None:
                                balance.imu = imu
                                balance_on = imu.ok
                    dpad_up_prev = dpad_up
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
                    if tof_on and tof is not None and jump_u is None:
                        mm = tof.poll()
                        if mm >= 20:
                            cmd_left = follow_speed(mm)
                            cmd_right = cmd_left

                    # 右摇杆上推升高（u 增大、走向伸直点），下拉降低（走向收缩点）。
                    if lift_ok and jump_u is None:
                        height_u = max(
                            0.0,
                            min(1.0, height_u + deadzone(height_s) * args.height_rate * dt_real),
                        )
                    # 扳机联动：高度随当前轮速同步变化。按下过程中速度一涨，
                    # 高度上限立刻跟着降：LT 封顶 1/3，RT 封顶最低。
                    vel_mag = max(abs(cmd_left), abs(cmd_right))
                    speed_frac = min(1.0, vel_mag / max(args.vmax_fast, 1.0))
                    if rt_on:
                        height_cap = 1.0 - speed_frac
                    elif lt_on:
                        height_cap = 1.0 - speed_frac * (2.0 / 3.0)
                    else:
                        height_cap = 1.0
                    trig_now = 2 if rt_on else (1 if lt_on else 0)
                    if trig_now != trig_prev:
                        if trig_now == 2:
                            print("RT 联动：高度随速度同步降到最低。", flush=True)
                        elif trig_now == 1:
                            print("LT 联动：高度随速度同步降到 1/3 以内。", flush=True)
                        trig_prev = trig_now
                    if height_u > height_cap:
                        height_u = height_cap

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
                        lean_deg, d_hip, d_neck, roll_u = balance.update(dt_real, balance_active)
                    else:
                        lean_deg, d_hip, d_neck, roll_u = 0.0, 0.0, 0.0, 0.0
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

                    pose.command_position(bus, pose_ids, stand_deg, torque_nm)
                    leg_u = jump_u if jump_u is not None else height_u
                    # 起跳时两腿一起动。十字键左右把机身倾斜 50%。
                    # 按下后左右自平衡暂停 3 秒，按住期间会一直顺延。
                    # 正的 side 是左腿多屈、右腿多伸，机身向左倾。
                    if jump_u is not None or not lift_ok:
                        dance_u = 0.0
                        side = 0.0
                    else:
                        dpad = pad.dpad_x()
                        if dpad != 0.0:
                            roll_pause_until = loop_t + DANCE_PAUSE_S
                        dance_goal = 0.0 if dpad == 0.0 else (DANCE_U if dpad < 0.0 else -DANCE_U)
                        dance_u = ramp(dance_u, dance_goal, DANCE_RATE_U_S, dt_real)
                        if loop_t < roll_pause_until:
                            side = dance_u
                            if balance is not None:
                                balance.roll_u = dance_u
                        else:
                            side = roll_u
                    left_u = max(0.0, min(1.0, leg_u - side))
                    right_u = max(0.0, min(1.0, leg_u + side))
                    lift_deg = {}
                    for mid in lift_ids:
                        if mid not in stand_deg or mid not in extended_deg:
                            continue
                        side_u = left_u if mid in LEFT_LIFT else right_u
                        lift_deg[mid] = stand_deg[mid] + (extended_deg[mid] - stand_deg[mid]) * side_u
                    for hid, sign in HIP_SIGN.items():
                        if hid in lift_deg:
                            lift_deg[hid] += sign * d_hip
                    knee_ids = [mid for mid in lift_ids if mid not in HIP_SIGN and mid in lift_deg]
                    hip_ids = [mid for mid in lift_ids if mid in HIP_SIGN and mid in lift_deg]
                    knee_tqe = hip_tqe_now if balance_active or jump_u is not None else torque_nm
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
                        head_goal = head_goal_for(
                            neck_cmd.get(NECK_PITCH_ID, 0.0), stand_deg, upright_deg
                        )
                        if head_goal is not None:
                            # 升降时头仍慢跟上。进入初始姿态时目标就是实测头角，不再先脖子后头。
                            neck_cmd[11] = ramp(neck_cmd.get(11, head_goal), head_goal, HEAD_SLEW_DEG_S, dt_real)
                        pose.command_position(bus, neck_ids, neck_cmd, torque_nm)
                    if mouth_on:
                        command_mouth(bus, pad.rt_level(), mouth_closed)
                    send_speed(bus, bus.motors[left_id], cmd_left * wheel_sign(left_id), args.wheel_tqe)
                    send_speed(bus, bus.motors[right_id], cmd_right * wheel_sign(right_id), args.wheel_tqe)

                    bus.read_status(left_id)
                    bus.read_status(right_id)
                    for hid in HIP_SIGN:
                        if hid in bus.motors:
                            bus.read_status(hid)
                    bus.collect(timeout=0.002)

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
                    volts = bus.motors[WHEEL_LEFT].state.voltage
                    if led is not None:
                        led.set_torque_level(torque_ratio)
                        if volts >= 8.0:
                            led.set_battery(battery_percent(volts))
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
                            f"{0.0 if balance is None or balance.roll_deg is None else balance.roll_deg:.3f}",
                            f"{roll_u:.4f}",
                        ]
                    )

                    if loop_t >= next_print:
                        bus.read_voltage(WHEEL_LEFT)
                        bus.collect(timeout=0.001)
                        volts = bus.motors[WHEEL_LEFT].state.voltage
                        print(
                            f"t={elapsed:5.1f}s  杆 前{fwd_s:+.2f} 转{turn_s:+.2f} 自{spin_s:+.2f}  "
                            f"档{gear_names[gear_now]}  升降 {height_u:4.2f}/≤{height_cap:4.2f}  "
                            f"前倾 {lean_deg:+5.1f}° 补偿 {d_hip:+5.2f}° "
                            f"侧倾 {0.0 if balance is None or balance.roll_deg is None else balance.roll_deg:+5.1f}° 屈伸 {roll_u:+.2f} "
                            f"髋力矩 {hip_torque:4.2f}/{hip_tqe_now:.1f} "
                            f"{'髋力矩顶住' if hip_frozen else ''} "
                            f"颈 {neck_cmd.get(NECK_PITCH_ID, 0.0):+5.1f}° 头 {neck_cmd.get(11, 0.0):+5.1f}° "
                            f"嘴 {MOUTH_OPEN_DEG * pad.rt_level():4.0f}° "
                            f"自平衡{'开' if balance_active else '关'}  "
                            f"轮指令 左{cmd_left:+6.2f} 右{cmd_right:+6.2f}  "
                            f"反馈 左{fb_left:+6.2f} 右{fb_right:+6.2f}  "
                            f"电压 {volts:.1f} V 电量 {battery_text(volts)}  灯 {led.label}  "
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

            if tof_on and tof is not None:
                tof.stop()
                tof_on = False
                imu = BodyImu()
                if balance is not None:
                    balance.imu = imu
            print("停车：轮速归零，然后直接卸力。脖子停在当前角度。")
            t_b = time.monotonic()
            while time.monotonic() - t_b < 0.3:
                pose.command_position(bus, pose_ids, stand_deg, torque_nm)
                pose.command_position(bus, lift_ids, lift_deg, torque_nm)
                if neck_ids:
                    pose.command_position(bus, neck_ids, neck_cmd, torque_nm)
                if mouth_on:
                    pad.poll()
                    command_mouth(bus, pad.rt_level(), mouth_closed)
                send_speed(bus, bus.motors[left_id], 0.0, args.wheel_tqe)
                send_speed(bus, bus.motors[right_id], 0.0, args.wheel_tqe)
                time.sleep(period)

            print("卸力：失能全部关节 ...")
            for mid in ids:
                bus.disable(mid)
            if mouth_on:
                bus.disable(MOUTH_ID)
            time.sleep(0.05)
            print("已卸力。")
            if not running:
                break
            print("长按 Y 约 1 秒恢复站立和驾驶。")
        return 0
    finally:
        if tof is not None:
            tof.stop()
        if pad is not None:
            pad.close()
        if imu is not None:
            imu.close()
        if led is not None:
            led.close()
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
