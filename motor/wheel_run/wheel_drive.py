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
    升降同时脖子（11、12、13、14 号）从 drive 角度同步滑向 stand 角度：
    腿完全伸直时机身最高，脖子正好到达 stand 的脖子方向。
  B 键：停车并退出
  X 键：切换灯效（和 watch_policy.py 同一套灯环）

全程力矩保护 1 N·m：站立保持、升降和轮电机的力矩上限都限制在 1 N·m。

结束流程：先停车，然后脖子（11、12、13、14 号）平滑回到 0 位，最后整机卸力。
退出前请扶稳机器人。

用法（在 motor 目录）：
  python3 wheel_run/wheel_drive.py
  python3 wheel_run/wheel_drive.py --vmax 6
  python3 wheel_run/wheel_drive.py --swap-wheels
"""

from __future__ import annotations

import argparse
import csv
import os
import select
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

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
        cmd = ["/usr/bin/python3", "-u", str(script), "--remote", "--fifo", str(self.fifo)]
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
                    i for i, (key, _label) in enumerate(self.ORDER) if key == "flow"
                )
                self.label = self.ORDER[self.index][1]
                print("灯环已接通，当前流水灯。手柄 X 切换灯效。")
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
            os.write(self.fifo_fd, (key + "\n").encode())
        except OSError:
            return f"{label}（灯环写失败）"
        return label

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


class Gamepad:
    """读 /dev/input/js* 的最新摇杆位置。轴号和 watch_policy.py 一致。"""

    PATHS = ("/dev/input/js0", "/dev/input/js1", "/dev/input/js2", "/dev/input/js3")
    # 模拟扳机轴。数字键按北通常见布局：左侧 6/4，右侧 7/5。
    LT_AXIS = 5

    def __init__(self) -> None:
        self.axis = [0.0] * 8
        self.buttons = [False] * 16
        self.button_b = False
        self.button_x = False
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
                        if not init and value == 1 and number == 1:
                            self.button_b = True
                        # 北通丝印 X 在 js 2，内核 BTN_X 在 js 3，两个都算。
                        if not init and value == 1 and number in (2, 3):
                            self.button_x = True
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
    parser.add_argument("--height-rate", type=float, default=0.15, help="升降全行程速率 1/s，右摇杆全推约 7 秒走完")
    parser.add_argument("--swap-wheels", action="store_true", help="交换左右轮（改回 1 号当左轮、6 号当右轮）")
    parser.add_argument("--neck-seconds", type=float, default=1.5, help="结束时脖子回 0 位的过渡秒数")
    parser.add_argument("--no-led", action="store_true", help="不接灯环，手柄 X 无效")
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
    try:
        led = LedRing()
        if not args.no_led:
            led.start()
        # 进入站立：只给非轮关节发站立指令。轮电机保持失能，禁止状态下不转。
        for mid in hold_ids:
            bus.enable(mid)
            bus.set_torque_limit(mid, torque_nm)
        for wid in WHEELS:
            bus.disable(wid)
        print("站立阶段轮电机（1、6 号）保持失能，不发指令。")
        time.sleep(0.05)
        start_deg = pose.read_degrees(bus, hold_ids)
        duration = max(0.5, float(cfg.get("transition_s", 3.0)))
        print(f"进入站立姿态 {args.pose}，过渡 {duration:.1f}s ...")

        t0 = time.monotonic()
        while running:
            now = time.monotonic()
            u = (now - t0) / duration
            s = pose.smootherstep(u)
            cmd_deg = {
                mid: start_deg[mid] + (stand_deg[mid] - start_deg[mid]) * s for mid in hold_ids
            }
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
            bus.set_torque_limit(wid, args.wheel_tqe)
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
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        log_path = LOG_DIR / f"drive_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        log_f = log_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(log_f)
        writer.writerow(
            [
                "t_s", "fwd", "turn", "spin", "gear", "height_stick", "height_u", "height_cap",
                "cmd_left_rad_s", "cmd_right_rad_s",
                "fb_left_rad_s", "fb_right_rad_s", "torque_left_nm", "torque_right_nm",
                "voltage_v", "battery_pct",
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
            f"普通档 ±{args.vmax:.0f} rad/s，按住 LT 切高速档 ±{args.vmax_fast:.0f} rad/s。LT 时高度随速度压到 1/3 以内，RT 时压到最低。"
            f"全程力矩保护 {torque_nm:.1f} N·m。B 键或 Ctrl+C 停车退出。"
        )
        drive_t0 = time.monotonic()
        next_print = drive_t0
        prev_t = drive_t0
        boost_prev = 0
        trig_prev = 0
        gear_names = ("普通", "高速")
        gear_caps = (args.vmax, args.vmax_fast)
        try:
            while running and takeover_ok:
                loop_t = time.monotonic()
                # 用真实循环间隔算斜坡，循环被拖慢时指令也不会滞后。
                dt_real = max(0.001, min(0.1, loop_t - prev_t))
                prev_t = loop_t
                fwd_s, turn_s, spin_s, boost, height_s, lt_on, rt_on = pad.poll()
                gear_now = int(boost)
                if pad.button_b:
                    print("B 键停车，退出驾驶。")
                    break
                if pad.button_x:
                    pad.button_x = False
                    print(f"灯效：{led.cycle()}", flush=True)
                led.drain()
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

                # 右摇杆上推升高（u 增大、走向伸直点），下拉降低（走向收缩点）。
                if lift_ok:
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

                pose.command_position(bus, pose_ids, stand_deg, torque_nm)
                lift_deg = {
                    mid: stand_deg[mid] + (extended_deg[mid] - stand_deg[mid]) * height_u
                    for mid in lift_ids
                }
                pose.command_position(bus, lift_ids, lift_deg, torque_nm)
                if neck_ids:
                    # 脖子同步：u=0（收缩）时在 drive 方向，u=1（伸直最高）到达 stand 方向。
                    neck_deg = {
                        mid: stand_deg[mid]
                        + (upright_deg[mid] - stand_deg[mid]) * height_u
                        for mid in neck_ids
                    }
                    pose.command_position(bus, neck_ids, neck_deg, torque_nm)
                send_speed(bus, bus.motors[left_id], cmd_left * wheel_sign(left_id), args.wheel_tqe)
                send_speed(bus, bus.motors[right_id], cmd_right * wheel_sign(right_id), args.wheel_tqe)

                bus.read_status(left_id)
                bus.read_status(right_id)
                bus.collect(timeout=0.002)

                lm = bus.motors[left_id]
                rm = bus.motors[right_id]
                fb_left = lm.state.velocity * wheel_sign(left_id)
                fb_right = rm.state.velocity * wheel_sign(right_id)
                tor_left = lm.state.torque * wheel_sign(left_id)
                tor_right = rm.state.torque * wheel_sign(right_id)
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
                    ]
                )

                if loop_t >= next_print:
                    bus.read_voltage(WHEEL_LEFT)
                    bus.collect(timeout=0.001)
                    volts = bus.motors[WHEEL_LEFT].state.voltage
                    print(
                        f"t={elapsed:5.1f}s  杆 前{fwd_s:+.2f} 转{turn_s:+.2f} 自{spin_s:+.2f}  "
                        f"档{gear_names[gear_now]}  升降 {height_u:4.2f}/≤{height_cap:4.2f}  "
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

        print("停车：轮速归零，其余关节保持站立 ...")
        t_b = time.monotonic()
        while time.monotonic() - t_b < 0.5:
            pose.command_position(bus, hold_ids, stand_deg, torque_nm)
            send_speed(bus, bus.motors[left_id], 0.0, args.wheel_tqe)
            send_speed(bus, bus.motors[right_id], 0.0, args.wheel_tqe)
            time.sleep(period)

        # 脖子（11..14）平滑回 0 位，其余关节继续保持站立，轮子保持停止。
        neck_ids = [mid for mid in NECK_IDS if mid in ids]
        if neck_ids:
            print(f"脖子 {neck_ids} 回 0 位，用时 {args.neck_seconds:.1f}s。请扶稳机器人，随后卸力。")
            try:
                neck_start = pose.read_degrees(bus, neck_ids)
            except RuntimeError:
                neck_start = {mid: stand_deg[mid] for mid in neck_ids}
            neck_t0 = time.monotonic()
            while True:
                now = time.monotonic()
                u = (now - neck_t0) / max(0.2, args.neck_seconds)
                s = pose.smootherstep(u)
                cmd_deg = dict(stand_deg)
                for mid in neck_ids:
                    cmd_deg[mid] = neck_start[mid] * (1.0 - s)
                pose.command_position(bus, hold_ids, cmd_deg, torque_nm)
                send_speed(bus, bus.motors[left_id], 0.0, args.wheel_tqe)
                send_speed(bus, bus.motors[right_id], 0.0, args.wheel_tqe)
                if u >= 1.0:
                    break
                time.sleep(period)

        print("卸力：失能全部关节 ...")
        for mid in ids:
            bus.disable(mid)
        time.sleep(0.05)
        print("已卸力。")
        pad.close()
        return 0
    finally:
        if led is not None:
            led.close()
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
