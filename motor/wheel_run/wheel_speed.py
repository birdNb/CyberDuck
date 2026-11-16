#!/usr/bin/env python3
"""轮行速度测试：只驱动指定电机，缓慢扫速度，看皮带何时跟不上。

默认只测 1 号。6 号用 --id 6。不会给总线上其他电机发速度。

速度控制走单电机普通帧 0x07 0x07：位置字 0x8000 表示不限位置，
后面是目标速度和力矩上限。软件侧再按 --accel 慢慢改目标速度。

启动时先自检连接：接口不存在或未拉起、电机无应答，都只提醒并退出，
不会自动 sudo，也不会发速度。

用法（在 motor 目录）：
  python3 wheel_run/wheel_speed.py
  python3 wheel_run/wheel_speed.py --id 1
  python3 wheel_run/wheel_speed.py --id 6 --vmax 10 --accel 0.5
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ht_can import (
    ACC_SCALE,
    MODE_VEL_ACC,
    MODE_VELOCITY,
    Motor,
    MotorBus,
    TWO_PI,
    _clip_i16,
    i16_le,
    vel_to_int,
)

LOG_DIR = Path(__file__).resolve().parent / "logs"
# 与 locomotion/scripts/watch_policy.py 的 battery_text 相同：12.8 V 为 0%，16.8 V 为 100%。
BATT_EMPTY_V = 12.8
BATT_FULL_V = 16.8


def iface_down_reason(iface: str) -> str | None:
    sysfs = f"/sys/class/net/{iface}"
    if not os.path.exists(sysfs):
        return f"{iface} 不存在（PCAN-USB 未识别到）"
    try:
        with open(f"{sysfs}/flags", encoding="utf-8") as f:
            flags = int(f.read().strip(), 16)
    except OSError:
        return f"无法读取 {iface} 状态"
    if not (flags & 0x1):
        return f"{iface} 处于 DOWN"
    return None


def send_speed(
    bus: MotorBus, motor: Motor, vel_rad_s: float, tqe_nm: float = 0.0, reply: bool = False
) -> None:
    """1、6 号轮驱：速度+加速度（0x1E），不写力矩上限。

    位置帧 0x1F 会把轮子力矩卡在 1 N·m，车重压着走不动。
    tqe_nm 保留参数，轮驱忽略它。
    reply 为真时带 0x0E，停车回读速度才用；驾驶循环保持不回包。
    """
    _ = tqe_nm
    acc_raw = _clip_i16(int(80.0 / TWO_PI * ACC_SCALE))
    # 查询码 0x00：不让轮子每拍回状态。0x0E 回包会堵住发送，摇杆和侧倾就丢、就慢。
    query = 0x0E if reply else 0x00
    data = (
        bytes([MODE_VEL_ACC, query])
        + i16_le(vel_to_int(vel_rad_s))
        + i16_le(acc_raw)
    )
    bus.send_cmd(motor.id, data, reply=False)


def battery_percent(volts: float) -> float:
    span = BATT_FULL_V - BATT_EMPTY_V
    return max(0.0, min(100.0, (volts - BATT_EMPTY_V) / span * 100.0))


def battery_text(volts: float) -> str:
    """12.8 V 为 0%，16.8 V 为 100%。颜色从红经黄到绿。"""
    pct = battery_percent(volts)
    t = pct / 100.0
    r = 255 if t < 0.5 else int(round(255 * (1.0 - t) * 2))
    g = int(round(255 * t * 2)) if t < 0.5 else 255
    return f"\033[38;2;{r};{g};0m{pct:3.0f}%\033[0m"


def yellow_nm(nm: float) -> str:
    return f"\033[33m{nm:+6.3f}\033[0m"


def poll_motor(bus: MotorBus, motor_id: int) -> None:
    """读关节状态和母线电压。电压寄存器 0x0D，1 LSB = 0.1 V。"""
    bus.read_status(motor_id)
    bus.collect(timeout=0.004)
    bus.read_voltage(motor_id)
    bus.collect(timeout=0.002)


def power_text(motor: Motor) -> str:
    volts = motor.state.voltage
    return f"电压 {volts:.1f} V 电量 {battery_text(volts)}  力矩 {yellow_nm(motor.state.torque)} Nm"


def step_toward(cur: float, goal: float, accel: float, dt: float) -> float:
    dv = abs(accel) * dt
    if cur < goal:
        return min(goal, cur + dv)
    if cur > goal:
        return max(goal, cur - dv)
    return goal


def brake(bus: MotorBus, motor: Motor, tqe_nm: float, seconds: float = 0.6) -> None:
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        send_speed(bus, motor, 0.0, tqe_nm)
        time.sleep(0.02)


def main() -> int:
    parser = argparse.ArgumentParser(description="轮行皮带打滑速度测试")
    parser.add_argument("--id", type=int, default=1, help="电机号，默认 1。测 6 号时加 --id 6")
    parser.add_argument("--iface", default="can0")
    parser.add_argument("--type", default="3532_0")
    parser.add_argument("--vmax", type=float, default=20.0, help="峰值速度 rad/s，扫到 +vmax 再扫到 -vmax")
    parser.add_argument("--accel", type=float, default=1.0, help="速度变化率 rad/s^2，越小越慢")
    parser.add_argument("--hold", type=float, default=2.0, help="在 ±vmax 停留秒数")
    parser.add_argument("--tqe", type=float, default=3.0, help="力矩上限 Nm")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--slip", type=float, default=1.5, help="反馈落后指令超过该值 rad/s 记一次异常")
    parser.add_argument("--countdown", type=float, default=3.0)
    args = parser.parse_args()

    if args.id not in (1, 6):
        print(f"这次轮行只允许 1 号或 6 号，收到 {args.id}", file=sys.stderr)
        return 2
    if args.vmax <= 0 or args.vmax > 25:
        print("峰值速度要在 0 到 25 rad/s 之间", file=sys.stderr)
        return 2
    if args.accel <= 0 or args.accel > 2:
        print("加速度要在 0 到 2 rad/s^2 之间", file=sys.stderr)
        return 2

    print("自检连接 ...")
    down = iface_down_reason(args.iface)
    if down:
        print(f"未连接：{down}", file=sys.stderr)
        print("请检查 PCAN-USB 是否插好。接口在但未拉起时先执行：sudo ./setup_can0.sh", file=sys.stderr)
        print("本次未打开总线，也没有给任何电机发指令。", file=sys.stderr)
        return 1
    print(f"  接口 {args.iface} 已拉起")

    try:
        bus = MotorBus(args.iface)
    except OSError as exc:
        print(f"未连接：打开 {args.iface} 失败：{exc}", file=sys.stderr)
        print("请检查 PCAN-USB 和 CAN 线。本次没有给任何电机发指令。", file=sys.stderr)
        return 1

    motor = bus.add_motor(Motor(args.id, args.type, f"wheel{args.id}"))
    running = True
    started = False

    def _stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        found = []
        for _ in range(2):
            found = bus.scan([args.id], timeout_each=0.1)
            if args.id in found:
                break
            time.sleep(0.05)
        if args.id not in found:
            print(f"未连接：{args.id} 号电机没有应答。", file=sys.stderr)
            print("请检查电机电源、CAN 高低线和终端电阻。本次没有发速度。", file=sys.stderr)
            return 2
        print(f"  {args.id} 号电机在线")

        bus.set_timeout_ms(args.id, 300)
        bus.set_torque_limit(args.id, args.tqe)
        bus.set_mode(args.id, MODE_VELOCITY)
        time.sleep(0.05)
        poll_motor(bus, args.id)
        print(f"  母线 {power_text(motor)}")
        print(
            f"开始前 pos={motor.state.position:+.3f} rad  "
            f"vel={motor.state.velocity:+.3f} rad/s  {power_text(motor)}"
        )

        up_s = args.vmax / args.accel
        span_s = (2.0 * args.vmax) / args.accel
        total = up_s + args.hold + span_s + args.hold + up_s
        print(
            f"只转 {args.id} 号：0 → +{args.vmax:.1f} → -{args.vmax:.1f} → 0 rad/s\n"
            f"加速度 {args.accel:.2f} rad/s^2，约 {total:.0f} 秒。"
            f"力矩上限 {args.tqe:.1f} Nm。Ctrl+C 立刻减速停。"
        )
        print("看皮带的时候，以屏幕上的「指令」速度为准。反馈明显落后时会打出异常。")
        for left in range(int(args.countdown), 0, -1):
            if not running:
                print("已取消。")
                return 0
            print(f"  {left} ...")
            time.sleep(1.0)
        if not running:
            print("已取消。")
            return 0

        print(f"先用 +0.4 rad/s 探一下 {args.id} 号是否听从速度帧 ...")
        probe_fb = 0.0
        probe_t = time.monotonic()
        while running and time.monotonic() - probe_t < 1.5:
            send_speed(bus, motor, 0.4, args.tqe)
            poll_motor(bus, args.id)
            probe_fb = motor.state.velocity
            time.sleep(0.02)
        send_speed(bus, motor, 0.0, args.tqe)
        time.sleep(0.15)
        print(f"探速反馈 {probe_fb:+.3f} rad/s  {power_text(motor)}")
        if not running:
            brake(bus, motor, args.tqe, 0.3)
            return 0
        if abs(probe_fb) < 0.08:
            print(
                f"{args.id} 号几乎没转。单电机速度帧 0x07 0x07 没有生效，已停止，不会继续加到 10 rad/s。",
                file=sys.stderr,
            )
            brake(bus, motor, args.tqe, 0.3)
            return 3

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = LOG_DIR / f"m{args.id}_{stamp}.csv"
        log_f = log_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(log_f)
        writer.writerow(
            [
                "t_s",
                "cmd_rad_s",
                "fb_vel_rad_s",
                "err_rad_s",
                "torque_nm",
                "pos_rad",
                "voltage_v",
                "battery_pct",
            ]
        )

        period = 1.0 / max(args.hz, 1.0)
        phases = [(args.vmax, args.hold), (-args.vmax, args.hold), (0.0, 0.4)]
        phase_i = 0
        holding = False
        hold_until = 0.0
        cmd = 0.0
        bad_for = 0.0
        events: list[tuple[float, float, float]] = []
        t0 = time.monotonic()
        next_print = t0
        started = True
        print("开始。")

        while running and phase_i < len(phases):
            loop_t = time.monotonic()
            goal, hold_s = phases[phase_i]
            if holding:
                if loop_t >= hold_until:
                    holding = False
                    phase_i += 1
                    if phase_i >= len(phases):
                        break
                    goal = phases[phase_i][0]
                    print(f"--- 转向 {goal:+.2f} rad/s ---", flush=True)
            cmd = step_toward(cmd, goal, args.accel, period)
            if not holding and abs(cmd - goal) < 1e-3:
                cmd = goal
                if hold_s > 0:
                    holding = True
                    hold_until = time.monotonic() + hold_s
                    print(f"--- 保持 {cmd:+.2f} rad/s  {hold_s:.1f}s ---", flush=True)
                else:
                    phase_i += 1

            send_speed(bus, motor, cmd, args.tqe)
            poll_motor(bus, args.id)
            fb = motor.state.velocity
            err = cmd - fb
            tor = motor.state.torque
            volts = motor.state.voltage
            pct = battery_percent(volts)
            elapsed = time.monotonic() - t0
            writer.writerow(
                [
                    f"{elapsed:.3f}",
                    f"{cmd:.4f}",
                    f"{fb:.4f}",
                    f"{err:.4f}",
                    f"{tor:.4f}",
                    f"{motor.state.position:.4f}",
                    f"{volts:.2f}",
                    f"{pct:.1f}",
                ]
            )

            if abs(cmd) >= 1.0 and abs(err) >= args.slip:
                bad_for += period
            else:
                bad_for = 0.0
            if bad_for >= 0.4:
                events.append((cmd, fb, tor))
                print(
                    f"*** 跟踪异常  t={elapsed:5.1f}s  指令 {cmd:+6.2f}  "
                    f"反馈 {fb:+6.2f}  误差 {err:+5.2f}  {power_text(motor)}",
                    flush=True,
                )
                bad_for = 0.0

            if loop_t >= next_print:
                print(
                    f"t={elapsed:5.1f}s  指令 {cmd:+6.2f}  反馈 {fb:+6.2f}  "
                    f"误差 {err:+5.2f}  {power_text(motor)}",
                    flush=True,
                )
                next_print = loop_t + 0.5

            dt = time.monotonic() - loop_t
            if dt < period:
                time.sleep(period - dt)

        log_f.flush()
        log_f.close()
        print("减速停止 ...", flush=True)
        brake(bus, motor, args.tqe)
        poll_motor(bus, args.id)
        print(f"停下后 vel={motor.state.velocity:+.3f} rad/s  {power_text(motor)}")
        if events:
            cmd0, fb0, tor0 = events[0]
            print(
                f"最早一次跟踪异常：指令 {cmd0:+.2f} rad/s，反馈 {fb0:+.2f} rad/s，力矩 {tor0:+.2f} Nm。"
                f"共 {len(events)} 次。日志 {log_path}"
            )
        else:
            print(
                f"整段反馈都跟上了指令（阈值 {args.slip:.1f} rad/s）。"
                f"若皮带在电机编码器外侧，打滑时反馈仍可能正常，请对照屏幕指令速度看轮子。日志 {log_path}"
            )
        return 0
    finally:
        if started:
            try:
                brake(bus, motor, args.tqe, 0.4)
            except OSError:
                pass
        print(f"失能 {args.id} 号 ...")
        try:
            bus.disable(args.id)
            time.sleep(0.05)
        except OSError:
            pass
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
