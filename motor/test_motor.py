#!/usr/bin/env python3
"""对应原工程 example/motor_run.cpp：pos_vel_MAXtqe 小幅摆动并打印反馈。

默认只扫描 + 读状态，加 --move 才发运动指令。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

from ht_can import Motor, MotorBus, format_state

try:
    import yaml
except ImportError:
    yaml = None


def load_motors(path: Path):
    if yaml is None or not path.exists():
        return "can0", [Motor(1, "4538_19", "L_low_foot")]
    cfg = yaml.safe_load(path.read_text()) or {}
    iface = cfg.get("interface", "can0")
    motors = []
    for item in cfg.get("motors") or []:
        motors.append(
            Motor(
                int(item["id"]),
                str(item.get("type", "4538_19")),
                str(item.get("name", f"motor{item['id']}")),
            )
        )
    if not motors:
        motors = [Motor(1, "4538_19", "L_low_foot")]
    return iface, motors


def main() -> int:
    parser = argparse.ArgumentParser(description="HighTorque PCAN 电机测试")
    parser.add_argument("-i", "--iface", default=None)
    parser.add_argument("-c", "--config", default=str(Path(__file__).with_name("motors.yaml")))
    parser.add_argument("--scan", action="store_true", help="忽略 yaml，扫描 1-14")
    parser.add_argument("--move", action="store_true", help="发 pos_vel_MAXtqe 小幅摆动")
    parser.add_argument("--velocity", type=float, default=None, metavar="RAD_S", help="速度模式，单位 rad/s，例如 0.5")
    parser.add_argument("--angle", type=float, default=0.2, help="摆动幅度 rad，默认 0.2")
    parser.add_argument("--vel", type=float, default=0.3, help="位置模式下的速度限制 rad/s")
    parser.add_argument("--tqe", type=float, default=2.0, help="最大力矩 Nm")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--period", type=float, default=2.0, help="换向周期秒")
    parser.add_argument("--seconds", type=float, default=0.0, help="运行秒数，0 表示直到 Ctrl+C")
    args = parser.parse_args()

    cfg_iface, cfg_motors = load_motors(Path(args.config))
    iface = args.iface or cfg_iface

    print(f"打开 {iface} ...")
    try:
        bus = MotorBus(iface)
    except OSError as exc:
        print(f"打开 {iface} 失败: {exc}", file=sys.stderr)
        print("请先: sudo ./setup_can0.sh", file=sys.stderr)
        return 1

    running = True

    def _stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        if args.scan:
            print("扫描电机 ID 1..14 ...")
            found = bus.scan(range(1, 15), timeout_each=0.05)
            if not found:
                print("未发现电机。请改跑: python3 probe_can.py")
                return 2
            print(f"发现: {found}")
            motors = [bus.motors[i] for i in found]
            for m in motors:
                if m.type == "4538_19" and m.name.startswith("motor"):
                    m.type = cfg_motors[0].type if cfg_motors else "4538_19"
        else:
            motors = []
            for m in cfg_motors:
                bus.add_motor(m)
                motors.append(m)
            print(f"配置电机: {[m.id for m in motors]}，先 ping ...")
            found = bus.scan([m.id for m in motors], timeout_each=0.08)
            missing = [m.id for m in motors if m.id not in found]
            if missing:
                print(f"未应答: {missing}。先不要发运动指令。")
                print("请改跑: python3 probe_can.py")
                if args.move or args.velocity is not None:
                    return 2

        for m in motors:
            bus.enable(m.id)
            if args.velocity is not None:
                bus.set_torque_limit(m.id, args.tqe)
        time.sleep(0.1)
        bus.poll_status([m.id for m in motors], timeout=0.08)
        for m in motors:
            print("使能后 " + format_state(m))

        if args.velocity is not None:
            seconds = args.seconds if args.seconds > 0 else 5.0
            print(
                f"速度控制: {args.velocity:+.3f} rad/s, tqe_max={args.tqe} Nm, "
                f"{seconds:.1f}s 后停止（Ctrl+C 也可停）"
            )
            period = 1.0 / max(args.hz, 1.0)
            t_start = time.monotonic()
            while running:
                if (time.monotonic() - t_start) >= seconds:
                    break
                t0 = time.monotonic()
                for m in motors:
                    bus.control_velocity(m, args.velocity)
                bus.collect(timeout=min(0.01, period * 0.4))
                if int((t0 - t_start) * args.hz) % max(int(args.hz / 5), 1) == 0:
                    bus.poll_status([m.id for m in motors], timeout=0.02)
                    for m in motors:
                        print(format_state(m), flush=True)
                dt = time.monotonic() - t0
                if dt < period:
                    time.sleep(period - dt)
            print("减速停止 ...")
            t_br = time.monotonic()
            while time.monotonic() - t_br < 0.4:
                for m in motors:
                    bus.control_velocity(m, 0.0)
                time.sleep(0.01)
            return 0

        if not args.move:
            print("只读状态循环（Ctrl+C 退出）。运动请加 --move")
            period = 1.0 / max(args.hz, 1.0)
            while running:
                t0 = time.monotonic()
                bus.poll_status([m.id for m in motors], timeout=period * 0.6)
                for m in motors:
                    print(format_state(m), flush=True)
                dt = time.monotonic() - t0
                if dt < period:
                    time.sleep(period - dt)
            return 0

        print(
            f"运动: ±{args.angle} rad, vel={args.vel} rad/s, "
            f"tqe_max={args.tqe} Nm, {args.hz:.0f} Hz（Ctrl+C 停止）"
        )
        period = 1.0 / max(args.hz, 1.0)
        t_start = time.monotonic()
        while running:
            if args.seconds > 0 and (time.monotonic() - t_start) >= args.seconds:
                break
            t0 = time.monotonic()
            phase = int((t0 - t_start) / args.period) % 2
            target = args.angle if phase == 0 else -args.angle
            for i, m in enumerate(motors):
                cmd = target if i < (len(motors) + 1) // 2 else -target
                bus.pos_vel_maxtqe(m, cmd, args.vel, args.tqe)
            bus.collect(timeout=min(0.01, period * 0.4))
            if int((t0 - t_start) * args.hz) % max(int(args.hz / 5), 1) == 0:
                bus.poll_status([m.id for m in motors], timeout=0.02)
                for m in motors:
                    print(format_state(m), flush=True)
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)
        return 0
    finally:
        print("\n失能电机 ...")
        try:
            for m in bus.motors.values():
                bus.disable(m.id)
            time.sleep(0.05)
        except OSError:
            pass
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
