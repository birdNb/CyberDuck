#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""触摸测试：GPIO_57 输出高电平，检测 GPIO_27 电平。

Radxa Dragon Q6A（gpiochip4 / f100000.pinctrl）：
  GPIO_57 = PIN_22  → 输出 1
  GPIO_27 = PIN_18  → 输入检测

用法：
  python3 test_touch.py
  python3 test_touch.py --hz 20
  python3 test_touch.py --active-low
Ctrl+C 退出。
"""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time


GPIO_CHIP = "4"
DRIVE_LINE = 57  # PIN_22
SENSE_LINE = 27  # PIN_18
DRIVE_PIN = "PIN_22"
SENSE_PIN = "PIN_18"


def run_capture(cmd: list[str]) -> str:
    return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT).strip()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=f"触摸测试：GPIO_{DRIVE_LINE}=1，读 GPIO_{SENSE_LINE}"
    )
    parser.add_argument("--chip", default=GPIO_CHIP)
    parser.add_argument("--drive", type=int, default=DRIVE_LINE, help=f"输出脚，默认 {DRIVE_LINE}")
    parser.add_argument("--sense", type=int, default=SENSE_LINE, help=f"检测脚，默认 {SENSE_LINE}")
    parser.add_argument("--hz", type=float, default=10.0, help="采样频率，默认 10")
    parser.add_argument(
        "--active-low",
        action="store_true",
        help=f"触摸时 GPIO_{SENSE_LINE} 为低电平（默认高电平算触摸）",
    )
    parser.add_argument(
        "--pull",
        choices=("pull-up", "pull-down", "disable", "as-is"),
        default="pull-up",
        help="检测脚偏置，默认 pull-up",
    )
    args = parser.parse_args()

    for tool in ("gpioset", "gpioget"):
        if subprocess.call(["which", tool], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0:
            print(f"缺少 {tool}，请安装 gpiod 工具包。", file=sys.stderr)
            return 1

    drive = subprocess.Popen(
        [
            "gpioset",
            "-m",
            "signal",
            "-B",
            "disable",
            str(args.chip),
            f"{args.drive}=1",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    time.sleep(0.05)
    if drive.poll() is not None:
        err = drive.stderr.read().strip() if drive.stderr else ""
        print(f"GPIO_{args.drive} 拉高失败: {err or f'exit={drive.returncode}'}", file=sys.stderr)
        return 1

    running = True

    def _stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    print(
        f"gpiochip{args.chip}: GPIO_{args.drive}({DRIVE_PIN})=1, "
        f"读 GPIO_{args.sense}({SENSE_PIN}), {args.hz:.0f} Hz"
    )
    print("Ctrl+C 退出")

    period = 1.0 / max(args.hz, 0.5)
    t0 = time.monotonic()
    try:
        while running:
            try:
                raw = run_capture(
                    ["gpioget", "-B", args.pull, str(args.chip), str(args.sense)]
                )
                level = int(raw.split()[0])
            except (subprocess.CalledProcessError, ValueError, IndexError) as exc:
                print(f"读 GPIO_{args.sense} 失败: {exc}", file=sys.stderr)
                time.sleep(period)
                continue

            touched = (level == 0) if args.active_low else (level == 1)
            state = "TOUCH" if touched else "idle "
            print(
                f"\r[{time.monotonic() - t0:7.2f}s]  "
                f"GPIO_{args.drive}=1  GPIO_{args.sense}={level}  {state}   ",
                end="",
                flush=True,
            )
            time.sleep(period)
        print()
    finally:
        drive.send_signal(signal.SIGTERM)
        try:
            drive.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            drive.kill()
            drive.wait()
        print(f"已释放 GPIO_{args.drive}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
