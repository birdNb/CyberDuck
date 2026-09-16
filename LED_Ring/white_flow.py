#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Radxa Dragon Q6A：GPIO_22（排针 PIN_8）上的 16 灯 WS2812 白色流水灯。

接线（与 mixly.ino 同一类灯环）：
  红 5V  -> 排针 5V（灯珠多时请外接 5V，共地）
  黑 GND -> 排针 GND
  黄 DIN -> GPIO_22，即 40 针排针第 8 脚

GPIO_22 默认是调试串口 TX，程序会临时占用该脚。需要 root：
  sudo python3 white_flow.py
关灯：
  sudo python3 white_flow.py --off
Ctrl+C 退出并关灯。
"""
from __future__ import annotations

import argparse
import ctypes
import os
import signal
import subprocess
import sys
import time

DIR = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(DIR, "ws2812_gpio.c")
SO = os.path.join(DIR, "ws2812_gpio.so")

LED_COUNT = 16
GPIO_CHIP = 4
GPIO_LINE = 22
BRIGHTNESS = 64
STEP_S = 0.05
TAIL = 5


def build_so() -> None:
    if os.path.exists(SO) and os.path.getmtime(SO) >= os.path.getmtime(SRC):
        return
    subprocess.check_call(
        ["gcc", "-O2", "-shared", "-fPIC", "-o", SO, SRC],
        cwd=DIR,
    )


def load_lib() -> ctypes.CDLL:
    build_so()
    lib = ctypes.CDLL(SO)
    lib.ws2812_open.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
    lib.ws2812_open.restype = ctypes.c_int
    lib.ws2812_show.argtypes = [ctypes.POINTER(ctypes.c_uint8)]
    lib.ws2812_show.restype = None
    lib.ws2812_hold_low.argtypes = []
    lib.ws2812_hold_low.restype = None
    lib.ws2812_close.argtypes = []
    lib.ws2812_close.restype = None
    lib.ws2812_error.argtypes = []
    lib.ws2812_error.restype = ctypes.c_char_p
    return lib


def maybe_rerun_as_root() -> None:
    if os.geteuid() == 0:
        return
    print("WS2812 需要直接操作 GPIO_22 / TLMM，正在用 sudo 重新启动…")
    os.execvp("sudo", ["sudo", "-E", sys.executable, os.path.abspath(__file__), *sys.argv[1:]])


def comet_frame(pos: int, count: int, tail: int, brightness: int) -> bytearray:
    """GRB 顺序，白色彗星拖尾。"""
    buf = bytearray(count * 3)
    for t in range(tail):
        i = (pos - t) % count
        level = int(brightness * (tail - t) / tail)
        # WS2812 字节序：G, R, B
        buf[i * 3 + 0] = level
        buf[i * 3 + 1] = level
        buf[i * 3 + 2] = level
    return buf


def main() -> int:
    parser = argparse.ArgumentParser(description="GPIO_22 白色流水灯")
    parser.add_argument("--count", type=int, default=LED_COUNT, help="灯珠数量，默认 16")
    parser.add_argument("--chip", type=int, default=GPIO_CHIP, help="gpiochip 编号，默认 4")
    parser.add_argument("--line", type=int, default=GPIO_LINE, help="GPIO 编号，默认 22")
    parser.add_argument("--brightness", type=int, default=BRIGHTNESS, help="亮度 1-255，默认 64")
    parser.add_argument("--delay", type=float, default=STEP_S, help="每步间隔秒，默认 0.05")
    parser.add_argument("--tail", type=int, default=TAIL, help="拖尾长度，默认 5")
    parser.add_argument("--off", action="store_true", help="发送全灭后退出")
    args = parser.parse_args()
    args.brightness = max(1, min(255, args.brightness))
    args.tail = max(1, min(args.count, args.tail))

    maybe_rerun_as_root()
    lib = load_lib()

    if args.off:
        n = max(args.count, 64)
        if lib.ws2812_open(args.chip, args.line, n) != 0:
            err = lib.ws2812_error() or b"unknown"
            print(f"打开 GPIO_{args.line} 失败: {err.decode('utf-8', 'replace')}", file=sys.stderr)
            return 1
        off = (ctypes.c_uint8 * (n * 3))()
        for _ in range(3):
            lib.ws2812_show(off)
        lib.ws2812_hold_low()
        print("已发送全灭帧，GPIO_22 DIN 保持低电平（不再绑回串口）。")
        print("确认灯灭后 Ctrl+C 退出。")
        try:
            signal.pause()
        except KeyboardInterrupt:
            pass
        lib.ws2812_hold_low()
        lib.ws2812_close()
        return 0

    if lib.ws2812_open(args.chip, args.line, args.count) != 0:
        err = lib.ws2812_error() or b"unknown"
        print(f"打开 GPIO_{args.line} 失败: {err.decode('utf-8', 'replace')}", file=sys.stderr)
        print("请用 sudo 运行，并确认 DIN 接到 40 针排针第 8 脚（GPIO_22）。", file=sys.stderr)
        return 1

    running = True

    def _stop(_signum=None, _frame=None):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    print(
        f"白色流水灯：GPIO_{args.line}，{args.count} 灯，亮度 {args.brightness}。"
        " Ctrl+C 退出。"
    )
    pos = 0
    try:
        while running:
            frame = comet_frame(pos, args.count, args.tail, args.brightness)
            buf = (ctypes.c_uint8 * len(frame)).from_buffer_copy(frame)
            lib.ws2812_show(buf)
            pos = (pos + 1) % args.count
            time.sleep(args.delay)
        off = (ctypes.c_uint8 * (args.count * 3))()
        lib.ws2812_show(off)
    finally:
        lib.ws2812_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
