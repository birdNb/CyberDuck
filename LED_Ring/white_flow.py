#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Radxa Dragon Q6A：GPIO_22（排针 PIN_8）上的 12 灯 WS2812 灯效。

接线（与 mixly.ino 同一类灯环）：
  红 5V  -> 排针 5V（灯珠多时请外接 5V，共地）
  黑 GND -> 排针 GND
  黄 DIN -> GPIO_22，即 40 针排针第 8 脚

GPIO_22 默认是调试串口 TX，程序会临时占用该脚。需要 root：
  sudo python3 white_flow.py
  sudo python3 white_flow.py --effect ironman
  sudo python3 white_flow.py --boot
  sudo python3 white_flow.py --boot-demo
关灯：
  sudo python3 white_flow.py --off
Ctrl+C 退出并关灯。开机灯效结束时会熄灭。
"""
from __future__ import annotations

import argparse
import ctypes
import math
import os
import random
import signal
import subprocess
import sys
import time
from typing import Callable, Dict, List, Optional, Tuple

DIR = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(DIR, "ws2812_gpio.c")
SO = os.path.join(DIR, "ws2812_gpio.so")

LED_COUNT = 12
GPIO_CHIP = 4
GPIO_LINE = 22
BRIGHTNESS = 64
STEP_S = 0.04
TAIL = 5
HOLD_S = 8.0
IDLE_BRIGHTNESS = 30
BOOT_SINE_PERIOD_S = 2.0
BOOT_SINE_CYCLES = 3
BOOT_TIMEOUT_S = 180.0
BOOT_DEMO_FLOW_S = 6.0


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
    if os.geteuid() == 0 or os.environ.get("INVOCATION_ID"):
        return
    print("WS2812 需要直接操作 GPIO_22 / TLMM，正在用 sudo 重新启动…")
    os.execvp("sudo", ["sudo", "-E", sys.executable, os.path.abspath(__file__), *sys.argv[1:]])


def hsv_to_grb(h: float, s: float, v: float) -> Tuple[int, int, int]:
    """h 为 0..1，返回 WS2812 的 (G, R, B) 0..255。"""
    h = h % 1.0
    if s <= 0:
        x = int(max(0.0, min(1.0, v)) * 255.0 + 0.5)
        return x, x, x
    i = int(h * 6.0)
    f = h * 6.0 - i
    p = v * (1.0 - s)
    q = v * (1.0 - f * s)
    t = v * (1.0 - (1.0 - f) * s)
    i %= 6
    if i == 0:
        r, g, b = v, t, p
    elif i == 1:
        r, g, b = q, v, p
    elif i == 2:
        r, g, b = p, v, t
    elif i == 3:
        r, g, b = p, q, v
    elif i == 4:
        r, g, b = t, p, v
    else:
        r, g, b = v, p, q
    return (
        int(max(0.0, min(1.0, g)) * 255.0 + 0.5),
        int(max(0.0, min(1.0, r)) * 255.0 + 0.5),
        int(max(0.0, min(1.0, b)) * 255.0 + 0.5),
    )


def put_grb(buf: bytearray, i: int, grb: Tuple[int, int, int]) -> None:
    buf[i * 3 + 0] = grb[0]
    buf[i * 3 + 1] = grb[1]
    buf[i * 3 + 2] = grb[2]


def scale_v(brightness: int) -> float:
    return max(1, min(255, brightness)) / 255.0


def breathe_level(n: int, period: int = 90) -> float:
    """0.06..1.0 的平滑呼吸，首尾相接无跳变。"""
    phase = (n % period) / period
    return 0.06 + 0.94 * (0.5 - 0.5 * math.cos(2.0 * math.pi * phase))


class SparkleState:
    def __init__(self) -> None:
        self.sparks: List[List[float]] = []

    def reset(self) -> None:
        self.sparks.clear()


EffectFn = Callable[[int, int, int, int, SparkleState], bytearray]


def effect_flow(n: int, count: int, brightness: int, tail: int, _st: SparkleState) -> bytearray:
    """白色流水，直线走，不绕环。"""
    buf = bytearray(count * 3)
    pos = n % count
    for t in range(tail):
        i = pos - t
        if i < 0 or i >= count:
            continue
        level = int(brightness * (tail - t) / tail)
        put_grb(buf, i, (level, level, level))
    return buf


def effect_color_flow(n: int, count: int, brightness: int, tail: int, _st: SparkleState) -> bytearray:
    """彩色流水，颜色随时间慢慢变。"""
    buf = bytearray(count * 3)
    pos = n % count
    hue = (n * 0.012) % 1.0
    v = scale_v(brightness)
    for t in range(tail):
        i = pos - t
        if i < 0 or i >= count:
            continue
        put_grb(buf, i, hsv_to_grb(hue, 1.0, v * (tail - t) / tail))
    return buf


def effect_neon(n: int, count: int, brightness: int, _tail: int, _st: SparkleState) -> bytearray:
    """霓虹彩虹绕圈旋转。"""
    buf = bytearray(count * 3)
    v = scale_v(brightness)
    shift = n * 0.018
    for i in range(count):
        h = (i / max(count, 1) + shift) % 1.0
        put_grb(buf, i, hsv_to_grb(h, 1.0, v))
    return buf


def effect_breathe(n: int, count: int, brightness: int, _tail: int, _st: SparkleState) -> bytearray:
    """整圈白色呼吸。"""
    buf = bytearray(count * 3)
    level = int(brightness * breathe_level(n) + 0.5)
    pixel = (level, level, level)
    for i in range(count):
        put_grb(buf, i, pixel)
    return buf


def effect_neon_breathe(n: int, count: int, brightness: int, _tail: int, _st: SparkleState) -> bytearray:
    """整圈霓虹变色 + 呼吸。"""
    buf = bytearray(count * 3)
    v = scale_v(brightness) * breathe_level(n, period=100)
    h = (n * 0.004) % 1.0
    pixel = hsv_to_grb(h, 1.0, v)
    for i in range(count):
        put_grb(buf, i, pixel)
    return buf


def effect_wave(n: int, count: int, brightness: int, _tail: int, _st: SparkleState) -> bytearray:
    """单峰波浪从一端走到另一端，不绕环。"""
    buf = bytearray(count * 3)
    v = scale_v(brightness)
    pos = (n % (count * 2)) / 2.0
    hue = (n * 0.006) % 1.0
    width = max(2.0, count / 4.0)
    for i in range(count):
        d = abs(i - pos)
        gain = math.exp(-0.5 * (d / width) ** 2)
        if gain < 0.02:
            continue
        put_grb(buf, i, hsv_to_grb(hue, 1.0, v * gain))
    return buf


def effect_fill(n: int, count: int, brightness: int, _tail: int, _st: SparkleState) -> bytearray:
    """从一端堆到另一端，再熄回去。"""
    buf = bytearray(count * 3)
    cycle = count * 2
    k = n % cycle
    lit = k + 1 if k < count else cycle - k
    hue = ((n // cycle) * 0.18) % 1.0
    v = scale_v(brightness)
    pixel = hsv_to_grb(hue, 0.85, v)
    for i in range(min(lit, count)):
        put_grb(buf, i, pixel)
    return buf


def effect_ironman(n: int, count: int, brightness: int, _tail: int, _st: SparkleState) -> bytearray:
    """钢铁侠胸灯：电蓝反应堆，三瓣能量旋转 + 心跳脉动。"""
    buf = bytearray(count * 3)
    v0 = scale_v(min(255, int(brightness * 1.35)))
    pulse = 0.58 + 0.42 * (0.5 - 0.5 * math.cos(2.0 * math.pi * n / 52.0))
    shimmer = 0.90 + 0.10 * math.sin(n * 0.81)
    spin = n * 0.14
    for i in range(count):
        spoke = 0.5 + 0.5 * math.cos(2.0 * math.pi * 3.0 * i / max(count, 1) - spin)
        spoke = spoke * spoke
        chase = 0.5 + 0.5 * math.cos(2.0 * math.pi * (i / max(count, 1)) + spin * 0.35)
        gain = (0.28 + 0.55 * spoke + 0.22 * chase) * pulse * shimmer
        sat = 1.0 - 0.12 * spoke
        hue = 0.64 + 0.02 * math.sin(n * 0.05 + i)
        g, r, b = hsv_to_grb(hue, sat, v0 * min(1.0, gain))
        g = int(g * 0.35)
        r = int(r * 0.45)
        b = min(255, int(b * 1.35 + 0.5))
        put_grb(buf, i, (g, r, b))
    return buf


def effect_sparkle(n: int, count: int, brightness: int, _tail: int, st: SparkleState) -> bytearray:
    """星闪：随机亮点淡出。"""
    buf = bytearray(count * 3)
    if random.random() < 0.35 and len(st.sparks) < count // 2 + 2:
        st.sparks.append([random.randrange(count), 1.0, random.random()])
    alive: List[List[float]] = []
    v = scale_v(brightness)
    for i, life, hue in st.sparks:
        idx = int(i) % count
        put_grb(buf, idx, hsv_to_grb(hue, 0.4, v * life))
        life *= 0.82
        if life > 0.05:
            alive.append([i, life, hue])
    st.sparks = alive
    return buf


EFFECTS: Dict[str, Tuple[str, EffectFn]] = {
    "ironman": ("钢铁侠胸灯", effect_ironman),
    "flow": ("白色流水", effect_flow),
    "color-flow": ("彩色流水", effect_color_flow),
    "neon": ("霓虹变换", effect_neon),
    "breathe": ("呼吸灯", effect_breathe),
    "neon-breathe": ("霓虹呼吸", effect_neon_breathe),
    "wave": ("波浪", effect_wave),
    "fill": ("堆积", effect_fill),
    "sparkle": ("星闪", effect_sparkle),
}

CYCLE_ORDER = [
    "ironman",
    "neon",
    "breathe",
    "neon-breathe",
    "color-flow",
    "wave",
    "flow",
    "fill",
    "sparkle",
]


def show_frame(lib, frame: bytearray) -> None:
    buf = (ctypes.c_uint8 * len(frame)).from_buffer_copy(frame)
    lib.ws2812_show(buf)


def solid_white(count: int, level: int) -> bytearray:
    x = max(0, min(255, int(level)))
    buf = bytearray(count * 3)
    pixel = (x, x, x)
    for i in range(count):
        put_grb(buf, i, pixel)
    return buf


def system_is_up() -> bool:
    try:
        r = subprocess.run(
            ["systemctl", "is-system-running"],
            capture_output=True,
            text=True,
            timeout=1,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return (r.stdout or "").strip() in ("running", "degraded")


def tick_delay(next_t: float, delay: float) -> float:
    next_t += delay
    remain = next_t - time.monotonic()
    if remain > 0:
        time.sleep(remain)
        return next_t
    return time.monotonic()


def run_boot(
    lib,
    count: int,
    brightness: int,
    tail: int,
    delay: float,
    idle: int,
    demo_s: Optional[float],
    timeout_s: float,
) -> None:
    running = True

    def _stop(_signum=None, _frame=None):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    hi = max(1, min(255, brightness))
    lo = 0
    idle = max(0, min(255, idle))
    st = SparkleState()
    n = 0
    next_t = time.monotonic()
    t0 = next_t
    if demo_s is not None:
        print(
            f"开机演示：白色流水 {demo_s:.0f}s，随后正弦 {BOOT_SINE_CYCLES} 次，最后熄灭"
        )
        until = t0 + max(0.0, demo_s)
        wait_boot = False
    elif system_is_up():
        print("系统已就绪，先走一段白色流水再进入完成灯效")
        until = t0 + BOOT_DEMO_FLOW_S
        wait_boot = False
    else:
        print("开机：白色流水灯，等待系统就绪…")
        until = t0 + max(5.0, timeout_s)
        wait_boot = True

    try:
        while running:
            show_frame(lib, effect_flow(n, count, hi, tail, st))
            n += 1
            next_t = tick_delay(next_t, delay)
            now = time.monotonic()
            if wait_boot and system_is_up():
                print("开机完成")
                break
            if now >= until:
                if wait_boot:
                    print("等待开机超时，进入完成灯效")
                break

        if running:
            print(
                f"正弦白光 {lo}–{hi}，周期 {BOOT_SINE_PERIOD_S:.0f}s × {BOOT_SINE_CYCLES}"
            )
            sine_t0 = time.monotonic()
            next_t = sine_t0
            duration = BOOT_SINE_PERIOD_S * BOOT_SINE_CYCLES
            while running:
                elapsed = time.monotonic() - sine_t0
                if elapsed >= duration:
                    break
                phase = 2.0 * math.pi * elapsed / BOOT_SINE_PERIOD_S
                level = lo + (hi - lo) * (0.5 + 0.5 * math.cos(phase))
                show_frame(lib, solid_white(count, int(level + 0.5)))
                next_t = tick_delay(next_t, delay)

        print("熄灭")
        off = (ctypes.c_uint8 * (count * 3))()
        for _ in range(3):
            lib.ws2812_show(off)
        lib.ws2812_hold_low()
    finally:
        lib.ws2812_close()


def run_loop(
    lib,
    names: List[str],
    count: int,
    brightness: int,
    tail: int,
    delay: float,
    hold_s: Optional[float],
) -> None:
    running = True

    def _stop(_signum=None, _frame=None):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    labels = ", ".join(f"{n}({EFFECTS[n][0]})" for n in names)
    print(f"GPIO_{GPIO_LINE}  {count} 灯  亮度 {brightness}  灯效: {labels}")
    if hold_s and len(names) > 1:
        print(f"每种约 {hold_s:.0f}s 轮换。Ctrl+C 退出。")
    else:
        print("Ctrl+C 退出。")

    st = SparkleState()
    idx = 0
    n = 0
    started = time.monotonic()
    next_t = started
    try:
        while running:
            name = names[idx]
            fn = EFFECTS[name][1]
            show_frame(lib, fn(n, count, brightness, tail, st))
            n += 1
            if hold_s and len(names) > 1 and (time.monotonic() - started) >= hold_s:
                idx = (idx + 1) % len(names)
                n = 0
                st.reset()
                started = time.monotonic()
                print(f"→ {names[idx]}（{EFFECTS[names[idx]][0]}）", flush=True)
            next_t += delay
            remain = next_t - time.monotonic()
            if remain > 0:
                time.sleep(remain)
            else:
                next_t = time.monotonic()
        off = (ctypes.c_uint8 * (count * 3))()
        lib.ws2812_show(off)
    finally:
        lib.ws2812_close()


def main() -> int:
    parser = argparse.ArgumentParser(description="GPIO_22 WS2812 灯环灯效")
    parser.add_argument("--count", type=int, default=LED_COUNT, help="灯珠数量，默认 12")
    parser.add_argument("--chip", type=int, default=GPIO_CHIP, help="gpiochip 编号，默认 4")
    parser.add_argument("--line", type=int, default=GPIO_LINE, help="GPIO 编号，默认 22")
    parser.add_argument("--brightness", type=int, default=BRIGHTNESS, help="亮度 1-255，默认 64")
    parser.add_argument("--delay", type=float, default=STEP_S, help="每帧间隔秒，默认 0.04")
    parser.add_argument("--tail", type=int, default=TAIL, help="流水拖尾长度，默认 5")
    parser.add_argument(
        "--effect",
        default="flow",
        metavar="NAME",
        help="灯效名，见 --list，默认 flow",
    )
    parser.add_argument("--cycle", action="store_true", help="轮播全部灯效")
    parser.add_argument("--hold", type=float, default=HOLD_S, help="轮播时每种持续秒，默认 8")
    parser.add_argument("--boot", action="store_true", help="开机灯效：流水 → 正弦3次 → 熄灭")
    parser.add_argument("--boot-demo", action="store_true", help="演示开机灯效（不等待 systemd）")
    parser.add_argument("--idle", type=int, default=IDLE_BRIGHTNESS, help="保留参数，开机结束已改为熄灭")
    parser.add_argument("--list", action="store_true", help="列出灯效")
    parser.add_argument("--off", action="store_true", help="发送全灭后退出")
    parser.add_argument("--no-sudo", action="store_true", help="不要自动 sudo（systemd 用）")
    args = parser.parse_args()

    if args.list:
        for name in EFFECTS:
            print(f"  {name:14}  {EFFECTS[name][0]}")
        print("  cycle           轮播全部")
        print("  --boot          开机：流水 → 正弦3次 → 熄灭")
        print("  --boot-demo     演示开机灯效")
        return 0

    args.brightness = max(1, min(255, args.brightness))
    args.tail = max(1, min(args.count, args.tail))
    args.idle = max(0, min(255, args.idle))

    if not args.no_sudo:
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

    if args.boot or args.boot_demo:
        if lib.ws2812_open(args.chip, args.line, args.count) != 0:
            err = lib.ws2812_error() or b"unknown"
            print(f"打开 GPIO_{args.line} 失败: {err.decode('utf-8', 'replace')}", file=sys.stderr)
            print("请用 sudo 运行，并确认 DIN 接到 40 针排针第 8 脚（GPIO_22）。", file=sys.stderr)
            return 1
        run_boot(
            lib,
            args.count,
            args.brightness,
            args.tail,
            args.delay,
            args.idle,
            BOOT_DEMO_FLOW_S if args.boot_demo else None,
            BOOT_TIMEOUT_S,
        )
        return 0

    if args.cycle or args.effect == "cycle":
        names = list(CYCLE_ORDER)
        hold: Optional[float] = max(1.0, args.hold)
    else:
        if args.effect not in EFFECTS:
            print(f"未知灯效 {args.effect!r}，可用: {', '.join(EFFECTS)}", file=sys.stderr)
            return 1
        names = [args.effect]
        hold = None

    if lib.ws2812_open(args.chip, args.line, args.count) != 0:
        err = lib.ws2812_error() or b"unknown"
        print(f"打开 GPIO_{args.line} 失败: {err.decode('utf-8', 'replace')}", file=sys.stderr)
        print("请用 sudo 运行，并确认 DIN 接到 40 针排针第 8 脚（GPIO_22）。", file=sys.stderr)
        return 1

    run_loop(lib, names, args.count, args.brightness, args.tail, args.delay, hold)
    return 0


if __name__ == "__main__":
    sys.exit(main())
