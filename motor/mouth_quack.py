#!/usr/bin/env python3
"""15 号嘴部电机：当前角设为零位，在 -5~15° 做不规律正弦，张嘴时短促鸭叫，灯环同步闪。"""

from __future__ import annotations

import argparse
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from ht_can import Motor, MotorBus, format_state

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "speaker"))
sys.path.insert(0, str(ROOT / "LED_Ring"))

from duck import Speaker, render, variant_count  # noqa: E402
from play import play_pcm, stop_playback  # noqa: E402
import white_flow as leds  # noqa: E402

DEG = math.pi / 180.0
SHORT_TAGS = ("peck", "chirp", "greet", "alarm")
SHORT_WEIGHTS = (3, 3, 2, 1)
MIN_DEG = -5.0
MAX_DEG = 15.0
SPAN_DEG = MAX_DEG - MIN_DEG


def maybe_rerun_as_root() -> None:
    if os.geteuid() == 0:
        return
    print("灯环需要 GPIO_22，正在用 sudo 重新启动…")
    os.execvp(
        "sudo",
        ["sudo", "-E", sys.executable, os.path.abspath(__file__), *sys.argv[1:]],
    )


def iface_is_up(iface: str) -> bool:
    path = Path(f"/sys/class/net/{iface}/flags")
    try:
        flags = int(path.read_text(encoding="utf-8").strip(), 16)
    except OSError:
        return False
    return bool(flags & 0x1)


def ensure_can(iface: str) -> None:
    if iface_is_up(iface):
        return
    setup = Path(__file__).with_name("setup_can0.sh")
    print(f"{iface} 未启动，尝试 {setup} ...")
    try:
        subprocess.check_call([str(setup), iface])
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"抬起 {iface} 失败: {exc}", file=sys.stderr)


class NaturalMouth:
    """有停顿的缓开缓合，避免连续正弦和过快翻嘴。"""

    REST = "rest"
    OPENING = "open"
    HOLD = "hold"
    DIP = "dip"
    CLOSING = "close"

    def __init__(self) -> None:
        self.mode = self.REST
        self.t = 0.0
        self.dur = random.uniform(0.5, 1.0)
        self.start = 0.0
        self.end = random.uniform(-2.5, 0.8)
        self.peak = 12.0
        self.double = False
        self.filt = 0.0
        self.event: str | None = None

    @staticmethod
    def _ease(u: float) -> float:
        u = max(0.0, min(1.0, u))
        return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)

    def _go_rest(self) -> None:
        self.mode = self.REST
        self.t = 0.0
        self.start = self.filt
        self.end = random.uniform(-3.0, 1.0)
        self.dur = random.uniform(0.55, 1.6)

    def _go_open(self, from_dip: bool = False) -> None:
        self.mode = self.OPENING
        self.t = 0.0
        self.start = self.filt
        self.peak = random.uniform(8.5, 15.0)
        self.end = self.peak
        self.dur = random.uniform(0.32, 0.52)
        self.double = (not from_dip) and random.random() < 0.28
        self.event = "open"

    def _go_close(self) -> None:
        self.mode = self.CLOSING
        self.t = 0.0
        self.start = self.filt
        self.end = random.uniform(-3.0, 1.0)
        self.dur = random.uniform(0.40, 0.70)

    def step(self, dt: float) -> float:
        self.event = None
        dt = max(1e-4, min(0.04, dt))
        self.t += dt
        u = self.t / max(self.dur, 1e-3)
        if self.mode == self.REST:
            target = self.start + (self.end - self.start) * self._ease(min(u, 1.0))
            if u >= 1.0:
                self._go_open()
        elif self.mode == self.OPENING:
            target = self.start + (self.end - self.start) * self._ease(min(u, 1.0))
            if u >= 1.0:
                self.mode = self.HOLD
                self.t = 0.0
                self.dur = random.uniform(0.08, 0.20)
                self.start = self.end
                target = self.start
        elif self.mode == self.HOLD:
            target = self.start
            if u >= 1.0:
                if self.double:
                    self.mode = self.DIP
                    self.t = 0.0
                    self.start = self.peak
                    self.end = random.uniform(2.5, 6.5)
                    self.dur = random.uniform(0.16, 0.28)
                    self.double = False
                    target = self.start
                else:
                    self._go_close()
                    target = self.start
        elif self.mode == self.DIP:
            target = self.start + (self.end - self.start) * self._ease(min(u, 1.0))
            if u >= 1.0:
                self._go_open(from_dip=True)
                target = self.filt
        else:
            target = self.start + (self.end - self.start) * self._ease(min(u, 1.0))
            if u >= 1.0:
                self._go_rest()
                target = self.filt

        tau = 0.08
        alpha = 1.0 - math.exp(-dt / tau)
        self.filt += (target - self.filt) * alpha
        return max(MIN_DEG, min(MAX_DEG, self.filt))


class QuackPlayer:
    def __init__(self, speaker: Speaker) -> None:
        self.speaker = speaker
        self._lock = threading.Lock()
        self._playing = False
        self._thread: threading.Thread | None = None

    def busy(self) -> bool:
        with self._lock:
            return self._playing

    def start(self, tag: str, variant: int) -> bool:
        with self._lock:
            if self._playing:
                return False
            self._playing = True
        buf = render(tag, self.speaker.personality, variant)

        def _run() -> None:
            try:
                used = play_pcm(buf, self.speaker.device)
                self.speaker.device = self.speaker.device or used
            except Exception as exc:
                print(f"鸭叫播放失败: {exc}", flush=True)
            finally:
                with self._lock:
                    self._playing = False

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        stop_playback()
        with self._lock:
            self._playing = False


class MouthLeds:
    """亮度跟张嘴走；鸭叫时整圈同步闪。"""

    def __init__(self, lib, count: int = 12, brightness: int = 96) -> None:
        self.lib = lib
        self.count = count
        self.brightness = brightness
        self.flash_t0 = 0.0
        self.flash_until = 0.0

    def poke_flash(self) -> None:
        now = time.monotonic()
        self.flash_t0 = now
        self.flash_until = now + 0.32

    def render(self, deg: float, quacking: bool) -> None:
        open_n = (deg - MIN_DEG) / SPAN_DEG
        open_n = max(0.0, min(1.0, open_n))
        now = time.monotonic()
        flashing = quacking or now < self.flash_until
        if flashing:
            phase = (now - self.flash_t0) * 11.0
            blink = 1.0 if (phase % 1.0) < 0.42 else 0.08
        else:
            blink = 1.0
        v = (0.04 + 0.96 * open_n) * blink
        hue = 0.08 + 0.04 * open_n
        sat = 0.55 if flashing and blink > 0.5 else 0.85
        pixel = leds.hsv_to_grb(hue, sat, v * (self.brightness / 255.0))
        buf = bytearray(self.count * 3)
        for i in range(self.count):
            leds.put_grb(buf, i, pixel)
        leds.show_frame(self.lib, buf)

    def off(self) -> None:
        leds.show_frame(self.lib, bytearray(self.count * 3))


def pick_short_tag() -> tuple[str, int]:
    tag = random.choices(SHORT_TAGS, weights=SHORT_WEIGHTS, k=1)[0]
    return tag, random.randrange(variant_count(tag))


def open_leds() -> MouthLeds | None:
    lib = leds.load_lib()
    if lib.ws2812_open(leds.GPIO_CHIP, leds.GPIO_LINE, leds.LED_COUNT) != 0:
        err = lib.ws2812_error() or b"unknown"
        print(f"打开灯环失败: {err.decode('utf-8', 'replace')}", file=sys.stderr)
        return None
    ring = MouthLeds(lib, leds.LED_COUNT)
    ring.off()
    return ring


def main() -> int:
    parser = argparse.ArgumentParser(description="15 号嘴部：设零 + 不规律张嘴鸭叫 + 灯环同步")
    parser.add_argument("-i", "--iface", default="can0")
    parser.add_argument("--id", type=int, default=15)
    parser.add_argument("--type", default="4538_19")
    parser.add_argument("--tqe", type=float, default=0.7, help="最大力矩 Nm")
    parser.add_argument("--vel", type=float, default=0.8, help="速度上限 rad/s")
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--invert", action="store_true", help="反向：正角度往闭合方向")
    parser.add_argument("--persist", action="store_true", help="把零位写入 flash")
    parser.add_argument("--no-sound", action="store_true")
    parser.add_argument("--no-led", action="store_true", help="不驱动灯环")
    parser.add_argument("--seconds", type=float, default=0.0, help="运行秒数，0 直到 Ctrl+C")
    args = parser.parse_args()

    if not args.no_led:
        maybe_rerun_as_root()

    ensure_can(args.iface)
    if not iface_is_up(args.iface):
        print(f"{args.iface} 仍未 UP。请先: sudo ./setup_can0.sh", file=sys.stderr)
        return 1

    ring = None if args.no_led else open_leds()
    if not args.no_led and ring is None:
        return 1

    mid = args.id
    print(f"打开 {args.iface}，只控制电机 {mid} ...")
    try:
        bus = MotorBus(args.iface)
    except OSError as exc:
        print(f"打开 {args.iface} 失败: {exc}", file=sys.stderr)
        if ring is not None:
            ring.off()
            ring.lib.ws2812_close()
        return 1

    motor = bus.add_motor(Motor(mid, args.type, "mouth"))
    speaker = Speaker()
    player = QuackPlayer(speaker)
    running = True

    def _stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        found = bus.scan([mid], timeout_each=0.08)
        if mid not in found:
            print(f"电机 {mid} 无应答。先跑: python3 probe_can.py --start {mid} --end {mid}")
            return 2

        bus.disable(mid)
        bus.set_timeout_ms(mid, 0)
        time.sleep(0.05)
        bus.poll_status([mid], timeout=0.08)
        print(f"设零前 {format_state(motor)}")

        bus.set_zero(mid)
        time.sleep(1.0)
        if args.persist:
            bus.save_config(mid)
            time.sleep(0.3)
            print("已请求写入 flash（彻底生效需给该电机重新上电）")
        bus.poll_status([mid], timeout=0.08)
        after = motor.state.position
        print(f"设零后 {format_state(motor)}")
        zero = after
        print(f"以当前角为 0 位，软件补偿 {zero / DEG:.2f}°（硬件 raw={motor.state.raw_pos}）")

        bus.enable(mid)
        bus.set_torque_limit(mid, args.tqe)
        time.sleep(0.08)

        wave = NaturalMouth()
        period = 1.0 / max(args.hz, 1.0)
        t_start = time.monotonic()
        prev = t_start
        last_print = t_start
        n_quack = 0
        sign = -1.0 if args.invert else 1.0

        print(
            f"{MIN_DEG:.0f}~{MAX_DEG:.0f}° 缓开缓合，张嘴时短叫，"
            f"灯环同步闪。Ctrl+C 停止"
        )

        while running:
            now = time.monotonic()
            t = now - t_start
            if args.seconds > 0 and t >= args.seconds:
                break
            dt = now - prev
            prev = now
            deg = wave.step(dt)
            target = zero + sign * deg * DEG
            bus.command_pos_vel_tqe(motor, target, args.vel, args.tqe)
            bus.collect(timeout=min(0.003, period * 0.25))

            if not args.no_sound and wave.event == "open" and not player.busy():
                tag, variant = pick_short_tag()
                if player.start(tag, variant):
                    n_quack += 1
                    if ring is not None:
                        ring.poke_flash()
                    print(f"  张嘴 {deg:5.1f}°  {tag}/{variant}", flush=True)

            if ring is not None:
                ring.render(deg, player.busy())

            if now - last_print >= 1.0:
                last_print = now
                bus.poll_status([mid], timeout=0.02)
                fb = (motor.state.position - zero) / DEG * sign
                print(
                    f"t={t:5.1f}s  cmd={deg:5.1f}°  fb={fb:5.1f}°  "
                    f"mode={wave.mode:5s}  quacks={n_quack}  {format_state(motor)}",
                    flush=True,
                )

            spent = time.monotonic() - now
            if spent < period:
                time.sleep(period - spent)

        print("回 0° 并失能 ...")
        t_home = time.monotonic()
        while time.monotonic() - t_home < 0.6:
            bus.command_pos_vel_tqe(motor, zero, args.vel, args.tqe)
            if ring is not None:
                ring.render(0.0, False)
            time.sleep(0.02)
        return 0
    finally:
        player.stop()
        if ring is not None:
            try:
                ring.off()
                ring.lib.ws2812_close()
            except Exception:
                pass
        try:
            bus.disable(mid)
            time.sleep(0.05)
        except OSError:
            pass
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
