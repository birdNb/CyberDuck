#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""让机器人发出鸭叫。合成来自 ~/microduck/sounds，播放走本机 USB 喇叭。

用法：
  python3 test_speaker.py
  python3 test_speaker.py greet
  python3 test_speaker.py chirp --variant 3
  python3 test_speaker.py --all
  python3 test_speaker.py --random
  python3 test_speaker.py greet --save greet.wav
  python3 test_speaker.py --list
"""
from __future__ import annotations

import argparse
import random
import sys
import time

from duck import Speaker, TAGS, hardware_seed, render, to_wav, variant_count
from play import detect_device, play_pcm, stop_playback


def pick_random_tag(last: str | None) -> str:
    tags = list(TAGS)
    others = [t for t in tags if t != "wheee"]
    if random.randrange(15) == 0:
        return "wheee"
    choices = [t for t in others if t != last] or others
    return random.choice(choices)


def random_loop(speaker: Speaker, gap: float) -> int:
    last = None
    n = 0
    print("随机轮播中，Ctrl+C 停止（wheee 概率 1/15）")
    try:
        while True:
            tag = pick_random_tag(last)
            last = tag
            variant = random.randrange(variant_count(tag))
            n += 1
            print(f"[{n}] {tag}/{variant}", flush=True)
            buf = render(tag, speaker.personality, variant)
            used = play_pcm(buf, speaker.device)
            speaker.device = speaker.device or used
            if gap > 0:
                time.sleep(gap)
    except KeyboardInterrupt:
        stop_playback()
        print(f"\n已停止，共 {n} 声", flush=True)
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="合成并播放鸭叫")
    parser.add_argument(
        "tag",
        nargs="?",
        default="greet",
        help="声音标签，默认 greet（打招呼的鸭叫）",
    )
    parser.add_argument("--list", action="store_true", help="列出全部标签")
    parser.add_argument("--all", action="store_true", help="依次播放全部标签")
    parser.add_argument("--random", action="store_true", help="随机轮播，Ctrl+C 停止")
    parser.add_argument("--gap", type=float, default=0.4, help="随机轮播间隔秒，默认 0.4")
    parser.add_argument("--seed", type=lambda s: int(s, 0), default=None)
    parser.add_argument("--variant", type=int, default=None)
    parser.add_argument("--device", default=None, help="ALSA 设备，默认自动探测")
    parser.add_argument("--save", default=None, help="同时写成 wav")
    args = parser.parse_args()

    if args.list:
        for name in TAGS:
            print(f"  {name:8}  {variant_count(name)} 个变体")
        return 0

    seed = hardware_seed() if args.seed is None else args.seed
    speaker = Speaker(seed=seed, device=args.device)
    device = args.device or detect_device()
    print(f"seed={speaker.seed}  pitch={speaker.personality.pitch_center_hz:.1f} Hz")
    print(f"device={device}")
    print(
        "  quackiness={:.2f}  brightness={:.2f}  speed={:.2f}".format(
            speaker.personality.quackiness,
            speaker.personality.brightness,
            speaker.personality.speed,
        )
    )

    if args.random:
        return random_loop(speaker, gap=max(0.0, args.gap))

    tags = list(TAGS) if args.all else [args.tag]
    for tag in tags:
        if tag not in TAGS:
            print(f"未知标签 {tag!r}，可选: {', '.join(TAGS)}", file=sys.stderr)
            return 1
        variant = 0 if args.variant is None else args.variant
        if args.all and args.variant is None:
            variant = 0
        elif args.variant is None:
            buf = speaker.quack(tag)
            print(f"played {tag}  samples={buf.size}")
            if args.save:
                to_wav(buf, args.save)
                print(f"wrote {args.save}")
            continue
        buf = render(tag, speaker.personality, variant)
        used = play_pcm(buf, args.device)
        print(f"played {tag}/{variant} via {used}  samples={buf.size}")
        if args.save:
            to_wav(buf, args.save)
            print(f"wrote {args.save}")
        if args.all:
            time.sleep(0.25)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
