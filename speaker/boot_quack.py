#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""音频启动确认：连叫三声不同鸭叫。"""
from __future__ import annotations

import sys
import time

from duck import Speaker

# 三声各不相同，开机一听就知道喇叭起来了。
TAGS = ("greet", "chirp", "peck")
GAP_S = 0.25


def main() -> int:
    speaker = Speaker()
    for i, tag in enumerate(TAGS, 1):
        print(f"开机鸭叫 {i}/3：{tag}", flush=True)
        try:
            speaker.quack(tag, variant=0)
        except Exception as exc:
            print(f"开机鸭叫失败 {tag}：{exc}", file=sys.stderr, flush=True)
            return 1
        if i < len(TAGS):
            time.sleep(GAP_S)
    print("开机鸭叫完成。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
