#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 指令 V {0-100} → 系统播放音量（PipeWire wpctl / PulseAudio / ALSA）。"""

from __future__ import annotations

import os
import subprocess
from typing import Callable, Optional, Tuple

LogFn = Callable[[str], None]


def _audio_env() -> dict:
    uid = (
        os.environ.get("DUCK_BLE_UID")
        or os.environ.get("BIRD_BLE_UID")
        or str(os.getuid() if os.getuid() != 0 else 1000)
    )
    runtime = os.environ.get("XDG_RUNTIME_DIR") or f"/run/user/{uid}"
    env = os.environ.copy()
    env["XDG_RUNTIME_DIR"] = runtime
    env.setdefault("PULSE_SERVER", f"unix:{runtime}/pulse/native")
    env.setdefault("PIPEWIRE_RUNTIME_DIR", runtime)
    return env


def _run(cmd: list, env: dict, timeout: float = 3.0) -> Tuple[int, str]:
    try:
        r = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            env=env,
        )
        out = ((r.stderr or "") + "\n" + (r.stdout or "")).strip()
        return r.returncode, out
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError) as e:
        return 127, str(e)


class VolumeController:
    """V 10 表示将系统主音量设为 10%。"""

    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._last_percent: Optional[int] = None

    def handle(self, percent_str: str) -> bool:
        try:
            pct = max(0, min(100, int(percent_str.strip())))
        except ValueError:
            self._log(f"[V] 无效音量: {percent_str!r}")
            return False
        return self._apply(pct)

    @property
    def last_percent(self) -> Optional[int]:
        return self._last_percent

    def set_percent(self, pct: int) -> bool:
        try:
            value = max(0, min(100, int(pct)))
        except (TypeError, ValueError):
            return False
        return self._apply(value)

    def _apply(self, pct: int) -> bool:
        env = _audio_env()
        attempts = (
            (["wpctl", "set-volume", "@DEFAULT_AUDIO_SINK@", f"{pct}%"], "wpctl"),
            (["amixer", "-D", "pulse", "sset", "Master", f"{pct}%"], "pulse Master"),
            (["amixer", "sset", "Master", f"{pct}%"], "alsa Master"),
            (["amixer", "-c", "0", "sset", "PCM", f"{pct}%"], "alsa PCM"),
        )
        last_err = ""
        for cmd, label in attempts:
            rc, err = _run(cmd, env)
            if rc == 0:
                self._last_percent = pct
                self._log(f"[V] 音量设为 {pct}% ({label})")
                return True
            if rc == 127:
                continue
            if err:
                last_err = err.splitlines()[-1][:160]
        self._log(f"[V] 设置音量失败 ({pct}%): {last_err or '无可用混音器'}")
        return False
