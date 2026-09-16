#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把合成的单声道 PCM 送到本机喇叭。

Dragon Q6A 上是杰理 USB 声卡 UACDemoV1.0（48 kHz / S16_LE / 立体声），
原工程 microduck 则是 TLV320AIC3104（plughw:aic3104）。两者都试一遍。
"""
from __future__ import annotations

import os
import shutil
import subprocess
from typing import List, Optional, Sequence

import numpy as np

from synth import SR, to_i16


def audio_env() -> dict:
    uid = os.getuid() if os.getuid() != 0 else 1000
    runtime = os.environ.get("XDG_RUNTIME_DIR") or f"/run/user/{uid}"
    env = os.environ.copy()
    env["XDG_RUNTIME_DIR"] = runtime
    env.setdefault("PULSE_SERVER", f"unix:{runtime}/pulse/native")
    env.setdefault("PIPEWIRE_RUNTIME_DIR", runtime)
    return env


def list_alsa_cards() -> List[str]:
    cards: List[str] = []
    path = "/proc/asound/cards"
    if not os.path.isfile(path):
        return cards
    try:
        text = open(path, encoding="utf-8", errors="replace").read()
    except OSError:
        return cards
    for line in text.splitlines():
        if "]" not in line or "[" not in line:
            continue
        name = line.split("[", 1)[1].split("]", 1)[0].strip()
        if name:
            cards.append(name)
    return cards


def detect_device() -> Optional[str]:
    """优先本机 USB 喇叭，其次原工程 codec，最后交给 PipeWire 默认设备。"""
    cards = list_alsa_cards()
    preferred = ("UACDemoV10", "UACDemo", "aic3104")
    for needle in preferred:
        for card in cards:
            if needle.lower() in card.lower():
                return f"plughw:{card}"
    if cards:
        return f"plughw:{cards[0]}"
    return "default"


_current: Optional[subprocess.Popen] = None


def stop_playback() -> None:
    """立刻掐掉正在播的 aplay，给 Ctrl+C 用。"""
    global _current
    proc = _current
    _current = None
    if proc is None or proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=0.4)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def _stereo_bytes(buf: np.ndarray) -> bytes:
    pcm = to_i16(np.asarray(buf, dtype=np.float32))
    stereo = np.empty(pcm.size * 2, dtype=np.int16)
    stereo[0::2] = pcm
    stereo[1::2] = pcm
    return stereo.tobytes()


def _aplay(device: Optional[str], pcm: bytes, env: dict) -> bool:
    global _current
    if shutil.which("aplay") is None:
        return False
    cmd = [
        "aplay",
        "-q",
        "-t",
        "raw",
        "-f",
        "S16_LE",
        "-c",
        "2",
        "-r",
        str(SR),
    ]
    if device:
        cmd.extend(["-D", device])
    try:
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
            start_new_session=True,
        )
    except OSError:
        return False
    _current = proc
    try:
        proc.communicate(input=pcm, timeout=30)
        return proc.returncode == 0
    except KeyboardInterrupt:
        stop_playback()
        raise
    except subprocess.TimeoutExpired:
        stop_playback()
        return False
    finally:
        if _current is proc:
            _current = None


def play_pcm(buf: np.ndarray, device: Optional[str] = None) -> str:
    """播放 f32 单声道缓冲。成功返回实际使用的设备名，失败抛 RuntimeError。"""
    pcm = _stereo_bytes(buf)
    env = audio_env()
    tried: List[Optional[str]] = []
    if device:
        tried.append(device)
    auto = detect_device()
    if auto and auto not in tried:
        tried.append(auto)
    for extra in ("plughw:UACDemoV10", "plughw:0", "hw:0,0", "default", None):
        if extra not in tried:
            tried.append(extra)

    for dev in tried:
        if _aplay(dev, pcm, env):
            return dev or "default"
    raise RuntimeError("aplay 播放失败，检查喇叭 / ALSA / PipeWire")


def play_wav(path: str, device: Optional[str] = None) -> str:
    if shutil.which("aplay") is None:
        raise RuntimeError("未找到 aplay")
    env = audio_env()
    devices: Sequence[Optional[str]] = (device, detect_device(), "default", None)
    seen = set()
    for dev in devices:
        key = dev or ""
        if key in seen:
            continue
        seen.add(key)
        cmd = ["aplay", "-q"]
        if dev:
            cmd.extend(["-D", dev])
        cmd.append(path)
        try:
            proc = subprocess.run(
                cmd,
                env=env,
                timeout=30,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if proc.returncode == 0:
                return dev or "default"
        except (OSError, subprocess.TimeoutExpired):
            continue
    raise RuntimeError(f"无法播放 {path}")
