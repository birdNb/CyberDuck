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


_codec_armed = False


def ensure_codec_path() -> None:
    """打开板载 HPH/LO。关掉时 aplay 仍返回 0，但听不见。"""
    global _codec_armed
    if _codec_armed:
        return
    if shutil.which("tinymix") is None:
        _codec_armed = True
        return
    for args in (
        ("set", "LO Switch", "1"),
        ("set", "HPHL Switch", "1"),
        ("set", "HPHR Switch", "1"),
        ("set", "CLSH Switch", "1"),
        ("set", "HPHL_RDAC Switch", "1"),
        ("set", "HPHR_RDAC Switch", "1"),
        ("set", "AUX_RDAC Switch", "1"),
        ("set", "HPHL Volume", "24"),
        ("set", "HPHR Volume", "24"),
    ):
        try:
            subprocess.run(
                ["tinymix", *args],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=0.5,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            pass
    _codec_armed = True


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


def _aplay(device: Optional[str], pcm: bytes, env: dict) -> tuple[bool, str]:
    global _current
    if shutil.which("aplay") is None:
        return False, "未找到 aplay"
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
            stderr=subprocess.PIPE,
            env=env,
            start_new_session=True,
        )
    except OSError as exc:
        return False, str(exc)
    _current = proc
    try:
        _out, err = proc.communicate(input=pcm, timeout=30)
        text = (err or b"").decode("utf-8", "replace").strip().replace("\n", " | ")
        return proc.returncode == 0, text[:240]
    except KeyboardInterrupt:
        stop_playback()
        raise
    except subprocess.TimeoutExpired:
        stop_playback()
        return False, "超时"
    finally:
        if _current is proc:
            _current = None


def play_pcm(buf: np.ndarray, device: Optional[str] = None) -> str:
    """播放 f32 单声道缓冲。成功返回实际使用的设备名，失败抛 RuntimeError。

    先走 PipeWire 的 default。这块板的 plughw:QCM6490IDP 会 write error，
    先开它会把正在用的喇叭路径打掉，所以只在 default 失败后才试 USB 声卡。
    """
    pcm = _stereo_bytes(buf)
    env = audio_env()
    ensure_codec_path()
    tried: List[Optional[str]] = []
    if device:
        tried.append(device)
    for extra in ("default", None, "plughw:UACDemoV10"):
        if extra not in tried:
            tried.append(extra)

    fails: List[str] = []
    for dev in tried:
        ok, err = _aplay(dev, pcm, env)
        if ok:
            return dev or "default"
        fails.append(f"{dev or 'pipewire'}: {err or '失败'}")
    raise RuntimeError("aplay 播放失败；" + " | ".join(fails))


def sink_status() -> str:
    """给排查日志用。不播放。"""
    env = audio_env()
    try:
        proc = subprocess.run(
            ["wpctl", "get-volume", "@DEFAULT_AUDIO_SINK@"],
            capture_output=True,
            text=True,
            timeout=1.0,
            env=env,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f"wpctl 失败 {exc}"
    text = ((proc.stdout or "") + (proc.stderr or "")).strip().replace("\n", " ")
    return f"rc={proc.returncode} {text or '无输出'}"


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
