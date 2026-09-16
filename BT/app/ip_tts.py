#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 IPv4 转成中文朗读文本，并用 espeak-ng 合成临时 WAV。"""

from __future__ import annotations

import array
import ctypes
import os
import shutil
import subprocess
import tempfile
import threading
import wave
from typing import Optional

_DIGIT_ZH = str.maketrans("0123456789.", "零一二三四五六七八九点")

_lock = threading.Lock()
_inited = False
_sample_rate = 22050
_lib = None
_CALLBACK = None
_chunks: list = []


def format_ip_zh(ip: str) -> str:
    """192.168.1.10 → 当前IP地址一九二点一六八点一点一零"""
    ip = (ip or "").strip()
    if not ip:
        return ""
    return "当前IP地址" + ip.translate(_DIGIT_ZH)


def format_lan_connected_ip_zh(ip: str) -> str:
    body = format_ip_zh(ip)
    if not body:
        return ""
    return "局域网已连接，" + body


def _ensure_espeak() -> bool:
    global _inited, _sample_rate, _lib, _CALLBACK, _chunks
    if _inited:
        return _lib is not None
    with _lock:
        if _inited:
            return _lib is not None
        try:
            lib = ctypes.CDLL("libespeak-ng.so.1")
        except OSError:
            _inited = True
            _lib = None
            return False

        AUDIO_OUTPUT_RETRIEVAL = 1
        lib.espeak_Initialize.argtypes = [
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        lib.espeak_Initialize.restype = ctypes.c_int
        lib.espeak_SetVoiceByName.argtypes = [ctypes.c_char_p]
        lib.espeak_SetVoiceByName.restype = ctypes.c_int
        CALLBACK = ctypes.CFUNCTYPE(
            ctypes.c_int, ctypes.POINTER(ctypes.c_short), ctypes.c_int, ctypes.c_void_p
        )
        lib.espeak_SetSynthCallback.argtypes = [CALLBACK]
        lib.espeak_Synth.argtypes = [
            ctypes.c_char_p,
            ctypes.c_size_t,
            ctypes.c_uint,
            ctypes.c_int,
            ctypes.c_uint,
            ctypes.c_uint,
            ctypes.c_void_p,
            ctypes.c_void_p,
        ]
        lib.espeak_Synth.restype = ctypes.c_int
        lib.espeak_Synchronize.restype = ctypes.c_int

        rate = int(lib.espeak_Initialize(AUDIO_OUTPUT_RETRIEVAL, 0, None, 0))
        if rate > 1000:
            _sample_rate = rate

        for voice in (b"cmn-latn-pinyin", b"cmn", b"zh", b"zh-cn", b"Chinese"):
            if int(lib.espeak_SetVoiceByName(voice)) == 0:
                break

        chunks: list = []

        @CALLBACK
        def _cb(wav, numsamples, _event):
            if wav and numsamples > 0:
                chunks.append(ctypes.string_at(wav, numsamples * 2))
            return 0

        lib.espeak_SetSynthCallback(_cb)
        _CALLBACK = _cb
        _chunks = chunks
        _lib = lib
        _inited = True
        return True


def _synthesize_cli(text: str, out_path: str) -> Optional[str]:
    voice = os.environ.get("VOICE_ESPEAK_VOICE", "zh")
    for bin_name in ("espeak-ng", "espeak"):
        if shutil.which(bin_name) is None:
            continue
        cmd = [bin_name, "-v", voice, "-s", "150", "-w", out_path, text]
        try:
            r = subprocess.run(cmd, capture_output=True, timeout=15, check=False)
        except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
            continue
        if (
            r.returncode == 0
            and os.path.isfile(out_path)
            and os.path.getsize(out_path) > 64
        ):
            return out_path
    return None


def synthesize_wav(text: str, out_path: Optional[str] = None) -> Optional[str]:
    text = (text or "").strip()
    if not text:
        return None
    if not out_path:
        fd, out_path = tempfile.mkstemp(prefix="duck_ip_", suffix=".wav")
        os.close(fd)
    if _ensure_espeak() and _lib is not None:
        with _lock:
            _chunks.clear()
            raw = text.encode("utf-8")
            rc = int(_lib.espeak_Synth(raw, len(raw) + 1, 0, 1, 0, 1, None, None))
            if rc == 0:
                _lib.espeak_Synchronize()
                pcm = b"".join(_chunks)
                if len(pcm) >= 64:
                    with wave.open(out_path, "wb") as w:
                        w.setnchannels(1)
                        w.setsampwidth(2)
                        w.setframerate(_sample_rate)
                        w.writeframes(pcm)
                    return out_path
    return _synthesize_cli(text, out_path)


def synthesize_ip_wav(ip: str, out_path: Optional[str] = None) -> Optional[str]:
    spoken = format_lan_connected_ip_zh(ip)
    if not spoken:
        return None
    return synthesize_wav(spoken, out_path=out_path)


DIGIT_SEC = 0.5
IP_REPEAT = 2
IP_GAP_SEC = 3.0
PREFIX_SEC = 1.5
MANDARIN_VOICE = "cmn-latn-pinyin"
MANDARIN_SPEED = "180"
_VOICE_STAMP = "cmn-latn-pinyin-v2"

# 拼音带声调，espeak 的 zh/cmn 会把汉字按英文念
_PINYIN = {
    "0": "ling2",
    "1": "yi1",
    "2": "er4",
    "3": "san1",
    "4": "si4",
    "5": "wu3",
    "6": "liu4",
    "7": "qi1",
    "8": "ba1",
    "9": "jiu3",
    ".": "dian3",
}
PREFIX_PINYIN = "dang1 qian2 di4 zhi3"  # 当前地址，避免拉丁字母 IP 被读成英语

_ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "ip_digits")
_ZH = _PINYIN  # ip_clip_sequence 仍按 IP 字符查找


def _clip_path(token: str) -> str:
    if token == "prefix":
        return os.path.join(_ASSETS_DIR, "prefix.wav")
    if token == ".":
        return os.path.join(_ASSETS_DIR, "dot.wav")
    return os.path.join(_ASSETS_DIR, f"d{token}.wav")


def _synthesize_pinyin(pinyin: str, out_path: str) -> Optional[str]:
    if shutil.which("espeak-ng") is None:
        return None
    cmd = [
        "espeak-ng",
        "-v",
        MANDARIN_VOICE,
        "-s",
        MANDARIN_SPEED,
        "-w",
        out_path,
        pinyin,
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, timeout=15, check=False)
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return None
    if r.returncode == 0 and os.path.isfile(out_path) and os.path.getsize(out_path) > 64:
        return out_path
    return None


def _fit_wav_duration(src: str, dst: str, seconds: float) -> bool:
    try:
        with wave.open(src, "rb") as r:
            nch = r.getnchannels()
            sw = r.getsampwidth()
            fr = r.getframerate()
            data = r.readframes(r.getnframes())
    except (OSError, wave.Error):
        return False
    frame_size = nch * sw
    if frame_size <= 0 or fr <= 0 or sw != 2:
        return False
    samples = array.array("h")
    samples.frombytes(data[: len(data) - (len(data) % 2)])
    # 去掉开头静音，避免 0.5 秒窗口切掉汉字
    thresh = 400
    start = 0
    for i, s in enumerate(samples):
        if abs(s) > thresh:
            start = max(0, i - int(fr * 0.02))
            break
    samples = samples[start:]
    target = int(fr * seconds)
    if len(samples) > target:
        samples = samples[:target]
    else:
        samples.extend([0] * (target - len(samples)))
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with wave.open(dst, "wb") as w:
        w.setnchannels(nch)
        w.setsampwidth(sw)
        w.setframerate(fr)
        w.writeframes(samples.tobytes())
    return os.path.isfile(dst) and os.path.getsize(dst) > 64


def _make_clip(pinyin: str, dst: str, seconds: float) -> bool:
    fd, tmp = tempfile.mkstemp(prefix="duck_digit_", suffix=".wav")
    os.close(fd)
    try:
        path = _synthesize_pinyin(pinyin, tmp)
        if not path:
            return False
        return _fit_wav_duration(path, dst, seconds)
    finally:
        try:
            os.unlink(tmp)
        except OSError:
            pass


def ensure_digit_clips() -> bool:
    """生成普通话数字/点/前缀 WAV（拼音音色），每个数字固定 0.5 秒。"""
    os.makedirs(_ASSETS_DIR, exist_ok=True)
    stamp_path = os.path.join(_ASSETS_DIR, ".voice")
    stamp = ""
    try:
        with open(stamp_path, encoding="utf-8") as f:
            stamp = f.read().strip()
    except OSError:
        stamp = ""
    if stamp != _VOICE_STAMP:
        for name in os.listdir(_ASSETS_DIR):
            if name.endswith(".wav"):
                try:
                    os.unlink(os.path.join(_ASSETS_DIR, name))
                except OSError:
                    pass
    need = [("prefix", PREFIX_PINYIN, PREFIX_SEC)]
    for d, py in _PINYIN.items():
        need.append((d, py, DIGIT_SEC))
    ok = True
    for token, pinyin, sec in need:
        dst = _clip_path(token)
        if os.path.isfile(dst) and os.path.getsize(dst) > 64:
            continue
        if not _make_clip(pinyin, dst, sec):
            ok = False
    if ok:
        try:
            with open(stamp_path, "w", encoding="utf-8") as f:
                f.write(_VOICE_STAMP + "\n")
        except OSError:
            pass
    return ok


def ip_clip_sequence(ip: str) -> list:
    """[(path, duration_sec), ...] 一轮汉语 IP 播报。"""
    ip = (ip or "").strip()
    seq = []
    prefix = _clip_path("prefix")
    if os.path.isfile(prefix):
        seq.append((prefix, PREFIX_SEC))
    for ch in ip:
        if ch not in _PINYIN:
            continue
        path = _clip_path(ch)
        if os.path.isfile(path):
            seq.append((path, DIGIT_SEC))
    return seq
