#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""鸭子嗓音：合成 + 播放入口，对应 microduck 的 sounds crate。"""
from __future__ import annotations

import hashlib
import os
import wave
from pathlib import Path
from typing import Callable, Dict, Optional

import numpy as np

from personality import Personality
from play import play_pcm
from synth import SR, to_i16
from voices import alarm, chirp, coo, greet, inquire, peck, wheee

Recipe = Callable[[Personality, int], np.ndarray]

TAGS: Dict[str, Recipe] = {
    "alarm": alarm,
    "greet": greet,
    "inquire": inquire,
    "peck": peck,
    "chirp": chirp,
    "coo": coo,
    "wheee": wheee,
}


def variant_count(tag: str) -> int:
    if tag in ("greet", "chirp"):
        return 12
    if tag == "wheee":
        return 6
    return 10


def seed_from_id(machine_id: str) -> int:
    digest = hashlib.sha256(machine_id.encode("utf-8")).digest()
    return int.from_bytes(digest[:4], "big")


def hardware_seed() -> int:
    named = os.environ.get("DUCK_IDENTITY")
    if named:
        return seed_from_id(named)
    serial = Path("/proc/device-tree/serial-number")
    if serial.is_file():
        raw = serial.read_bytes().split(b"\x00", 1)[0].decode("utf-8", "replace").strip()
        if raw:
            return seed_from_id(raw)
    machine = Path("/etc/machine-id")
    if machine.is_file():
        text = machine.read_text(encoding="utf-8", errors="replace").strip()
        if text:
            return seed_from_id(text)
    return seed_from_id("microDuckPro")


def render(tag: str, personality: Personality, variant: int = 0) -> np.ndarray:
    recipe = TAGS.get(tag)
    if recipe is None:
        raise KeyError(f"未知标签 {tag!r}，可选: {', '.join(TAGS)}")
    return recipe(personality, variant)


def to_wav(buffer: np.ndarray, path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pcm = to_i16(np.asarray(buffer, dtype=np.float32))
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(SR)
        wav.writeframes(pcm.tobytes())


class Speaker:
    """一只鸭子的喇叭：固定种子，随机变体，播到 USB 扬声器。"""

    def __init__(self, seed: Optional[int] = None, device: Optional[str] = None) -> None:
        self.seed = hardware_seed() if seed is None else seed & 0xFFFFFFFF
        self.personality = Personality.from_seed(self.seed)
        self.device = device
        self.volume = 1.0

    def quack(self, tag: str = "greet", variant: Optional[int] = None) -> np.ndarray:
        if variant is None:
            variant = int.from_bytes(os.urandom(2), "little") % variant_count(tag)
        buf = render(tag, self.personality, variant)
        vol = max(0.0, min(1.0, float(self.volume)))
        if vol < 0.999:
            buf = np.clip(buf * vol, -1.0, 1.0)
        used = play_pcm(buf, self.device)
        self.device = self.device or used
        return buf
