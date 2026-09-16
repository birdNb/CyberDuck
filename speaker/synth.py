#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DSP 原语，对应 microduck/sounds/src/synth.rs。原生 48 kHz，匹配本机 USB 喇叭。"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np

from rng import Rng

SR = 48_000
TUNED_SR = 22_050.0


def t_axis(duration_s: float) -> np.ndarray:
    n = max(1, int(round(duration_s * SR)))
    return (np.arange(n, dtype=np.float32) / np.float32(SR))


def lerp(t: np.ndarray, points: Sequence[Tuple[float, float]]) -> np.ndarray:
    xp = np.array([p[0] for p in points], dtype=np.float64)
    fp = np.array([p[1] for p in points], dtype=np.float64)
    return np.interp(t.astype(np.float64), xp, fp).astype(np.float32)


def expdecay(t: np.ndarray, attack_s: float, decay_s: float) -> np.ndarray:
    attack = np.float32(max(attack_s, 1e-4))
    decay = np.float32(max(decay_s, 1e-4))
    a = np.clip(t / attack, 0.0, 1.0)
    d = np.exp(-(np.maximum(t - attack, 0.0) / decay))
    return (a * d).astype(np.float32)


def bell(t: np.ndarray, attack_s: float, release_s: float) -> np.ndarray:
    total = float(t[-1]) if t.size else 0.0
    rel_start = max(total - release_s, 0.0)
    attack = max(attack_s, 1e-4)
    release = max(release_s, 1e-4)
    x = t.astype(np.float64)
    v = np.ones_like(x)
    if attack_s > 0:
        v = np.where(x < attack_s, x / attack, v)
    v = np.where(x > rel_start, np.maximum(1.0 - (x - rel_start) / release, 0.0), v)
    return np.clip(v, 0.0, 1.0).astype(np.float32)


def phase_from_freq(freq: np.ndarray) -> np.ndarray:
    acc = np.cumsum(freq.astype(np.float64))
    return (2.0 * math.pi * acc / float(SR)).astype(np.float32)


def harmonic_osc(phase: np.ndarray, weights: Sequence[float]) -> np.ndarray:
    out = np.zeros(phase.shape[0], dtype=np.float32)
    for n, w in enumerate(weights, start=1):
        if w == 0.0:
            continue
        out += np.float32(w) * np.sin(np.float32(n) * phase)
    return out


def vibrato(t: np.ndarray, rate_hz: float, depth_semitones: float, phase: float) -> np.ndarray:
    if rate_hz <= 0.0 or depth_semitones <= 0.0:
        return np.ones(t.shape[0], dtype=np.float32)
    lfo = np.sin(2.0 * math.pi * rate_hz * t.astype(np.float64) + phase)
    return np.power(2.0, depth_semitones * lfo / 12.0).astype(np.float32)


def jitter(t: np.ndarray, depth_semitones: float, rng: Rng) -> np.ndarray:
    if depth_semitones <= 0.0:
        return np.ones(t.shape[0], dtype=np.float32)
    raw = np.array(rng.standard_normal_vec(int(t.shape[0])), dtype=np.float32)
    window = max(1, int(round(64.0 * float(SR) / TUNED_SR)))
    kernel = np.ones(window, dtype=np.float32) / np.float32(window)
    smoothed = np.convolve(raw, kernel, mode="same")
    return np.power(2.0, depth_semitones * smoothed.astype(np.float64) / 12.0).astype(np.float32)


def pink_noise(n: int, rng: Rng) -> np.ndarray:
    a = np.float32(0.985 ** (TUNED_SR / float(SR)))
    leak = 0.0
    pink = np.empty(n, dtype=np.float32)
    for i in range(n):
        leak = a * leak + rng.standard_normal()
        pink[i] = leak
    peak = float(np.max(np.abs(pink))) + 1e-9
    pink /= np.float32(peak)
    return pink


def click(n: int, rng: Rng, length: int) -> np.ndarray:
    out = np.zeros(n, dtype=np.float32)
    length = min(length, n)
    for i in range(length):
        fade = 1.0 - i / max(length, 1)
        out[i] = rng.uniform(-1.0, 1.0) * fade * fade
    return out


def normalise(x: np.ndarray, peak_dbfs: float) -> np.ndarray:
    peak = float(np.max(np.abs(x))) + 1e-9
    target = 10.0 ** (peak_dbfs / 20.0)
    x *= np.float32(target / peak)
    return x


def to_i16(x: np.ndarray) -> np.ndarray:
    scaled = np.clip(x * 32767.0, -32768.0, 32767.0)
    return scaled.astype(np.int16)
