#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""各标签配方，对应 microduck/sounds/src/voices.rs。"""
from __future__ import annotations

import copy
import math
from typing import List, Sequence, Tuple

import numpy as np

from personality import Personality
from rng import Rng
from synth import SR
from synth import bell, click, expdecay, jitter, lerp, normalise, phase_from_freq
from synth import harmonic_osc, pink_noise, t_axis, vibrato


def attack(p: Personality, dur: float, snappy: float) -> float:
    soft = 0.04 * dur
    sharp = 0.003 * dur
    return max(0.001, soft + (sharp - soft) * p.attack_sharpness * snappy)


def voice(
    p: Personality,
    t: np.ndarray,
    freq: np.ndarray,
    rng: Rng,
    am_scale: float,
    breath_scale: float,
) -> np.ndarray:
    vib = vibrato(t, p.vibrato_rate_hz, p.vibrato_depth, rng.uniform(0.0, 2.0 * math.pi))
    jit = jitter(t, p.jitter_depth, rng)
    f = freq * vib * jit
    phase = phase_from_freq(f)
    body = harmonic_osc(phase, p.harmonics())

    am_d = p.am_depth * am_scale * p.quackiness
    if am_d > 0.01:
        am = 1.0 - am_d * (0.5 + 0.5 * np.sin(2.0 * math.pi * p.am_rate_hz * t.astype(np.float64)))
        body *= am.astype(np.float32)

    breath = p.breath * breath_scale
    if breath > 0.0:
        body += np.float32(breath) * pink_noise(int(t.shape[0]), rng)
    return body


def alarm(p: Personality, variant: int) -> np.ndarray:
    rng = p.variant_rng("alarm", variant)
    dur = (0.20 + 0.12 * rng.random()) / p.speed
    t = t_axis(dur)
    f0 = p.pitch_center_hz * (1.25 + 0.35 * p.pitch_spread) * (0.94 + 0.12 * rng.random())
    peak_mul = 1.15 + 0.25 * p.pitch_spread + 0.10 * rng.random()
    fall_mul = 0.75 + 0.20 * (1.0 - p.pitch_spread)
    freq = lerp(t, [(0.0, f0), (0.05 * dur, f0 * peak_mul), (dur, f0 * fall_mul)])
    env = expdecay(t, attack(p, dur, 1.0), dur * (0.40 + 0.20 * rng.random()))
    sig = voice(p, t, freq, rng, 0.5, 1.0) * env
    crackle = 0.04 + 0.10 * p.brightness
    noise = np.array(rng.standard_normal_vec(int(t.shape[0])), dtype=np.float32)
    sig += np.float32(crackle) * noise * env
    return normalise(sig, -3.0)


def greet_syllable(p: Personality, rng: Rng, dur_scale: float, f0_scale: float) -> np.ndarray:
    dur = (0.32 + 0.25 * rng.random()) * dur_scale / p.speed
    t = t_axis(dur)
    f0 = p.pitch_center_hz * (0.9 + 0.15 * rng.random()) * f0_scale
    bias = p.glide_bias
    bend = 0.10 + 0.15 * p.pitch_spread
    start = f0 * (1.0 - bias * bend * 0.5)
    mid = f0 * (1.0 + bias * bend)
    end = f0 * (1.0 - bias * bend * 0.3) * (0.92 + 0.08 * rng.random())
    freq = lerp(t, [(0.0, start), (0.18 * dur, mid), (dur, end)])
    env = expdecay(t, attack(p, dur, 0.5), dur * 0.7)
    return voice(p, t, freq, rng, 1.0, 1.0) * env


def greet(p: Personality, variant: int) -> np.ndarray:
    rng = p.variant_rng("greet", variant)
    parts: List[np.ndarray] = [greet_syllable(p, rng, 1.0, 1.0)]
    if rng.random() < 0.4:
        gap_n = int((0.05 + 0.06 * rng.random()) / p.speed * SR)
        parts.append(np.zeros(gap_n, dtype=np.float32))
        parts.append(greet_syllable(p, rng, 0.8, 0.95 + 0.06 * rng.random()))
    return normalise(np.concatenate(parts), -3.0)


def inquire(p: Personality, variant: int) -> np.ndarray:
    rng = p.variant_rng("inquire", variant)
    dur = (0.42 + 0.25 * rng.random()) / p.speed
    t = t_axis(dur)
    f0 = p.pitch_center_hz * (0.88 + 0.10 * rng.random())
    rise = 1.15 + 0.50 * p.pitch_spread + 0.20 * max(p.glide_bias, 0.0) + 0.10 * rng.random()
    freq = lerp(t, [(0.0, f0 * 0.92), (0.30 * dur, f0 * 0.95), (dur, f0 * rise)])
    env = bell(t, dur * (0.06 + 0.10 * (1.0 - p.attack_sharpness)), dur * 0.32)
    sig = voice(p, t, freq, rng, 0.6, 1.0) * env
    return normalise(sig, -3.0)


def peck(p: Personality, variant: int) -> np.ndarray:
    rng = p.variant_rng("peck", variant)
    dur = (0.16 + 0.12 * rng.random()) / p.speed
    t = t_axis(dur)
    f0 = p.pitch_center_hz * (0.45 + 0.20 * rng.random())
    freq = lerp(t, [(0.0, f0 * 1.5), (0.04 * dur, f0), (dur, f0 * 0.80)])
    env = expdecay(t, attack(p, dur, 1.0), dur * 0.35)
    body = voice(p, t, freq, rng, 0.3, 0.5) * env
    click_len = int((0.003 + 0.006 * p.attack_sharpness) * SR)
    click_gain = 0.4 + 0.4 * p.attack_sharpness
    body += np.float32(click_gain) * click(int(t.shape[0]), rng, click_len)
    return normalise(body, -3.0)


def chirp_syllable(
    p: Personality,
    rng: Rng,
    f0: float,
    dur: float,
    contour: Sequence[Tuple[float, float]],
) -> np.ndarray:
    t = t_axis(dur)
    warble = vibrato(t, p.warble_hz, p.warble_depth, rng.uniform(0.0, 2.0 * math.pi))
    freq = np.float32(f0) * lerp(t, contour) * warble
    env = expdecay(t, attack(p, dur, 0.7), dur * 0.55)
    p_soft = copy.copy(p)
    p_soft.quackiness *= 0.2
    p_soft.brightness *= 0.5
    p_soft.formant_gain *= 0.5
    return voice(p_soft, t, freq, rng, 1.0, 0.4) * env


def chirp(p: Personality, variant: int) -> np.ndarray:
    rng = p.variant_rng("chirp", variant)
    shape = variant % 4
    f0 = p.pitch_center_hz * (0.95 + 0.75 * rng.random())
    if shape == 0:
        dur = (0.10 + 0.10 * rng.random()) / p.speed
        peak = 1.12 + 0.10 * rng.random()
        sig = chirp_syllable(p, rng, f0, dur, [(0.0, 0.88), (0.5 * dur, peak), (dur, 1.05)])
    elif shape == 1:
        dur = (0.10 + 0.10 * rng.random()) / p.speed
        start = 1.12 + 0.10 * rng.random()
        sig = chirp_syllable(p, rng, f0, dur, [(0.0, start), (0.3 * dur, 1.0), (dur, 0.78)])
    elif shape == 2:
        dur = (0.22 + 0.14 * rng.random()) / p.speed
        p_trill = copy.copy(p)
        p_trill.warble_depth = max(p.warble_depth, 0.7) * 1.6
        sig = chirp_syllable(
            p_trill, rng, f0, dur, [(0.0, 1.0), (0.5 * dur, 1.06), (dur, 0.94)]
        )
    else:
        dur = (0.08 + 0.05 * rng.random()) / p.speed
        first = chirp_syllable(p, rng, f0, dur, [(0.0, 0.95), (0.4 * dur, 1.10), (dur, 0.88)])
        gap_n = int((0.03 + 0.03 * rng.random()) / p.speed * SR)
        dur_b = dur * 0.9
        f0_b = f0 * (0.92 + 0.10 * rng.random())
        second = chirp_syllable(
            p, rng, f0_b, dur_b, [(0.0, 0.95), (0.4 * dur_b, 1.10), (dur_b, 0.88)]
        )
        sig = np.concatenate([first, np.zeros(gap_n, dtype=np.float32), second])
    return normalise(sig, -6.0)


def coo(p: Personality, variant: int) -> np.ndarray:
    rng = p.variant_rng("coo", variant)
    dur = (0.85 + 0.55 * rng.random()) / p.speed
    t = t_axis(dur)
    f0 = (
        p.pitch_center_hz
        * (0.42 + 0.15 * (1.0 - p.attack_sharpness))
        * (0.94 + 0.12 * rng.random())
    )
    drift_a = 1.0 + 0.05 * rng.random() + 0.04 * p.glide_bias
    freq = lerp(t, [(0.0, f0 * 0.94), (dur * 0.5, f0 * drift_a), (dur, f0 * 0.90)])
    env = bell(t, dur * (0.18 + 0.10 * (1.0 - p.attack_sharpness)), dur * 0.30)
    p_soft = copy.copy(p)
    p_soft.breath = max(p.breath, 0.12) + 0.10
    p_soft.quackiness = p.quackiness * 0.25
    p_soft.am_rate_hz = p.am_rate_hz * 0.30
    p_soft.vibrato_rate_hz = p.vibrato_rate_hz * 0.45
    p_soft.vibrato_depth = p.vibrato_depth * 0.7
    sig = voice(p_soft, t, freq, rng, 1.0, 1.0) * env
    return normalise(sig, -5.0)


def wheee_segments(p: Personality, variant: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    rng = p.variant_rng("wheee", variant)
    d_start = (0.80 + 0.30 * rng.random()) / p.speed
    d_loop = (1.60 + 0.60 * rng.random()) / p.speed
    d_end = (0.55 + 0.25 * rng.random()) / p.speed
    total = d_start + d_loop + d_end
    t = t_axis(total)
    t1, t2 = d_start, d_start + d_loop
    f0 = p.pitch_center_hz * (0.95 + 0.10 * rng.random())
    top = 1.6 + 0.5 * p.pitch_spread + 0.25 * rng.random()
    base = lerp(
        t,
        [
            (0.0, f0 * 0.85),
            (0.15 * d_start, f0),
            (t1, f0 * top),
            (t2, f0 * top),
            (t2 + 0.25 * d_end, f0 * top * 1.04),
            (total, f0 * 0.60),
        ],
    )
    wob_hz = 4.5 + 3.0 * rng.random()
    swell = lerp(t, [(0.0, 0.15), (t1, 1.0), (total, 1.0)])
    wob = np.sin(2.0 * math.pi * wob_hz * t.astype(np.float64))
    freq = base * np.power(2.0, 0.5 * swell.astype(np.float64) * wob / 12.0).astype(np.float32)
    env = lerp(
        t,
        [
            (0.0, 0.0),
            (min(0.06, 0.5 * d_start), 1.0),
            (t2 + 0.40 * d_end, 1.0),
            (total, 0.0),
        ],
    )
    p_joy = copy.copy(p)
    p_joy.vibrato_depth = p.vibrato_depth * 0.5
    p_joy.quackiness = p.quackiness * 0.5
    sig = voice(p_joy, t, freq, rng, 0.5, 0.5) * env
    normalise(sig, -4.0)

    n1 = int(t1 * SR)
    n2 = int(t2 * SR)
    start = sig[:n1].copy()
    loop_seg = sig[n1:n2].copy()
    end = sig[n2:].copy()
    nx = min(int(0.08 * SR), loop_seg.size // 2, n1)
    if nx > 1:
        w = np.linspace(0.0, 1.0, nx, dtype=np.float32)
        loop_seg[-nx:] = (1.0 - w) * loop_seg[-nx:] + w * sig[n1 - nx : n1]
    return start, loop_seg, end


def wheee(p: Personality, variant: int) -> np.ndarray:
    start, loop_seg, end = wheee_segments(p, variant)
    return np.concatenate([start, loop_seg, loop_seg, end])
