#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从种子派生一只鸭子的嗓音，对应 microduck/sounds/src/personality.rs。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

from rng import Rng, crc32


@dataclass
class Personality:
    seed: int
    pitch_center_hz: float
    register: float
    pitch_spread: float
    glide_bias: float
    brightness: float
    tilt: float
    nasal: float
    harmonic_skew: float
    formant_n: int
    formant_gain: float
    vibrato_rate_hz: float
    vibrato_depth: float
    jitter_depth: float
    breath: float
    quackiness: float
    am_rate_hz: float
    am_depth: float
    warble_hz: float
    warble_depth: float
    attack_sharpness: float
    speed: float

    @classmethod
    def from_seed(cls, seed: int) -> "Personality":
        rng = Rng.from_seed(seed & 0xFFFFFFFF)
        register = rng.choice([-1.0, 0.0, 0.0, 1.0]) + rng.uniform(-0.4, 0.4)
        base = rng.uniform(160.0, 380.0)
        pitch = min(620.0, max(110.0, base * (2.0 ** (register * 0.45))))
        return cls(
            seed=seed & 0xFFFFFFFF,
            pitch_center_hz=pitch,
            register=register,
            pitch_spread=rng.uniform(0.4, 1.2),
            glide_bias=rng.uniform(-1.0, 1.0),
            brightness=rng.uniform(0.05, 0.55),
            tilt=rng.uniform(1.4, 2.8),
            nasal=rng.uniform(0.1, 1.0),
            harmonic_skew=rng.uniform(-1.0, 1.0),
            formant_n=int(rng.integers(1, 6)),
            formant_gain=rng.uniform(0.0, 1.4),
            vibrato_rate_hz=rng.uniform(3.5, 9.5),
            vibrato_depth=rng.uniform(0.0, 0.7),
            jitter_depth=rng.uniform(0.03, 0.35),
            breath=rng.uniform(0.0, 0.30),
            quackiness=rng.uniform(0.2, 1.0),
            am_rate_hz=rng.uniform(18.0, 55.0),
            am_depth=rng.uniform(0.15, 0.70),
            warble_hz=rng.uniform(7.0, 18.0),
            warble_depth=rng.uniform(0.0, 1.4),
            attack_sharpness=rng.uniform(0.0, 1.0),
            speed=rng.uniform(0.82, 1.22),
        )

    def variant_rng(self, tag: str, variant: int) -> Rng:
        h = (
            (self.seed * 1_000_003)
            ^ crc32(tag.encode("utf-8"))
            ^ ((variant & 0xFFFFFFFF) * 2_654_435_761)
        ) & 0xFFFFFFFFFFFFFFFF
        return Rng.from_seed(h & 0xFFFFFFFF)

    def harmonics(self) -> List[float]:
        n_harm = 7
        weights: List[float] = []
        for n in range(1, n_harm + 1):
            base = 1.0 / (n ** self.tilt)
            high_lift = self.brightness * ((n / n_harm) ** 1.5)
            nasal = self.nasal * (0.6 if n in (2, 3) else 0.0)
            if self.harmonic_skew >= 0.0:
                skew = self.harmonic_skew * (0.4 if n % 2 == 0 else -0.2)
            else:
                skew = -self.harmonic_skew * (-0.3 if n % 2 == 0 else 0.4)
            formant = self.formant_gain if n == self.formant_n else 0.0
            weights.append(max(0.0, base + high_lift + nasal + skew + formant * base * 1.5))
        weights[0] = max(weights[0], 0.7)
        return weights
