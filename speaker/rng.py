#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""xoshiro256++ + Box–Muller，对应 microduck/sounds/src/rng.rs。

音色由种子派生，生成器必须固定，不能用 numpy / random 的算法。
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence


class Rng:
    """确定性随机流。"""

    def __init__(self, seed: int) -> None:
        x = seed & 0xFFFFFFFF
        state: List[int] = []
        for _ in range(4):
            x = (x + 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
            z = x
            z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & 0xFFFFFFFFFFFFFFFF
            z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & 0xFFFFFFFFFFFFFFFF
            state.append(z ^ (z >> 31))
        self._s = state
        self._spare_normal: Optional[float] = None

    @classmethod
    def from_seed(cls, seed: int) -> "Rng":
        return cls(seed)

    def next_u64(self) -> int:
        s = self._s
        result = ((s[0] + s[3]) & 0xFFFFFFFFFFFFFFFF)
        result = ((result << 23) | (result >> (64 - 23))) & 0xFFFFFFFFFFFFFFFF
        result = (result + s[0]) & 0xFFFFFFFFFFFFFFFF
        t = (s[1] << 17) & 0xFFFFFFFFFFFFFFFF
        s[2] ^= s[0]
        s[3] ^= s[1]
        s[1] ^= s[2]
        s[0] ^= s[3]
        s[2] ^= t
        s[3] = ((s[3] << 45) | (s[3] >> (64 - 45))) & 0xFFFFFFFFFFFFFFFF
        return result

    def random(self) -> float:
        return (self.next_u64() >> 11) * (1.0 / float(1 << 53))

    def uniform(self, lo: float, hi: float) -> float:
        return lo + (hi - lo) * self.random()

    def integers(self, lo: int, hi: int) -> int:
        return lo + int(math.floor(self.random() * (hi - lo)))

    def choice(self, choices: Sequence[float]) -> float:
        return float(choices[self.integers(0, len(choices))])

    def standard_normal(self) -> float:
        if self._spare_normal is not None:
            z = self._spare_normal
            self._spare_normal = None
            return z
        r = 1.0 - self.random()
        theta = 2.0 * math.pi * self.random()
        mag = math.sqrt(-2.0 * math.log(r))
        self._spare_normal = mag * math.sin(theta)
        return mag * math.cos(theta)

    def standard_normal_vec(self, n: int) -> List[float]:
        return [self.standard_normal() for _ in range(n)]


def crc32(data: bytes) -> int:
    """IEEE CRC-32，与 zlib.crc32 / Rust 实现对齐。"""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            mask = -(crc & 1)
            crc = (crc >> 1) ^ (0xEDB88320 & mask)
            crc &= 0xFFFFFFFF
    return (~crc) & 0xFFFFFFFF
