#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LSM6DSV16X：I2C6，40 针 pin31=SDA（GPIO24）、pin11=SCL（GPIO25）。"""
from __future__ import annotations

import ctypes
import os
import subprocess
import time
from dataclasses import dataclass

DIR = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(DIR, "i2c_gpio.c")
SO = os.path.join(DIR, "i2c_gpio.so")

GPIO_CHIP = 4
SDA_LINE = 24  # pin31, I2C6 SDA
SCL_LINE = 25  # pin11, I2C6 SCL

WHO_AM_I = 0x0F
WHO_AM_I_VAL = 0x70
CTRL1_XL = 0x10
CTRL2_G = 0x11
CTRL3_C = 0x12
CTRL6_C = 0x15
CTRL8_XL = 0x17
CTRL9_XL = 0x18
OUTX_L_G = 0x22

SW_RESET = 1 << 0
IF_INC = 1 << 2
BDU = 1 << 6
BOOT = 1 << 7

XL_ODR_240HZ = 0x07
XL_MODE_HP = 0x03
GYRO_FS_2000 = 0x04
XL_FS_2G = 0x00
XL_FS_8G = 0x02

# ±8 g → 0.244 mg/LSB；±2000 dps → 70 mdps/LSB
ACC_LSB_G = 0.000244
GYRO_LSB_DPS = 0.070
GYRO_LSB_RAD = GYRO_LSB_DPS * 3.141592653589793 / 180.0

ADDRS = (0x6A, 0x6B)


def _load_mount() -> tuple[tuple[float, float, float], ...]:
    """R @ v_sensor = v_base_link。文件缺失时用单位阵，不再套官方鸭子的安装旋转。"""
    root = os.path.dirname(DIR)
    paths = (
        os.path.join(root, "sport_ctrl", "locomotion", "config", "imu_mount.json"),
        os.path.join(root, "locomotion", "config", "imu_mount.json"),
    )
    try:
        import json

        for path in paths:
            if not os.path.isfile(path):
                continue
            with open(path, encoding="utf-8") as f:
                rows = json.load(f)["R"]
            return tuple(tuple(float(v) for v in row) for row in rows)
    except (OSError, KeyError, TypeError, ValueError):
        pass
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


_MOUNT = _load_mount()


def rotate_mount(v: tuple[float, float, float]) -> tuple[float, float, float]:
    """sensor → base_link，乘 locomotion/config/imu_mount.json 里的 R。"""
    return tuple(sum(_MOUNT[i][k] * v[k] for k in range(3)) for i in range(3))


def normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
    if n < 1e-9:
        return (0.0, 0.0, -1.0)
    return (v[0] / n, v[1] / n, v[2] / n)


@dataclass
class PolicyImu:
    """策略观测用 IMU：对齐 microduck ImuData 的 gyro + gravity。"""

    gyro: tuple[float, float, float]  # trunk, rad/s
    gravity: tuple[float, float, float]  # trunk, 单位向量；直立 ≈ [0, 0, -1]


class I2CGpioError(RuntimeError):
    pass


def build_so() -> None:
    if os.path.exists(SO) and os.path.getmtime(SO) >= os.path.getmtime(SRC):
        return
    subprocess.check_call(["gcc", "-O2", "-shared", "-fPIC", "-o", SO, SRC], cwd=DIR)


def load_lib() -> ctypes.CDLL:
    build_so()
    lib = ctypes.CDLL(SO)
    lib.i2c_gpio_open.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    lib.i2c_gpio_open.restype = ctypes.c_int
    lib.i2c_gpio_close.argtypes = []
    lib.i2c_gpio_close.restype = None
    lib.i2c_gpio_xfer.argtypes = [
        ctypes.c_uint8,
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_int,
    ]
    lib.i2c_gpio_xfer.restype = ctypes.c_int
    lib.i2c_gpio_scan.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_int]
    lib.i2c_gpio_scan.restype = ctypes.c_int
    lib.i2c_gpio_recover.argtypes = []
    lib.i2c_gpio_recover.restype = None
    lib.i2c_gpio_error.argtypes = []
    lib.i2c_gpio_error.restype = ctypes.c_char_p
    return lib


@dataclass
class ImuSample:
    ax: int
    ay: int
    az: int
    gx: int
    gy: int
    gz: int

    @property
    def acc_g(self) -> tuple[float, float, float]:
        return self.ax * ACC_LSB_G, self.ay * ACC_LSB_G, self.az * ACC_LSB_G

    @property
    def gyro_dps(self) -> tuple[float, float, float]:
        return self.gx * GYRO_LSB_DPS, self.gy * GYRO_LSB_DPS, self.gz * GYRO_LSB_DPS

    def to_policy(self) -> PolicyImu:
        """转成策略推理字段（gyro rad/s + 投影重力）。

        正式机重力来自 SFLP 四元数；直连 I2C 测试用 −normalize(acc) 近似。
        """
        gyro = rotate_mount(
            (
                self.gx * GYRO_LSB_RAD,
                self.gy * GYRO_LSB_RAD,
                self.gz * GYRO_LSB_RAD,
            )
        )
        ax, ay, az = self.acc_g
        gravity = normalize(tuple(-c for c in rotate_mount((ax, ay, az))))
        return PolicyImu(gyro=gyro, gravity=gravity)


class LSM6DSV16X:
    def __init__(self, lib: ctypes.CDLL, addr: int):
        self.lib = lib
        self.addr = addr

    def _err(self) -> str:
        raw = self.lib.i2c_gpio_error()
        return (raw or b"unknown").decode("utf-8", "replace")

    def _xfer(self, w: bytes = b"", rlen: int = 0) -> bytes:
        wbuf = ctypes.create_string_buffer(w, len(w)) if w else None
        rbuf = ctypes.create_string_buffer(rlen) if rlen else None
        if self.lib.i2c_gpio_xfer(self.addr, wbuf, len(w), rbuf, rlen) != 0:
            raise I2CGpioError(self._err())
        return rbuf.raw if rbuf else b""

    def write_reg(self, reg: int, val: int) -> None:
        self._xfer(bytes([reg & 0xFF, val & 0xFF]))

    def read_reg(self, reg: int, n: int = 1) -> bytes:
        return self._xfer(bytes([reg & 0xFF]), n)

    def who_am_i(self) -> int:
        return self.read_reg(WHO_AM_I)[0]

    def init(self) -> None:
        # 与 LQ_LSM6DSV16X_Hard.c 中 LQ_LSM60DSV16X_Init 一致
        self.write_reg(CTRL3_C, SW_RESET)
        time.sleep(0.01)
        self.write_reg(CTRL3_C, BOOT)
        time.sleep(0.02)
        self.write_reg(CTRL3_C, BDU | IF_INC)
        self.write_reg(CTRL9_XL, 0x02)
        self.write_reg(CTRL1_XL, (XL_MODE_HP << 4) | XL_ODR_240HZ)
        self.write_reg(CTRL2_G, (XL_MODE_HP << 4) | XL_ODR_240HZ)
        self.write_reg(CTRL6_C, GYRO_FS_2000 | 0x01)
        self.write_reg(CTRL8_XL, XL_FS_8G)
        time.sleep(0.02)

    def read_raw(self) -> ImuSample:
        buf = self.read_reg(OUTX_L_G, 12)

        def s16(lo: int, hi: int) -> int:
            v = lo | (hi << 8)
            return v - 0x10000 if v & 0x8000 else v

        return ImuSample(
            gx=s16(buf[0], buf[1]),
            gy=s16(buf[2], buf[3]),
            gz=s16(buf[4], buf[5]),
            ax=s16(buf[6], buf[7]),
            ay=s16(buf[8], buf[9]),
            az=s16(buf[10], buf[11]),
        )


def open_bus(lib: ctypes.CDLL, pullup: bool) -> None:
    if lib.i2c_gpio_open(GPIO_CHIP, SDA_LINE, SCL_LINE, 1 if pullup else 0) != 0:
        raw = lib.i2c_gpio_error() or b"unknown"
        raise I2CGpioError(raw.decode("utf-8", "replace"))


def scan_bus(lib: ctypes.CDLL) -> list[int]:
    buf = (ctypes.c_uint8 * 16)()
    n = lib.i2c_gpio_scan(buf, 16)
    if n < 0:
        raw = lib.i2c_gpio_error() or b"unknown"
        raise I2CGpioError(raw.decode("utf-8", "replace"))
    return [int(buf[i]) for i in range(n)]
