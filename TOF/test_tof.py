#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""编译并运行 VL53L5CX 8x8 测距（I2C6 GPIO_24/25）。

接线与 IMU 相同：SDA=PIN_3(GPIO_24)，SCL=PIN_5(GPIO_25)，3.3V + GND。
首次启动会通过 I2C 下载约 84KB 固件，软件 I2C 可能要几十秒。
"""
from __future__ import annotations

import os
import subprocess
import sys

DIR = os.path.dirname(os.path.abspath(__file__))
IMU = os.path.join(os.path.dirname(DIR), "IMU")
ULD = os.path.join(DIR, "VL53L5CX_ULD_driver")
BIN = os.path.join(DIR, "test_tof")
SRCS = [
    os.path.join(DIR, "test_tof.c"),
    os.path.join(ULD, "Platform", "platform.c"),
    os.path.join(ULD, "VL53L5CX_ULD_API", "src", "vl53l5cx_api.c"),
    os.path.join(IMU, "i2c_gpio.c"),
]


def need_build() -> bool:
    if not os.path.exists(BIN):
        return True
    bt = os.path.getmtime(BIN)
    return any(os.path.getmtime(s) > bt for s in SRCS)


def build() -> None:
    cmd = [
        "gcc",
        "-O2",
        "-o",
        BIN,
        f"-I{os.path.join(ULD, 'VL53L5CX_ULD_API', 'inc')}",
        f"-I{os.path.join(ULD, 'Platform')}",
        f"-I{IMU}",
        *SRCS,
    ]
    print("编译 VL53L5CX ULD…")
    subprocess.check_call(cmd)


def main() -> int:
    if need_build():
        build()
    os.execv(BIN, [BIN, *sys.argv[1:]])


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as e:
        print(f"编译失败: {e}", file=sys.stderr)
        raise SystemExit(1)
