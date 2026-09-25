#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在 I2C6（GPIO_24 SDA / GPIO_25 SCL）上跑 LSM6DSV16X。

只打印策略推理需要的 IMU 观测（对齐 microduck duck-control obs）：
  gyro[3]     trunk 系角速度，rad/s
  gravity[3]  trunk 系投影重力，单位向量（直立 ≈ 0 0 -1）

接线（40 针排针）：
  VCC -> PIN_1  3.3V
  GND -> PIN_6  GND
  SDA -> PIN_3  GPIO_24
  SCL -> PIN_5  GPIO_25

用法：
  python3 test_imu.py
  python3 test_imu.py --hz 50
  python3 test_imu.py --scan
Ctrl+C 退出。
"""
from __future__ import annotations

import argparse
import atexit
import fcntl
import os
import signal
import sys
import time

from lsm6dsv16x import (
    ADDRS,
    GPIO_CHIP,
    SCL_LINE,
    SDA_LINE,
    WHO_AM_I_VAL,
    I2CGpioError,
    LSM6DSV16X,
    load_lib,
    open_bus,
    scan_bus,
)


LOCK_PATH = "/tmp/microduck-imu-i2c6.lock"


def acquire_lock() -> int:
    fd = os.open(LOCK_PATH, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd)
        raise I2CGpioError("GPIO 24/25 已被占用：请先 Ctrl+C 停掉正在跑的 test_imu.py")
    os.write(fd, f"{os.getpid()}\n".encode())
    return fd


def maybe_rerun_as_root(need_pullup: bool) -> None:
    if not need_pullup or os.geteuid() == 0:
        return
    print("打开 SoC 内部上拉需要 /dev/mem，正在用 sudo 重新启动…")
    os.execvp("sudo", ["sudo", "-E", sys.executable, os.path.abspath(__file__), *sys.argv[1:]])


def probe(lib, pullup: bool) -> LSM6DSV16X:
    open_bus(lib, pullup=pullup)
    found = scan_bus(lib)
    addrs = [a for a in found if a in ADDRS] or list(ADDRS)
    last_err = "未找到 LSM6DSV16X"
    for addr in addrs:
        imu = LSM6DSV16X(lib, addr)
        try:
            wid = imu.who_am_i()
        except I2CGpioError as e:
            last_err = str(e)
            continue
        if wid == WHO_AM_I_VAL:
            imu.init()
            print(f"LSM6DSV16X @ 0x{addr:02X}  OK  (SDA=GPIO_{SDA_LINE} SCL=GPIO_{SCL_LINE})")
            return imu
        last_err = f"0x{addr:02X} WHO_AM_I=0x{wid:02X}，期望 0x{WHO_AM_I_VAL:02X}"
    raise I2CGpioError(last_err)


def main() -> int:
    parser = argparse.ArgumentParser(description="打印策略用 IMU：gyro + gravity")
    parser.add_argument("--scan", action="store_true", help="只扫描总线后退出")
    parser.add_argument("--hz", type=float, default=50.0, help="打印频率，默认 50")
    parser.add_argument(
        "--pullup",
        action="store_true",
        help="用 TLMM 打开 GPIO_24/25 内部上拉（需要 sudo）",
    )
    args = parser.parse_args()
    maybe_rerun_as_root(args.pullup)

    try:
        lock_fd = acquire_lock()
    except I2CGpioError as e:
        print(f"打开 IMU 失败: {e}", file=sys.stderr)
        return 1

    lib = load_lib()
    atexit.register(lib.i2c_gpio_close)
    signal.signal(signal.SIGTERM, lambda *_a: sys.exit(0))

    try:
        try:
            if args.scan:
                open_bus(lib, pullup=args.pullup)
                found = scan_bus(lib)
                print("扫描到: " + (", ".join(f"0x{a:02X}" for a in found) if found else "（无）"))
                return 0
            imu = probe(lib, pullup=args.pullup)
        except I2CGpioError as e:
            print(f"打开 IMU 失败: {e}", file=sys.stderr)
            return 1

        print("obs: gyro[3] rad/s + gravity[3]  (trunk, 直立 gravity≈0 0 -1)")
        print("Ctrl+C 退出")
        period = 1.0 / max(args.hz, 1.0)
        misses = 0
        while True:
            t0 = time.monotonic()
            try:
                p = imu.read_raw().to_policy()
                misses = 0
            except I2CGpioError as e:
                misses += 1
                print(f"I2C 失败 ({misses}): {e}", file=sys.stderr)
                imu.lib.i2c_gpio_recover()
                time.sleep(0.02)
                if misses >= 30:
                    return 1
                continue
            gx, gy, gz = p.gyro
            grx, gry, grz = p.gravity
            print(
                f"gyro {gx:+7.3f} {gy:+7.3f} {gz:+7.3f}   "
                f"gravity {grx:+6.3f} {gry:+6.3f} {grz:+6.3f}",
                flush=True,
            )
            spent = time.monotonic() - t0
            if spent < period:
                time.sleep(period - spent)
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        lib.i2c_gpio_close()
        os.close(lock_fd)


if __name__ == "__main__":
    sys.exit(main())
