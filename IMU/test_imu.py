#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在 I2C6（GPIO_24 SDA / GPIO_25 SCL）上跑 LSM6DSV16X。

接线（40 针排针）：
  VCC -> PIN_1  3.3V
  GND -> PIN_6  GND
  SDA -> PIN_3  GPIO_24
  SCL -> PIN_5  GPIO_25

同一条 I2C6 上还有 VL53L5CX（0x29），本程序只访问 LSM6DSV16X（0x6A/0x6B）。

用法：
  python3 test_imu.py
  python3 test_imu.py --scan
Ctrl+C 退出。不要加 --pullup（那是缺上拉时才需要的 sudo 路径）。
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
TOF_ADDR = 0x29


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
    print(
        f"I2C6 GPIO 开漏：gpiochip{GPIO_CHIP}  SDA=GPIO_{SDA_LINE}  SCL=GPIO_{SCL_LINE}"
        f"{'  (TLMM 内部上拉)' if pullup else ''}"
    )
    if found:
        names = []
        for a in found:
            tag = ""
            if a in ADDRS:
                tag = "(LSM6DSV16X)"
            elif a == TOF_ADDR:
                tag = "(VL53L5CX TOF)"
            names.append(f"0x{a:02X}{tag}")
        print("扫描到: " + ", ".join(names))
    else:
        print("扫描到: （无设备）")

    addrs = [a for a in found if a in ADDRS] or list(ADDRS)
    last_err = "未找到 LSM6DSV16X"
    for addr in addrs:
        imu = LSM6DSV16X(lib, addr)
        try:
            wid = imu.who_am_i()
        except I2CGpioError as e:
            last_err = str(e)
            continue
        print(f"0x{addr:02X} WHO_AM_I=0x{wid:02X}" + ("  OK" if wid == WHO_AM_I_VAL else "  （不是 0x70）"))
        if wid == WHO_AM_I_VAL:
            imu.init()
            return imu
        last_err = f"0x{addr:02X} WHO_AM_I=0x{wid:02X}，期望 0x{WHO_AM_I_VAL:02X}"
    raise I2CGpioError(last_err)


def main() -> int:
    parser = argparse.ArgumentParser(description="LSM6DSV16X on I2C6 GPIO 24/25")
    parser.add_argument("--scan", action="store_true", help="只扫描总线后退出")
    parser.add_argument("--hz", type=float, default=20.0, help="打印频率，默认 20")
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

    def _release(*_a):
        lib.i2c_gpio_close()

    atexit.register(_release)
    signal.signal(signal.SIGTERM, lambda *_a: sys.exit(0))

    try:
        try:
            imu = probe(lib, pullup=args.pullup)
        except I2CGpioError as e:
            print(f"打开 IMU 失败: {e}", file=sys.stderr)
            err = str(e)
            if "busy" in err.lower() or "占用" in err:
                print("请先结束占用 GPIO_24/25 的进程后再跑。", file=sys.stderr)
            else:
                print("请确认：VCC=3.3V, GND, SDA=PIN_3(GPIO_24), SCL=PIN_5(GPIO_25)", file=sys.stderr)
                if not args.pullup and "上拉" in err:
                    print("若模块没有上拉电阻，再试: sudo python3 test_imu.py --pullup", file=sys.stderr)
            return 1

        if args.scan:
            return 0

        print("加速度 ±8g，陀螺仪 ±2000 dps，240 Hz。Ctrl+C 退出。")
        period = 1.0 / max(args.hz, 1.0)
        misses = 0
        while True:
            try:
                s = imu.read_raw()
                misses = 0
            except I2CGpioError as e:
                misses += 1
                print(f"I2C 瞬时失败 ({misses}): {e}", file=sys.stderr)
                imu.lib.i2c_gpio_recover()
                time.sleep(0.02)
                if misses == 5:
                    try:
                        print("连续失败，尝试重新初始化 IMU…", file=sys.stderr)
                        imu.init()
                    except I2CGpioError as e2:
                        print(f"重新初始化失败: {e2}", file=sys.stderr)
                if misses >= 30:
                    print(
                        "I2C 连续失败。晃动时掉线更像是焊点/地线抖动，"
                        "请检查 3.3V、GND、SDA、SCL 焊点。",
                        file=sys.stderr,
                    )
                    return 1
                continue
            ax, ay, az = s.acc_g
            gx, gy, gz = s.gyro_dps
            sat = any(abs(v) > 30000 for v in (s.ax, s.ay, s.az))
            print(
                f"acc(g)  {ax:7.3f} {ay:7.3f} {az:7.3f}   "
                f"gyro(dps) {gx:8.1f} {gy:8.1f} {gz:8.1f}   "
                f"raw a[{s.ax:6d} {s.ay:6d} {s.az:6d}] "
                f"g[{s.gx:6d} {s.gy:6d} {s.gz:6d}]"
                + ("  SAT" if sat else "")
            )
            time.sleep(period)
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        lib.i2c_gpio_close()
        os.close(lock_fd)


if __name__ == "__main__":
    sys.exit(main())
