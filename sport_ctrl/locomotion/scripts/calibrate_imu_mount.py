#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 LSM6 芯片轴对准仿真 base_link，并核对方向是否一致。

约定见 locomotion/htdk_sim2real.md 第 2.1 节和第 6 节。

  R @ v_sensor = v_body
  body：+X 朝前，+Y 朝左，+Z 朝上
  直立静止 projected_gravity ≈ (0, 0, -1)
  右肩向下 wx>0，低头 wy>0，俯视逆时针 wz>0

本脚本读芯片原始轴，不用 IMU 驱动里写死的安装旋转
（sensor → (z, y, -x)，来自官方 MicroDuck）。那只是待核对的猜测。
六项里有一项失败就不写 imu_mount.json。

在 locomotion/ 目录下：

  python3 scripts/calibrate_imu_mount.py --self-test
  python3 scripts/calibrate_imu_mount.py --snapshot
  python3 scripts/calibrate_imu_mount.py --live
  python3 scripts/calibrate_imu_mount.py --lsm6 --out config/imu_mount.json
  python3 scripts/calibrate_imu_mount.py --csv imu_log.csv --out config/imu_mount.json
  python3 scripts/calibrate_imu_mount.py --serial /dev/ttyUSB0 --baud 115200 --out config/imu_mount.json
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import dataclass

G = 9.80665
POSES = (
    ("upright", "static", "站直放稳。力矩可以是 0，人扶住躯干。"),
    ("nose_down", "static", "前倾大约 20° 到 40° 后稳住。只低头，不要侧倾。"),
    ("left_down", "static", "机器人自己的左侧降低大约 20° 到 40°，稳住。"),
    ("roll_right_down", "gyro", "缓慢地把右肩往下转。整段采集期间都在转，不要停。"),
    ("pitch_nose_down", "gyro", "缓慢低头。整段采集期间都在转，不要停。"),
    ("yaw_ccw", "gyro", "从上往下看，缓慢逆时针转。整段采集期间都在转，不要停。"),
)

# 当前 IMU/cpp 与 IMU/lsm6dsv16x.py 写死的安装旋转：(x, y, z) → (z, y, -x)
DRIVER_R = (
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0),
    (-1.0, 0.0, 0.0),
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOCO_DIR = os.path.dirname(SCRIPT_DIR)
REPO_DIR = os.path.dirname(LOCO_DIR)
DEFAULT_OUT = os.path.join(LOCO_DIR, "config", "imu_mount.json")


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a):
    return math.sqrt(dot(a, a))


def normalize(a):
    n = norm(a)
    if n < 1e-9:
        raise ValueError("向量长度几乎为 0")
    return scale(a, 1.0 / n)


def matvec(m, v):
    return tuple(dot(m[i], v) for i in range(3))


def matmul(a, b):
    bt = transpose(b)
    return tuple(tuple(dot(a[i], bt[j]) for j in range(3)) for i in range(3))


def transpose(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def det(m):
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def rotation_angle_deg(a, b):
    m = matmul(transpose(a), b)
    c = (m[0][0] + m[1][1] + m[2][2] - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def ry(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def mean(rows):
    if not rows:
        raise ValueError("没有样本")
    acc = (0.0, 0.0, 0.0)
    for row in rows:
        acc = add(acc, row)
    return scale(acc, 1.0 / len(rows))


def fmt(v):
    return f"{v[0]:+7.3f} {v[1]:+7.3f} {v[2]:+7.3f}"


def projected_gravity(r, accel):
    """静止时 projected_gravity = -normalize(R @ accel)。"""
    return normalize(scale(matvec(r, accel), -1.0))


def angle_to_upright(grav):
    c = max(-1.0, min(1.0, dot(grav, (0.0, 0.0, -1.0))))
    return math.degrees(math.acos(c))


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


def solve_mount(acc_up, acc_nose):
    """用直立和前倾两段加速度求 R。左侧降低只用来验收，不参与拟合。

    直立比力对准 body +Z。前倾比力落在 body 的 XZ 平面、X 分量为负，
    所以 normalize(a_nose × z) 对准 body +Y。角度未知也够用。
    """
    z_s = normalize(acc_up)
    y_raw = cross(normalize(acc_nose), z_s)
    if norm(y_raw) < 0.20:
        raise ValueError("前倾和直立的加速度几乎平行，倾角不够，请重采 nose_down")
    y_s = normalize(y_raw)
    x_s = cross(y_s, z_s)
    # 列是 body 轴在传感器坐标里的方向，R @ 这些列 = I
    sensor_axes = (
        (x_s[0], y_s[0], z_s[0]),
        (x_s[1], y_s[1], z_s[1]),
        (x_s[2], y_s[2], z_s[2]),
    )
    mount = transpose(sensor_axes)
    if det(mount) < 0.5:
        raise ValueError("求出的 R 不是右手旋转，请重采，不要手工翻某一轴")
    return mount


def _gyro_bias(gyro):
    return mean([gyro["upright"], gyro["nose_down"], gyro["left_down"]])


def evaluate(mount, acc, gyro):
    checks = []
    grav_up = projected_gravity(mount, acc["upright"])
    up_deg = angle_to_upright(grav_up)
    checks.append(
        Check(
            "直立重力",
            up_deg < 8.0,
            f"gravity {fmt(grav_up)}  与 (0,0,-1) 夹角 {up_deg:.1f}°（要通过 < 8°）",
        )
    )

    def tilt_check(name, grav, axis, positive_hint):
        tilt = angle_to_upright(grav)
        main = grav[axis]
        leak = abs(grav[1 - axis])  # 0↔1，2 不走这里
        ok = main > 0.25 and leak < min(0.30, 0.5 * main) and 12.0 <= tilt <= 55.0
        checks.append(
            Check(
                name,
                ok,
                f"gravity {fmt(grav)}  倾角 {tilt:.1f}°  {positive_hint}",
            )
        )

    tilt_check("前倾", projected_gravity(mount, acc["nose_down"]), 0, "期望 gx>0，几乎没有 gy")
    tilt_check("左侧降低", projected_gravity(mount, acc["left_down"]), 1, "期望 gy>0，几乎没有 gx")

    bias = _gyro_bias(gyro)
    axes = (
        ("右肩向下", "roll_right_down", 0, "wx"),
        ("低头", "pitch_nose_down", 1, "wy"),
        ("俯视逆时针", "yaw_ccw", 2, "wz"),
    )
    body_rates = {}
    for name, key, axis, label in axes:
        omega = matvec(mount, sub(gyro[key], bias))
        body_rates[key] = omega
        others = max(abs(omega[i]) for i in range(3) if i != axis)
        ok = omega[axis] > 0.08 and omega[axis] > others
        checks.append(
            Check(
                name,
                ok,
                f"角速度 {fmt(omega)} rad/s  期望 {label}>0 且是最大分量",
            )
        )
    return checks, grav_up, up_deg, bias, body_rates


def looks_like_deg_per_sec(gyro):
    for key in ("roll_right_down", "pitch_nose_down", "yaw_ccw"):
        if norm(gyro[key]) > 6.0:
            return True
    return False


def accel_unit_scale(acc_up):
    n = norm(acc_up)
    if 0.6 <= n <= 1.6:
        return G, "g"
    if 6.0 <= n <= 16.0:
        return 1.0, "m/s^2"
    raise ValueError(f"直立加速度模长 {n:.3f}，既不像 g 也不像 m/s^2")


def means_from_groups(groups, gyro_unit):
    acc, gyro = {}, {}
    gyro_scale = math.pi / 180.0 if gyro_unit == "deg" else 1.0
    for name, _kind, _hint in POSES:
        rows = groups.get(name) or []
        if len(rows) < 5:
            raise ValueError(f"{name} 样本太少（{len(rows)}）")
        acc[name] = mean([row[:3] for row in rows])
        gyro[name] = scale(mean([row[3:] for row in rows]), gyro_scale)
    scale_acc, unit = accel_unit_scale(acc["upright"])
    if scale_acc != 1.0:
        acc = {k: scale(v, scale_acc) for k, v in acc.items()}
    return acc, gyro, unit


def calibrate(groups, gyro_unit, mount=None):
    """mount 为空时由加速度求 R。传入 mount 时只验收，不改旋转。"""
    acc, gyro, unit = means_from_groups(groups, gyro_unit)
    if looks_like_deg_per_sec(gyro) and gyro_unit == "rad":
        raise ValueError("陀螺仪数值像 deg/s。串口或 CSV 请加 --gyro-unit deg")
    solved = mount is None
    if mount is None:
        mount = solve_mount(acc["upright"], acc["nose_down"])
    checks, grav_up, up_deg, bias, rates = evaluate(mount, acc, gyro)
    return {
        "solved": solved,
        "R": mount,
        "checks": checks,
        "gravity_upright": grav_up,
        "upright_angle_deg": up_deg,
        "accel_unit": unit,
        "gyro_bias_sensor": bias,
        "body_rates": rates,
        "driver_mount_angle_deg": rotation_angle_deg(mount, DRIVER_R),
    }


def all_ok(result):
    return all(item.ok for item in result["checks"])


def print_result(result):
    print()
    for item in result["checks"]:
        mark = "通过" if item.ok else "失败"
        print(f"[{mark}] {item.name}  {item.detail}")
    angle = result["driver_mount_angle_deg"]
    print(f"加速度单位按 {result['accel_unit']} 识别")
    print(f"与当前驱动写死的安装旋转相差 {angle:.1f}°")
    if angle < 8.0:
        print("这次的 R 和驱动里的 [z, y, -x] 接近。")
    else:
        print("驱动里的 [z, y, -x] 不能当成 base_link。部署要用标定得到的 R。")
    if result["solved"]:
        print("R 由直立和前倾求出，左侧降低和三轴慢转只做验收。")
    else:
        print("这次没有重新求 R，只验收给定的安装旋转。")


def save_mount(path, result):
    payload = {
        "R": [[round(v, 8) for v in row] for row in result["R"]],
        "convention": "R @ v_sensor = v_base_link; +X forward, +Y left, +Z up",
        "upright_gravity": [round(v, 6) for v in result["gravity_upright"]],
        "upright_angle_deg": round(result["upright_angle_deg"], 3),
        "driver_mount_angle_deg": round(result["driver_mount_angle_deg"], 3),
        "checks": {item.name: item.ok for item in result["checks"]},
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")


def finish_calibration(groups, gyro_unit, out_path, mount=None):
    try:
        result = calibrate(groups, gyro_unit, mount=mount)
    except ValueError as exc:
        print(f"校准失败: {exc}", file=sys.stderr)
        print("不写 imu_mount.json。", file=sys.stderr)
        return 1
    print_result(result)
    if not all_ok(result):
        print("有一项未通过，不写 imu_mount.json。请整段重采，不要手工翻转某一列。")
        return 1
    save_mount(out_path, result)
    print(f"六项通过，已写入 {out_path}")
    print("角速度三项和直立重力都对上之前，不要闭合策略，也不要把力矩上限调过 0.5 N·m。")
    return 0


def load_csv(path):
    groups = {name: [] for name, _kind, _hint in POSES}
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        need = ["pose", "ax", "ay", "az", "gx", "gy", "gz"]
        missing = [name for name in need if name not in fields]
        if missing:
            raise ValueError("CSV 需要列 " + ",".join(need))
        for row in reader:
            pose = (row.get("pose") or "").strip()
            if pose not in groups:
                raise ValueError(f"未知 pose: {pose}")
            groups[pose].append(tuple(float(row[k]) for k in need[1:]))
    absent = [name for name, rows in groups.items() if not rows]
    if absent:
        raise ValueError("CSV 缺少姿态: " + ", ".join(absent))
    return groups


def collect_interactive(read_window, seconds):
    print("共六段。静止的三段放稳再按回车；慢转的三段按回车后立刻开始转，并保持转到采集结束。")
    print("失败会拒绝保存。不要为了凑符号去翻某一列数据。")
    groups = {}
    for name, kind, hint in POSES:
        print()
        print(f"下一段: {name}  ({'静止' if kind == 'static' else '慢转'})")
        print(hint)
        input("准备好后按回车开始… ")
        if kind == "gyro":
            print("开始转。")
        rows = read_window(seconds)
        groups[name] = rows
        acc = mean([row[:3] for row in rows]) if rows else (0.0, 0.0, 0.0)
        rate = mean([row[3:] for row in rows]) if rows else (0.0, 0.0, 0.0)
        print(f"  {len(rows)} 个样本  acc {fmt(acc)}  gyro {fmt(rate)}")
    return groups


def parse_six(line):
    parts = line.replace(",", " ").split()
    if len(parts) < 6:
        return None
    try:
        return tuple(float(parts[i]) for i in range(6))
    except ValueError:
        return None


class SerialWindow:
    def __init__(self, path, baud):
        import termios

        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        speed = getattr(termios, f"B{baud}", None)
        if speed is None:
            raise ValueError(f"不支持波特率 {baud}")
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[3] = 0
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self.buf = b""

    def close(self):
        os.close(self.fd)

    def read_window(self, seconds):
        rows = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                chunk = os.read(self.fd, 256)
            except OSError:
                chunk = b""
            if chunk:
                self.buf += chunk
                while b"\n" in self.buf:
                    line, self.buf = self.buf.split(b"\n", 1)
                    sample = parse_six(line.decode("utf-8", "replace"))
                    if sample is not None:
                        rows.append(sample)
            else:
                time.sleep(0.002)
        return rows


class Lsm6Window:
    """读芯片原始轴。加速度 m/s^2，角速度 rad/s。不套驱动里的安装旋转。"""

    def __init__(self, pullup):
        sys.path.insert(0, os.path.join(REPO_DIR, "IMU"))
        import fcntl

        from lsm6dsv16x import (
            ADDRS,
            GYRO_LSB_RAD,
            WHO_AM_I_VAL,
            I2CGpioError,
            LSM6DSV16X,
            load_lib,
            open_bus,
            scan_bus,
        )

        self._I2CGpioError = I2CGpioError
        self._gyro_lsb = GYRO_LSB_RAD
        self.lock_fd = os.open("/tmp/microduck-imu-i2c6.lock", os.O_CREAT | os.O_RDWR, 0o644)
        try:
            fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(self.lock_fd)
            self.lock_fd = None
            raise I2CGpioError("GPIO 24/25 已被占用，先停掉 test_imu.py 或 duck_locomotion")
        os.write(self.lock_fd, f"{os.getpid()}\n".encode())
        self.lib = None
        self.imu = None
        try:
            self.lib = load_lib()
            open_bus(self.lib, pullup=pullup)
            found = scan_bus(self.lib)
            addrs = [a for a in found if a in ADDRS] or list(ADDRS)
            last = "未找到 LSM6DSV16X"
            for addr in addrs:
                candidate = LSM6DSV16X(self.lib, addr)
                try:
                    who = candidate.who_am_i()
                except I2CGpioError as exc:
                    last = str(exc)
                    continue
                if who == WHO_AM_I_VAL:
                    candidate.init()
                    self.imu = candidate
                    print(f"LSM6DSV16X @ 0x{addr:02X}  原始芯片轴，尚未乘安装旋转")
                    break
                last = f"0x{addr:02X} WHO_AM_I=0x{who:02X}"
            if self.imu is None:
                raise I2CGpioError(last)
        except Exception:
            self.close()
            raise

    def close(self):
        if getattr(self, "lib", None) is not None:
            self.lib.i2c_gpio_close()
        if getattr(self, "lock_fd", None) is not None:
            os.close(self.lock_fd)
            self.lock_fd = None

    def read_one(self):
        sample = self.imu.read_raw()
        ax, ay, az = sample.acc_g
        acc = (ax * G, ay * G, az * G)
        gyro = (
            sample.gx * self._gyro_lsb,
            sample.gy * self._gyro_lsb,
            sample.gz * self._gyro_lsb,
        )
        return acc + gyro

    def read_window(self, seconds):
        rows = []
        misses = 0
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                rows.append(self.read_one())
                misses = 0
            except self._I2CGpioError:
                misses += 1
                self.lib.i2c_gpio_recover()
                time.sleep(0.01)
                if misses >= 30:
                    raise
        return rows


def load_mount_arg(text):
    if text is None or text == "driver":
        return DRIVER_R, "当前驱动写死的 [z, y, -x]"
    with open(text, encoding="utf-8") as f:
        payload = json.load(f)
    rows = payload["R"]
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError("imu_mount.json 里的 R 不是 3x3")
    mount = tuple(tuple(float(v) for v in row) for row in rows)
    return mount, text


def print_live_line(mount, sample):
    acc, gyro = sample[:3], sample[3:]
    try:
        grav = projected_gravity(mount, acc)
        up = angle_to_upright(grav)
        grav_txt = f"{fmt(grav)}  夹角 {up:5.1f}°"
    except ValueError:
        grav_txt = "加速度无效"
    body_gyro = matvec(mount, gyro)
    print(f"gyro {fmt(body_gyro)}   gravity {grav_txt}", flush=True)


def run_snapshot(source, mount, label):
    print(f"采集 1 秒。映射: {label}")
    print("这一步只看当前姿态的重力。机器人要先放直。三轴慢转还没做，不能据此闭合策略。")
    rows = source.read_window(1.0)
    acc = mean([row[:3] for row in rows])
    gyro = mean([row[3:] for row in rows])
    print(f"样本 {len(rows)}")
    print(f"芯片加速度 {fmt(acc)} m/s^2   |a|={norm(acc):.2f}")
    print(f"芯片角速度 {fmt(gyro)} rad/s")
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    angles = {}
    for title, candidate in (("不旋转", identity), (label, mount)):
        grav = projected_gravity(candidate, acc)
        up = angle_to_upright(grav)
        angles[title] = up
        mark = "直立重力接近仿真" if up < 8.0 else "直立重力与仿真不一致"
        print(f"{title}: gravity {fmt(grav)}  夹角 {up:.1f}°  {mark}")
    raw_ok = angles["不旋转"] < 8.0
    mount_ok = angles[label] < 8.0
    if mount_ok:
        print("这份映射的直立重力对上了。还要用 --live 看右肩向下、低头、逆时针，或跑完整六段采集。")
        return 0
    if raw_ok and label.startswith("当前驱动"):
        print("芯片轴的竖直方向已经接近 base_link。驱动里多乘的 [z, y, -x] 把重力转到了水平方向，不要用它。")
        print("前后和左右、以及三轴角速度还没核对。请做六段采集，不要闭合策略。")
        return 1
    print("当前映射下直立重力未对齐。不要闭合策略。先确认机器人放直，再做六段采集。")
    return 1


def run_live(source, mount, label):
    print(f"实时核对。映射: {label}")
    print("直立静止 gravity 应接近 0 0 -1。右肩向下 wx>0，低头 wy>0，俯视逆时针 wz>0。")
    print("Ctrl+C 退出。")
    period = 0.1
    while True:
        t0 = time.monotonic()
        try:
            sample = source.read_one()
        except Exception as exc:
            print(f"读取失败: {exc}", file=sys.stderr)
            time.sleep(0.05)
            continue
        print_live_line(mount, sample)
        spent = time.monotonic() - t0
        if spent < period:
            time.sleep(period - spent)


def self_test():
    def body_to_sensor(mount, vector):
        return matvec(transpose(mount), vector)

    def synthesize(mount, gyro_bias=(0.02, -0.01, 0.015)):
        nose = (-math.sin(math.radians(30.0)), 0.0, math.cos(math.radians(30.0)))
        left = (0.0, -math.sin(math.radians(25.0)), math.cos(math.radians(25.0)))
        acc_body = {
            "upright": (0.0, 0.0, G),
            "nose_down": scale(nose, G),
            "left_down": scale(left, G),
        }
        rates = {
            "roll_right_down": (0.60, 0.02, -0.03),
            "pitch_nose_down": (0.01, 0.55, 0.04),
            "yaw_ccw": (-0.02, 0.03, 0.70),
        }
        groups = {}
        for name, vector in acc_body.items():
            sensor_acc = body_to_sensor(mount, vector)
            row = sensor_acc + gyro_bias
            groups[name] = [row] * 20
        upright_acc = body_to_sensor(mount, acc_body["upright"])
        for name, rate in rates.items():
            sensor_rate = add(body_to_sensor(mount, rate), gyro_bias)
            groups[name] = [upright_acc + sensor_rate] * 20
        return groups

    mounts = {
        "单位阵": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        "驱动安装": DRIVER_R,
        "组合旋转": matmul(rz(20.0), matmul(ry(-15.0), rx(10.0))),
    }
    for label, mount in mounts.items():
        result = calibrate(synthesize(mount), "rad")
        err = rotation_angle_deg(result["R"], mount)
        if err > 0.05 or not all_ok(result):
            print(f"self-test 失败: {label} 恢复误差 {err:.3f}°")
            print_result(result)
            return 1
        print(f"self-test: {label} 恢复误差 {err:.4f}°")

    flipped = synthesize(DRIVER_R)
    bias = (0.02, -0.01, 0.015)
    flipped_rows = []
    for row in flipped["roll_right_down"]:
        rate_body = matvec(DRIVER_R, sub(row[3:], bias))
        rate_body = (-rate_body[0], rate_body[1], rate_body[2])
        flipped_rows.append(row[:3] + add(matvec(transpose(DRIVER_R), rate_body), bias))
    flipped["roll_right_down"] = flipped_rows
    rejected = calibrate(flipped, "rad")
    roll = next(item for item in rejected["checks"] if item.name == "右肩向下")
    if roll.ok or all_ok(rejected):
        print("self-test 失败: 右肩向下反号时仍被接受")
        print_result(rejected)
        return 1
    print("self-test: 右肩向下反号已拒绝，未当成通过")

    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    mismatched = calibrate(synthesize(DRIVER_R), "rad", mount=identity)
    if all_ok(mismatched):
        print("self-test 失败: 装反的 R 仍被接受")
        return 1
    print("self-test: 与数据不符的安装旋转已拒绝")
    print("self-test 通过")
    return 0


def main():
    parser = argparse.ArgumentParser(description="核对 IMU 轴是否与仿真 base_link 一致，并求安装旋转 R")
    parser.add_argument("--self-test", action="store_true", help="不接硬件，用合成数据检查求 R 和验收")
    parser.add_argument("--snapshot", action="store_true", help="读 1 秒，只看当前姿态的直立重力")
    parser.add_argument("--live", action="store_true", help="连续打印机体系角速度和重力，人眼看三项慢转")
    parser.add_argument("--lsm6", action="store_true", help="从本机 LSM6 交互采集六段")
    parser.add_argument("--serial", help="串口，每行 ax ay az gx gy gz")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--csv", help="列名 pose,ax,ay,az,gx,gy,gz")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--gyro-unit", choices=("rad", "deg"), default="rad")
    parser.add_argument("--out", default=DEFAULT_OUT)
    parser.add_argument(
        "--mount",
        help="live/snapshot 使用的 R。driver 或 imu_mount.json 路径。默认 driver",
    )
    parser.add_argument(
        "--verify",
        help="六段采集后不重新求 R，只验收这份旋转。driver 或 json 路径",
    )
    parser.add_argument("--pullup", action="store_true", help="打开 GPIO 内部上拉，需要 root")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    modes = sum(bool(x) for x in (args.snapshot, args.live, args.lsm6, args.serial, args.csv))
    if modes != 1:
        parser.print_help()
        print("\n请只选一种：--self-test、--snapshot、--live、--lsm6、--serial、--csv", file=sys.stderr)
        return 2

    if args.csv:
        try:
            groups = load_csv(args.csv)
        except (OSError, ValueError) as exc:
            print(f"读取 CSV 失败: {exc}", file=sys.stderr)
            return 1
        mount = None
        if args.verify:
            mount, _label = load_mount_arg(args.verify)
        return finish_calibration(groups, args.gyro_unit, args.out, mount=mount)

    if args.pullup and os.geteuid() != 0:
        print("打开内部上拉需要 root，正在用 sudo 重新启动…")
        os.execvp("sudo", ["sudo", "-E", sys.executable, os.path.abspath(__file__), *sys.argv[1:]])

    source = None
    try:
        if args.serial:
            source = SerialWindow(args.serial, args.baud)
            groups = collect_interactive(source.read_window, args.seconds)
            mount = None
            if args.verify:
                mount, _label = load_mount_arg(args.verify)
            return finish_calibration(groups, args.gyro_unit, args.out, mount=mount)

        source = Lsm6Window(pullup=args.pullup)
        if args.snapshot or args.live:
            mount, label = load_mount_arg(args.mount)
            if args.snapshot:
                return run_snapshot(source, mount, label)
            run_live(source, mount, label)
            return 0
        groups = collect_interactive(source.read_window, args.seconds)
        mount = None
        if args.verify:
            mount, _label = load_mount_arg(args.verify)
        return finish_calibration(groups, "rad", args.out, mount=mount)
    except KeyboardInterrupt:
        print()
        return 0
    except Exception as exc:
        print(f"IMU 失败: {exc}", file=sys.stderr)
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    sys.exit(main())
