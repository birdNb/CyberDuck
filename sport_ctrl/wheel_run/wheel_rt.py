#!/usr/bin/env python3
"""轮速和手柄单独进程。

主程序里的灯、头、嘴、声音、腿位置一旦堵住，原先会把最后一帧轮速留在电机上，
摇杆和 B 都不再被读到。这里的进程只做三件事：读手柄、算轮速、发两帧速度。
主程序只负责身体。它卡住时，松杆或按 B 仍然由本进程把轮子停住。

另一个看门进程盯着本进程的心跳。心跳断了就自己把轮速打成 0。
"""
from __future__ import annotations

import os
import struct
import time
from multiprocessing import Process, get_context
from multiprocessing.shared_memory import SharedMemory

SIZE = 256

# 轮速进程写
OFF_AXES = 0          # 8f
OFF_SEEN = 32         # 8B
OFF_BTN = 40          # 16B
OFF_EDGE = 56         # 5I  a b x y rb
OFF_FWD = 76          # 4f fwd turn spin height
OFF_GEAR = 92         # B
OFF_LT = 93
OFF_RT = 94
OFF_BOGUS = 95
OFF_XBOX = 96
OFF_ESTOP = 97        # 轮速进程置 1，主程序清 0
OFF_CMDL = 100        # f
OFF_CMDR = 104
OFF_RT_HB = 112       # d

# 主程序写
OFF_DRIVE = 128       # B
OFF_FORCE0 = 129
OFF_QUIT = 130
OFF_QUIET = 131       # 起立时主程序独占总线，轮速进程先别发

# 看门进程写
OFF_WD_HB = 136       # d


def _u8(buf, off: int) -> int:
    return int(buf[off])


def _set_u8(buf, off: int, val: int) -> None:
    buf[off] = int(val) & 0xFF


def _f32(buf, off: int) -> float:
    return struct.unpack_from("<f", buf, off)[0]


def _set_f32(buf, off: int, val: float) -> None:
    struct.pack_into("<f", buf, off, float(val))


def _f64(buf, off: int) -> float:
    return struct.unpack_from("<d", buf, off)[0]


def _set_f64(buf, off: int, val: float) -> None:
    struct.pack_into("<d", buf, off, float(val))


class PadView:
    """主程序看到的手柄。数据来自轮速进程，不自己占 /dev/input/js。"""

    LT_AXIS = 5

    def __init__(self, shm: SharedMemory) -> None:
        self._shm = shm
        self.axis = [0.0] * 8
        self.axis_seen = [False] * 8
        self.buttons = [False] * 16
        self.button_a = False
        self.button_b = False
        self.button_x = False
        self.button_y = False
        self.button_rb = False
        self.bogus = False
        self._bogus_warned = False
        self.xbox = False
        self._edge = {"a": 0, "b": 0, "x": 0, "y": 0, "rb": 0}
        self.fd = 1

    def poll(self) -> tuple[float, float, float, int, float, bool, bool]:
        buf = self._shm.buf
        self.axis = [struct.unpack_from("<f", buf, OFF_AXES + 4 * i)[0] for i in range(8)]
        self.axis_seen = [bool(_u8(buf, OFF_SEEN + i)) for i in range(8)]
        self.buttons = [bool(_u8(buf, OFF_BTN + i)) for i in range(16)]
        self.xbox = bool(_u8(buf, OFF_XBOX))
        self.bogus = bool(_u8(buf, OFF_BOGUS))
        edges = struct.unpack_from("<5I", buf, OFF_EDGE)
        for key, val, attr in (
            ("a", edges[0], "button_a"),
            ("b", edges[1], "button_b"),
            ("x", edges[2], "button_x"),
            ("y", edges[3], "button_y"),
            ("rb", edges[4], "button_rb"),
        ):
            if val != self._edge[key]:
                self._edge[key] = val
                setattr(self, attr, True)
        fwd, turn, spin, height = struct.unpack_from("<4f", buf, OFF_FWD)
        gear = _u8(buf, OFF_GEAR)
        lt = bool(_u8(buf, OFF_LT))
        rt = bool(_u8(buf, OFF_RT))
        return fwd, turn, spin, gear, height, lt, rt

    def lb_held(self) -> bool:
        """Xbox 的 LB 是 js 4。这只北通的 Y 占了 js 4，LB 在 js 6。"""
        idx = 4 if self.xbox else 6
        return idx < len(self.buttons) and bool(self.buttons[idx])

    def y_held(self) -> bool:
        idx = 3 if self.xbox else 4
        return idx < len(self.buttons) and self.buttons[idx]

    def rt_level(self) -> float:
        if self.buttons[9] or self.buttons[5]:
            return 1.0
        if not self.axis_seen[4]:
            return 0.0
        raw = self.axis[4]
        return max(0.0, min(1.0, (raw + 1.0) * 0.5))

    def close(self) -> None:
        return None


class WheelLink:
    def __init__(self, shm: SharedMemory, rt: Process, wd: Process) -> None:
        self.shm = shm
        self.rt = rt
        self.wd = wd
        self.view = PadView(shm)

    @classmethod
    def start(cls, cfg: dict) -> "WheelLink":
        ctx = get_context("spawn")
        shm = SharedMemory(create=True, size=SIZE, name=f"cyberduck-rt-{os.getpid()}")
        buf = shm.buf
        for i in range(SIZE):
            buf[i] = 0
        ppid = os.getpid()
        rt = ctx.Process(
            target=_rt_main, name="wheel-rt", args=(shm.name, ppid, cfg), daemon=False
        )
        wd = ctx.Process(
            target=_wd_main,
            name="wheel-wd",
            args=(shm.name, ppid, cfg["iface"], int(cfg["left_id"]), int(cfg["right_id"])),
            daemon=False,
        )
        rt.start()
        wd.start()
        return cls(shm, rt, wd)

    def set_drive(self, on: bool) -> None:
        _set_u8(self.shm.buf, OFF_DRIVE, 1 if on else 0)

    def set_force_zero(self, on: bool) -> None:
        _set_u8(self.shm.buf, OFF_FORCE0, 1 if on else 0)

    def set_quiet(self, on: bool) -> None:
        """起立过渡时不要发轮子。停止帧的回包会把腿的位置帧挤掉，动作就一顿一顿。"""
        _set_u8(self.shm.buf, OFF_QUIET, 1 if on else 0)

    def clear_estop(self) -> None:
        _set_u8(self.shm.buf, OFF_ESTOP, 0)

    def estop(self) -> bool:
        return bool(_u8(self.shm.buf, OFF_ESTOP))

    def commands(self) -> tuple[float, float]:
        buf = self.shm.buf
        return _f32(buf, OFF_CMDL), _f32(buf, OFF_CMDR)

    def close(self) -> None:
        try:
            _set_u8(self.shm.buf, OFF_DRIVE, 0)
            _set_u8(self.shm.buf, OFF_QUIT, 1)
        except Exception:
            pass
        for proc in (self.rt, self.wd):
            proc.join(timeout=1.2)
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=0.5)
        try:
            self.shm.close()
            self.shm.unlink()
        except FileNotFoundError:
            pass
        except Exception:
            pass


def _retry(send) -> None:
    for _ in range(6):
        try:
            send()
            return
        except OSError:
            time.sleep(0.001)


def _send_pair(bus, motors, left_id: int, right_id: int, left: float, right: float, sign) -> None:
    from wheel_speed import send_speed

    for mid, spd in ((left_id, left), (right_id, right)):
        _retry(lambda mid=mid, spd=spd: send_speed(bus, motors[mid], spd * sign(mid), 0.0, reply=False))


def _cut_wheels(bus, left_id: int, right_id: int) -> None:
    """速度 0 丢一帧，电机会一直保持上一档转速。松杆和停车直接断力矩。"""
    from ht_can import MODE_STOP, QUERY_STATUS

    frame = bytes([MODE_STOP, QUERY_STATUS])
    for mid in (left_id, right_id):
        _retry(lambda mid=mid: bus.send_cmd(mid, frame, reply=False))


def _rt_main(shm_name: str, ppid: int, cfg: dict) -> None:
    from wheel_drive import Gamepad, lowpass, ramp, wheel_sign, wheel_targets

    shm = SharedMemory(name=shm_name)
    buf = shm.buf
    try:
        os.nice(-5)
    except OSError:
        pass
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(15))
    except (AttributeError, OSError, PermissionError):
        pass
    from ht_can import Motor, MotorBus

    iface = str(cfg["iface"])
    left_id = int(cfg["left_id"])
    right_id = int(cfg["right_id"])
    bus = MotorBus(iface)
    motors = {
        left_id: bus.add_motor(Motor(left_id)),
        right_id: bus.add_motor(Motor(right_id)),
    }
    pad = Gamepad()
    edges = [0, 0, 0, 0, 0]
    cmd_l = 0.0
    cmd_r = 0.0
    spd_l = 0.0
    spd_r = 0.0
    prev = time.monotonic()
    prev_drive = False
    arm_t = 0.0
    was_sending = False
    period = float(cfg["period"])
    arm_s = float(cfg["arm_s"])
    zero_burst = 0
    print("轮速进程已起：只读手柄、只发两轮。主程序堵住时摇杆和 B 仍然有效。", flush=True)
    try:
        while _u8(buf, OFF_QUIT) == 0:
            try:
                bus.pump(limit=16)
            except Exception:
                pass
            now = time.monotonic()
            dt = max(0.001, min(0.05, now - prev))
            prev = now
            if os.getppid() != ppid:
                break
            fwd, turn, spin, gear, height, lt, rt = pad.poll()
            bogus = Gamepad.sticks_look_bogus(fwd, turn, spin)
            if pad.button_b or (pad.fd >= 0 and len(pad.buttons) > 1 and pad.buttons[1]):
                _set_u8(buf, OFF_ESTOP, 1)
            names = (
                (pad.button_a, 0),
                (pad.button_b, 1),
                (pad.button_x, 2),
                (pad.button_y, 3),
                (pad.button_rb, 4),
            )
            for flag, idx in names:
                if flag and not (bogus and idx != 1):
                    edges[idx] = (edges[idx] + 1) & 0xFFFFFFFF
            pad.button_a = False
            pad.button_b = False
            pad.button_x = False
            pad.button_y = False
            pad.button_rb = False
            if bogus:
                fwd = turn = spin = height = 0.0
                gear = 0
                lt = False
                rt = False
            for i, val in enumerate(pad.axis):
                _set_f32(buf, OFF_AXES + 4 * i, val)
                _set_u8(buf, OFF_SEEN + i, 1 if pad.axis_seen[i] else 0)
            for i, val in enumerate(pad.buttons):
                _set_u8(buf, OFF_BTN + i, 1 if val else 0)
            struct.pack_into("<5I", buf, OFF_EDGE, *edges)
            struct.pack_into("<4f", buf, OFF_FWD, fwd, turn, spin, height)
            _set_u8(buf, OFF_GEAR, int(gear))
            _set_u8(buf, OFF_LT, 1 if lt else 0)
            _set_u8(buf, OFF_RT, 1 if rt else 0)
            _set_u8(buf, OFF_BOGUS, 1 if bogus else 0)
            _set_u8(buf, OFF_XBOX, 1 if pad.xbox else 0)

            drive = _u8(buf, OFF_DRIVE) == 1
            force0 = _u8(buf, OFF_FORCE0) == 1
            estop = _u8(buf, OFF_ESTOP) == 1
            if drive and not prev_drive:
                arm_t = now
            prev_drive = drive
            vmax = float(cfg["vmax_fast"] if gear else cfg["vmax"])
            tgt_l, tgt_r = wheel_targets(
                fwd, turn, spin, vmax, float(cfg["turn"]), float(cfg["spin"])
            )
            if (not drive) or estop or force0 or (now - arm_t) < arm_s or pad.fd < 0:
                tgt_l = tgt_r = 0.0
            idle = abs(tgt_l) < 1e-6 and abs(tgt_r) < 1e-6
            # 摇杆回中就立刻清掉斜坡。否则松杆后滤波还在往前送，轮子停不下来。
            if idle or not drive or estop or force0 or pad.fd < 0 or (now - arm_t) < arm_s:
                cmd_l = cmd_r = spd_l = spd_r = 0.0
            else:
                cmd_l = ramp(cmd_l, tgt_l, float(cfg["accel"]), dt)
                cmd_r = ramp(cmd_r, tgt_r, float(cfg["accel"]), dt)
                spd_l = lowpass(spd_l, cmd_l, float(cfg["smooth"]), dt)
                spd_r = lowpass(spd_r, cmd_r, float(cfg["smooth"]), dt)
            _set_f32(buf, OFF_CMDL, spd_l)
            _set_f32(buf, OFF_CMDR, spd_r)
            quiet = _u8(buf, OFF_QUIET) == 1
            # 驾驶中松杆：每拍都发速度 0，电机才会从上一档转速刹住。
            # 断力矩会让轮子靠惯性空转。只有退出驾驶、按 B、手柄掉线才断力矩。
            # 起立时主程序在发腿的位置，这里停发，避免停止帧回包把动作挤断。按 B 仍断力矩。
            if quiet and not estop:
                zero_burst = 0
            elif drive and not estop and not force0 and pad.fd >= 0:
                _send_pair(bus, motors, left_id, right_id, spd_l, spd_r, wheel_sign)
                # 刚松杆时连发几帧速度 0。丢一帧电机就会保持上一档转速，要转好几秒才停。
                if abs(spd_l) < 1e-6 and abs(spd_r) < 1e-6:
                    if zero_burst < 4:
                        zero_burst += 1
                        _send_pair(bus, motors, left_id, right_id, 0.0, 0.0, wheel_sign)
                else:
                    zero_burst = 0
            else:
                zero_burst = 0
                _cut_wheels(bus, left_id, right_id)
            _set_f64(buf, OFF_RT_HB, time.monotonic())
            spent = time.monotonic() - now
            if spent < period:
                time.sleep(period - spent)
        _cut_wheels(bus, left_id, right_id)
    finally:
        try:
            bus.close()
        except Exception:
            pass
        shm.close()


def _wd_main(shm_name: str, ppid: int, iface: str, left_id: int, right_id: int) -> None:
    from ht_can import Motor, MotorBus

    shm = SharedMemory(name=shm_name)
    buf = shm.buf
    bus = None
    motors = None
    try:
        while _u8(buf, OFF_QUIT) == 0:
            time.sleep(0.05)
            if os.getppid() != ppid:
                break
            hb = _f64(buf, OFF_RT_HB)
            drive = _u8(buf, OFF_DRIVE) == 1
            age = time.monotonic() - hb if hb > 0 else 0.0
            _set_f64(buf, OFF_WD_HB, time.monotonic())
            if hb <= 0 or age < 0.25:
                continue
            if not drive and age < 1.0:
                continue
            if bus is None:
                bus = MotorBus(iface)
                motors = {
                    left_id: bus.add_motor(Motor(left_id)),
                    right_id: bus.add_motor(Motor(right_id)),
                }
            print(f"轮速进程心跳断了 {age * 1000:.0f}ms，看门切断轮子力矩。", flush=True)
            for _ in range(8):
                if _u8(buf, OFF_QUIT):
                    break
                _cut_wheels(bus, left_id, right_id)
                time.sleep(0.02)
            time.sleep(0.4)
    finally:
        if bus is not None:
            try:
                if motors is not None:
                    _cut_wheels(bus, left_id, right_id)
                bus.close()
            except Exception:
                pass
        shm.close()
