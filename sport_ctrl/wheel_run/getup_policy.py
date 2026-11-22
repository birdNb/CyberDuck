#!/usr/bin/env python3
"""倒地起身。这份权重和行走策略不是同一套。

身体位置关节发位置。左右轮的速度输出算出来后屏蔽，不写到电机。
"""

from __future__ import annotations

import io
import math
import pickle
import time
import zipfile
from collections import OrderedDict
from pathlib import Path

import numpy as np

POLICY_DIR = Path(__file__).resolve().parents[1] / "policies" / "倒地起身"
TORQUE_NM = 2.0
# 这台脖子正角度越大越缩。缩着压不到地，起不来。
# 按 LB 后先收到 0° 伸直，到位后再只跟策略目标相对这一拍的增量。
NECK_CAN = 12
NECK_ARRIVE_DEG = 8.0
NECK_STRAIGHT_S = 1.5
# rsl_rl EmpiricalNormalization 的默认 eps。这份权重的 std 最小约 0.11，这项几乎不动结果。
NORM_EPS = 1e-2
WHEEL_ACTION_LIMIT = 2.0
WHEEL_VEL_SCALE = 10.0

# 位置关节，和 model_3250.pt 里的 joint_pos 顺序一致。
# (名字, 电机或舵机号, 是否舵机, 仿真限位 lo/hi, 默认角, latent_offset, SIGN)
# SIGN：仿真角 = SIGN × 电机弧度。编码器 0 对仿真 0，和行走标定在腿上的结果相同。
POS_JOINTS = (
    ("r_hip_yaw", 5, False, -0.436, 0.436, 0.0, 0.0, -1.0),
    ("r_hip_roll", 4, False, -0.384, 0.384, 0.0, 0.0, -1.0),
    ("r_hip_pitch", 3, False, -1.571, 1.571, 0.490, 0.3226523995399475, -1.0),
    ("r_calf", 2, False, -1.571, 1.571, -0.444, -0.2905299961566925, 1.0),
    ("l_hip_yaw", 10, False, -0.436, 0.436, 0.0, 0.0, -1.0),
    ("l_hip_roll", 9, False, -0.384, 0.384, 0.0, 0.0, 1.0),
    ("l_hip_pitch", 8, False, -1.571, 1.571, 0.490, 0.3226523995399475, 1.0),
    ("l_calf", 7, False, -1.571, 1.571, -0.444, -0.2905299961566925, -1.0),
    ("neck_pitch", 12, False, -1.571, 1.877, -0.360, -0.3068446218967438, 1.0),
    ("head_pitch", 11, False, -1.571, 1.571, 0.405, 0.26374778151512146, 1.0),
    ("head_yaw", 13, True, -2.967, 2.967, 0.0, 0.0, 1.0),
    ("head_roll", 14, True, -0.208, 0.208, 0.011, 0.05293400213122368, 1.0),
)
# 速度观测在 12 个位置关节之后再加右轮、左轮。这两维是速度输出。
WHEELS = (
    ("r_ankle", 1, -1.0),
    ("l_ankle", 6, 1.0),
)


def _artanh(x: float) -> float:
    x = max(-0.999999, min(0.999999, x))
    return 0.5 * math.log((1.0 + x) / (1.0 - x))


def _encode_pos(q_sim: float, lo: float, hi: float, latent: float) -> float:
    mid = 0.5 * (lo + hi)
    half = 0.5 * (hi - lo)
    if half < 1e-6:
        return 0.0
    return _artanh((q_sim - mid) / half) - latent


def _decode_pos(action: float, lo: float, hi: float, latent: float) -> float:
    mid = 0.5 * (lo + hi)
    half = 0.5 * (hi - lo)
    q = mid + half * math.tanh(action + latent)
    return min(hi, max(lo, q))


class _Blob(dict):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.args = args

    def __setstate__(self, state):
        self.state = state


class _Unpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if name == "OrderedDict":
            return OrderedDict
        if name == "dict":
            return dict

        def ctor(*args, **kwargs):
            blob = _Blob(*args, **kwargs)
            blob.module = module
            blob.name = name
            return blob

        return ctor

    def persistent_load(self, pid):
        return pid


def _find_pt() -> Path | None:
    if not POLICY_DIR.is_dir():
        return None
    found = sorted(POLICY_DIR.glob("*.pt"))
    return found[-1] if found else None


def _load_actor(path: Path) -> dict[str, np.ndarray]:
    """读 rsl_rl 的 pt，只取 actor。不依赖 torch。"""
    with zipfile.ZipFile(path) as zf:
        root = zf.namelist()[0].split("/")[0]
        obj = _Unpickler(io.BytesIO(zf.read(f"{root}/data.pkl"))).load()
        actor = obj["actor_state_dict"]
        out: dict[str, np.ndarray] = {}
        for key, blob in actor.items():
            storage = blob.args[0]
            shape = tuple(int(n) for n in blob.args[2])
            key_id = storage[2]
            numel = int(storage[4])
            payload = zf.read(f"{root}/data/{key_id}")
            # count 是 int64。推理只用 float 权重和归一化统计。
            if len(payload) != numel * 4:
                continue
            raw = np.frombuffer(payload, dtype="<f4").copy()
            out[key] = raw.reshape(shape)
        return out


def _elu(x: np.ndarray) -> np.ndarray:
    return np.where(x > 0.0, x, np.expm1(x))


class GetupPolicy:
    TORQUE_NM = TORQUE_NM

    def __init__(self) -> None:
        self.path = _find_pt()
        self.ready = False
        self.reason = ""
        self.active = False
        self.last_action = np.zeros(14, dtype=np.float64)
        self.q_prev: np.ndarray | None = None
        self.servo_deg = {"yaw": 0.0, "roll": 0.0}
        self.wheel_vel = {can: 0.0 for _name, can, _sign in WHEELS}
        self.neck_origin: float | None = None
        self.neck_straight_t = 0.0
        self._w: dict[str, np.ndarray] = {}
        self._load()

    def status_line(self) -> str:
        if self.ready and self.path is not None:
            return (
                f"倒地起身已加载 {self.path.name}。"
                f"按 LB 就跑，不看倒地；再按 LB 退出，回到默认姿态。"
                f"身体位置力矩 {TORQUE_NM:.0f} N·m，左右轮输出已屏蔽。"
            )
        return f"倒地起身没加载（{self.reason}）。"

    def _load(self) -> None:
        if self.path is None:
            self.reason = f"{POLICY_DIR} 里没有 pt"
            return
        try:
            self._w = _load_actor(self.path)
            weight = self._w["mlp.0.weight"]
            if weight.shape != (512, 46):
                self.reason = f"actor 输入是 {weight.shape[1]}，这份起身权重要 46"
                self._w = {}
                return
        except Exception as exc:
            self.reason = f"模型打不开（{exc}）"
            self._w = {}
            return
        self.ready = True

    def update(
        self,
        dt: float,
        gravity: tuple[float, float, float],
        gyro: tuple[float, float, float],
        bus,
        servo_deg: dict[str, float],
        arm: bool = False,
    ) -> dict[int, float] | None:
        """按 LB 进入，再按 LB 退出。不看倒地。轮子不在返回值里。"""
        if not self.ready:
            return None
        dt = max(0.001, min(0.05, dt))
        if self.active:
            if arm:
                self._stop()
                print("LB：退出起身策略，回到默认姿态。", flush=True)
                return None
            return self._step(dt, gravity, gyro, bus, servo_deg)
        if not arm:
            return None
        self.active = True
        self.last_action[:] = 0.0
        self.q_prev = None
        self.neck_origin = None
        self.neck_straight_t = 0.0
        print(
            f"LB：起身策略生效，不看倒地。轮子输出已屏蔽，身体力矩 {TORQUE_NM:.0f} N·m。"
            f"脖子先到 0° 伸直，到位后再跟位置增量。再按 LB 回到默认姿态。",
            flush=True,
        )
        return self._step(dt, gravity, gyro, bus, servo_deg)

    def _stop(self) -> None:
        self.active = False
        self.last_action[:] = 0.0
        self.q_prev = None
        self.servo_deg = {"yaw": 0.0, "roll": 0.0}
        self.wheel_vel = {can: 0.0 for _name, can, _sign in WHEELS}
        self.neck_origin = None
        self.neck_straight_t = 0.0

    def _motor_rad(self, bus, can: int) -> tuple[float, float]:
        motor = bus.motors.get(can)
        if motor is None:
            return 0.0, 0.0
        return float(motor.state.position), float(motor.state.velocity)

    def _q_sim(self, bus, servo_deg: dict[str, float]) -> np.ndarray:
        q = np.zeros(14, dtype=np.float64)
        for i, (_name, can, servo, _lo, _hi, _off, _lat, sign) in enumerate(POS_JOINTS):
            if servo:
                deg = float(servo_deg.get("yaw" if _name == "head_yaw" else "roll", 0.0))
                q[i] = sign * math.radians(deg)
            else:
                pos, _vel = self._motor_rad(bus, can)
                q[i] = sign * pos
        for j, (_name, can, sign) in enumerate(WHEELS):
            pos, _vel = self._motor_rad(bus, can)
            q[12 + j] = sign * pos
        return q

    def _q_dot(self, dt: float, q: np.ndarray, bus) -> np.ndarray:
        # 位置关节用仿真角速度。舵机没有速度反馈，用相邻两拍差分。
        qd = np.zeros(14, dtype=np.float64)
        if self.q_prev is None:
            self.q_prev = q.copy()
        for i, (_name, can, servo, _lo, _hi, _off, _lat, sign) in enumerate(POS_JOINTS):
            if servo:
                qd[i] = (q[i] - self.q_prev[i]) / dt
            else:
                _pos, vel = self._motor_rad(bus, can)
                qd[i] = sign * vel
        for j, (_name, can, sign) in enumerate(WHEELS):
            _pos, vel = self._motor_rad(bus, can)
            qd[12 + j] = sign * vel
        self.q_prev = q.copy()
        return qd

    def _step(self, dt, gravity, gyro, bus, servo_deg) -> dict[int, float] | None:
        q = self._q_sim(bus, servo_deg)
        qd = self._q_dot(dt, q, bus)
        obs = np.zeros(46, dtype=np.float64)
        obs[0:3] = gyro
        obs[3:6] = gravity
        for i, (_name, _can, _servo, lo, hi, _off, lat, _sign) in enumerate(POS_JOINTS):
            obs[6 + i] = _encode_pos(q[i], lo, hi, lat)
        obs[18:32] = qd
        obs[32:46] = self.last_action
        action = self._infer(obs)
        if action is None:
            return None
        action = action.astype(np.float64)
        action[12] = min(WHEEL_ACTION_LIMIT, max(-WHEEL_ACTION_LIMIT, float(action[12])))
        action[13] = min(WHEEL_ACTION_LIMIT, max(-WHEEL_ACTION_LIMIT, float(action[13])))
        self.last_action = action
        # 轮速维仍回灌到上一拍动作，但输出屏蔽，不写到 CAN 1 / 6。
        for _name, can, _sign in WHEELS:
            self.wheel_vel[can] = 0.0
        cmd: dict[int, float] = {}
        for i, (_name, can, servo, lo, hi, _off, lat, sign) in enumerate(POS_JOINTS):
            q_tgt = _decode_pos(float(action[i]), lo, hi, lat)
            deg = math.degrees(sign * q_tgt)
            if servo:
                key = "yaw" if _name == "head_yaw" else "roll"
                self.servo_deg[key] = deg
            else:
                cmd[can] = deg
        self._rebase_neck(dt, bus, cmd)
        return cmd

    def _neck_meas_deg(self, bus) -> float | None:
        motor = bus.motors.get(NECK_CAN)
        if motor is None or motor.state.t_pos <= 0.0:
            return None
        if time.monotonic() - motor.state.t_pos > 0.4:
            return None
        return math.degrees(float(motor.state.position))

    def _rebase_neck(self, dt: float, bus, cmd: dict[int, float]) -> None:
        """绝对角会把已经缩着的脖子继续往回收。先伸直到 0°，再只跟目标的变化。"""
        policy_deg = cmd.get(NECK_CAN)
        if policy_deg is None:
            return
        if self.neck_origin is None:
            self.neck_straight_t += dt
            meas = self._neck_meas_deg(bus)
            arrived = meas is not None and abs(meas) <= NECK_ARRIVE_DEG
            cmd[NECK_CAN] = 0.0
            if not arrived and self.neck_straight_t < NECK_STRAIGHT_S:
                return
            self.neck_origin = float(policy_deg)
            if arrived:
                print(
                    f"脖子已到 {meas:+.1f}°，从 0° 起按策略位置增量走。",
                    flush=True,
                )
            else:
                shown = "无反馈" if meas is None else f"{meas:+.1f}°"
                print(
                    f"脖子伸直到时（{shown}），从 0° 起按策略位置增量走。",
                    flush=True,
                )
            return
        lo, hi, sign = -1.571, 1.877, 1.0
        for _name, can, _servo, jlo, jhi, _off, _lat, jsign in POS_JOINTS:
            if can == NECK_CAN and not _servo:
                lo, hi, sign = jlo, jhi, jsign
                break
        ends = (math.degrees(sign * lo), math.degrees(sign * hi))
        delta = float(policy_deg) - self.neck_origin
        cmd[NECK_CAN] = min(max(ends), max(min(ends), delta))

    def _infer(self, obs: np.ndarray) -> np.ndarray | None:
        try:
            mean = self._w["obs_normalizer._mean"].reshape(-1)
            std = self._w["obs_normalizer._std"].reshape(-1)
            x = (obs - mean) / (std + NORM_EPS)
            x = _elu(x @ self._w["mlp.0.weight"].T + self._w["mlp.0.bias"])
            x = _elu(x @ self._w["mlp.2.weight"].T + self._w["mlp.2.bias"])
            x = _elu(x @ self._w["mlp.4.weight"].T + self._w["mlp.4.bias"])
            y = x @ self._w["mlp.6.weight"].T + self._w["mlp.6.bias"]
        except Exception as exc:
            print(f"起身策略推理失败：{exc}", flush=True)
            self.ready = False
            self.reason = str(exc)
            return None
        return np.asarray(y, dtype=np.float64).reshape(-1)[:14]
