#!/usr/bin/env python3
"""前左翻滚策略。权重来自 sport_ctrl/policies/cyberduck_v2_roulade_forward_left。

观测 48 维：gyro3 + gravity3 + joint_pos12 + joint_vel14 + last_action14 + phase2。
动作 14 维：12 路位置 + 2 路踝速度（策略踝速不写电机；翻滚时手柄轮行仍可用）。
按 A 进入，再按 A 或满 EPISODE_S 退出。
"""

from __future__ import annotations

import io
import math
import pickle
import zipfile
from collections import OrderedDict
from pathlib import Path

import numpy as np

POLICY_DIR = Path(__file__).resolve().parents[1] / "policies" / "cyberduck_v2_roulade_forward_left"
PRIMARY_PT = POLICY_DIR / "model.pt"
TORQUE_NM = 2.0
NORM_EPS = 1e-2
OBS_DIM = 48
EPISODE_S = 5.0
WHEEL_ACTION_LIMIT = 2.0

# 与 getup / 训练 joint_pos 顺序一致。
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
    if PRIMARY_PT.is_file():
        return PRIMARY_PT
    if not POLICY_DIR.is_dir():
        return None
    found = sorted(POLICY_DIR.glob("*.pt"))
    return found[-1] if found else None


def _load_actor(path: Path) -> dict[str, np.ndarray]:
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
            if len(payload) != numel * 4:
                continue
            raw = np.frombuffer(payload, dtype="<f4").copy()
            out[key] = raw.reshape(shape)
        return out


def _elu(x: np.ndarray) -> np.ndarray:
    return np.where(x > 0.0, x, np.expm1(x))


class RouladePolicy:
    TORQUE_NM = TORQUE_NM
    EPISODE_S = EPISODE_S

    def __init__(self) -> None:
        self.path = _find_pt()
        self.ready = False
        self.reason = ""
        self.active = False
        self.last_action = np.zeros(14, dtype=np.float64)
        self.q_prev: np.ndarray | None = None
        self.servo_deg = {"yaw": 0.0, "roll": 0.0}
        self.wheel_vel = {can: 0.0 for _name, can, _sign in WHEELS}
        self.elapsed = 0.0
        self._w: dict[str, np.ndarray] = {}
        self._load()

    def status_line(self) -> str:
        if self.ready and self.path is not None:
            return (
                f"前左翻滚已加载 {self.path.name}。"
                f"站立后按 A 触发，再按 A 或满 {EPISODE_S:.0f}s 退出。"
                f"身体力矩 {TORQUE_NM:.0f} N·m；翻滚时手柄轮行仍开，可推摇杆给前进速度。"
            )
        return f"前左翻滚没加载（{self.reason}）。"

    def _load(self) -> None:
        if self.path is None:
            self.reason = f"{POLICY_DIR} 里没有 pt"
            return
        try:
            self._w = _load_actor(self.path)
            weight = self._w["mlp.0.weight"]
            if weight.shape != (512, OBS_DIM):
                self.reason = f"actor 输入是 {weight.shape[1]}，这份翻滚权重要 {OBS_DIM}"
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
        """按 A 进入，再按 A 或超时退出。轮子不在返回值里。"""
        if not self.ready:
            return None
        dt = max(0.001, min(0.05, dt))
        if self.active:
            if arm:
                self._stop()
                print("A：退出前左翻滚，回到默认姿态。", flush=True)
                return None
            self.elapsed += dt
            if self.elapsed >= EPISODE_S:
                self._stop()
                print(f"A：前左翻滚已跑满 {EPISODE_S:.0f}s，自动退出。", flush=True)
                return None
            return self._step(dt, gravity, gyro, bus, servo_deg)
        if not arm:
            return None
        self.active = True
        self.last_action[:] = 0.0
        self.q_prev = None
        self.elapsed = 0.0
        print(
            f"A：前左翻滚生效（{self.path.name if self.path else '?'}）。"
            f"身体力矩 {TORQUE_NM:.0f} N·m；手柄轮行仍可用，推摇杆给前进速度。"
            f"再按 A 或满 {EPISODE_S:.0f}s 退出。",
            flush=True,
        )
        return self._step(dt, gravity, gyro, bus, servo_deg)

    def _stop(self) -> None:
        self.active = False
        self.last_action[:] = 0.0
        self.q_prev = None
        self.servo_deg = {"yaw": 0.0, "roll": 0.0}
        self.wheel_vel = {can: 0.0 for _name, can, _sign in WHEELS}
        self.elapsed = 0.0

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

    def _phase(self) -> tuple[float, float]:
        # 回合进度相位：sin/cos(2π t/T)。训练里常见的 2 维 phase 编码。
        u = max(0.0, min(1.0, self.elapsed / EPISODE_S))
        ang = 2.0 * math.pi * u
        return math.sin(ang), math.cos(ang)

    def _step(self, dt, gravity, gyro, bus, servo_deg) -> dict[int, float] | None:
        q = self._q_sim(bus, servo_deg)
        qd = self._q_dot(dt, q, bus)
        obs = np.zeros(OBS_DIM, dtype=np.float64)
        obs[0:3] = gyro
        obs[3:6] = gravity
        for i, (_name, _can, _servo, lo, hi, _off, lat, _sign) in enumerate(POS_JOINTS):
            obs[6 + i] = _encode_pos(q[i], lo, hi, lat)
        obs[18:32] = qd
        obs[32:46] = self.last_action
        s, c = self._phase()
        obs[46] = s
        obs[47] = c
        action = self._infer(obs)
        if action is None:
            return None
        action = action.astype(np.float64)
        action[12] = min(WHEEL_ACTION_LIMIT, max(-WHEEL_ACTION_LIMIT, float(action[12])))
        action[13] = min(WHEEL_ACTION_LIMIT, max(-WHEEL_ACTION_LIMIT, float(action[13])))
        self.last_action = action
        # 仿真权重未实机标定：轮速维回灌观测，但不写 CAN。
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
        return cmd

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
            print(f"前左翻滚推理失败：{exc}", flush=True)
            self.ready = False
            self.reason = str(exc)
            return None
        return np.asarray(y, dtype=np.float64).reshape(-1)[:14]
