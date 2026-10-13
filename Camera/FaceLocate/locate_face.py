#!/usr/bin/env python3
"""人脸跟踪：进模式后脖子先到 stand，再由 11(pitch)/13(yaw) 把人脸锁在画面中心。

控制律对齐 Bird_ws/locate_face_cpp（死区、P 增益、步长钳位、EMA、丢脸回中）。
退出时脖子 11~14 平滑回电机零位（0°）后失能泄力。
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CAMERA_DIR = HERE.parent
ROOT = CAMERA_DIR.parent
MOTOR_DIR = ROOT / "motor"
POSE_DIR = MOTOR_DIR / "pose_contrl"
POSE_CONFIG = POSE_DIR / "pose_config.yaml"
NECK_STATE_FILE = Path("/tmp/locate_face_neck.state")

if "--gui" in sys.argv and os.environ.get("DISPLAY"):
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
else:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, str(CAMERA_DIR))
sys.path.insert(0, str(MOTOR_DIR))
sys.path.insert(0, str(POSE_DIR))

from camera_csi import (  # noqa: E402
    CAM2_ID,
    CAM3_ID,
    DEFAULT_CAMERA,
    gst_pipeline,
    list_libcamera_ids,
    require_libcamerasrc,
)
from face_detect import YuNetDetector  # noqa: E402
from ht_can import Motor, MotorBus  # noqa: E402
from pose import (  # noqa: E402
    command_position,
    ensure_can,
    iface_down_reason,
    load_config,
    read_degrees,
    smootherstep,
)

import cv2 as cv  # noqa: E402

# ----- 与 locate_face_cpp/include/Common.h 对齐 -----
DEAD_BAND_X = 0.04
DEAD_BAND_Y = 0.05
K_YAW_DEG = 20.0
K_PITCH_DEG = 15.0
MAX_STEP_YAW_DEG = 6.0
MAX_STEP_PITCH_DEG = 5.0
TARGET_EMA_ALPHA = 0.60
YAW_LIMIT_DEG = 80.0
PITCH_UP_DEG = -40.0
PITCH_DOWN_DEG = 60.0
NECK_HZ = 50
NO_FACE_RETURN_HOME_SEC = 1.0
RETURN_HOME_RATE_DEG_PER_SEC = 45.0
PARK_RATE_DEG_PER_SEC = 30.0
FORCE_EXIT_SEC = 8.0
TRACK_TORQUE_NM = 0.1
TRACK_PITCH_TORQUE_NM = 0.3
CAM_FPS = 15

PITCH_ID = 11
YAW_ID = 13
NECK_IDS = (11, 12, 13, 14)
TRACK_IDS = (11, 13)
RESOLUTIONS = ((1280, 720), (1640, 1232), (640, 480), (3272, 2464))


def command_neck(bus: MotorBus, ids: list[int], degrees: dict[int, float]) -> None:
    pitch = [mid for mid in ids if mid == PITCH_ID]
    rest = [mid for mid in ids if mid != PITCH_ID]
    if pitch:
        command_position(bus, pitch, degrees, TRACK_PITCH_TORQUE_NM)
    if rest:
        command_position(bus, rest, degrees, TRACK_TORQUE_NM)


def clampf(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def stand_neck_home(cfg: dict) -> dict[int, float]:
    joints = ((cfg.get("poses") or {}).get("stand") or {}).get("joints") or {}
    home: dict[int, float] = {}
    missing = []
    for mid in NECK_IDS:
        key = str(mid)
        if key not in joints:
            missing.append(mid)
            continue
        home[mid] = float(joints[key])
    if missing:
        raise SystemExit(
            f"pose_config.yaml 的 stand 缺少脖子角度 {missing}。"
            "先: python3 motor/pose_contrl/pose.py capture stand"
        )
    return home


def open_neck_bus(cfg: dict) -> tuple[MotorBus, list[int]]:
    iface = str(cfg.get("interface", "can0"))
    ensure_can(iface)
    down = iface_down_reason(iface)
    if down:
        raise SystemExit(f"{down}\n请先: sudo {MOTOR_DIR}/setup_can0.sh")
    motor_type = str(cfg.get("motor_type", "4538_19"))
    bus = MotorBus(iface)
    for mid in NECK_IDS:
        bus.add_motor(Motor(mid, motor_type, f"J{mid}"))
    found = bus.scan(list(NECK_IDS), timeout_each=0.06)
    missing = [i for i in NECK_IDS if i not in found]
    if missing:
        print(f"脖子关节未应答，本次跳过: {missing}")
    if PITCH_ID not in found or YAW_ID not in found:
        bus.close()
        raise SystemExit(f"跟踪需要 {PITCH_ID}(pitch) 和 {YAW_ID}(yaw) 在线。")
    for mid in found:
        bus.set_timeout_ms(mid, 0)
    time.sleep(0.05)
    return bus, found


def go_neck_home(
    bus: MotorBus,
    ids: list[int],
    home: dict[int, float],
    cfg: dict,
    running,
    seconds: float = 2.0,
    label: str = "stand",
    start: dict[int, float] | None = None,
) -> dict[int, float]:
    if start is None:
        start = read_degrees(bus, ids)
    duration = max(0.5, float(seconds))
    hz = max(20.0, float(cfg.get("hz", NECK_HZ)))
    period = 1.0 / hz
    torque = abs(float(cfg.get("torque_nm", 1.2)))
    print(f"脖子过渡到 {label}:")
    for mid in ids:
        print(f"  ID {mid:2d}  {start[mid]:+8.2f}° → {home[mid]:+8.2f}°")
    for mid in ids:
        bus.enable(mid)
        bus.set_torque_limit(mid, torque)
    time.sleep(0.05)
    t0 = time.monotonic()
    last = dict(start)
    while running():
        now = time.monotonic()
        u = (now - t0) / duration
        s = smootherstep(u)
        cmd = {mid: start[mid] + (home[mid] - start[mid]) * s for mid in ids}
        command_position(bus, ids, cmd, torque)
        last = cmd
        bus.collect(timeout=min(0.004, period * 0.3))
        if u >= 1.0:
            break
        spent = time.monotonic() - now
        if spent < period:
            time.sleep(period - spent)
    hold = home if running() else last
    for _ in range(int(hz * 0.3)):
        command_position(bus, ids, hold, torque)
        time.sleep(period)
    return hold


def park_neck_and_release(
    bus: MotorBus,
    ids: list[int],
    cfg: dict,
    running,
    fallback: dict[int, float],
) -> None:
    """脖子平滑回电机零位（0°）后失能。"""
    try:
        start = read_degrees(bus, ids)
    except RuntimeError as exc:
        print(f"读脖子角度失败，按最后指令估计起点: {exc}", flush=True)
        start = {mid: fallback.get(mid, 0.0) for mid in ids}
    zero = {mid: 0.0 for mid in ids}
    peak = max((abs(start[mid]) for mid in ids), default=0.0)
    seconds = max(1.0, peak / PARK_RATE_DEG_PER_SEC)
    try:
        go_neck_home(bus, ids, zero, cfg, running, seconds, label="零位", start=start)
    finally:
        for mid in ids:
            bus.disable(mid)
        time.sleep(0.05)
        print("脖子已泄力", flush=True)


class NeckServo:
    """相对 stand 的 yaw/pitch 偏移（度），50Hz 下发 11/13，12/14 锁 stand。"""

    def __init__(
        self,
        invert_yaw: bool = False,
        invert_pitch: bool = False,
    ) -> None:
        self.invert_yaw = -1.0 if invert_yaw else 1.0
        self.invert_pitch = -1.0 if invert_pitch else 1.0
        self._lock = threading.Lock()
        self.ctrl_yaw = 0.0
        self.ctrl_pitch = 0.0
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self.last_face_t = time.monotonic()
        self.last_track_t = 0.0
        self.has_face = False
        self.dx_n = 0.0
        self.dy_n = 0.0
        self.lost_sec = 0.0
        self.state = "NoFace"

    def snapshot(self) -> tuple[float, float, bool, float, float, str]:
        with self._lock:
            return (
                self.target_yaw,
                self.target_pitch,
                self.has_face,
                self.dx_n,
                self.dy_n,
                self.state,
            )

    def write_state_file(self) -> None:
        yaw, pitch, *_ = self.snapshot()
        try:
            NECK_STATE_FILE.write_text(f"{yaw:.3f} {pitch:.3f}\n", encoding="ascii")
        except OSError:
            pass

    def update_from_error(self, dx_n: float, dy_n: float) -> None:
        if abs(dx_n) < DEAD_BAND_X:
            dx_n = 0.0
        if abs(dy_n) < DEAD_BAND_Y:
            dy_n = 0.0
        delta_yaw = clampf(-K_YAW_DEG * dx_n, -MAX_STEP_YAW_DEG, MAX_STEP_YAW_DEG)
        delta_pitch = clampf(K_PITCH_DEG * dy_n, -MAX_STEP_PITCH_DEG, MAX_STEP_PITCH_DEG)
        a = TARGET_EMA_ALPHA
        with self._lock:
            raw_yaw = self.ctrl_yaw + delta_yaw
            raw_pitch = self.ctrl_pitch + delta_pitch
            self.ctrl_yaw = self.ctrl_yaw * (1.0 - a) + raw_yaw * a
            self.ctrl_pitch = self.ctrl_pitch * (1.0 - a) + raw_pitch * a
            self.ctrl_yaw = clampf(self.ctrl_yaw, -YAW_LIMIT_DEG, YAW_LIMIT_DEG)
            self.ctrl_pitch = clampf(self.ctrl_pitch, PITCH_UP_DEG, PITCH_DOWN_DEG)
            self.target_yaw = self.ctrl_yaw
            self.target_pitch = self.ctrl_pitch
            self.has_face = True
            self.dx_n = dx_n
            self.dy_n = dy_n
            self.lost_sec = 0.0
            self.state = "Tracking"
            self.last_face_t = time.monotonic()
            self.last_track_t = self.last_face_t

    def step_homing(self, dt: float) -> None:
        step = RETURN_HOME_RATE_DEG_PER_SEC * dt
        with self._lock:
            if abs(self.target_yaw) > 1e-4:
                self.target_yaw -= math.copysign(min(step, abs(self.target_yaw)), self.target_yaw)
                self.ctrl_yaw = self.target_yaw
            else:
                self.target_yaw = self.ctrl_yaw = 0.0
            if abs(self.target_pitch) > 1e-4:
                self.target_pitch -= math.copysign(
                    min(step, abs(self.target_pitch)), self.target_pitch
                )
                self.ctrl_pitch = self.target_pitch
            else:
                self.target_pitch = self.ctrl_pitch = 0.0

    def apply_no_face(self, dt: float) -> None:
        now = time.monotonic()
        with self._lock:
            lost = now - self.last_face_t
            self.has_face = False
            self.dx_n = 0.0
            self.dy_n = 0.0
            self.lost_sec = lost
        if lost > NO_FACE_RETURN_HOME_SEC:
            self.step_homing(dt)
            with self._lock:
                self.state = "Homing"
        else:
            with self._lock:
                self.state = "NoFace"

    def on_detect(self, result) -> None:
        now = time.monotonic()
        dt = 0.033
        if self.last_track_t > 0:
            dt = clampf(now - self.last_track_t, 0.02, 0.2)
        self.last_track_t = now
        if result is None:
            self.apply_no_face(dt)
            return
        dx_n, dy_n, _fx, _fy = result
        self.update_from_error(dx_n, dy_n)

    def tick(self, dt: float) -> None:
        """检测线程卡住时，按丢脸处理，避免一直锁在旧角度。"""
        now = time.monotonic()
        stale = now - self.last_track_t if self.last_track_t > 0 else 0.0
        if stale > 0.35:
            self.apply_no_face(dt)

    def command_degrees(self, home: dict[int, float], ids: list[int]) -> dict[int, float]:
        yaw, pitch, *_ = self.snapshot()
        cmd: dict[int, float] = {}
        for mid in ids:
            if mid == PITCH_ID:
                cmd[mid] = home[PITCH_ID] + self.invert_pitch * pitch
            elif mid == YAW_ID:
                cmd[mid] = home[YAW_ID] + self.invert_yaw * yaw
        return cmd


def open_track_camera(width: int, height: int, fps: int):
    require_libcamerasrc()
    ids = list_libcamera_ids()
    if ids:
        print("libcamera 检测到:", ", ".join(ids))
    sizes = [(width, height)] + [s for s in RESOLUTIONS if s != (width, height)]
    names = []
    for name in ids + [DEFAULT_CAMERA, None, CAM3_ID, CAM2_ID]:
        key = name or "__default__"
        if key not in names:
            names.append(key)
    for w, h in sizes:
        for key in names:
            cam_name = None if key == "__default__" else key
            pipeline = gst_pipeline(cam_name, width=w, height=h, fps=fps)
            print("尝试打开:", pipeline)
            cap = cv.VideoCapture(pipeline, cv.CAP_GSTREAMER)
            if cap.isOpened():
                print(f"相机已打开 {w}x{h} @ {fps}fps")
                return cap
            cap.release()
    print("无法打开 CSI 摄像头。若视频流占用了镜头，先: Camera/stop.sh")
    sys.exit(1)


def draw_overlay(frame, telem, fps_show: float):
    yaw, pitch, has_face, dx_n, dy_n, state = telem
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv.drawMarker(frame, (cx, cy), (255, 255, 255), cv.MARKER_CROSS, 24, 2)
    dx_pix = int(DEAD_BAND_X * w / 2.0)
    dy_pix = int(DEAD_BAND_Y * h / 2.0)
    cv.rectangle(
        frame,
        (cx - dx_pix, cy - dy_pix),
        (cx + dx_pix, cy + dy_pix),
        (90, 90, 90),
        1,
    )
    if has_face:
        fx = int(cx + dx_n * w / 2.0)
        fy = int(cy + dy_n * h / 2.0)
        cv.circle(frame, (fx, fy), 8, (0, 255, 0), 2)
    label = (
        f"FPS {fps_show:.1f}  yaw={yaw:+.1f} pitch={pitch:+.1f}  {state}"
    )
    cv.putText(frame, label, (20, 40), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return frame


def main() -> int:
    parser = argparse.ArgumentParser(description="人脸跟踪：脖子 stand 后 11/13 锁人脸居中")
    parser.add_argument("--gui", action="store_true", help="弹出预览窗口")
    parser.add_argument("--no-motor", action="store_true", help="只检测不转脖子")
    parser.add_argument("--invert-yaw", action="store_true", help="左右跟反了再加")
    parser.add_argument("--invert-pitch", action="store_true", help="点头方向反了再加")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=CAM_FPS)
    parser.add_argument("--config", default=str(POSE_CONFIG))
    args = parser.parse_args()

    if args.gui and not os.environ.get("DISPLAY"):
        print("没有 DISPLAY，改为后台模式")
        args.gui = False
    if args.gui:
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

    cfg = load_config(Path(args.config))
    home = stand_neck_home(cfg)
    print(
        f"stand 脖子: 11(pitch)={home[11]:+.2f}°  12={home[12]:+.2f}°  "
        f"13(yaw)={home[13]:+.2f}°  14={home[14]:+.2f}°"
    )

    detector = YuNetDetector()
    servo = NeckServo(invert_yaw=args.invert_yaw, invert_pitch=args.invert_pitch)
    running = True
    stop_pub = threading.Event()
    sig_hits = 0

    def _force_exit_later() -> None:
        time.sleep(FORCE_EXIT_SEC)
        print("退出超时，强制结束", flush=True)
        os._exit(130)

    def _stop(*_):
        nonlocal running, sig_hits
        sig_hits += 1
        running = False
        stop_pub.set()
        if sig_hits == 1:
            print("\n收到中断，正在退出（再按一次 Ctrl+C 强制结束）", flush=True)
            threading.Thread(target=_force_exit_later, daemon=True).start()
        else:
            print("强制退出", flush=True)
            os._exit(130)

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    bus = None
    ids: list[int] = []
    neck_ids: list[int] = []
    if not args.no_motor:
        bus, ids = open_neck_bus(cfg)
        neck_ids = list(ids)
        try:
            go_neck_home(bus, ids, home, cfg, lambda: running)
        except Exception:
            bus.close()
            raise
        if not running:
            if sig_hits < 2:
                park_neck_and_release(bus, neck_ids, cfg, lambda: sig_hits < 2, home)
            bus.close()
            return 0
        track_ids = [mid for mid in TRACK_IDS if mid in ids]
        for mid in track_ids:
            limit = TRACK_PITCH_TORQUE_NM if mid == PITCH_ID else TRACK_TORQUE_NM
            bus.set_torque_limit(mid, limit)
        ids = track_ids
        print(
            f"进入视觉伺服：只动 11(pitch {TRACK_PITCH_TORQUE_NM:.2f} Nm) "
            f"和 13(yaw {TRACK_TORQUE_NM:.2f} Nm)，12/14 锁在 stand。"
            "Ctrl+C 回零位并泄力后退出"
        )
    else:
        print("仅检测，不控制电机")

    cap = open_track_camera(args.width, args.height, args.fps)
    period = 1.0 / NECK_HZ

    def publisher() -> None:
        tick = 0
        while not stop_pub.is_set() and running:
            t0 = time.monotonic()
            servo.tick(period)
            if bus is not None:
                cmd = servo.command_degrees(home, ids)
                command_neck(bus, ids, cmd)
                bus.collect(timeout=min(0.003, period * 0.25))
            if tick % 5 == 0:
                servo.write_state_file()
            tick += 1
            spent = time.monotonic() - t0
            if spent < period:
                time.sleep(period - spent)

    pub_thread = threading.Thread(target=publisher, daemon=True)
    pub_thread.start()

    detect_lock = threading.Lock()
    detect_in = {"frame": None}
    detect_out = {"result": None}

    def detect_loop() -> None:
        while running and not stop_pub.is_set():
            with detect_lock:
                frame = detect_in["frame"]
                detect_in["frame"] = None
            if frame is None:
                time.sleep(0.005)
                continue
            try:
                result = detector.process(frame)
            except Exception as exc:
                print(f"[face] 检测失败: {exc}", flush=True)
                result = None
            with detect_lock:
                detect_out["result"] = result
                detect_out["t"] = time.monotonic()
            servo.on_detect(result)

    detect_thread = threading.Thread(target=detect_loop, daemon=True)
    detect_thread.start()

    frame_lock = threading.Lock()
    latest_frame = {"img": None}

    def capture_loop() -> None:
        while running and not stop_pub.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            view = frame
            det = frame.copy()
            with frame_lock:
                latest_frame["img"] = view
            with detect_lock:
                detect_in["frame"] = det

    capture_thread = threading.Thread(target=capture_loop, daemon=True)
    capture_thread.start()

    if args.gui:
        cv.namedWindow("Locate Face", cv.WINDOW_NORMAL)

    fps_t0 = time.monotonic()
    fps_n = 0
    fps_show = 0.0
    last_log = 0.0
    try:
        while running:
            with frame_lock:
                frame = latest_frame["img"]
            if frame is None:
                time.sleep(0.01)
                continue
            with detect_lock:
                result = detect_out["result"]

            fps_n += 1
            now = time.monotonic()
            if fps_n >= 10:
                fps_show = fps_n / max(now - fps_t0, 1e-3)
                fps_t0 = now
                fps_n = 0

            telem = servo.snapshot()
            if args.gui:
                draw_overlay(frame, telem, fps_show)
                cv.imshow("Locate Face", frame)
                key = cv.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    running = False
                    break

            time.sleep(0.02)
            if now - last_log >= 1.0:
                yaw, pitch, has_face, dx_n, dy_n, state = telem
                print(
                    f"[track] face={'Y' if has_face else 'N'}  FPS={fps_show:.1f}  "
                    f"yaw={yaw:+.1f}  pitch={pitch:+.1f}  "
                    f"dx={dx_n:+.2f} dy={dy_n:+.2f}  {state}",
                    flush=True,
                )
                last_log = now
    finally:
        print("退出 → 脖子回零位并泄力", flush=True)
        stop_pub.set()
        pub_thread.join(timeout=0.5)
        detect_thread.join(timeout=0.5)
        if bus is not None and sig_hits < 2:
            last_cmd = dict(home)
            last_cmd.update(servo.command_degrees(home, ids))
            try:
                park_neck_and_release(bus, neck_ids, cfg, lambda: sig_hits < 2, last_cmd)
            except Exception as exc:
                print(f"脖子回零/泄力失败: {exc}", flush=True)
        servo.write_state_file()
        releaser = threading.Thread(target=cap.release, daemon=True)
        releaser.start()
        releaser.join(timeout=1.0)
        if args.gui:
            try:
                cv.destroyAllWindows()
            except Exception:
                pass
        if bus is not None:
            try:
                bus.close()
            except OSError:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
