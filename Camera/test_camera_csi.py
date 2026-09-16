#!/usr/bin/env python3
# encoding: utf-8
"""Radxa Dragon Q6A IMX219 CSI 预览（CAM3）。

本机是 QCS6490，使用 libcamerasrc，不能用 NVIDIA 的 nvarguscamerasrc。
有桌面显示时弹出窗口，按 q 退出；否则保存 snapshot.jpg。
"""
import os
import sys
import time

os.environ.setdefault("GST_INSTALL_PLUGINS_HELPER", "/bin/true")


def setup_display_env():
    """Must run before importing cv2, otherwise Qt may abort()."""
    runtime = f"/run/user/{os.getuid()}"
    os.environ.setdefault("XDG_RUNTIME_DIR", runtime)
    display = os.environ.get("DISPLAY")
    # 仅使用当前会话已授权的 DISPLAY；乱连 :0 会让 Qt abort
    if display:
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
        return True
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    return False


HAS_GUI = setup_display_env()

import cv2 as cv

WIDTH = 3272
HEIGHT = 2464
FPS = 30
CAM2_ID = "/base/soc@0/cci@ac4b000/i2c-bus@0/camera@10"
CAM3_ID = "/base/soc@0/cci@ac4b000/i2c-bus@1/camera@10"
DEFAULT_CAMERA = CAM3_ID
SNAPSHOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "snapshot.jpg")


def require_libcamerasrc():
    try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        if Gst.ElementFactory.find("libcamerasrc") is None:
            raise RuntimeError("no libcamerasrc")
    except Exception:
        print("未找到 GStreamer 插件 libcamerasrc。")
        sys.exit(1)


def gst_pipeline(camera_name=None):
    src = "libcamerasrc"
    if camera_name:
        src += f' camera-name="{camera_name}"'
    return (
        f"{src} ! "
        f"video/x-raw, width={WIDTH}, height={HEIGHT}, framerate={FPS}/1 ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def list_libcamera_ids():
    names = []
    try:
        from libcamera import CameraManager

        cm = CameraManager.singleton()
        cameras = getattr(cm, "cameras", None)
        if cameras is None and hasattr(cm, "get"):
            cameras = cm.get()
        for cam in cameras or []:
            ident = getattr(cam, "id", None)
            if callable(ident):
                ident = ident()
            if ident:
                names.append(str(ident))
    except Exception:
        pass
    return names


def open_camera():
    require_libcamerasrc()
    ids = list_libcamera_ids()
    if ids:
        print("libcamera 检测到:", ", ".join(ids))
    candidates = ids + [DEFAULT_CAMERA, None, CAM3_ID, CAM2_ID]
    seen = set()
    for name in candidates:
        key = name or "__default__"
        if key in seen:
            continue
        seen.add(key)
        pipeline = gst_pipeline(name)
        print("尝试打开:", pipeline)
        cap = cv.VideoCapture(pipeline, cv.CAP_GSTREAMER)
        if cap.isOpened():
            return cap
        cap.release()
    print("无法打开 CSI 摄像头。")
    sys.exit(1)


def draw_fps(frame, fps):
    cv.putText(
        frame,
        "FPS : " + str(int(fps)),
        (20, 30),
        cv.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 0, 255),
        1,
    )


def save_snapshot(frame):
    ok = cv.imwrite(SNAPSHOT, frame)
    if ok:
        print("已保存:", SNAPSHOT)
    else:
        print("保存失败:", SNAPSHOT)


def main():
    capture = open_camera()
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    print("显示模式:", "窗口" if HAS_GUI else "无桌面，保存 snapshot.jpg")

    # 等自动曝光稳定后再取图
    frame = None
    for _ in range(12):
        ret, frame = capture.read()
        if not ret:
            frame = None

    if frame is None:
        print("读取画面失败")
        capture.release()
        sys.exit(1)

    if not HAS_GUI:
        dt = 1.0 / max(capture.get(cv.CAP_PROP_FPS), 1)
        draw_fps(frame, 1.0 / dt)
        save_snapshot(frame)
        capture.release()
        return

    while capture.isOpened():
        start = time.time()
        ret, frame = capture.read()
        if not ret or frame is None:
            print("读取画面失败")
            break
        draw_fps(frame, 1.0 / max(time.time() - start, 1e-6))
        cv.imshow("frame", frame)
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            save_snapshot(frame)
            break

    capture.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
