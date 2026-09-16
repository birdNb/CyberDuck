#!/usr/bin/env python3
# encoding: utf-8
"""Radxa Dragon Q6A IMX219 CSI 打开封装（默认 CAM3）。"""
import os
import sys

os.environ.setdefault("GST_INSTALL_PLUGINS_HELPER", "/bin/true")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2 as cv

WIDTH = 3272
HEIGHT = 2464
FPS = 30
CAM2_ID = "/base/soc@0/cci@ac4b000/i2c-bus@0/camera@10"
CAM3_ID = "/base/soc@0/cci@ac4b000/i2c-bus@1/camera@10"
DEFAULT_CAMERA = CAM3_ID


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


def gst_pipeline(camera_name=None, width=WIDTH, height=HEIGHT, fps=FPS):
    src = "libcamerasrc"
    if camera_name:
        src += f' camera-name="{camera_name}"'
    return (
        f"{src} ! "
        f"video/x-raw, width={width}, height={height}, framerate={fps}/1 ! "
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


def open_camera(width=WIDTH, height=HEIGHT, fps=FPS):
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
        pipeline = gst_pipeline(name, width, height, fps)
        print("尝试打开:", pipeline)
        cap = cv.VideoCapture(pipeline, cv.CAP_GSTREAMER)
        if cap.isOpened():
            return cap
        cap.release()
    print("无法打开 CSI 摄像头。")
    sys.exit(1)
