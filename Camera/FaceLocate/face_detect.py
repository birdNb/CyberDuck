#!/usr/bin/env python3
"""人脸检测。

OpenCV 4.6 的 FaceDetectorYN 跑不了 2023 YuNet（Eltwise 尺寸断言）。
这里用 Haar，接口仍返回 (dx_n, dy_n, face_cx, face_cy)。
"""

from __future__ import annotations

from pathlib import Path

import cv2 as cv

DETECT_SCORE = 0.4
PROC_MAX_W = 320
HERE = Path(__file__).resolve().parent
HAAR_PATH = HERE / "model" / "haarcascade_frontalface_default.xml"


def _proc_size(w: int, h: int, max_w: int) -> tuple[int, int]:
    if w <= max_w:
        return w, h
    return max_w, max(1, int(h * max_w / w))


class FaceDetector:
    def __init__(self, model_path: Path | None = None) -> None:
        path = Path(model_path) if model_path else HAAR_PATH
        if not path.is_file():
            raise FileNotFoundError(f"找不到人脸模型: {path}")
        self._det = cv.CascadeClassifier(str(path))
        if self._det.empty():
            raise RuntimeError(f"Haar 级联加载失败: {path}")
        print(f"[face] backend Haar ({path.name})")

    def process(self, bgr):
        """返回 (dx_n, dy_n, face_cx, face_cy) 或 None。"""
        h, w = bgr.shape[:2]
        pw, ph = _proc_size(w, h, PROC_MAX_W)
        if (pw, ph) != (w, h):
            proc = cv.resize(bgr, (pw, ph), interpolation=cv.INTER_AREA)
            sx = w / float(pw)
            sy = h / float(ph)
        else:
            proc = bgr
            sx = sy = 1.0

        gray = cv.cvtColor(proc, cv.COLOR_BGR2GRAY)
        gray = cv.equalizeHist(gray)
        min_side = max(32, min(pw, ph) // 8)
        faces = self._det.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            flags=cv.CASCADE_SCALE_IMAGE,
            minSize=(min_side, min_side),
        )
        if faces is None or len(faces) == 0:
            return None

        x, y, bw, bh = max(faces, key=lambda f: int(f[2]) * int(f[3]))
        fx = (float(x) + float(bw) / 2.0) * sx
        fy = (float(y) + float(bh) / 2.0) * sy
        dx_n = (fx - w / 2.0) / (w / 2.0)
        dy_n = (fy - h / 2.0) / (h / 2.0)
        return dx_n, dy_n, fx, fy


# 兼容旧导入名
YuNetDetector = FaceDetector
