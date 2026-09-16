#!/usr/bin/env python3
# encoding: utf-8
"""CSI 摄像头 MJPEG 视频流：浏览器打开即可看实时画面。"""
import argparse
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2 as cv

from camera_csi import FPS, HEIGHT, WIDTH, open_camera

INDEX_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>microDuck CSI 实时视频</title>
  <style>
    html, body { margin: 0; background: #111; color: #eee; font-family: sans-serif; }
    header { padding: 12px 16px; background: #1c1c1c; }
    img { display: block; width: 100%; max-width: 3272px; margin: 0 auto; background: #000; }
  </style>
</head>
<body>
  <header>IMX219 CAM3 实时视频 · %WIDTH%x%HEIGHT% · %FPS% fps · 端口 %PORT%</header>
  <img src="/stream" alt="camera stream">
</body>
</html>
"""


class FrameHub:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.seq = 0
        self.running = True

    def set_jpeg(self, data):
        with self.lock:
            self.jpeg = data
            self.seq += 1

    def get_jpeg(self):
        with self.lock:
            return self.jpeg, self.seq


def capture_loop(hub, quality, fps, width, height):
    cap = open_camera(width=width, height=height, fps=fps)
    encode = [int(cv.IMWRITE_JPEG_QUALITY), quality]
    interval = 1.0 / max(fps, 1)
    next_t = time.perf_counter()
    try:
        while hub.running:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            scale = max(frame.shape[1] / 1280.0, 1.0)
            cv.putText(
                frame,
                "FPS : %d  %dx%d" % (fps, frame.shape[1], frame.shape[0]),
                (20, int(40 * scale)),
                cv.FONT_HERSHEY_SIMPLEX,
                0.9 * scale,
                (0, 0, 255),
                max(1, int(2 * scale)),
            )
            ok, buf = cv.imencode(".jpg", frame, encode)
            if ok:
                hub.set_jpeg(buf.tobytes())
            next_t += interval
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()
    finally:
        cap.release()


def local_ips():
    ips = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127.") and ip not in ips:
                ips.append(ip)
    except OSError:
        pass
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        if ip not in ips:
            ips.insert(0, ip)
    except OSError:
        pass
    finally:
        s.close()
    return ips or ["127.0.0.1"]


def make_handler(hub, port, fps, width, height):
    index = (
        INDEX_HTML.replace("%PORT%", str(port))
        .replace("%FPS%", str(fps))
        .replace("%WIDTH%", str(width))
        .replace("%HEIGHT%", str(height))
        .encode("utf-8")
    )
    interval = 1.0 / max(fps, 1)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            print("[%s] " % self.address_string() + fmt % args)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(index)))
                self.end_headers()
                self.wfile.write(index)
                return
            if self.path != "/stream":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()
            last_seq = -1
            try:
                while hub.running:
                    jpeg, seq = hub.get_jpeg()
                    if jpeg is None or seq == last_seq:
                        time.sleep(0.005)
                        continue
                    last_seq = seq
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(jpeg)).encode()
                        + b"\r\n\r\n"
                        + jpeg
                        + b"\r\n"
                    )
                    time.sleep(interval)
            except (BrokenPipeError, ConnectionResetError):
                return

    return Handler


def main():
    parser = argparse.ArgumentParser(description="启动 IMX219 CSI 实时视频流")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--quality", type=int, default=60)
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument("--width", type=int, default=WIDTH)
    parser.add_argument("--height", type=int, default=HEIGHT)
    args = parser.parse_args()

    hub = FrameHub()
    worker = threading.Thread(
        target=capture_loop,
        args=(hub, args.quality, args.fps, args.width, args.height),
        daemon=True,
    )
    worker.start()

    deadline = time.time() + 25
    while hub.get_jpeg()[0] is None:
        if not worker.is_alive():
            print("采集线程已退出")
            sys.exit(1)
        if time.time() > deadline:
            print("等待摄像头首帧超时")
            sys.exit(1)
        time.sleep(0.05)

    handler = make_handler(hub, args.port, args.fps, args.width, args.height)
    httpd = ThreadingHTTPServer((args.host, args.port), handler)
    print(
        "CSI 实时视频流已启动  %dx%d @ %d fps"
        % (args.width, args.height, args.fps)
    )
    print("  本机:   http://127.0.0.1:%d/" % args.port)
    for ip in local_ips():
        print("  局域网: http://%s:%d/" % (ip, args.port))
    print("浏览器打开上述地址即可播放")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n正在停止...")
    finally:
        hub.running = False
        httpd.server_close()


if __name__ == "__main__":
    main()
