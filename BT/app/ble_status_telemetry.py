#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人 → 小程序 状态遥测（FFE2 notify）。

  IP:192.168.19.11   局域网 IPv4
  pwr:88             电量固定 88%（本机无电池话题）
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Callable, Optional

from ble_log import log_info, log_tx, log_warn

FIXED_BATTERY_PCT = 88
IP_POLL_SEC = 15.0
PWR_POLL_SEC = 5.0
# 小程序订阅 FFE2 后连发局域网 IP：立即一次，之后每隔 2 秒，共 3 次
IP_BURST_COUNT = 3
IP_BURST_INTERVAL_SEC = 2.0

NotifyFn = Callable[[bytes], None]
BatteryFn = Callable[[], Optional[int]]


def read_lan_ip() -> Optional[str]:
    """返回局域网 IPv4 四段，如 192.168.19.11。"""
    candidates = []
    try:
        out = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
        for item in out:
            ip = item[4][0]
            if not ip.startswith("127.") and ip.count(".") == 3:
                candidates.append(ip)
    except OSError:
        pass

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.168.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        if not ip.startswith("127."):
            candidates.insert(0, ip)
    except OSError:
        pass

    for ip in candidates:
        parts = ip.split(".")
        if len(parts) == 4 and (parts[0] == "192" or parts[0] == "10"):
            return ip
    for ip in candidates:
        parts = ip.split(".")
        if len(parts) == 4:
            return ip
    return None


def read_battery_percent() -> int:
    return FIXED_BATTERY_PCT


class BleStatusTelemetry:
    def __init__(
        self,
        notify: NotifyFn,
        battery_fn: Optional[BatteryFn] = None,
        **_ignored,
    ) -> None:
        self._notify = notify
        self._battery_fn = battery_fn or (lambda: FIXED_BATTERY_PCT)
        self._stop = threading.Event()
        self._subscribed = threading.Event()
        self._lock = threading.Lock()
        self._last_ip: Optional[str] = None
        self._last_pwr_sent: Optional[int] = None
        self._poll_thread: Optional[threading.Thread] = None
        self._ip_burst_stop = threading.Event()
        self._ip_burst_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._poll_thread is not None:
            return
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        log_info("状态遥测已启动（IP / pwr:88 → FFE2）")

    def stop(self) -> None:
        self._stop.set()
        self._cancel_ip_burst()

    def on_subscribed(self, emit_snapshot: bool = True) -> None:
        self._subscribed.set()
        with self._lock:
            self._last_pwr_sent = None
        if emit_snapshot:
            self._push_snapshot()

    def on_unsubscribed(self) -> None:
        self._subscribed.clear()
        self._cancel_ip_burst()

    def _cancel_ip_burst(self) -> None:
        self._ip_burst_stop.set()

    def _push_snapshot(self) -> None:
        ip = read_lan_ip()
        if ip:
            self._send_ip(ip, force=True)
        self._maybe_send_pwr(self._read_pwr(), force=True)

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            ip = read_lan_ip()
            if ip:
                self._send_ip(ip)
            self._maybe_send_pwr(self._read_pwr())
            time.sleep(PWR_POLL_SEC)

    def _read_pwr(self) -> int:
        try:
            pct = self._battery_fn()
        except Exception:
            pct = None
        if pct is None:
            return FIXED_BATTERY_PCT
        return max(0, min(100, int(pct)))

    def _tx(self, text: str) -> bool:
        if not self._subscribed.is_set():
            return False
        payload = text.encode("utf-8", errors="replace")[:180]
        try:
            self._notify(payload)
            log_tx(text)
            return True
        except Exception as e:
            log_warn(f"notify 失败: {e}")
            return False

    def _send_ip(self, ip: str, force: bool = False) -> None:
        with self._lock:
            if not force and ip == self._last_ip:
                return
        if not self._tx(f"IP:{ip}"):
            return
        with self._lock:
            self._last_ip = ip

    def _maybe_send_pwr(self, raw_pct: int, force: bool = False) -> None:
        if not self._subscribed.is_set():
            return
        pct = max(0, min(100, int(raw_pct)))
        with self._lock:
            prev = self._last_pwr_sent
            should_send = force or prev is None or pct < prev
        if not should_send:
            return
        if self._tx(f"pwr:{pct}"):
            with self._lock:
                self._last_pwr_sent = pct

    def push_ip(self, force: bool = True) -> Optional[str]:
        ip = read_lan_ip()
        if ip:
            self._send_ip(ip, force=force)
        return ip

    def push_ip_burst(
        self,
        count: int = IP_BURST_COUNT,
        interval: float = IP_BURST_INTERVAL_SEC,
    ) -> None:
        """连接后连发 IP：立即一次，之后每隔 interval 秒，共 count 次。"""
        self._cancel_ip_burst()
        stop = threading.Event()
        self._ip_burst_stop = stop
        total = max(1, int(count))
        gap = max(0.0, float(interval))

        def _run() -> None:
            for i in range(total):
                if stop.is_set() or self._stop.is_set() or not self._subscribed.is_set():
                    return
                ip = read_lan_ip()
                if ip:
                    self._send_ip(ip, force=True)
                    log_info(f"连接后上报 IP ({i + 1}/{total}): {ip}")
                else:
                    log_warn(f"连接后上报 IP ({i + 1}/{total}): 尚未拿到局域网地址")
                if i + 1 < total and stop.wait(gap):
                    return

        thread = threading.Thread(target=_run, name="ble-ip-burst", daemon=True)
        self._ip_burst_thread = thread
        thread.start()
