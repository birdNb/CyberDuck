#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FFE1 指令分类：音量 / 配网 / 改名 / 对话语音短码；M_default 仅 ACK。"""

from __future__ import annotations

import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Deque, Optional, Tuple

MAX_PACKET_BYTES = 200
QUEUE_MAX = 32
TICK_INTERVAL_SEC = 0.05

try:
    from ble_wifi_manager import WIFI_PAYLOAD_SEP
except ImportError:
    WIFI_PAYLOAD_SEP = "\x1e"

try:
    from ble_device_name import RENAME_RE as _RENAME_RE
except ImportError:
    _RENAME_RE = re.compile(r"^rename\s+HT_(\d{8})$", re.IGNORECASE)

STICK_RE = re.compile(
    r"^X:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Y:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Z:\s*([+-]?\d+(?:\.\d+)?)"
    r"(?:\s*,\s*N:\s*\d+)?\s*$",
    re.IGNORECASE,
)
HANDSHAKE_RE = re.compile(r"^M_default$", re.IGNORECASE)
HEARTBEAT_RE = re.compile(r"^1$", re.IGNORECASE)
VOLUME_RE = re.compile(r"^V\s+(\d{1,3})$", re.IGNORECASE)
CONV_CODE_RE = re.compile(r"^[A-Z][A-Z0-9]{1,31}$")
# 小程序仍可能下发，板端识别后丢弃，避免刷「无法识别」
IGNORED_RE = re.compile(
    r"^(M_(init|protect|resetzero|tech)$"
    r"|PULL\s+(ON|OFF)"
    r"|GAIT\s+(ON|OFF)"
    r"|LT\s+(ON|OFF)"
    r"|MP\s+(ON|OFF)"
    r"|sound\s+(ON|OFF)(?:\s+\d+)?"
    r"|locate_face\s+(ON|OFF)"
    r"|neck0"
    r"|[Pp][+-]?\d+Y[+-]?\d+"
    r"|LT\+RT\+\w+"
    r"|RT\+[AXYB]"
    r"|LT\+DP[URLD]"
    r"|[AX]"
    r")$",
    re.IGNORECASE,
)


class CommandKind(str, Enum):
    HEARTBEAT = "heartbeat"
    HANDSHAKE = "handshake"
    STICK = "stick"
    VOLUME = "volume"
    RENAME = "rename"
    WIFI = "wifi"
    CONVERSATION = "conversation"
    IGNORED = "ignored"
    UNKNOWN = "unknown"


@dataclass
class QueuedCommand:
    kind: CommandKind
    wire: str
    payload: str


LogFn = Callable[[str], None]
HandleFn = Callable[[CommandKind, str], None]
AckFn = Callable[[str], None]
EchoConfirmFn = Callable[[str], None]


def _sanitize_wire_text(text: str) -> str:
    return text.replace("\x00", "").replace("\r", "").replace("\n", "").strip()


def _conversation_code_exists(code: str) -> bool:
    if not CONV_CODE_RE.match((code or "").strip().upper()):
        return False
    try:
        from ble_voice import conversation_code_exists

        return bool(conversation_code_exists(code))
    except ImportError:
        return False


def classify_payload(text: str) -> Tuple[CommandKind, str, str]:
    raw = _sanitize_wire_text(text)
    if not raw:
        return CommandKind.UNKNOWN, "", ""

    if HEARTBEAT_RE.match(raw):
        return CommandKind.HEARTBEAT, "1", "1"

    if HANDSHAKE_RE.match(raw):
        return CommandKind.HANDSHAKE, "m_default", "M_default"

    if STICK_RE.match(raw):
        return CommandKind.STICK, raw, raw

    m_vol = VOLUME_RE.match(raw)
    if m_vol:
        pct = max(0, min(100, int(m_vol.group(1))))
        wire = f"V {pct}"
        return CommandKind.VOLUME, str(pct), wire

    m_rename = _RENAME_RE.match(raw)
    if m_rename:
        digits = m_rename.group(1)
        name = f"HT_{digits}"
        return CommandKind.RENAME, name, f"rename {name}"

    if re.match(r"^WIFI\s+", raw, re.IGNORECASE):
        try:
            from ble_wifi_manager import parse_wifi_command
        except ImportError:
            parse_wifi_command = None  # type: ignore
        parsed = parse_wifi_command(raw) if parse_wifi_command else None
        if parsed is None:
            return CommandKind.UNKNOWN, raw, raw
        ssid, password = parsed
        payload = f"{ssid}{WIFI_PAYLOAD_SEP}{password}"
        return CommandKind.WIFI, payload, f"WIFI {ssid}"

    if IGNORED_RE.match(raw):
        return CommandKind.IGNORED, raw, raw

    if CONV_CODE_RE.match(raw.strip().upper()) and _conversation_code_exists(raw):
        code = raw.strip().upper()
        return CommandKind.CONVERSATION, code, code

    return CommandKind.UNKNOWN, raw, raw


def make_notify_reply(wire_text: str) -> bytes:
    return f"ACK:{wire_text}".encode("utf-8", errors="replace")[:180]


class CommandDispatcher:
    def __init__(
        self,
        handle: HandleFn,
        ack: Optional[AckFn] = None,
        echo_confirm: Optional[EchoConfirmFn] = None,
        log_rx: LogFn = print,
        log_warn: Optional[LogFn] = None,
    ) -> None:
        self._handle = handle
        self._ack = ack
        self._echo_confirm = echo_confirm
        self._log_rx = log_rx
        self._log_warn = log_warn or log_rx
        self._queue: Deque[QueuedCommand] = deque()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def on_disconnect(self) -> None:
        with self._lock:
            self._queue.clear()

    def dispatch(self, data: bytes) -> None:
        if len(data) > MAX_PACKET_BYTES:
            return
        try:
            text = data.decode("utf-8", errors="ignore")
        except UnicodeDecodeError:
            return
        text = _sanitize_wire_text(text)
        if not text:
            return

        kind, payload, wire = classify_payload(text)
        if kind == CommandKind.UNKNOWN:
            self._log_rx(f"无法识别: {text!r}")
            return

        if kind in (CommandKind.HEARTBEAT, CommandKind.STICK, CommandKind.IGNORED):
            return

        if kind == CommandKind.HANDSHAKE:
            # 兼容旧小程序仍可能写入 M_default；只 ACK，不进入任何模式
            self._log_rx("M_default (ack only)")
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"M_default 处理失败: {e}")
                return
            if self._ack is not None:
                self._ack("M_default")
            return

        if kind == CommandKind.VOLUME:
            self._log_rx(f"V: {wire}")
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"音量处理失败: {e}")
            if self._ack is not None:
                self._ack(wire)
            return

        if kind == CommandKind.RENAME:
            self._log_rx(f"rename: {wire}")
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"rename 处理失败: {e}")
            return

        if kind == CommandKind.WIFI:
            self._log_rx(f"wifi: {wire}")
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"WIFI 处理失败: {e}")
            return

        if kind == CommandKind.CONVERSATION:
            self._log_rx(f"conv: {wire}")
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"对话语音处理失败: {e}")
                return
            if self._ack is not None:
                self._ack(wire)
            return

        cmd = QueuedCommand(kind=kind, wire=wire, payload=payload)
        with self._lock:
            if len(self._queue) >= QUEUE_MAX:
                self._queue.popleft()
            self._queue.append(cmd)

    def _worker(self) -> None:
        while not self._stop.is_set():
            cmd: Optional[QueuedCommand] = None
            with self._lock:
                if self._queue:
                    cmd = self._queue.popleft()
            if cmd is None:
                time.sleep(TICK_INTERVAL_SEC)
                continue
            try:
                self._handle(cmd.kind, cmd.payload)
            except Exception as e:
                self._log_warn(f"处理失败: {e}")
            time.sleep(TICK_INTERVAL_SEC)
