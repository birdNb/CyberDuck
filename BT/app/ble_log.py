#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 终端日志：RX 红 / TX 绿。"""

from __future__ import annotations

import sys
import time

_USE_COLOR = hasattr(sys.stdout, "isatty") and sys.stdout.isatty()
_RED = "\033[31m" if _USE_COLOR else ""
_GREEN = "\033[32m" if _USE_COLOR else ""
_YELLOW = "\033[33m" if _USE_COLOR else ""
_RESET = "\033[0m" if _USE_COLOR else ""


def _ts() -> str:
    return time.strftime("%H:%M:%S")


def log_info(msg: str) -> None:
    line = f"[{_ts()}] {msg}"
    print(line, flush=True)
    try:
        with open("/tmp/duck-ble.log", "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except OSError:
        pass


def log_warn(msg: str) -> None:
    line = f"[{_ts()}] {msg}"
    print(f"{_YELLOW}{line}{_RESET}", flush=True)
    try:
        with open("/tmp/duck-ble.log", "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except OSError:
        pass


def log_rx(msg: str) -> None:
    print(f"{_RED}[{_ts()}] RX {msg}{_RESET}", flush=True)


def log_tx(msg: str) -> None:
    print(f"{_GREEN}[{_ts()}] TX {msg}{_RESET}", flush=True)
