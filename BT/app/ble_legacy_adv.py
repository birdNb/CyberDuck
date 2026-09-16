#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 广播：RK Realtek / AIC8800 走 HCI；Orin USB 优先 btmgmt。

- Realtek RTL8822：btmgmt advertising 会走 Extended，常 Disallowed/Busy。
- AIC8800D80（Q6A）：btmgmt / D-Bus LEAdvertisingManager 走 mgmt 会阻塞，
  ActiveInstances 一直为 0。GATT 仍用 D-Bus，空中广播改 Legacy HCI，
  失败则用 Extended HCI（legacy PDU / ADV_IND），彻底跳过 btmgmt。
"""

from __future__ import annotations

import os
import re
import subprocess
import threading
import time
from typing import Callable, List, Optional

from platform_detect import PlatformInfo, detect_platform

LogFn = Callable[[str], None]

FFE0_UUID128_LE = bytes(
    [
        0xE0, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
        0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB,
    ]
)

ADV_INTERVAL_MIN = 0x0030  # ~30ms，iOS 扫描窗口短
ADV_INTERVAL_MAX = 0x0050  # ~50ms

_keeper_stop = threading.Event()
_keeper_thread: Optional[threading.Thread] = None


def build_adv_payload(name: str = "") -> bytes:
    """主包：Flags + 16-bit FFE0 + 设备名（微信按服务 UUID 扫描需要 UUID 在 ADV 里）。"""
    name_b = (name or "HT_BLE").encode("utf-8")
    base = bytes([0x02, 0x01, 0x06, 0x03, 0x03, 0xE0, 0xFF])
    budget = 31 - len(base) - 2
    name_b = name_b[: max(0, budget)]
    return (base + bytes([len(name_b) + 1, 0x09]) + name_b)[:31]


def build_scan_rsp(name: str = "") -> bytes:
    """Scan Response：128-bit FFE0，供 nRF/部分手机二次匹配。"""
    _ = name
    return (bytes([0x11, 0x07]) + FFE0_UUID128_LE)[:31]


def _run(argv: List[str], timeout: float = 5.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        argv, capture_output=True, text=True, timeout=timeout, check=False
    )


def _hci_ok(out: str) -> bool:
    lines = [ln.strip() for ln in (out or "").splitlines() if ln.strip()]
    for i, ln in enumerate(lines):
        if "HCI Event" in ln and i + 1 < len(lines):
            toks = lines[i + 1].split()
            if len(toks) >= 4:
                return toks[3] == "00"
    return bool(re.search(r"\b00\s*$", out or ""))


_ADV_BACKEND = "legacy"  # legacy | extended | hciconfig


def _adv_log(msg: str, log: Optional[LogFn] = None) -> None:
    if log:
        log(msg)
    line = time.strftime("%H:%M:%S ") + msg + "\n"
    for path in (
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "ble_adv.log"),
        "/tmp/duck-ble-adv.log",
    ):
        try:
            with open(path, "a", encoding="utf-8") as f:
                f.write(line)
            try:
                os.chmod(path, 0o666)
            except OSError:
                pass
        except OSError:
            pass


def hci_le_cmd(hci_dev: str, *args: str, quiet: bool = False) -> bool:
    try:
        r = _run(["hcitool", "-i", hci_dev, "cmd", "0x08", *args], timeout=3.0)
        out = (r.stdout or "") + (r.stderr or "")
        ok = _hci_ok(out) and "Send failed" not in out
        if not ok and not quiet:
            brief = " ".join(out.split())[:220]
            _adv_log(f"hci_le_cmd fail 0x08 {' '.join(args[:2])}: {brief}")
        return ok
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError) as e:
        if not quiet:
            _adv_log(f"hci_le_cmd exception: {e}")
        return False


def btmgmt(hci_dev: str, *args: str) -> None:
    plat = detect_platform()
    if plat.skip_btmgmt:
        return
    idx = hci_dev.replace("hci", "")
    try:
        _run(["btmgmt", "--index", idx, *args], timeout=1.5)
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        pass


def prepare_adapter(hci_dev: str, name: str, platform: Optional[PlatformInfo] = None) -> None:
    plat = platform or detect_platform()
    _run(["rfkill", "unblock", "bluetooth"], timeout=2.0)
    _run(["hciconfig", hci_dev, "up"], timeout=2.0)
    _run(["hciconfig", hci_dev, "name", name], timeout=2.0)
    _run(["hciconfig", hci_dev, "noscan"], timeout=2.0)
    if plat.skip_btmgmt:
        # AIC8800：btmgmt / bluetoothctl 会卡住，只保留 hciconfig + HCI
        time.sleep(0.4)
        return
    for cmd in (
        ["bluetoothctl", "power", "on"],
    ):
        _run(cmd)
    btmgmt(hci_dev, "le", "on")
    btmgmt(hci_dev, "connectable", "on")
    btmgmt(hci_dev, "discov", "off")
    btmgmt(hci_dev, "pairable", "off")
    btmgmt(hci_dev, "bondable", "off")
    if plat.adv_mode != "legacy_hci":
        btmgmt(hci_dev, "advertising", "off")
    hci_le_cmd(hci_dev, "0x000a", "0x00")
    btmgmt(hci_dev, "name", name)
    _run(["bluetoothctl", "system-alias", name])
    if plat.is_onboard_bt:
        _run(["hciconfig", hci_dev, "noscan"])
    time.sleep(0.2)


def stop_advertising(hci_dev: str) -> None:
    stop_adv_keeper()
    hci_le_cmd(hci_dev, "0x000a", "0x00", quiet=True)
    hci_le_cmd(hci_dev, "0x0039", "0x00", "0x00", quiet=True)
    _run(["hciconfig", hci_dev, "noleadv"], timeout=2.0)


def _pad31(payload: bytes) -> List[str]:
    data = (payload + bytes(31 - len(payload)))[:31]
    return [f"0x{len(payload):02x}"] + [f"0x{b:02x}" for b in data]


def _set_params(hci_dev: str) -> bool:
    lo_min, hi_min = ADV_INTERVAL_MIN & 0xFF, (ADV_INTERVAL_MIN >> 8) & 0xFF
    lo_max, hi_max = ADV_INTERVAL_MAX & 0xFF, (ADV_INTERVAL_MAX >> 8) & 0xFF
    return hci_le_cmd(
        hci_dev, "0x0006",
        f"0x{lo_min:02x}", f"0x{hi_min:02x}",
        f"0x{lo_max:02x}", f"0x{hi_max:02x}",
        "0x00", "0x00", "0x00", "0x00", "0x00", "0x00",
        "0x00", "0x00", "0x00", "0x07", "0x00",
    )


def refresh_legacy_hci(hci_dev: str, name: str) -> bool:
    hci_le_cmd(hci_dev, "0x000a", "0x00", quiet=True)
    hci_le_cmd(hci_dev, "0x0039", "0x00", "0x00", quiet=True)
    time.sleep(0.05)
    if not _set_params(hci_dev):
        return False
    adv, sr = build_adv_payload(name), build_scan_rsp(name)
    if not hci_le_cmd(hci_dev, "0x0008", *_pad31(adv)):
        return False
    if not hci_le_cmd(hci_dev, "0x0009", *_pad31(sr), quiet=True):
        pass
    return hci_le_cmd(hci_dev, "0x000a", "0x01")


def start_legacy_hci(hci_dev: str, name: str) -> bool:
    return refresh_legacy_hci(hci_dev, name)


def _u24(n: int) -> List[str]:
    return [f"0x{n & 0xFF:02x}", f"0x{(n >> 8) & 0xFF:02x}", f"0x{(n >> 16) & 0xFF:02x}"]


def _ext_data_args(payload: bytes) -> List[str]:
    # handle=1, operation=Complete, fragment_pref=Host no preference, len, data
    return [
        "0x01",
        "0x03",
        "0x01",
        f"0x{len(payload):02x}",
        *[f"0x{b:02x}" for b in payload],
    ]


def refresh_extended_hci(hci_dev: str, name: str) -> bool:
    """BT5 Extended Advertising，Event Properties 带 Use Legacy PDU → 空中仍是 ADV_IND。"""
    hci_le_cmd(hci_dev, "0x000a", "0x00", quiet=True)
    hci_le_cmd(hci_dev, "0x0039", "0x00", "0x00", quiet=True)
    hci_le_cmd(hci_dev, "0x003c", "0x01", quiet=True)
    time.sleep(0.05)
    # handle 1, properties 0x0013 = Connectable | Scannable | Legacy PDUs
    params = [
        "0x01",
        "0x13", "0x00",
        *_u24(ADV_INTERVAL_MIN),
        *_u24(ADV_INTERVAL_MAX),
        "0x07",
        "0x00",
        "0x00",
        "0x00", "0x00", "0x00", "0x00", "0x00", "0x00",
        "0x00",
        "0x7f",
        "0x01",
        "0x00",
        "0x01",
        "0x00",
        "0x00",
    ]
    if not hci_le_cmd(hci_dev, "0x0036", *params):
        return False
    adv, sr = build_adv_payload(name), build_scan_rsp(name)
    if not hci_le_cmd(hci_dev, "0x0037", *_ext_data_args(adv)):
        return False
    hci_le_cmd(hci_dev, "0x0038", *_ext_data_args(sr), quiet=True)
    return hci_le_cmd(
        hci_dev, "0x0039", "0x01", "0x01", "0x01", "0x00", "0x00", "0x00"
    )


def start_extended_hci(hci_dev: str, name: str) -> bool:
    return refresh_extended_hci(hci_dev, name)


def start_hciconfig_leadv(hci_dev: str) -> bool:
    r = _run(["hciconfig", hci_dev, "leadv", "0"], timeout=3.0)
    out = ((r.stdout or "") + (r.stderr or "")).lower()
    return r.returncode == 0 and "not permitted" not in out and "failed" not in out


def start_btmgmt_adv(hci_dev: str, name: str, force: bool = False) -> bool:
    idx = hci_dev.replace("hci", "")
    adv, sr = build_adv_payload(name), build_scan_rsp(name)
    if force:
        btmgmt(hci_dev, "rm-adv", "1")
        btmgmt(hci_dev, "rm-adv", "2")
    r = _run(
        ["btmgmt", "--index", idx, "add-adv", "-d", adv.hex(), "-s", sr.hex(), "-c", "1"],
        timeout=6,
    )
    if r.returncode != 0:
        return False
    r2 = _run(["btmgmt", "--index", idx, "advertising", "on"])
    out = ((r2.stdout or "") + (r2.stderr or "")).lower()
    return "failed" not in out and "busy" not in out


def stop_adv_keeper() -> None:
    global _keeper_thread
    _keeper_stop.set()
    th = _keeper_thread
    if th is not None and th.is_alive():
        th.join(timeout=2.0)
    _keeper_thread = None


def start_adv_keeper(
    hci_dev: str,
    name: str,
    platform: Optional[PlatformInfo] = None,
    interval_sec: float = 10.0,
    log: Optional[LogFn] = None,
) -> None:
    global _keeper_thread
    stop_adv_keeper()
    _keeper_stop.clear()
    plat = platform or detect_platform()

    def _loop() -> None:
        while not _keeper_stop.wait(interval_sec):
            if plat.skip_btmgmt or plat.adv_mode == "legacy_hci":
                if _ADV_BACKEND == "extended":
                    ok = refresh_extended_hci(hci_dev, name)
                elif _ADV_BACKEND == "hciconfig":
                    ok = start_hciconfig_leadv(hci_dev)
                else:
                    ok = refresh_legacy_hci(hci_dev, name) or refresh_extended_hci(
                        hci_dev, name
                    )
            else:
                ok = start_btmgmt_adv(hci_dev, name, force=True) or refresh_legacy_hci(
                    hci_dev, name
                )
            if log and not ok:
                log(f"[adv] 保活重发失败: {name}")

    _keeper_thread = threading.Thread(target=_loop, daemon=True, name="ble-adv-keeper")
    _keeper_thread.start()


def start_advertising(
    hci_dev: str,
    name: str,
    platform: Optional[PlatformInfo] = None,
    force: bool = False,
    log: Optional[LogFn] = None,
    keepalive: bool = True,
) -> bool:
    global _ADV_BACKEND
    plat = platform or detect_platform()
    env_name = os.environ.get("BLE_DEVICE_NAME", "").strip()
    if env_name:
        name = env_name
    prepare_adapter(hci_dev, name, plat)

    if plat.skip_btmgmt or plat.adv_mode == "legacy_hci":
        ok = start_legacy_hci(hci_dev, name)
        if ok:
            _ADV_BACKEND = "legacy"
            msg = f"[adv] AIC/板载 Legacy HCI 广播已开启 ({plat.hw_desc}): {name}"
        else:
            ok = start_extended_hci(hci_dev, name)
            if ok:
                _ADV_BACKEND = "extended"
                msg = f"[adv] AIC Extended HCI(legacy PDU) 广播已开启: {name}"
            else:
                ok = start_hciconfig_leadv(hci_dev)
                _ADV_BACKEND = "hciconfig" if ok else "legacy"
                msg = (
                    f"[adv] hciconfig leadv 广播已开启: {name}"
                    if ok
                    else f"[adv] AIC HCI 广播全部失败: {name}"
                )
        _adv_log(msg, log)
        if ok and keepalive:
            interval = 2.0 if plat.skip_btmgmt else 10.0
            start_adv_keeper(hci_dev, name, plat, interval_sec=interval, log=log)
        return ok

    ok = start_btmgmt_adv(hci_dev, name, force=force)
    if not ok:
        ok = start_legacy_hci(hci_dev, name)
    if log:
        log(
            f"[adv] USB 蓝牙广播已开启 ({plat.hw_desc}): {name}"
            if ok else f"[adv] USB 蓝牙广播失败: {name}"
        )
    if ok and keepalive:
        start_adv_keeper(hci_dev, name, plat, log=log)
    return ok
