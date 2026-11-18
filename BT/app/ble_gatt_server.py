#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
microDuck BLE GATT 从机：微信小程序连接，FFE1 收指令，FFE2 回 ACK / IP / 电量。
保留：连接、IP、音量、WiFi 配网、改名。
已移除：动作调用、拖拽、ROS 遥控。
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import threading
import time
from typing import List, Optional

import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib

# ----- 量产 BLE 服务 FFE0 -----
SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
WRITE_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
NOTIFY_CHAR_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"

_APP_DIR = os.path.dirname(os.path.abspath(__file__))
if _APP_DIR not in sys.path:
    sys.path.insert(0, _APP_DIR)
# 包根目录（ble_device_name.conf / docs），发布包与源码均为 app/ 的上一级
_PKG_DIR = os.environ.get("PKG_DIR") or (
    os.path.dirname(_APP_DIR)
    if os.path.isfile(os.path.join(os.path.dirname(_APP_DIR), "ble_device_name.conf"))
    else _APP_DIR
)
_DUCK_WS = os.environ.get("DUCK_WS") or os.path.abspath(os.path.join(_PKG_DIR, ".."))
if _DUCK_WS not in sys.path:
    sys.path.insert(0, _DUCK_WS)

try:
    from ble_device_name import (
        DEFAULT_BLE_NAME,
        load_ble_name,
        persistent_name_file,
        save_ble_name,
    )
except ImportError:
    DEFAULT_BLE_NAME = "HT_88888888"

    def load_ble_name() -> str:
        return DEFAULT_BLE_NAME

    def save_ble_name(name: str) -> None:
        pass

try:
    from platform_detect import detect_platform
    from ble_legacy_adv import (
        start_advertising as _start_ble_advertising,
        stop_advertising as _stop_ble_advertising,
        stop_adv_keeper as _stop_adv_keeper,
        btmgmt as _btmgmt_adv,
    )
except ImportError:
    detect_platform = None  # type: ignore
    _start_ble_advertising = None  # type: ignore
    _stop_ble_advertising = None  # type: ignore
    _stop_adv_keeper = None  # type: ignore
    _btmgmt_adv = None  # type: ignore

BOARD_BDADDR = "00:00:00:00:00:00"
ADAPTER_PATH = "/org/bluez/hci0"
BLUEZ_SERVICE = "org.bluez"
GATT_MANAGER_IFACE = "org.bluez.GattManager1"
LE_ADV_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"
DBUS_OM_IFACE = "org.freedesktop.DBus.ObjectManager"
DBUS_PROP_IFACE = "org.freedesktop.DBus.Properties"
LE_ADV_IFACE = "org.bluez.LEAdvertisement1"
GATT_SERVICE_IFACE = "org.bluez.GattService1"
GATT_CHAR_IFACE = "org.bluez.GattCharacteristic1"
GATT_DESC_IFACE = "org.bluez.GattDescriptor1"
DEVICE_IFACE = "org.bluez.Device1"
ADAPTER_IFACE = "org.bluez.Adapter1"

LINK_IDLE_TIMEOUT_SEC = 30.0
LINK_WATCHDOG_TICK_MS = 1000
CONNECT_ATTEMPT_LOG_INTERVAL_SEC = 5.0
STICK_LOG_RE = re.compile(
    r"^X:\s*[+-]?\d+(?:\.\d+)?\s*,\s*Y:\s*[+-]?\d+(?:\.\d+)?\s*,\s*Z:\s*[+-]?\d+(?:\.\d+)?"
    r"(?:\s*,\s*N:\s*\d+)?$",
    re.IGNORECASE,
)


def _dbus_sv_opts() -> dbus.Dictionary:
    """BlueZ Register* 的 options 须为 a{sv}，空 dict 不能直接用 {}。"""
    return dbus.Dictionary({}, signature="sv")


from ble_log import log_info, log_rx, log_tx, log_warn


class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.freedesktop.DBus.Error.InvalidArgs"


class NotSupportedException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.bluez.Error.NotSupported"


class Application(dbus.service.Object):
    def __init__(self, bus: dbus.SystemBus):
        self.path = "/org/microduck/ble"
        self.services: List[dbus.service.Object] = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self) -> str:
        return dbus.ObjectPath(self.path)

    def add_service(self, service: dbus.service.Object) -> None:
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            for chrc in service.get_characteristics():
                response[chrc.get_path()] = chrc.get_properties()
                for desc in chrc.get_descriptors():
                    response[desc.get_path()] = desc.get_properties()
        return response


class Service(dbus.service.Object):
    PATH_BASE = "/org/microduck/ble/service"

    def __init__(self, bus: dbus.SystemBus, index: int, uuid: str, primary: bool):
        self.path = f"{self.PATH_BASE}{index}"
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics: List[dbus.service.Object] = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self) -> str:
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, chrc: dbus.service.Object) -> None:
        self.characteristics.append(chrc)

    def get_characteristics(self) -> List[dbus.service.Object]:
        return self.characteristics

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                "UUID": self.uuid,
                "Primary": self.primary,
                "Characteristics": dbus.Array(
                    [c.get_path() for c in self.characteristics], signature="o"
                ),
            }
        }


class Characteristic(dbus.service.Object):
    PATH_BASE = "/org/microduck/ble/char"

    def __init__(
        self,
        bus: dbus.SystemBus,
        index: int,
        uuid: str,
        flags: List[str],
        service: Service,
        on_write=None,
        on_notify_start=None,
        on_notify_stop=None,
    ):
        self.path = f"{self.PATH_BASE}{index}"
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.descriptors: List[dbus.service.Object] = []
        self._value = bytearray(b"Duck BLE ready")
        self._on_write = on_write
        self._on_notify_start = on_notify_start
        self._on_notify_stop = on_notify_stop
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self) -> str:
        return dbus.ObjectPath(self.path)

    def get_descriptors(self) -> List[dbus.service.Object]:
        return self.descriptors

    def get_properties(self):
        return {
            GATT_CHAR_IFACE: {
                "Service": self.service.get_path(),
                "UUID": self.uuid,
                "Flags": self.flags,
                "Value": dbus.Array(self._value, signature="y"),
            }
        }

    @dbus.service.method(GATT_CHAR_IFACE, in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        if self._on_write:
            self._on_write(bytes(self._value), {"event": "read"})
        return self._value

    @dbus.service.method(GATT_CHAR_IFACE, in_signature="aya{sv}")
    def WriteValue(self, value, options):
        data = bytes(value)
        self._value = bytearray(data)
        if self._on_write:
            self._on_write(data, options)

    @dbus.service.method(GATT_CHAR_IFACE)
    def StartNotify(self):
        if self._on_notify_start:
            self._on_notify_start()

    @dbus.service.method(GATT_CHAR_IFACE)
    def StopNotify(self):
        if self._on_notify_stop:
            self._on_notify_stop()

    def notify(self, data: bytes) -> None:
        if "notify" not in self.flags and "indicate" not in self.flags:
            return
        self._value = bytearray(data)
        self.PropertiesChanged(
            GATT_CHAR_IFACE,
            {"Value": dbus.Array(self._value, signature="y")},
            [],
        )

    @dbus.service.signal(DBUS_PROP_IFACE, signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass


class Advertisement(dbus.service.Object):
    PATH_BASE = "/org/microduck/ble/advertisement"

    def __init__(self, bus: dbus.SystemBus, index: int, local_name: str):
        self.path = f"{self.PATH_BASE}{index}"
        self.local_name = local_name
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        # FFE0 用 16-bit UUID，才能和名称一起塞进 31 字节广播包
        return {
            LE_ADV_IFACE: {
                "Type": "peripheral",
                "LocalName": self.local_name,
                "ServiceUUIDs": dbus.Array(
                    ["0000ffe0-0000-1000-8000-00805f9b34fb"], signature="s"
                ),
                "IncludeTxPower": dbus.Boolean(False),
            }
        }

    def get_path(self) -> dbus.ObjectPath:
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != LE_ADV_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[LE_ADV_IFACE]

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="ss", out_signature="v")
    def Get(self, interface, prop):
        props = self.GetAll(interface)
        if prop not in props:
            raise InvalidArgsException()
        return props[prop]

    @dbus.service.method(LE_ADV_IFACE, in_signature="", out_signature="")
    def Release(self):
        log_info("BLE 广播已释放")
        # BlueZ 主动 Release 时标记为未注册，断连恢复逻辑会重新注册
        server = getattr(self, "_server", None)
        if server is not None:
            server._adv_registered = False



class BleGattServer:
    def __init__(
        self,
        adapter: str,
        name: str,
        echo: bool,
        ros_control: bool = False,
    ):
        self.adapter = adapter
        self.name = name
        self.echo = echo
        self.ros_control = False
        self._notify_chrc: Optional[Characteristic] = None
        self._write_chrc: Optional[Characteristic] = None
        self._msg_count = 0
        self._connected_devices: set = set()
        self._adapter_path = ""
        self._volume = None
        self._wifi = None
        self._dispatcher = None
        self._telemetry = None
        self._voice = None
        self._conversation = None
        self._ble_ready_announced = False
        self._connect_hint_ids: List[int] = []
        self._msg_count_at_connect = 0
        self._adv: Optional[Advertisement] = None
        self._adv_manager = None
        self._adv_index = 0
        self._adv_reregister_pending = False
        self._adv_registered = False
        self._adv_stop_pending = False
        self._notify_active = False
        self._pending_notifies: List[bytes] = []
        self._link_last_downlink_ts = 0.0
        self._link_watchdog_source_id: Optional[int] = None
        self._link_watchdog_disconnect_pending = False
        self._last_connect_attempt_log_ts: dict[str, float] = {}
        self._notify_enabled = False
        self._ready_reported = False

        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self.bus = dbus.SystemBus()
        self.mainloop = GLib.MainLoop()

    def _device_label(self, path: str) -> str:
        try:
            props = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE, path), DBUS_PROP_IFACE
            )
            values = props.GetAll(DEVICE_IFACE)
            name = str(values.get("Name", "") or values.get("Alias", "") or "未知设备")
            addr = str(values.get("Address", path))
            return f"{name} ({addr})"
        except dbus.exceptions.DBusException:
            return path

    def _note_connect_attempt(self, path: str, trigger: str) -> None:
        now = time.monotonic()
        last = self._last_connect_attempt_log_ts.get(path, 0.0)
        if now - last < CONNECT_ATTEMPT_LOG_INTERVAL_SEC:
            return
        self._last_connect_attempt_log_ts[path] = now
        label = self._device_label(path)
        log_info(f"[conn] 发现连接尝试({trigger}): {label}")

    def _schedule_connect_hint(self) -> None:
        self._msg_count_at_connect = self._msg_count
        hint_id = GLib.timeout_add(5000, self._connect_no_data_hint)
        self._connect_hint_ids.append(hint_id)

    def _connect_no_data_hint(self) -> bool:
        if self._msg_count == self._msg_count_at_connect:
            log_warn("[tip] 已连接 5s 仍无 FFE1 写入 — 见 BLE_PROTOCOL.md")
            log_info("      小程序应订阅 FFE2；可选心跳 write: 1")
            log_info(f"      写入 UUID: {WRITE_CHAR_UUID}")
        return False

    def _start_link_watchdog(self) -> None:
        self._touch_link_watchdog()
        if self._link_watchdog_source_id is not None:
            return
        self._link_watchdog_source_id = GLib.timeout_add(
            LINK_WATCHDOG_TICK_MS, self._link_watchdog_cb
        )

    def _stop_link_watchdog(self) -> None:
        source_id = self._link_watchdog_source_id
        if source_id is not None:
            try:
                GLib.source_remove(source_id)
            except Exception:
                pass
        self._link_watchdog_source_id = None
        self._link_watchdog_disconnect_pending = False
        self._link_last_downlink_ts = 0.0

    def _touch_link_watchdog(self) -> None:
        self._link_last_downlink_ts = time.monotonic()

    def _disconnect_device(self, path: str, reason: str) -> None:
        try:
            dev = dbus.Interface(self.bus.get_object(BLUEZ_SERVICE, path), DEVICE_IFACE)
            dev.Disconnect()
            log_warn(f"[link] 主动断开 BLE 连接: {self._device_label(path)} | {reason}")
        except dbus.exceptions.DBusException as e:
            log_warn(f"[link] 主动断开失败: {self._device_label(path)} | {e}")

    def _disconnect_connected_devices(self, reason: str) -> None:
        for path in list(self._connected_devices):
            self._disconnect_device(path, reason)

    def _link_watchdog_cb(self) -> bool:
        if not self._connected_devices:
            self._link_watchdog_source_id = None
            self._link_watchdog_disconnect_pending = False
            return False
        if self._link_watchdog_disconnect_pending:
            return True
        last = self._link_last_downlink_ts
        if last <= 0.0:
            return True
        idle = time.monotonic() - last
        if idle < LINK_IDLE_TIMEOUT_SEC:
            return True
        self._link_watchdog_disconnect_pending = True
        self._disconnect_connected_devices(
            f"{idle:.1f}s 未收到下行指令/心跳 1（阈值 {LINK_IDLE_TIMEOUT_SEC:.1f}s）"
        )
        return True

    def _trust_device(self, path: str) -> None:
        """部分手机（尤其 Android）未 Trusted 时 GATT 写入/订阅会失败。"""
        try:
            props = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE, path), DBUS_PROP_IFACE
            )
            props.Set(DEVICE_IFACE, "Trusted", dbus.Boolean(True))
        except dbus.exceptions.DBusException:
            pass

    def _on_phone_connected(self, path: str) -> None:
        if path in self._connected_devices:
            return
        self._connected_devices.add(path)
        self._last_connect_attempt_log_ts.pop(path, None)
        self._link_watchdog_disconnect_pending = False
        # 小程序可能先订阅 FFE2，Connected 才到。这时不要把已订阅清掉。
        notifying = self._notify_active
        self._notify_enabled = notifying
        self._ready_reported = False
        self._trust_device(path)
        label = self._device_label(path)
        log_info(f"*** 手机已连接: {label} ***")
        log_info("    等待订阅 FFE2，随后上报 IP / pwr（无模式握手）")
        log_info(f"    写入 UUID: {WRITE_CHAR_UUID}")
        if self._voice is not None:
            self._voice.on_ble_connected()
        self._schedule_connect_hint()
        self._start_link_watchdog()
        if notifying:
            if self._telemetry is not None:
                self._telemetry.on_subscribed(emit_snapshot=False)
            self._try_report_ready()
            self._schedule_ip_burst()
        # AIC 上连接后立刻停广播容易打断 GATT 发现/订阅，延后到握手完成
        GLib.timeout_add(2500, self._deferred_stop_adv_cb)

    def _deferred_stop_adv_cb(self) -> bool:
        if self._connected_devices:
            self._stop_advertising_while_connected()
        return False

    def _stop_advertising_while_connected(self) -> None:
        """连接后停止 BLE 广播，避免其他手机扫到并抢占连接。"""
        if self._adv_stop_pending:
            return
        self._adv_stop_pending = True
        self._adv_reregister_pending = False

        hci_dev = self._adapter_path.split("/")[-1] if self._adapter_path else "hci0"
        plat = detect_platform() if detect_platform else None
        log_info("已连接：停止 BLE 广播（断连后自动恢复可扫描）")

        def _stop_hw() -> None:
            if self._uses_dbus_adv():
                return
            try:
                if _stop_adv_keeper is not None:
                    _stop_adv_keeper()
                if _stop_ble_advertising is not None:
                    _stop_ble_advertising(hci_dev)
                if (
                    plat is not None
                    and plat.adv_mode != "legacy_hci"
                    and _btmgmt_adv is not None
                ):
                    _btmgmt_adv(hci_dev, "advertising", "off")
                    _btmgmt_adv(hci_dev, "rm-adv", "1")
                    _btmgmt_adv(hci_dev, "rm-adv", "2")
            except Exception as e:
                log_warn(f"停止 BLE 广播失败: {e}")

        threading.Thread(target=_stop_hw, daemon=True).start()

        if not self._adv_registered or self._adv_manager is None or self._adv is None:
            return

        def _on_unregistered() -> None:
            self._adv_registered = False
            log_info("已连接：D-Bus 广播已注销")

        def _on_unregister_failed(error: dbus.DBusException) -> None:
            err = self._dbus_error_text(error)
            if "DoesNotExist" in err or "NotFound" in err:
                self._adv_registered = False
                return
            log_warn(f"注销 D-Bus 广播失败: {err}")

        try:
            self._adv_manager.UnregisterAdvertisement(
                self._adv.get_path(),
                reply_handler=_on_unregistered,
                error_handler=_on_unregister_failed,
            )
        except dbus.exceptions.DBusException as e:
            err = self._dbus_error_text(e)
            if "DoesNotExist" in err or "NotFound" in err:
                self._adv_registered = False
            else:
                log_warn(f"UnregisterAdvertisement 异常: {err}")

    def _on_phone_disconnected(self, path: str) -> None:
        if path not in self._connected_devices:
            return
        label = self._device_label(path)
        self._connected_devices.discard(path)
        self._last_connect_attempt_log_ts.pop(path, None)
        log_info(f"--- 手机已断开: {label} ---")
        if self._voice is not None:
            self._voice.on_ble_disconnected()
        if self._dispatcher is not None:
            self._dispatcher.on_disconnect()
        if not self._connected_devices:
            self._ble_ready_announced = False
            self._notify_enabled = False
            self._ready_reported = False
            self._stop_link_watchdog()
            self._notify_active = False
            self._pending_notifies.clear()
            self._adv_stop_pending = False
            if self._telemetry is not None:
                self._telemetry.on_unsubscribed()
            self._schedule_restart_advertising()
            GLib.timeout_add(2000, self._ble_ready_timeout_cb)

    @staticmethod
    def _dbus_error_text(error: dbus.DBusException) -> str:
        try:
            return f"{error.get_dbus_name()}: {error}"
        except Exception:
            return str(error)

    @staticmethod
    def _is_adv_already_exists(error: dbus.DBusException) -> bool:
        text = BleGattServer._dbus_error_text(error)
        return "AlreadyExists" in text or "Already Exists" in text

    def _refresh_advertising_after_disconnect(self, hci_dev: str = "hci0") -> None:
        """断连后恢复可扫描。Realtek 禁止 btmgmt advertising on。"""
        self._run_btmgmt("le", "on")
        self._run_btmgmt("connectable", "on")

        def _worker() -> None:
            try:
                self._patch_le_adv_data(hci_dev=hci_dev, force=True, initial_delay=0.2)
                log_info(f"断连后 BLE 广播已恢复: {self.name}")
            except Exception as e:
                log_warn(f"断连后刷新广播包失败: {e}")

        threading.Thread(target=_worker, daemon=True).start()

    def _schedule_restart_advertising(self) -> None:
        """断连后延迟恢复 BLE 广播，便于小程序重新扫描连接。"""
        if self._connected_devices or self._adv is None:
            return
        if self._adv_reregister_pending:
            return
        self._adv_reregister_pending = True
        GLib.timeout_add(400, self._restart_advertising_cb)

    def _restart_advertising_cb(self) -> bool:
        self._adv_reregister_pending = False
        if self._connected_devices or self._adv is None:
            return False

        hci_dev = self._adapter_path.split("/")[-1] if self._adapter_path else "hci0"
        plat = detect_platform() if detect_platform else None

        if plat is not None and plat.adv_mode == "legacy_hci":
            log_info("断连后恢复 Legacy HCI 广播...")
            self._refresh_advertising_after_disconnect(hci_dev)
            return False

        # Orin：断连后 BlueZ 仍可能保留 Advertisement 注册
        if self._adv_registered or self._adv_manager is None:
            log_info("断连后恢复 BLE 可扫描状态...")
            self._refresh_advertising_after_disconnect(hci_dev)
            return False

        def adv_done() -> None:
            self._adv_registered = True
            log_info(f"断连后 BLE 广播已重新注册: {self.name}")
            self._refresh_advertising_after_disconnect(hci_dev)

        def adv_failed(error: dbus.DBusException) -> None:
            if self._is_adv_already_exists(error):
                self._adv_registered = True
                log_info("断连后 BLE 广播仍注册，刷新广播包")
                self._refresh_advertising_after_disconnect(hci_dev)
                return
            log_warn(f"断连后广播恢复失败: {self._dbus_error_text(error)}，尝试注销后重注册")
            self._reregister_advertisement(hci_dev)

        try:
            self._adv_manager.RegisterAdvertisement(
                self._adv.get_path(),
                _dbus_sv_opts(),
                reply_handler=adv_done,
                error_handler=adv_failed,
            )
        except dbus.exceptions.DBusException as e:
            if self._is_adv_already_exists(e):
                self._adv_registered = True
                log_info("断连后 BLE 广播仍注册，刷新广播包")
                self._refresh_advertising_after_disconnect(hci_dev)
            else:
                log_warn(f"断连后 RegisterAdvertisement 异常: {self._dbus_error_text(e)}")
                self._reregister_advertisement(hci_dev)
        return False

    def _reregister_advertisement(self, hci_dev: str) -> None:
        if self._adv is None or self._adv_manager is None:
            return

        def register_again() -> None:
            def on_registered() -> None:
                self._adv_registered = True
                log_info(f"断连后 BLE 广播已重新注册: {self.name}")
                self._refresh_advertising_after_disconnect(hci_dev)

            def on_register_failed(err: dbus.DBusException) -> None:
                log_warn(
                    f"断连后重注册广播仍失败: {self._dbus_error_text(err)}"
                )
                GLib.timeout_add(3000, self._restart_advertising_cb)

            try:
                self._adv_manager.RegisterAdvertisement(
                    self._adv.get_path(),
                    _dbus_sv_opts(),
                    reply_handler=on_registered,
                    error_handler=on_register_failed,
                )
            except dbus.exceptions.DBusException as e:
                log_warn(
                    f"断连后 RegisterAdvertisement 异常: {self._dbus_error_text(e)}"
                )
                GLib.timeout_add(3000, self._restart_advertising_cb)

        def unregister_failed(error: dbus.DBusException) -> None:
            err = self._dbus_error_text(error)
            if "DoesNotExist" in err or "NotFound" in err:
                register_again()
                return
            log_warn(f"断连后注销广播失败: {err}")
            GLib.timeout_add(3000, self._restart_advertising_cb)

        try:
            self._adv_manager.UnregisterAdvertisement(
                self._adv.get_path(),
                reply_handler=register_again,
                error_handler=unregister_failed,
            )
            self._adv_registered = False
        except dbus.exceptions.DBusException as e:
            err = self._dbus_error_text(e)
            if "DoesNotExist" in err or "NotFound" in err:
                register_again()
            else:
                log_warn(f"断连后 UnregisterAdvertisement 异常: {err}")
                GLib.timeout_add(3000, self._restart_advertising_cb)

    def _on_device_props_changed(
        self, interface: str, changed, invalidated, path: str = ""
    ) -> None:
        if interface != DEVICE_IFACE:
            return
        if not str(path).startswith(self._adapter_path):
            return
        if "Connected" in changed:
            if bool(changed["Connected"]):
                self._on_phone_connected(path)
            else:
                self._on_phone_disconnected(path)
            return
        if any(k in changed for k in ("ServicesResolved", "RSSI", "UUIDs", "Name", "Alias")):
            self._note_connect_attempt(path, "props")

    def _on_interfaces_added(self, path, interfaces) -> None:
        if DEVICE_IFACE not in interfaces:
            return
        if not str(path).startswith(self._adapter_path):
            return
        dev = interfaces[DEVICE_IFACE]
        self._note_connect_attempt(path, "added")
        if dev.get("Connected", False):
            self._on_phone_connected(path)

    def _on_interfaces_removed(self, path, interfaces) -> None:
        if DEVICE_IFACE not in interfaces:
            return
        self._on_phone_disconnected(path)

    def _watch_devices(self) -> None:
        self.bus.add_signal_receiver(
            self._on_interfaces_added,
            dbus_interface=DBUS_OM_IFACE,
            signal_name="InterfacesAdded",
        )
        self.bus.add_signal_receiver(
            self._on_interfaces_removed,
            dbus_interface=DBUS_OM_IFACE,
            signal_name="InterfacesRemoved",
        )
        self.bus.add_signal_receiver(
            self._on_device_props_changed,
            dbus_interface=DBUS_PROP_IFACE,
            signal_name="PropertiesChanged",
            path_keyword="path",
        )

    def _on_write(self, data: bytes, options) -> None:
        opt = dict(options) if options else {}
        if opt.get("event") == "read":
            log_info(f"手机读取 FFE1 ({len(data)} bytes) — 须 write")
            return

        self._msg_count += 1
        self._touch_link_watchdog()
        self._link_watchdog_disconnect_pending = False

        try:
            preview = data.decode("utf-8", errors="replace").strip()[:80]
        except Exception:
            preview = data[:24].hex()
        # 摇杆速度包频率高，日志刷屏，默认不打印。
        is_stick = bool(preview and STICK_LOG_RE.match(preview))
        if preview and not is_stick and not (len(data) >= 3 and data[0] == 0x0B):
            log_rx(f"FFE1 ← {preview}")

        # 兼容历史语音包：首字节 0x0B 的二进制数据直接忽略，不参与控制。
        if len(data) >= 3 and data[0] == 0x0B:
            return

        if self._dispatcher is not None:
            self._dispatcher.dispatch(data)

    def _notify_on_main_thread(self, data: bytes) -> None:
        """FFE2 notify 必须在 GLib 主线程发送，避免跨线程 D-Bus 导致断联。"""
        GLib.idle_add(self._do_notify, data)

    def _do_notify(self, data: bytes) -> bool:
        if self._notify_chrc is None:
            return False
        if not self._notify_active:
            self._pending_notifies.append(data)
            if len(self._pending_notifies) > 16:
                self._pending_notifies.pop(0)
            return False
        self._notify_chrc.notify(data)
        return False

    def _flush_pending_notifies(self) -> None:
        pending = list(self._pending_notifies)
        self._pending_notifies.clear()
        for payload in pending:
            if self._notify_chrc is not None:
                self._notify_chrc.notify(payload)

    def _send_ack(self, wire: str) -> None:
        if not self.echo or self._notify_chrc is None:
            return
        upper = wire.strip().upper()
        if upper in ("GAIT ON", "GAIT OFF"):
            return
        from ble_command_dispatcher import make_notify_reply

        payload = make_notify_reply(wire)
        self._notify_on_main_thread(payload)
        log_tx(f"ACK:{wire}")

    def _send_command_echo(self, wire: str) -> None:
        """FFE2 回传上行原文（无 ACK: 等前缀）。"""
        if not self.echo or self._notify_chrc is None:
            return
        plain = wire.strip()
        payload = plain.encode("utf-8", errors="replace")[:180]
        self._notify_on_main_thread(payload)
        log_tx(plain)

    def _send_plain_notify(self, text: str) -> None:
        if self._notify_chrc is None:
            return
        body = text.strip()
        payload = body.encode("utf-8", errors="replace")[:180]
        self._notify_on_main_thread(payload)
        log_tx(body)

    def _try_report_ready(self) -> None:
        if not self._connected_devices:
            return
        if self._ready_reported:
            return
        if not self._notify_enabled:
            log_info("[ready] 等待 FFE2 订阅后再上报 IP/pwr")
            return
        pct = self._read_battery_pct()
        self._send_plain_notify(f"pwr:{max(0, min(100, int(pct)))}")
        self._ready_reported = True
        log_info("[ready] 已上报 pwr（不含 mode/fsm）")
        self._stop_advertising_while_connected()

    def _schedule_ip_burst(self) -> None:
        """每次 FFE2 订阅都重发 IP。微信会先订一次再立刻退订重订，第一次发出的包会被丢掉。"""
        if not self._connected_devices or not self._notify_enabled:
            return
        if self._telemetry is None:
            return
        self._telemetry.push_ip_burst()

    def _handle_wifi(self, payload: str) -> None:
        """WIFI 配网：解析 SSID/密码 → nmcli 保存并连接 → FFE2 回最终结果。"""
        from ble_wifi_manager import WIFI_PAYLOAD_SEP, format_wifi_result

        if self._wifi is None:
            log_warn("[wifi] 管理器未就绪")
            self._send_command_echo(format_wifi_result(False, "unavailable"))
            return
        if WIFI_PAYLOAD_SEP not in (payload or ""):
            log_warn("[wifi] 载荷格式错误")
            self._send_command_echo(format_wifi_result(False, "bad_format"))
            return
        ssid, password = payload.split(WIFI_PAYLOAD_SEP, 1)
        ssid = (ssid or "").strip()
        password = password or ""
        if not ssid or not password:
            self._send_command_echo(format_wifi_result(False, "bad_format"))
            return

        if self._voice is not None:
            self._voice.on_wifi_changed()

        def _on_result(ok: bool, detail: str) -> None:
            if ok:
                self._notify_wifi_reconnected()
            else:
                self._send_command_echo(format_wifi_result(False, detail))
                log_warn(f"[wifi] 配网失败: {detail}")

        if not self._wifi.connect_async(ssid, password, on_result=_on_result):
            log_warn("[wifi] 已有配网任务进行中")
            self._send_command_echo(format_wifi_result(False, "busy"))

    def _notify_wifi_disconnected(self) -> None:
        from ble_wifi_manager import RESULT_DISCONNECTED

        self._send_command_echo(RESULT_DISCONNECTED)
        if self._voice is not None:
            self._voice.on_wifi_disconnected()

    def _notify_wifi_reconnected(self) -> None:
        from ble_wifi_manager import RESULT_OK

        self._send_command_echo(RESULT_OK)
        if self._voice is not None:
            self._voice.on_wifi_connected()
        self._send_wifi_ip()

    def _send_wifi_ip(self) -> None:
        """DHCP 可能晚于关联，轮询后再单独推 IP:x.x.x.x。"""
        try:
            from ble_status_telemetry import read_lan_ip
        except ImportError:
            read_lan_ip = None  # type: ignore
        ip = None
        if read_lan_ip is not None:
            for _ in range(20):
                ip = read_lan_ip()
                if ip:
                    break
                time.sleep(0.5)
        if ip:
            self._send_command_echo(f"IP:{ip}")
            if self._telemetry is not None:
                try:
                    with self._telemetry._lock:
                        self._telemetry._last_ip = ip
                except Exception:
                    pass
            # IP 只走 FFE2 上报，不再语音播报，避免和驾驶鸭叫抢喇叭。
            log_info(f"[wifi] 局域网 IP {ip}（语音播报已关闭）")
        else:
            log_warn("[wifi] 已连接但尚未拿到局域网 IP")

    def _handle_dispatched(self, kind, payload: str) -> None:
        if kind.value in ("heartbeat", "stick", "ignored"):
            return
        if kind.value == "handshake":
            log_info("[ble] 收到 M_default，仅 ACK，不做模式切换")
            self._try_report_ready()
            return
        if kind.value == "volume":
            if self._volume is not None:
                self._volume.handle(payload)
            return
        if kind.value == "rename":
            echo = f"rename {payload}"
            if self.rename_ble_device(payload):
                self._send_command_echo(echo)
            else:
                log_warn(f"BLE 改名失败: {payload}")
            return
        if kind.value == "wifi":
            self._handle_wifi(payload)
            return
        if kind.value == "conversation":
            bag = self._conversation
            if bag is None and self._voice is not None:
                bag = getattr(self._voice, "conversation", None)
            if bag is not None:
                bag.play(payload)
            else:
                log_warn("conversation_bag 未加载")
            return

    def _hci_index(self, hci_dev: str = "hci0") -> str:
        return hci_dev.replace("hci", "")

    def _build_le_adv_payload(self, name: Optional[str] = None) -> bytes:
        """构造 LE 广播 AD：Flags + 16-bit UUID(FFE0) + Complete Local Name。"""
        name_b = (name or self.name).encode("utf-8")
        base = bytes([0x02, 0x01, 0x06, 0x03, 0x03, 0xE0, 0xFF])
        budget = 31 - len(base) - 2
        name_b = name_b[: max(0, budget)]
        payload = base + bytes([len(name_b) + 1, 0x09]) + name_b
        return payload[:31]

    def _build_le_scan_rsp_payload(self, name: Optional[str] = None) -> bytes:
        name_b = (name or self.name).encode("utf-8")[:25]
        return bytes([len(name_b) + 1, 0x09]) + name_b

    def _btmgmt_set_local_name(self, hci_idx: str, name: str) -> None:
        if self._skip_btmgmt():
            return
        try:
            subprocess.run(
                ["btmgmt", "--index", hci_idx, "name", name],
                check=False,
                capture_output=True,
                timeout=4,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
            pass

    def _set_system_ble_name(self, new_name: str) -> None:
        hci_dev = self._adapter_path.split("/")[-1] if self._adapter_path else "hci0"
        hci_idx = self._hci_index(hci_dev)
        self._btmgmt_set_local_name(hci_idx, new_name)
        for cmd in (
            ["hciconfig", hci_dev, "name", new_name],
            ["bluetoothctl", "system-alias", new_name],
        ):
            try:
                subprocess.run(
                    cmd,
                    check=False,
                    capture_output=True,
                    timeout=4,
                )
            except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
                pass

    def rename_ble_device(self, new_name: str) -> bool:
        """更新 BLE 名称并刷新广播（可从分发线程调用）。"""
        if not new_name or not new_name.startswith("HT_"):
            return False
        done = threading.Event()
        ok: List[bool] = [False]

        def _start() -> bool:
            self._apply_ble_rename_on_main(new_name, done, ok)
            return False

        GLib.idle_add(_start)
        done.wait(timeout=3.0)
        return ok[0]

    def _apply_ble_rename_on_main(
        self, new_name: str, done: threading.Event, ok: List[bool]
    ) -> None:
        hci_dev = self._adapter_path.split("/")[-1] if self._adapter_path else "hci0"
        hci_idx = self._hci_index(hci_dev)

        def _patch_and_finish() -> None:
            try:
                self._patch_le_adv_data(
                    hci_dev=hci_dev, force=True, initial_delay=0.0
                )
                ok[0] = True
            except Exception as e:
                log_warn(f"BLE 广播刷新失败: {e}")
                ok[0] = False
            finally:
                done.set()

        if new_name == self.name:
            threading.Thread(target=_patch_and_finish, daemon=True).start()
            return

        self.name = new_name
        try:
            save_ble_name(new_name)
            log_info(f"广播名已持久化: {persistent_name_file()}")
        except OSError as e:
            log_warn(f"广播名持久化失败: {e}")
        self._btmgmt_set_local_name(hci_idx, new_name)

        if self._adapter_path:
            try:
                props = dbus.Interface(
                    self.bus.get_object(BLUEZ_SERVICE, self._adapter_path),
                    DBUS_PROP_IFACE,
                )
                props.Set(ADAPTER_IFACE, "Alias", new_name)
            except dbus.exceptions.DBusException as e:
                log_warn(f"设置 Alias 失败: {e}")

        if self._adv is not None:
            self._adv.local_name = new_name

        if self._connected_devices:
            log_info(
                f"BLE 名称已更新为 {new_name}（已连接，断连后请重新扫描）"
            )
        else:
            log_info(f"BLE 名称已更新: {new_name}，请手机重新扫描连接")

        threading.Thread(target=_patch_and_finish, daemon=True).start()

    def _on_notify_start(self) -> None:
        self._notify_active = True
        self._notify_enabled = True
        log_info("手机已订阅 FFE2 notify")
        self._flush_pending_notifies()
        if self._telemetry is not None:
            self._telemetry.on_subscribed(emit_snapshot=False)
        self._try_report_ready()
        self._schedule_ip_burst()

    def _on_notify_stop(self) -> None:
        self._notify_active = False
        self._notify_enabled = False
        self._pending_notifies.clear()
        log_info("手机已取消 notify 订阅")
        if self._telemetry is not None:
            self._telemetry.on_unsubscribed()

    def _find_adapter(self) -> str:
        om = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, "/"), DBUS_OM_IFACE
        )
        objects = om.GetManagedObjects()
        for path, ifaces in objects.items():
            if GATT_MANAGER_IFACE in ifaces and LE_ADV_MANAGER_IFACE in ifaces:
                if self.adapter == "hci0" and str(path).endswith("hci0"):
                    return str(path)
                if str(path) == self.adapter:
                    return str(path)
        for path, ifaces in objects.items():
            if GATT_MANAGER_IFACE in ifaces and LE_ADV_MANAGER_IFACE in ifaces:
                return str(path)
        raise RuntimeError("未找到支持 GATT/广播 的蓝牙适配器，请确认 bluetoothd 已启动")

    def _read_adapter_address(self, adapter_path: str) -> str:
        try:
            props = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE, adapter_path), DBUS_PROP_IFACE
            )
            return str(props.Get(ADAPTER_IFACE, "Address"))
        except dbus.exceptions.DBusException:
            return BOARD_BDADDR

    def _uses_dbus_adv(self) -> bool:
        plat = detect_platform() if detect_platform else None
        if plat is None:
            return True
        return plat.adv_mode != "legacy_hci"

    def _skip_btmgmt(self) -> bool:
        plat = detect_platform() if detect_platform else None
        return plat is not None and plat.skip_btmgmt

    def _run_btmgmt(self, *args: str) -> None:
        if self._skip_btmgmt() or self._uses_dbus_adv():
            # AIC/Q6A 上 btmgmt 会卡住；D-Bus 广播路径也不走 btmgmt
            return
        try:
            subprocess.run(
                ["btmgmt", "--index", "0", *args],
                check=False,
                capture_output=True,
                timeout=1,
                start_new_session=True,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired, OSError) as e:
            log_warn(f"btmgmt {' '.join(args)}: {e}")

    def _enter_ble_only_mode(self) -> None:
        """仅 BLE 广播，关闭经典蓝牙可发现/可配对，避免手机系统反复弹连接框。"""
        try:
            subprocess.run(
                ["hciconfig", "hci0", "noscan"],
                check=False,
                capture_output=True,
                timeout=2,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
            pass
        if self._skip_btmgmt():
            log_info("AIC8800：跳过 btmgmt，改用 HCI 空中广播")
            return
        if self._uses_dbus_adv():
            log_info("D-Bus 广播：跳过 btmgmt")
            return
        self._run_btmgmt("le", "on")
        self._run_btmgmt("connectable", "on")
        self._run_btmgmt("discov", "off")
        self._run_btmgmt("pairable", "off")
        self._run_btmgmt("bondable", "off")
        log_info("已切换 BLE-only：经典蓝牙不可配对（避免手机系统弹窗）")

    def _set_adapter_props(self, adapter_path: str) -> None:
        props = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path), DBUS_PROP_IFACE
        )
        try:
            props.Set(ADAPTER_IFACE, "Alias", self.name)
            log_info(f"BLE 广播名: {self.name}")
        except dbus.exceptions.DBusException as e:
            log_warn(f"设置 Alias 失败: {e}")
        try:
            props.Set(ADAPTER_IFACE, "Powered", dbus.Boolean(True))
            props.Set(ADAPTER_IFACE, "Discoverable", dbus.Boolean(False))
            props.Set(ADAPTER_IFACE, "Pairable", dbus.Boolean(False))
            props.Set(ADAPTER_IFACE, "DiscoverableTimeout", dbus.UInt32(0))
            props.Set(ADAPTER_IFACE, "PairableTimeout", dbus.UInt32(0))
        except dbus.exceptions.DBusException as e:
            log_warn(f"适配器属性: {e}")
        # RK 板载网卡：Discovering 只读，单独处理避免误报警
        try:
            props.Set(ADAPTER_IFACE, "Discovering", dbus.Boolean(False))
        except dbus.exceptions.DBusException:
            pass
        self._enter_ble_only_mode()

    def _verify_le_advertising(self, adapter_path: str) -> None:
        try:
            props = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE, adapter_path), DBUS_PROP_IFACE
            )
            alias = str(props.Get(ADAPTER_IFACE, "Alias"))
            addr = str(props.Get(ADAPTER_IFACE, "Address"))
            log_info(f"当前适配器 MAC: {addr}  Alias: {alias}")
            try:
                n = int(props.Get(LE_ADV_MANAGER_IFACE, "ActiveInstances"))
                log_info(f"LE 广播实例: ActiveInstances={n}")
                if n <= 0 and not self._skip_btmgmt() and self._uses_dbus_adv():
                    log_warn("ActiveInstances=0，手机将扫不到设备")
                elif n <= 0 and self._skip_btmgmt():
                    log_info("AIC HCI 广播不走 D-Bus，ActiveInstances=0 属正常")
            except dbus.exceptions.DBusException:
                pass
            log_info("请用微信小程序扫描 FFE0 / HT_，不要用系统蓝牙配对页")
        except dbus.exceptions.DBusException as e:
            log_warn(f"读取适配器信息失败: {e}")

    def _patch_le_adv_data(
        self, hci_dev: str = "hci0", force: bool = False, initial_delay: float = 0.4
    ) -> None:
        """RK/AIC：Legacy 或 Extended HCI 外发。D-Bus 广播路径不发 HCI，避免关掉已注册实例。"""
        if self._connected_devices:
            return
        if self._uses_dbus_adv() and not self._skip_btmgmt():
            return
        if initial_delay > 0:
            time.sleep(initial_delay)
        if _start_ble_advertising is not None:
            plat = detect_platform() if detect_platform else None
            _start_ble_advertising(
                hci_dev,
                self.name,
                platform=plat,
                force=force,
                log=log_info,
                keepalive=True,
            )
            return
        log_warn("[adv] ble_legacy_adv 未加载，跳过广播刷新")

    def _start_volume_manager(self) -> None:
        try:
            from ble_volume_manager import VolumeController
        except ImportError as e:
            log_warn(f"无法加载 ble_volume_manager: {e}")
            return
        self._volume = VolumeController(log=log_info)

    def _start_wifi_manager(self) -> None:
        try:
            from ble_wifi_manager import WifiManager
        except ImportError as e:
            log_warn(f"无法加载 ble_wifi_manager: {e}")
            return
        self._wifi = WifiManager(log=log_info)

        def _on_wifi_link_down() -> None:
            self._notify_wifi_disconnected()

        def _on_wifi_link_up() -> None:
            self._notify_wifi_reconnected()

        self._wifi.start_link_monitor(
            on_disconnected=_on_wifi_link_down,
            on_reconnected=_on_wifi_link_up,
        )
        log_info("WiFi 配网已就绪（WIFI <SSID> <PASSWORD>）")

    def _start_dispatcher(self) -> None:
        try:
            from ble_command_dispatcher import CommandDispatcher
        except ImportError as e:
            log_warn(f"无法加载 ble_command_dispatcher: {e}")
            return
        self._dispatcher = CommandDispatcher(
            handle=self._handle_dispatched,
            ack=self._send_ack if self.echo else None,
            echo_confirm=self._send_command_echo if self.echo else None,
            log_rx=log_rx,
            log_warn=log_warn,
        )
        self._dispatcher.start()
        log_info("指令分发器已启动")

    def _start_voice(self) -> None:
        try:
            from ble_voice import VoicePlayer
        except ImportError as e:
            log_warn(f"无法加载 ble_voice: {e}")
            return
        self._voice = VoicePlayer(log=log_info)
        self._voice.start()
        self._conversation = getattr(self._voice, "conversation", None)
        # 开机 IP 语音播报已关闭；局域网 IP 仍由状态遥测走 FFE2。
        log_info("[voice] 开机 IP 语音播报已关闭")

    def _notify_ble_ready(self) -> None:
        if self._ble_ready_announced:
            return
        self._ble_ready_announced = True
        log_info("BLE 广播就绪，等待小程序扫描连接（启动提示音已关闭）")
        # 不再播「蓝牙就绪，待连接」，避免服务重启时反复占喇叭。

    def _ble_ready_timeout_cb(self) -> bool:
        self._notify_ble_ready()
        return False

    def _start_telemetry(self) -> None:
        if self._notify_chrc is None:
            return
        try:
            from ble_status_telemetry import BleStatusTelemetry, FIXED_BATTERY_PCT
        except ImportError as e:
            log_warn(f"无法加载 ble_status_telemetry: {e}")
            return
        self._telemetry = BleStatusTelemetry(
            notify=self._notify_on_main_thread,
            battery_fn=self._read_battery_pct,
        )
        self._telemetry.start()
        log_info(f"电量固定上报 pwr:{FIXED_BATTERY_PCT}")

    def _read_battery_pct(self) -> int:
        try:
            from ble_status_telemetry import FIXED_BATTERY_PCT
        except ImportError:
            return 88
        return FIXED_BATTERY_PCT

    def run(self) -> int:
        adapter_path = self._find_adapter()
        self._adapter_path = adapter_path
        log_info(f"使用适配器: {adapter_path}")
        self._set_adapter_props(adapter_path)
        self._watch_devices()
        self._start_volume_manager()
        self._start_wifi_manager()
        self._start_voice()
        self._start_dispatcher()

        app = Application(self.bus)
        service = Service(self.bus, 0, SERVICE_UUID, True)
        self._write_chrc = Characteristic(
            self.bus,
            0,
            WRITE_CHAR_UUID,
            ["write", "write-without-response", "read"],
            service,
            on_write=self._on_write,
        )
        self._notify_chrc = Characteristic(
            self.bus,
            1,
            NOTIFY_CHAR_UUID,
            ["read", "notify"],
            service,
            on_notify_start=self._on_notify_start,
            on_notify_stop=self._on_notify_stop,
        )
        write_chrc = self._write_chrc
        service.add_characteristic(write_chrc)
        service.add_characteristic(self._notify_chrc)
        app.add_service(service)
        self._start_telemetry()

        adv = Advertisement(self.bus, 0, self.name)
        adv._server = self
        self._adv = adv
        adv_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path), LE_ADV_MANAGER_IFACE
        )
        self._adv_manager = adv_manager
        gatt_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path), GATT_MANAGER_IFACE
        )

        plat = detect_platform() if detect_platform else None
        use_dbus_adv = plat is None or (plat.adv_mode != "legacy_hci")

        def register_done() -> None:
            log_info("GATT 服务已注册，等待手机连接")
            hci_dev = adapter_path.split("/")[-1]
            threading.Thread(
                target=self._patch_le_adv_data,
                kwargs={"hci_dev": hci_dev},
                daemon=True,
            ).start()
            GLib.timeout_add(2500, self._ble_ready_timeout_cb)

        def register_failed(error: dbus.DBusException) -> None:
            err = self._dbus_error_text(error)
            log_warn(f"GATT 注册失败: {err}")
            if "AlreadyExists" in err:
                log_info("GATT 应用已存在，继续尝试广播")
                return
            log_info(
                "  尝试: ./start.sh --setup  "
                "或在 /etc/bluetooth/main.conf 加 Experimental=true"
            )

        def adv_done() -> None:
            self._adv_registered = True
            log_info(f"BLE 广播已开启，小程序应能扫到名称: {self.name}")
            log_info(f"    或按服务 UUID 扫描: {SERVICE_UUID}")
            self._verify_le_advertising(adapter_path)
            hci_dev = adapter_path.split("/")[-1]
            threading.Thread(
                target=self._patch_le_adv_data,
                kwargs={"hci_dev": hci_dev},
                daemon=True,
            ).start()
            self._notify_ble_ready()

        def adv_failed(error: dbus.DBusException) -> None:
            log_warn(f"广播注册失败: {error}")
            log_warn("GATT 仍保持运行，将 2s 后重试 RegisterAdvertisement")
            GLib.timeout_add(2000, self._restart_advertising_cb)

        gatt_manager.RegisterApplication(
            app.get_path(),
            _dbus_sv_opts(),
            reply_handler=register_done,
            error_handler=register_failed,
        )
        if use_dbus_adv:
            adv_manager.RegisterAdvertisement(
                adv.get_path(),
                _dbus_sv_opts(),
                reply_handler=adv_done,
                error_handler=adv_failed,
            )
        else:
            if plat:
                log_info(f"平台: {plat.platform_id} | {plat.hw_desc}")
            log_info("板载蓝牙：跳过 D-Bus 广播，使用 HCI 外发（AIC 同时尝试 Extended）")

        print()
        addr = self._read_adapter_address(adapter_path)
        log_info(">>> microDuck BLE 连接服务 <<<")
        log_info(f"    广播名: {self.name}  MAC: {addr}")
        log_info("    FFE0/FFE1/FFE2 | IP / 音量 / 固定电量 88%")
        log_info("    日志: RX红(收) TX绿(发)")
        log_info("    Ctrl+C 退出")
        print()

        try:
            self.mainloop.run()
        except KeyboardInterrupt:
            log_info("退出")
        finally:
            if self._telemetry is not None:
                self._telemetry.stop()
            if self._dispatcher is not None:
                self._dispatcher.stop()
            if self._wifi is not None:
                try:
                    self._wifi.stop_link_monitor()
                except Exception:
                    pass
            if self._voice is not None:
                self._voice.stop()
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="microDuck BLE GATT 连接服务")
    parser.add_argument(
        "--name",
        default=None,
        help=f"BLE 广播设备名 (默认从 ble_device_name.conf 读取，缺省 {DEFAULT_BLE_NAME})",
    )
    parser.add_argument("--adapter", default="hci0", help="hci0 或 dbus 路径")
    parser.add_argument(
        "--no-echo",
        action="store_true",
        help="不向 notify 特征回显 ACK",
    )
    args = parser.parse_args()
    ble_name = args.name if args.name else load_ble_name()
    server = BleGattServer(
        args.adapter,
        ble_name,
        echo=not args.no_echo,
    )
    return server.run()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        import traceback

        err = traceback.format_exc()
        try:
            with open("/tmp/duck-ble.log", "a", encoding="utf-8") as f:
                f.write(err)
        except OSError:
            pass
        print(err, file=sys.stderr, flush=True)
        sys.exit(1)
