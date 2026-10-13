#!/bin/bash
# 平台路径（cyberDuck/BT）

_pe_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export BT_DIR="${BT_DIR:-${_pe_dir}}"
if [ -f "${BT_DIR}/../ble_device_name.conf" ]; then
  export PKG_DIR="${PKG_DIR:-$(cd "${BT_DIR}/.." && pwd)}"
else
  export PKG_DIR="${PKG_DIR:-${BT_DIR}}"
fi
export DUCK_WS="${DUCK_WS:-$(cd "${PKG_DIR}/.." && pwd)}"
export BLE_DEVICE_NAME_FILE="${BLE_DEVICE_NAME_FILE:-/var/lib/duck-ble/ble_device_name.conf}"
_ws_owner="$(stat -c '%U' "${DUCK_WS}" 2>/dev/null || true)"
if [ -n "${DUCK_USER:-}" ]; then
  :
elif [ -n "$_ws_owner" ] && [ "$_ws_owner" != "root" ]; then
  export DUCK_USER="$_ws_owner"
else
  export DUCK_USER="${USER:-radxa}"
fi
export DUCK_HOME="${DUCK_HOME:-/home/${DUCK_USER}}"
export DUCK_BLE_UID="${DUCK_BLE_UID:-$(id -u "${DUCK_USER}" 2>/dev/null || echo 1000)}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/${DUCK_BLE_UID}}"
export PULSE_SERVER="${PULSE_SERVER:-unix:${XDG_RUNTIME_DIR}/pulse/native}"
unset _pe_dir _ws_owner
