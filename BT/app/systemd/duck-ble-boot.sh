#!/bin/bash
# systemd 开机启动入口
set -eo pipefail

if [ -f /etc/default/duck-ble ]; then
  # shellcheck disable=SC1091
  source /etc/default/duck-ble
fi

BT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$BT_DIR"

# shellcheck disable=SC1091
source "${BT_DIR}/platform_env.sh"
source "${BT_DIR}/platform_hw.sh"

export PULSE_SERVER="${PULSE_SERVER:-unix:/run/user/${DUCK_BLE_UID}/pulse/native}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/${DUCK_BLE_UID}}"

wait_hci() {
  local i
  for i in $(seq 1 20); do
    if hciconfig "${BLE_HCI_DEV}" 2>/dev/null | grep -q "${BLE_HCI_DEV}"; then
      return 0
    fi
    sleep 0.25
  done
  echo "[duck-ble] 超时：未检测到 ${BLE_HCI_DEV} (${BLE_HW_DESC})" >&2
  return 1
}

hci_up() {
  hciconfig "${BLE_HCI_DEV}" 2>/dev/null | grep -q "UP RUNNING"
}

prep_bluetooth() {
  if ! systemctl is-active --quiet bluetooth 2>/dev/null; then
    echo "[duck-ble] 启动 bluetooth.service"
    systemctl start bluetooth 2>/dev/null || true
    sleep 0.4
  fi
  wait_hci || return 1

  if ! hci_up; then
    hciconfig "${BLE_HCI_DEV}" up 2>/dev/null || true
  fi

  BLE_NAME="$(python3 -c "from ble_device_name import load_ble_name; print(load_ble_name())")"
  hciconfig "${BLE_HCI_DEV}" name "${BLE_NAME}" 2>/dev/null || true
  hciconfig "${BLE_HCI_DEV}" noscan 2>/dev/null || true
  timeout 1 bluetoothctl system-alias "${BLE_NAME}" >/dev/null 2>&1 || true
  timeout 1 bluetoothctl discoverable off >/dev/null 2>&1 || true
  timeout 1 bluetoothctl pairable off >/dev/null 2>&1 || true
}

prep_bluetooth

EXTRA_ARGS=()
if [ -f /etc/default/duck-ble ]; then
  # shellcheck disable=SC1091
  source /etc/default/duck-ble
fi

exec python3 "${BT_DIR}/ble_gatt_server.py" "${EXTRA_ARGS[@]}"
