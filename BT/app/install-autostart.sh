#!/bin/bash
# 安装 microDuck BLE 开机自启动（systemd）
set -euo pipefail

BT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${BT_DIR}/platform_env.sh"
SERVICE_NAME="duck-ble.service"
UNIT_SRC="${BT_DIR}/systemd/duck-ble.service"
BOOT_SRC="${BT_DIR}/systemd/duck-ble-boot.sh"
UNIT_DST="/etc/systemd/system/${SERVICE_NAME}"
DEFAULT_ENV="/etc/default/duck-ble"

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo $0"
  exit 1
fi

chmod +x "${BOOT_SRC}" "${BT_DIR}/start.sh"

PERSIST_NAME="/var/lib/duck-ble/ble_device_name.conf"
mkdir -p /var/lib/duck-ble
chmod 755 /var/lib/duck-ble
if [ ! -f "${PERSIST_NAME}" ] && [ -f "${PKG_DIR}/ble_device_name.conf" ]; then
  _n="$(head -1 "${PKG_DIR}/ble_device_name.conf" | tr -d '[:space:]')"
  if [ -n "${_n}" ]; then
    echo "${_n}" >"${PERSIST_NAME}"
    chmod 644 "${PERSIST_NAME}"
    echo "[ok] 已迁移广播名 → ${PERSIST_NAME}: ${_n}"
  fi
fi
unset PERSIST_NAME _n

if [ -f /etc/bluetooth/main.conf ] && ! grep -qE '^[[:space:]]*Experimental[[:space:]]*=[[:space:]]*true' /etc/bluetooth/main.conf 2>/dev/null; then
  echo "[setup] 配置 BlueZ Experimental=true ..."
  "${BT_DIR}/start.sh" --setup || true
fi

if ! python3 -c "import dbus; from gi.repository import GLib" 2>/dev/null; then
  echo "[error] 缺少依赖: sudo apt install -y bluez python3-dbus python3-gi"
  exit 1
fi

sed \
  -e "s|@BT_DIR@|${BT_DIR}|g" \
  -e "s|@PKG_DIR@|${PKG_DIR}|g" \
  -e "s|@DUCK_WS@|${DUCK_WS}|g" \
  -e "s|@DUCK_USER@|${DUCK_USER}|g" \
  -e "s|@DUCK_HOME@|${DUCK_HOME}|g" \
  -e "s|@DUCK_BLE_UID@|${DUCK_BLE_UID}|g" \
  "${UNIT_SRC}" >"${UNIT_DST}"

if [ ! -f "${DEFAULT_ENV}" ]; then
  cat >"${DEFAULT_ENV}" <<EOF
BLE_DEVICE_NAME_FILE=/var/lib/duck-ble/ble_device_name.conf
EXTRA_ARGS=()
EOF
  echo "[ok] 已创建 ${DEFAULT_ENV}"
elif ! grep -q '^BLE_DEVICE_NAME_FILE=' "${DEFAULT_ENV}" 2>/dev/null; then
  echo 'BLE_DEVICE_NAME_FILE=/var/lib/duck-ble/ble_device_name.conf' >>"${DEFAULT_ENV}"
fi

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

echo ""
echo "=========================================="
echo " microDuck BLE 已设置开机自启动"
echo " 服务名: ${SERVICE_NAME}"
echo "=========================================="
echo ""
echo "常用命令:"
echo "  sudo systemctl start ${SERVICE_NAME}"
echo "  sudo systemctl status ${SERVICE_NAME}"
echo "  journalctl -u ${SERVICE_NAME} -f"
