#!/bin/bash
# microDuck BLE 一键安装：系统依赖 + 开机自启
set -euo pipefail

PKG_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_DIR="${PKG_DIR}/app"
cd "${PKG_DIR}"

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo ./install.sh"
  exit 1
fi

INSTALL_USER="${SUDO_USER:-${DUCK_USER:-radxa}}"
INSTALL_HOME="$(getent passwd "${INSTALL_USER}" | cut -d: -f6)"
INSTALL_UID="$(id -u "${INSTALL_USER}" 2>/dev/null || echo 1000)"
INSTALL_GID="$(id -g "${INSTALL_USER}" 2>/dev/null || echo 1000)"
DUCK_WS="${DUCK_WS:-$(cd "${PKG_DIR}/.." && pwd)}"

echo "=========================================="
echo " microDuck BLE 安装"
echo " 安装目录: ${PKG_DIR}"
echo " 运行用户: ${INSTALL_USER}"
echo "=========================================="

echo "[1/4] 安装系统依赖..."
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-install-recommends \
  bluez bluez-tools \
  python3 python3-dbus python3-gi \
  rfkill alsa-utils wireplumber \
  espeak-ng

echo "[2/4] 配置 BlueZ Experimental..."
CONF="/etc/bluetooth/main.conf"
if [ -f "${CONF}" ] && ! grep -qE '^[[:space:]]*Experimental[[:space:]]*=[[:space:]]*true' "${CONF}" 2>/dev/null; then
  BACKUP="${CONF}.bak.$(date +%Y%m%d%H%M%S)"
  cp "${CONF}" "${BACKUP}"
  if grep -q '^\[General\]' "${CONF}"; then
    sed -i '/^\[General\]/a Experimental=true' "${CONF}"
  else
    printf '%s\n' '[General]' 'Experimental=true' >>"${CONF}"
  fi
  systemctl restart bluetooth
  sleep 2
  echo "[ok] 已启用 Experimental=true（备份: ${BACKUP}）"
fi

echo "[3/4] 写入环境 /etc/default/duck-ble ..."
cat >/etc/default/duck-ble <<EOF
DUCK_USER=${INSTALL_USER}
DUCK_HOME=${INSTALL_HOME}
DUCK_BLE_UID=${INSTALL_UID}
DUCK_WS=${DUCK_WS}
BLE_DEVICE_NAME_FILE=/var/lib/duck-ble/ble_device_name.conf
EXTRA_ARGS=()
EOF

mkdir -p /var/lib/duck-ble
chmod 755 /var/lib/duck-ble
if [ ! -f /var/lib/duck-ble/ble_device_name.conf ] && [ -f "${PKG_DIR}/ble_device_name.conf" ]; then
  _bn="$(tr -d '[:space:]' < "${PKG_DIR}/ble_device_name.conf")"
  if [ -n "${_bn}" ]; then
    echo "${_bn}" > /var/lib/duck-ble/ble_device_name.conf
    chmod 644 /var/lib/duck-ble/ble_device_name.conf
    echo "[ok] 广播名持久化: /var/lib/duck-ble/ble_device_name.conf (${_bn})"
  fi
fi
unset _bn

export DUCK_USER="${INSTALL_USER}"
export DUCK_HOME="${INSTALL_HOME}"
export DUCK_BLE_UID="${INSTALL_UID}"
export DUCK_WS="${DUCK_WS}"

chown "${INSTALL_USER}:${INSTALL_GID}" "${PKG_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod 664 "${PKG_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod +x "${APP_DIR}"/*.sh "${APP_DIR}/scripts/"*.sh "${APP_DIR}/systemd/"*.sh

echo "[4/4] 安装 systemd 开机自启..."
"${APP_DIR}/install-autostart.sh"

systemctl restart duck-ble.service || systemctl start duck-ble.service || true
sleep 3
echo ""
echo "=========================================="
echo " 安装完成"
echo "=========================================="
echo " 状态: sudo systemctl status duck-ble"
echo " 日志: journalctl -u duck-ble -f"
echo " 手动: cd ${APP_DIR} && ./start.sh"
"${APP_DIR}/scripts/check.sh" || true
