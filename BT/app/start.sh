#!/bin/bash
# microDuck BLE 连接服务启动
set -euo pipefail
cd "$(dirname "$0")"

# shellcheck disable=SC1091
source "$(pwd)/platform_env.sh"
source "$(pwd)/platform_hw.sh"

SUDO_PW="${DUCK_BLE_SUDO_PW:-}"

sudo_n() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  elif [ -n "${SUDO_PW}" ]; then
    echo "$SUDO_PW" | sudo -S -p '' "$@"
  else
    sudo "$@"
  fi
}

show_help() {
  cat <<'EOF'
用法: ./start.sh [选项]

启动 BLE GATT 从机：连接、IP 显示、音量调节。

小程序连接参数:
  设备名: HT_88888888  （可用 rename HT_12345678 修改后8位）
  服务 FFE0:  0000FFE0-0000-1000-8000-00805F9B34FB
  写入 FFE1:  0000FFE1-0000-1000-8000-00805F9B34FB
  通知 FFE2:  0000FFE2-0000-1000-8000-00805F9B34FB

重要: 只在【微信小程序】里连接，不要在手机系统蓝牙里点配对

选项:
  --name NAME     广播名称 (默认 ble_device_name.conf 或 HT_88888888)
  --no-echo       不回显 ACK 到 notify 特征
  --setup         开启 BlueZ Experimental 并重启 bluetooth
  -h, --help      显示帮助

开机自启动:
  sudo ./install-autostart.sh
  sudo systemctl status duck-ble
  journalctl -u duck-ble -f
EOF
}

NAME="$(python3 -c "from ble_device_name import load_ble_name; print(load_ble_name())")"
EXTRA_ARGS=()
DO_SETUP=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      show_help
      exit 0
      ;;
    --setup)
      DO_SETUP=1
      shift
      ;;
    --name)
      NAME="${2:-HT_88888888}"
      EXTRA_ARGS+=(--name "$NAME")
      shift 2
      ;;
    --no-echo)
      EXTRA_ARGS+=(--no-echo)
      shift
      ;;
    --no-ros)
      shift
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if [ "$DO_SETUP" -eq 1 ]; then
  echo "[setup] 检查 BlueZ Experimental..."
  CONF="/etc/bluetooth/main.conf"
  if [ -f "$CONF" ] && ! grep -qE '^[[:space:]]*Experimental[[:space:]]*=[[:space:]]*true' "$CONF" 2>/dev/null; then
    BACKUP="${CONF}.bak.$(date +%Y%m%d%H%M%S)"
    sudo_n cp "$CONF" "$BACKUP"
    echo "[setup] 已备份: $BACKUP"
    if grep -q '^\[General\]' "$CONF"; then
      sudo_n sed -i '/^\[General\]/a Experimental=true' "$CONF"
      echo "[setup] 已添加 Experimental=true"
    else
      printf '%s\n' '[General]' 'Experimental=true' | sudo_n tee -a "$CONF" >/dev/null
      echo "[setup] 已追加 [General] Experimental=true"
    fi
    sudo_n systemctl restart bluetooth
    sleep 2
  else
    echo "[setup] 已配置或无法修改 $CONF"
  fi
  sudo_n hciconfig hci0 up 2>/dev/null || true
  sudo_n hciconfig hci0 name "${NAME}" 2>/dev/null || true
  bluetoothctl power on 2>/dev/null || true
  bluetoothctl discoverable off 2>/dev/null || true
  bluetoothctl pairable off 2>/dev/null || true
  bluetoothctl system-alias "${NAME}" 2>/dev/null || true
fi

if ! systemctl is-active --quiet bluetooth 2>/dev/null; then
  echo "[warn] bluetooth 服务未运行，尝试启动..."
  sudo_n systemctl start bluetooth || true
  sleep 1
fi

_BLE_DEV="${BLE_HCI_DEV:-hci0}"
if ! hciconfig "${_BLE_DEV}" 2>/dev/null | grep -q "${_BLE_DEV}"; then
  echo "[error] 未检测到蓝牙 ${_BLE_DEV}"
  echo "  平台: ${BLE_PLATFORM} | ${BLE_HW_DESC}"
  echo "  恢复: sudo ./scripts/recover.sh"
  exit 1
fi
unset _BLE_DEV

echo "平台: ${BLE_PLATFORM} | ${BLE_HW_DESC} | 广播: ${BLE_ADV_MODE}"

if ! python3 -c "import dbus; from gi.repository import GLib" 2>/dev/null; then
  echo "缺少依赖，请执行:"
  echo "  sudo apt install -y bluez python3-dbus python3-gi"
  exit 1
fi

chmod +x ble_gatt_server.py ble_command_dispatcher.py ble_status_telemetry.py ble_log.py 2>/dev/null || true

sudo_n hciconfig hci0 up 2>/dev/null || true
sudo_n hciconfig hci0 name "${NAME}" 2>/dev/null || true
sudo_n hciconfig hci0 noscan 2>/dev/null || true
bluetoothctl system-alias "${NAME}" 2>/dev/null || true
bluetoothctl discoverable off 2>/dev/null || true
bluetoothctl pairable off 2>/dev/null || true
if [ "${BLE_PLATFORM:-}" != "qcs6490" ] && [ "${BLE_ADV_MODE:-}" != "legacy_hci" ]; then
  if command -v btmgmt >/dev/null && hciconfig hci0 2>/dev/null | grep -q "hci0"; then
    timeout 1 sudo_n btmgmt -i 0 le on 2>/dev/null || true
    timeout 1 sudo_n btmgmt -i 0 connectable on 2>/dev/null || true
    timeout 1 sudo_n btmgmt -i 0 discov off 2>/dev/null || true
    timeout 1 sudo_n btmgmt -i 0 pairable off 2>/dev/null || true
    timeout 1 sudo_n btmgmt -i 0 bondable off 2>/dev/null || true
  fi
else
  echo "[aic] 跳过 btmgmt（AIC8800 mgmt 会阻塞）"
fi

MAC=$(bluetoothctl show 2>/dev/null | awk '/Controller/ {print $2}' | head -1)
echo "========================================"
echo " microDuck BLE | 广播名: $NAME"
echo " MAC: ${MAC:-见 bluetoothctl show}"
echo " 协议: ${PKG_DIR}/docs/BLE_PROTOCOL.md"
echo " 诊断: ./scripts/check.sh"
echo "========================================"

if [ "$(id -u)" -ne 0 ]; then
  echo "[tip] 使用 sudo 注册 GATT 服务..."
  if [ -n "${SUDO_PW}" ]; then
    echo "$SUDO_PW" | sudo -S -p '' -E python3 "$(pwd)/ble_gatt_server.py" --name "$NAME" "${EXTRA_ARGS[@]}"
  else
    sudo -E python3 "$(pwd)/ble_gatt_server.py" --name "$NAME" "${EXTRA_ARGS[@]}"
  fi
  exit $?
fi

exec python3 "$(pwd)/ble_gatt_server.py" --name "$NAME" "${EXTRA_ARGS[@]}"
