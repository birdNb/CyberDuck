#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../platform_hw.sh"
cd "${BT_DIR}"

echo "========== 平台 =========="
echo "主控: ${BLE_PLATFORM} | ${BLE_BOARD_NAME}"
echo "蓝牙: ${BLE_HW_DESC} (${BLE_CHIP} → ${BLE_HCI_DEV})"
echo "广播模式: ${BLE_ADV_MODE}"
echo "工作目录: ${BT_DIR}"
echo ""

if ! hciconfig "${BLE_HCI_DEV}" 2>/dev/null | grep -q "${BLE_HCI_DEV}"; then
  echo "[!!] ${BLE_HCI_DEV} 不存在"
  echo "     sudo ${BT_DIR}/scripts/recover.sh"
  echo ""
fi
bluetoothctl show 2>/dev/null | grep -E 'Controller|Name:|Alias:|Powered|Discoverable' || true
MAC=$(bluetoothctl show 2>/dev/null | awk '/Controller/ {print $2}' | head -1)
echo ""

echo "========== BLE 服务 =========="
if pgrep -f 'ble_gatt_server\.py[c]?' >/dev/null; then
  echo "[OK] BLE GATT 进程运行中"
elif systemctl is-active --quiet duck-ble.service 2>/dev/null; then
  echo "[OK] duck-ble.service active"
else
  echo "[!!] 未运行 — sudo systemctl start duck-ble  或  cd ${BT_DIR} && ./start.sh"
fi
echo ""
echo "板子 MAC: ${MAC:-见 bluetoothctl show}"
echo "广播名: $(python3 -c "from ble_device_name import load_ble_name; print(load_ble_name())" 2>/dev/null || echo HT_88888888)"
echo ""
echo "说明:"
echo "  - 小程序内扫描 FFE0 / HT_88888888，勿用手机系统蓝牙配对"
echo "  - AIC8800 走 HCI 广播，bluetoothctl ActiveInstances=0 属正常"
echo "  - 连接后推送 IP:x.x.x.x 与 pwr:88"
echo "  - 音量指令: V 0-100"
