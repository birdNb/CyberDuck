#!/bin/bash
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo ./uninstall.sh"
  exit 1
fi
systemctl stop duck-ble.service 2>/dev/null || true
systemctl disable duck-ble.service 2>/dev/null || true
rm -f /etc/systemd/system/duck-ble.service
systemctl daemon-reload
echo "[ok] 已移除 duck-ble 开机自启（安装目录未删除）"
