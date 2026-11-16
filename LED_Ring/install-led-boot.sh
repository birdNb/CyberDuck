#!/usr/bin/env bash
# 安装并启用开机灯效（PIN_22，默认彗星流水灯，持续运行）
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
UNIT_SRC="${DIR}/led-boot.service"
UNIT_DST="/etc/systemd/system/led-boot.service"

if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

install -m 644 "${UNIT_SRC}" "${UNIT_DST}"
systemctl daemon-reload
systemctl enable led-boot.service
systemctl restart led-boot.service
systemctl --no-pager --full status led-boot.service || true
echo "OK: led-boot.service 已安装并启动"
echo "下次开机将自动点亮 PIN_22 彗星流水灯。"
