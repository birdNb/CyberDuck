#!/bin/bash
# 安装音频开机启动，并排在驾驶服务前面。
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
UNIT_DST="/etc/systemd/system/audio-boot.service"
DRIVE_UNIT="/etc/systemd/system/wheel-drive.service"
DRIVE_SRC="/home/aidlux/cyberDuck/sport_ctrl/wheel_run/wheel-drive.service"

if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

chmod 755 "${DIR}/audio-boot.sh"
install -m 644 "${DIR}/audio-boot.service" "${UNIT_DST}"

# 驾驶服务要在音频之后；仓库里的 unit 同步过去。
if [[ -f "${DRIVE_SRC}" ]]; then
  install -m 644 "${DRIVE_SRC}" "${DRIVE_UNIT}"
fi

if command -v loginctl >/dev/null 2>&1; then
  loginctl enable-linger aidlux || true
fi

systemctl daemon-reload
systemctl enable audio-boot.service
systemctl start audio-boot.service
systemctl --no-pager --full status audio-boot.service || true
echo "OK: audio-boot.service 已安装。下次开机先拉 PipeWire，再起 wheel-drive。"
