#!/bin/bash
# 安装轮行驾驶开机自启。已有手动运行的进程时不重复拉起。
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
UNIT_DST="/etc/systemd/system/wheel-drive.service"

if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

chmod 755 "${DIR}/wheel-drive-boot.sh"
install -m 644 "${DIR}/wheel-drive.service" "${UNIT_DST}"
systemctl daemon-reload
systemctl enable wheel-drive.service

if pgrep -f 'wheel_run/wheel_drive.py' >/dev/null 2>&1; then
  echo "已有手动运行的驾驶程序，这次只设置开机自启，不重复拉起。"
else
  systemctl start wheel-drive.service
fi

systemctl --no-pager --full status wheel-drive.service || true
echo "下次开机自动运行 wheel_drive.py。起来后仍是卸力，长按 Y 才上力矩。"
