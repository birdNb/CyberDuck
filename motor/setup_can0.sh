#!/usr/bin/env bash
# 抬起 PEAK PCAN-USB 的 can0：1 Mbps，采样点 0.875
set -euo pipefail
IFACE="${1:-can0}"

if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

modprobe can || true
modprobe can_raw || true
modprobe peak_usb || true

ip link set "${IFACE}" down 2>/dev/null || true
ip link set "${IFACE}" up type can bitrate 1000000 sample-point 0.875 restart-ms 100
ip -details link show "${IFACE}"
echo "OK: ${IFACE} UP @ 1Mbps"
