#!/bin/bash
# 开机拉起轮行驾驶。can0 晚到就重试，程序起来后仍卸力，要长按 Y 才上力矩。
set -u
MOTOR="/home/aidlux/cyberDuck/motor"
IFACE="can0"

for _ in $(seq 1 30); do
  if ip -details link show "${IFACE}" 2>/dev/null | grep -q "state UP"; then
    break
  fi
  "${MOTOR}/setup_can0.sh" "${IFACE}" || true
  sleep 1
done

cd "${MOTOR}"
exec /usr/bin/python3 -u wheel_run/wheel_drive.py
