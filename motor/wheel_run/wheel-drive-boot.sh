#!/bin/bash
# 每次启动都把 PCAN 拆掉再拉起。can0 晚到就重试。
# 程序起来后仍卸力，要长按 Y 才上力矩。
set -u
MOTOR="/home/aidlux/cyberDuck/motor"
IFACE="can0"

for _ in $(seq 1 30); do
  if "${MOTOR}/setup_can0.sh" "${IFACE}"; then
    break
  fi
  sleep 1
done

if ! ip -details link show "${IFACE}" 2>/dev/null | grep -q "state UP"; then
  echo "can0 没起来，仍启动驾驶程序，关节上线后会自己重扫。" >&2
fi

cd "${MOTOR}"
exec /usr/bin/python3 -u wheel_run/wheel_drive.py
