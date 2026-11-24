#!/bin/bash
# 每次启动都把 PCAN 拆掉再拉起。can0 晚到就重试。
# 程序起来后仍卸力，要长按 Y 才上力矩。
set -u
MOTOR="/home/aidlux/cyberDuck/motor"
IFACE="can0"

# 口已经 UP 就不要再 down/up，否则右轮 1 号经常晚回包被扫漏。
if ip -details link show "${IFACE}" 2>/dev/null | grep -q "state UP"; then
  echo "OK: ${IFACE} 已 UP，跳过重建"
else
  for _ in $(seq 1 30); do
    if "${MOTOR}/setup_can0.sh" "${IFACE}"; then
      break
    fi
    sleep 1
  done
fi

if ! ip -details link show "${IFACE}" 2>/dev/null | grep -q "state UP"; then
  echo "can0 没起来，仍启动驾驶程序，关节上线后会自己重扫。" >&2
fi

# 给电机一点时间再扫。
sleep 0.5
exec /home/aidlux/cyberDuck/build-wheel/sport_ctrl/wheel_run/cpp/wheel_drive --wheel-hz 50 --body-hz 50
