#!/usr/bin/env bash
# 开机进入轮行。先拉起 can0，再跑 wheel_drive.py。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "${ROOT}"

for _ in $(seq 1 20); do
  if [[ -e /sys/class/net/can0 ]]; then
    break
  fi
  sleep 0.5
done

"${ROOT}/setup_can0.sh" can0

exec /usr/bin/python3 -u "${ROOT}/wheel_run/wheel_drive.py"
