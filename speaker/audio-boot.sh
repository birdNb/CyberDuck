#!/bin/bash
# 开机拉起 aidlux 的 PipeWire。驾驶服务以 root 跑，鸭叫走 /run/user/1000。
set -u

USER_NAME="aidlux"
UID_NUM="$(id -u "${USER_NAME}" 2>/dev/null || echo 1000)"
RUNTIME="/run/user/${UID_NUM}"
export XDG_RUNTIME_DIR="${RUNTIME}"
export HOME="$(getent passwd "${USER_NAME}" | cut -d: -f6)"
export DBUS_SESSION_BUS_ADDRESS="unix:path=${RUNTIME}/bus"

# 没登录也要有用户级 systemd，否则 PipeWire 起不来。
if command -v loginctl >/dev/null 2>&1; then
  loginctl enable-linger "${USER_NAME}" >/dev/null 2>&1 || true
fi
systemctl start "user@${UID_NUM}.service" >/dev/null 2>&1 || true

for _ in $(seq 1 40); do
  if [[ -d "${RUNTIME}" ]]; then
    break
  fi
  sleep 0.25
done
if [[ ! -d "${RUNTIME}" ]]; then
  echo "音频启动失败：没有 ${RUNTIME}" >&2
  exit 1
fi

user_ctl() {
  sudo -u "${USER_NAME}" -H \
    XDG_RUNTIME_DIR="${RUNTIME}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS}" \
    systemctl --user "$@"
}

user_ctl start pipewire.socket pipewire.service \
  pipewire-pulse.socket pipewire-pulse.service \
  wireplumber.service >/dev/null 2>&1 || true

ready=0
for _ in $(seq 1 60); do
  if [[ -S "${RUNTIME}/pulse/native" ]] || [[ -S "${RUNTIME}/pipewire-0" ]]; then
    ready=1
    break
  fi
  sleep 0.25
done

SPEAKER_DIR="$(cd "$(dirname "$0")" && pwd)"

as_user() {
  sudo -u "${USER_NAME}" -H \
    XDG_RUNTIME_DIR="${RUNTIME}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS}" \
    PULSE_SERVER="unix:${RUNTIME}/pulse/native" \
    PIPEWIRE_RUNTIME_DIR="${RUNTIME}" \
    "$@"
}

if [[ "${ready}" -eq 1 ]]; then
  as_user wpctl set-mute @DEFAULT_AUDIO_SINK@ 0 >/dev/null 2>&1 || true
  as_user wpctl set-volume @DEFAULT_AUDIO_SINK@ 0.75 >/dev/null 2>&1 || true
  echo "OK: PipeWire 已就绪（${USER_NAME} ${RUNTIME}）"
  # 三声不同鸭叫，表示音频拉起来了。
  as_user /usr/bin/python3 -u "${SPEAKER_DIR}/boot_quack.py" || \
    echo "开机鸭叫没播出来，音频服务仍算启动成功。" >&2
  exit 0
fi

echo "音频启动超时：PipeWire 套接字还没出现，驾驶仍会启动。" >&2
exit 0
