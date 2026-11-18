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

as_user() {
  sudo -u "${USER_NAME}" -H \
    XDG_RUNTIME_DIR="${RUNTIME}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS}" \
    PULSE_SERVER="unix:${RUNTIME}/pulse/native" \
    PIPEWIRE_RUNTIME_DIR="${RUNTIME}" \
    "$@"
}

# 板载 WCD 通路：HPH/LO 关掉时 PipeWire 仍“播成功”，但听不见。
enable_codec_path() {
  command -v tinymix >/dev/null 2>&1 || return 0
  tinymix set "LO Switch" 1 >/dev/null 2>&1 || true
  tinymix set "HPHL Switch" 1 >/dev/null 2>&1 || true
  tinymix set "HPHR Switch" 1 >/dev/null 2>&1 || true
  tinymix set "CLSH Switch" 1 >/dev/null 2>&1 || true
  tinymix set "HPHL_RDAC Switch" 1 >/dev/null 2>&1 || true
  tinymix set "HPHR_RDAC Switch" 1 >/dev/null 2>&1 || true
  tinymix set "AUX_RDAC Switch" 1 >/dev/null 2>&1 || true
  tinymix set "HPHL Volume" 24 >/dev/null 2>&1 || true
  tinymix set "HPHR Volume" 24 >/dev/null 2>&1 || true
  tinymix set "RX_RX0 Digital Volume" 120 >/dev/null 2>&1 || true
  tinymix set "RX_RX1 Digital Volume" 120 >/dev/null 2>&1 || true
}

# PipeWire 卡死时 wpctl 会一直挂；所有调用都加 timeout。
wpctl_ok() {
  timeout 2 sudo -u "${USER_NAME}" -H \
    XDG_RUNTIME_DIR="${RUNTIME}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS}" \
    PULSE_SERVER="unix:${RUNTIME}/pulse/native" \
    PIPEWIRE_RUNTIME_DIR="${RUNTIME}" \
    wpctl get-volume @DEFAULT_AUDIO_SINK@ >/dev/null 2>&1
}

start_audio_stack() {
  user_ctl reset-failed \
    pipewire.service pipewire.socket \
    pipewire-pulse.service pipewire-pulse.socket \
    wireplumber.service >/dev/null 2>&1 || true
  user_ctl start pipewire.socket pipewire.service \
    pipewire-pulse.socket pipewire-pulse.service \
    wireplumber.service >/dev/null 2>&1 || true
}

recover_audio_stack() {
  echo "PipeWire 无响应，尝试复位…" >&2
  user_ctl stop pipewire-pulse.service wireplumber.service pipewire.service >/dev/null 2>&1 || true
  # 别等 systemctl 卡死
  timeout 3 user_ctl stop pipewire-pulse.socket pipewire.socket >/dev/null 2>&1 || true
  pkill -u "${USER_NAME}" -x pipewire-pulse >/dev/null 2>&1 || true
  pkill -u "${USER_NAME}" -x wireplumber >/dev/null 2>&1 || true
  pkill -u "${USER_NAME}" -x pipewire >/dev/null 2>&1 || true
  sleep 0.5
  rm -f "${RUNTIME}/pipewire-0" "${RUNTIME}/pipewire-0.lock" 2>/dev/null || true
  start_audio_stack
}

start_audio_stack

ready=0
for _ in $(seq 1 60); do
  if [[ -S "${RUNTIME}/pulse/native" ]] || [[ -S "${RUNTIME}/pipewire-0" ]]; then
    ready=1
    break
  fi
  sleep 0.25
done

SPEAKER_DIR="$(cd "$(dirname "$0")" && pwd)"

if [[ "${ready}" -eq 1 ]]; then
  # 等 sink 真正出来；套接字在、节点还没就绪时 wpctl 会报 -1。
  sink_ready=0
  for _ in $(seq 1 40); do
    if wpctl_ok; then
      sink_ready=1
      break
    fi
    sleep 0.25
  done
  if [[ "${sink_ready}" -ne 1 ]]; then
    recover_audio_stack
    for _ in $(seq 1 40); do
      if [[ -S "${RUNTIME}/pipewire-0" ]] && wpctl_ok; then
        sink_ready=1
        break
      fi
      sleep 0.25
    done
  fi

  enable_codec_path
  timeout 2 sudo -u "${USER_NAME}" -H \
    XDG_RUNTIME_DIR="${RUNTIME}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS}" \
    PULSE_SERVER="unix:${RUNTIME}/pulse/native" \
    PIPEWIRE_RUNTIME_DIR="${RUNTIME}" \
    wpctl set-mute @DEFAULT_AUDIO_SINK@ 0 >/dev/null 2>&1 || true
  timeout 2 sudo -u "${USER_NAME}" -H \
    XDG_RUNTIME_DIR="${RUNTIME}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS}" \
    PULSE_SERVER="unix:${RUNTIME}/pulse/native" \
    PIPEWIRE_RUNTIME_DIR="${RUNTIME}" \
    wpctl set-volume @DEFAULT_AUDIO_SINK@ 0.85 >/dev/null 2>&1 || true
  echo "OK: PipeWire 已就绪（${USER_NAME} ${RUNTIME} sink=${sink_ready}）"
  # timeout 不能直接调用 shell 函数，所以这里展开成 sudo。
  timeout 25 sudo -u "${USER_NAME}" -H \
    XDG_RUNTIME_DIR="${RUNTIME}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS}" \
    PULSE_SERVER="unix:${RUNTIME}/pulse/native" \
    PIPEWIRE_RUNTIME_DIR="${RUNTIME}" \
    /usr/bin/python3 -u "${SPEAKER_DIR}/boot_quack.py" || \
    echo "开机鸭叫没播出来，音频服务仍算启动成功。" >&2
  exit 0
fi

echo "音频启动超时：PipeWire 套接字还没出现，驾驶仍会启动。" >&2
exit 0
