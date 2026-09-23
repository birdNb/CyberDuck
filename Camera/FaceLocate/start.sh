#!/bin/bash
# 人脸跟踪：./start.sh 进入，./start.sh stop 退出
set -euo pipefail
cd "$(dirname "$0")"

export GST_INSTALL_PLUGINS_HELPER="${GST_INSTALL_PLUGINS_HELPER:-/bin/true}"
export PYTHONUNBUFFERED=1

PID_FILE="/tmp/microduck-locate-face.pid"
LOG_FILE="$(pwd)/locate_face.log"

usage() {
  cat <<EOF
用法: $(basename "$0") [start|stop|restart|status] [--gui] [--invert-yaw] [--invert-pitch]

  start     后台启动人脸跟踪（脖子先到 stand，再 11/13 锁人脸居中）
  stop      停止并回 stand
  restart   重启
  status    是否在运行

前台调试: python3 locate_face.py [--gui]
EOF
}

stream_pid() {
  if [[ -f "$PID_FILE" ]]; then
    cat "$PID_FILE"
  fi
}

is_running() {
  local pid
  pid="$(stream_pid)"
  [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null
}

cmd_start() {
  if is_running; then
    echo "人脸跟踪已在运行 (pid $(stream_pid))"
    return 0
  fi
  rm -f "$PID_FILE"
  nohup python3 -u locate_face.py "$@" >>"$LOG_FILE" 2>&1 &
  echo $! >"$PID_FILE"
  sleep 0.4
  if is_running; then
    echo "已启动人脸跟踪 (pid $(stream_pid))"
    echo "日志: $LOG_FILE"
    echo "关闭: $(pwd)/start.sh stop"
    return 0
  fi
  echo "启动失败，见 $LOG_FILE"
  tail -n 30 "$LOG_FILE" || true
  rm -f "$PID_FILE"
  return 1
}

cmd_stop() {
  local pid
  pid="$(stream_pid)"
  if [[ -n "${pid:-}" ]]; then
    kill "$pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.2
    done
    kill -9 "$pid" 2>/dev/null || true
  fi
  pkill -f '[l]ocate_face.py' 2>/dev/null || true
  rm -f "$PID_FILE"
  echo "已停止人脸跟踪"
}

cmd_status() {
  if is_running; then
    echo "运行中 (pid $(stream_pid))"
  else
    echo "未运行"
    return 1
  fi
}

action="${1:-start}"
if [[ "$action" == -* ]]; then
  set -- start "$@"
  action=start
else
  shift || true
fi

case "$action" in
  start) cmd_start "$@" ;;
  stop) cmd_stop ;;
  restart) cmd_stop; cmd_start "$@" ;;
  status) cmd_status ;;
  -h|--help|help) usage ;;
  *)
    usage
    exit 1
    ;;
esac
