#!/bin/bash
# CSI 实时视频流：./start.sh 打开，./start.sh stop 关闭
set -euo pipefail
cd "$(dirname "$0")"

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export GST_INSTALL_PLUGINS_HELPER="${GST_INSTALL_PLUGINS_HELPER:-/bin/true}"
export PYTHONUNBUFFERED=1

PID_FILE="/tmp/microduck-csi-stream.pid"
LOG_FILE="$(pwd)/stream.log"
PORT="${PORT:-8088}"
FPS="${FPS:-30}"
WIDTH="${WIDTH:-3272}"
HEIGHT="${HEIGHT:-2464}"

usage() {
  cat <<EOF
用法: $(basename "$0") [start|stop|restart|status]

  start     后台启动满分辨率视频流（默认 3272x2464 @ 30fps）
  stop      关闭视频流
  restart   重启
  status    查看是否在运行

浏览器: http://127.0.0.1:${PORT}/
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

print_urls() {
  echo "  本机:   http://127.0.0.1:${PORT}/"
  hostname -I 2>/dev/null | tr ' ' '\n' | awk 'NF && $0 !~ /^127\./ {print "  局域网: http://"$1":'"${PORT}"'/"}'
}

cmd_start() {
  if is_running; then
    echo "视频流已在运行 (pid $(stream_pid))  ${WIDTH}x${HEIGHT} @ ${FPS}fps"
    print_urls
    return 0
  fi
  rm -f "$PID_FILE"
  nohup python3 -u stream_camera.py --port "$PORT" --fps "$FPS" --width "$WIDTH" --height "$HEIGHT" >>"$LOG_FILE" 2>&1 &
  echo $! >"$PID_FILE"
  for _ in $(seq 1 30); do
    if is_running && ss -tlnH "sport = :${PORT}" 2>/dev/null | grep -q .; then
      echo "已打开视频流 (pid $(stream_pid))  ${WIDTH}x${HEIGHT} @ ${FPS}fps"
      print_urls
      echo "关闭: $(pwd)/stop.sh   或   $(pwd)/start.sh stop"
      return 0
    fi
    sleep 0.5
  done
  if is_running; then
    echo "已打开视频流 (pid $(stream_pid))  ${WIDTH}x${HEIGHT} @ ${FPS}fps （端口稍后就绪）"
    print_urls
    echo "关闭: $(pwd)/stop.sh   或   $(pwd)/start.sh stop"
    return 0
  fi
  echo "启动失败，见 $LOG_FILE"
  tail -n 20 "$LOG_FILE" || true
  rm -f "$PID_FILE"
  return 1
}

cmd_stop() {
  local pid
  pid="$(stream_pid)"
  if [[ -n "${pid:-}" ]]; then
    kill "$pid" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.2
    done
    kill -9 "$pid" 2>/dev/null || true
  fi
  pkill -f '[s]tream_camera.py' 2>/dev/null || true
  if command -v fuser >/dev/null 2>&1; then
    fuser -k "${PORT}/tcp" >/dev/null 2>&1 || true
  fi
  rm -f "$PID_FILE"
  echo "已关闭视频流"
}

cmd_status() {
  if is_running; then
    echo "运行中 (pid $(stream_pid))  ${WIDTH}x${HEIGHT} @ ${FPS}fps"
    print_urls
  else
    echo "未运行"
    return 1
  fi
}

case "${1:-start}" in
  start) cmd_start ;;
  stop) cmd_stop ;;
  restart) cmd_stop; cmd_start ;;
  status) cmd_status ;;
  -h|--help|help) usage ;;
  *)
    usage
    exit 1
    ;;
esac
