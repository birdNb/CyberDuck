#!/usr/bin/env bash
# 把 /home/radxa/microDuckPro 改名为 /home/radxa/cyberDuck，并更新系统服务里的路径。
# 用法：sudo bash /home/radxa/microDuckPro/rename_to_cyberDuck.sh
set -euo pipefail

OLD=/home/radxa/microDuckPro
NEW=/home/radxa/cyberDuck
SERVICES=(duck-ble.service led-boot.service)

if [ "$(id -u)" -ne 0 ]; then
  echo "需要 sudo 运行：sudo bash $0" >&2
  exit 1
fi
if [ ! -d "$OLD" ]; then
  echo "找不到 $OLD，可能已经改过名。" >&2
  exit 1
fi
if [ -e "$NEW" ]; then
  echo "$NEW 已存在，先处理掉再运行。" >&2
  exit 1
fi
if pgrep -f "$OLD/locomotion/scripts/watch_policy.py" >/dev/null; then
  echo "watch_policy.py 还在运行，先按 Ctrl+C 或 Q 退出再改名。" >&2
  exit 1
fi

echo "停止 duck-ble.service ..."
systemctl stop duck-ble.service || true

echo "改名 $OLD -> $NEW ..."
mv "$OLD" "$NEW"
chown radxa:radxa "$NEW"

for svc in "${SERVICES[@]}"; do
  f="/etc/systemd/system/$svc"
  if [ -f "$f" ]; then
    cp -a "$f" "$f.bak-microDuckPro"
    sed -i -e "s#$OLD#$NEW#g" -e 's#microDuckPro LED#cyberDuck LED#' "$f"
    echo "已更新 $f（备份 $f.bak-microDuckPro）"
  fi
done

systemctl daemon-reload
echo "启动 duck-ble.service ..."
systemctl start duck-ble.service
systemctl --no-pager --lines=0 status duck-ble.service || true

echo
echo "完成。新目录：$NEW"
echo "led-boot.service 下次开机生效。"
echo "Cursor 里用 File > Open Folder 重新打开 $NEW；已开的终端先 cd $NEW。"
