# microDuck BLE

从 `Bird_ws/Bt_voice_pull_bag` 移植的蓝牙 GATT 连接模块。

保留：**连接、IP 显示、音量调节、WiFi 配网**。  
移除：**动作调用、拖拽模式、ROS 遥控、头追、语音包**。  
电量：无硬件读数，固定上报 `pwr:88`。

开机若已连 WiFi，先播完「蓝牙就绪待连接」，再以 **80% 音量** 用中文播报 IP：每个数字 0.5 秒，连播 2 次、间隔 3 秒（需 `espeak-ng`）。

版本见 `VERSION`。

## 一键安装（开机自启）

```bash
cd ~/cyberDuck/BT
sudo ./install.sh
```

## 手动启动

```bash
cd ~/cyberDuck/BT/app
sudo ./start.sh
# 首次若 GATT 注册失败：
sudo ./start.sh --setup
```

## 常用命令

```bash
sudo systemctl status duck-ble
journalctl -u duck-ble -f
cd app && ./scripts/check.sh
```

## 小程序参数

| 项 | 值 |
| --- | --- |
| 广播名 | `HT_88888888`（可用 `rename HT_xxxxxxxx` 改） |
| 服务 | `0000FFE0-0000-1000-8000-00805F9B34FB` |
| 写入 | FFE1 |
| 通知 | FFE2 |
| 连接就绪 | 订阅 FFE2 后推送 `pwr:88`，`IP:` 每隔 2 秒连发 3 次（无 `M_default` 模式握手） |
| 对话语音 | FFE1 发拼音首字母短码，如 `BBMWX`，播放 `conversation_bag` |
| 音量 | `V 0` … `V 100` |

协议：`docs/BLE_PROTOCOL.md`
