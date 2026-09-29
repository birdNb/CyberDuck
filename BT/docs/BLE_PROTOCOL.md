# microDuck BLE 通讯协议（精简）

> 全链路 **UTF-8 纯文本**，单条不含换行，经 **FFE2 notify** / **FFE1 write** 传输。

本目录从 `Bird_ws/Bt_voice_pull_bag` 移植连接能力，**已移除**动作调用、拖拽、摇杆控制、ROS 遥控。

| UUID | 方向 | 用途 |
| ---- | ---- | ---- |
| FFE0 | — | 服务 |
| FFE1 | 小程序 → 机器 | 控制指令 |
| FFE2 | 机器 → 小程序 | ACK + IP / 电量 |

---

## 一、上行：小程序 → 机器（FFE1 write）

### 1.1 连接与心跳

订阅 FFE2 后，板端立即推送 `pwr:88`，并每隔 2 秒推送 `IP:x.x.x.x`，共 3 次。**不再下发 `mode:` / `fsm:`**。

旧小程序若仍写入 `M_default`，只回 `ACK:M_default`，不做模式切换。

心跳（可选，保持链路）：`1`

摇杆包 `X:…,Y:…,Z:…` 会被**静默忽略**。

### 1.2 音量

```text
V {0-100}
```

例：`V 10`。范围钳位到 0–100。板端优先 `wpctl`，其次 PulseAudio / ALSA。

回执：`ACK:V 10`

### 1.3 WiFi 配网

```text
WIFI <SSID> <PASSWORD>
```

成功：`WIFI OK`，再推 `IP:x.x.x.x`  
失败：`WIFI FAIL <reason>`  
链路断开：`WiFi disconnected`

### 1.4 改名

```text
rename HT_12345678
```

回显：`rename HT_12345678`（需重新扫描连接）

### 1.5 对话语音（conversation_bag）

小程序发送录音文案 **前 5 个汉字拼音首字母（大写）**，板端播放同名 WAV，并回 `ACK:<CODE>`。

```text
BBMWX
```

语音包目录：`app/assets/conversation_bag/`（`manifest.json` + `BBMWX.wav` 等）。新对话会打断当前播放。

不足 5 字则取全部汉字，例如 `行走模式` → `XZMS`。

### 1.6 已移除（收到后忽略）

`PULL ON/OFF`、组合键动作、`GAIT`、`MP`、`LT ON/OFF`、`locate_face`、`neck0`、`P{n}Y{m}`、`sound ON/OFF`，以及 `M_init` / `M_protect` / `M_resetzero` / `M_tech` 等模式指令。

---

## 二、下行：机器 → 小程序（FFE2 notify）

### 2.1 局域网 IP

```text
IP:192.168.98.240
```

订阅 FFE2 后立即推一次，之后每隔 2 秒再推，共 3 次；之后仅在 IP 变化时再推。

开机若已连接 WiFi，先**阻塞播完**「蓝牙就绪待连接」，再把播放音量设为 **80%**，用中文逐个播报 IP：每个数字 **0.5 秒**，整段连播 **2 次**，每次间隔 **3 秒**。配网成功拿到新 IP 后同样播报。

依赖：`espeak-ng`（`sudo apt install -y espeak-ng`）。

### 2.2 电量（固定）

```text
pwr:88
```

本机无电池话题，**写死 88%**。订阅 FFE2 时推送。

---

## 三、连接流程

手机连接成功后，GATT 发现完成再停 BLE 广播；断连后恢复可扫描。

```text
扫描 FFE0 → createBLEConnection → setBLEMTU(247)
→ 发现特征 → 订阅 FFE2
→ 收到 pwr:88 / IP:…
（可选）write FFE1 "M_default" → ACK:M_default
```

只在微信小程序里连接，不要在手机系统蓝牙里配对。不再使用 `mode:` / `fsm:`。
