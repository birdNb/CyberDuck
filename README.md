# CyberDuck —— 你的第一台桌面级机器人

桌面级赛博宠物 / 创意平台。鸭子会什么，由开发者持续定义。

当前默认主控：**Radxa Dragon Q6A 8G（QCS6490）**。  
可选评估主控：**立创·泰山派 RK3566**（见下文移植说明）。

---

## 硬件方案（概要）

| 子系统 | 方案 |
|---|---|
| 主控 | Radxa Dragon Q6A 8G / 评估：泰山派 RK3566 |
| 电源 | 4S 电池包（自带 BMS）；总电源分两路开关：主控电源 / 关节动力电源 |
| 电机 | HighTorque 关节模组；PCAN-USB → SocketCAN（`can0`，1 Mbps） |
| 摄像头 | IMX219 CSI（Q6A：CAM3 / MIPI CSI-2） |
| 姿态 | 两路 LSM6DSV16X IMU（I2C，地址 `0x6A` / `0x6B`） |
| 测距 | VL53L5CX 8×8 TOF（I2C，地址 `0x29`） |
| 音频 | Q6A 现用 USB 声卡；目标 / 泰山派可用板载功放或 ES8311+NS4150B |
| 其它 | 手柄、BLE 配网、触摸、WS2812 灯环 |

电路以成品模块为主，降低复刻门槛。开源 URDF 与强化学习策略相关部分，便于二次开发。

架构图（16:9 PPT 用）：`architecture_slides/`

---

## 仓库结构

| 目录 | 说明 |
|---|---|
| `motor/` | 电机总线 `ht_can.py`、姿态、轮驱 `wheel_run/`、`setup_can0.sh` |
| `locomotion/` | 50 Hz 运控、ONNX 策略引擎、手柄 |
| `Camera/` | CSI 采集、MJPEG 推流、人脸跟随 `FaceLocate/` |
| `IMU/` | LSM6DSV16X（当前为 GPIO 软件 I2C6） |
| `TOF/` | VL53L5CX 8×8 测距 |
| `speaker/` | 鸭叫合成与 ALSA 播放 |
| `BT/` | BLE GATT、配网、遥测 |
| `LED_Ring/` / `touch/` | 灯环、触摸 GPIO |
| `architecture_slides/` | 电路 / 软件 / 控制架构位图 |

---

## 当前平台快速上手（Q6A）

### 电机 CAN

```bash
cd motor
sudo ./setup_can0.sh          # can0 @ 1 Mbps
python3 scan_motors.py
python3 wheel_run/wheel_drive.py   # 轮驱（注意安全）
```

### 运控

见 `locomotion/README.md`。默认 50 Hz；Q6A 可用 ORT CPU，或 `engine qnn`（Hexagon HTP）。

### 摄像头

```bash
cd Camera
python3 stream_camera.py      # HTTP MJPEG，默认 8088
```

依赖 Qualcomm `libcamera` + GStreamer `libcamerasrc`（CAM3）。

### IMU / TOF

当前共用 Q6A **GPIO24/25 软件 I2C**，测试程序独占总线，不可同时跑。

```bash
cd IMU && python3 test_imu.py
cd TOF && python3 test_tof.py
```

### 音频 / 蓝牙

```bash
cd speaker && python3 test_speaker.py
cd BT && # 见 BT/README.md
```

---

## 移植到立创·泰山派 RK3566

**结论：业务逻辑与电机协议可复用；不能整仓拷贝即跑。** 摄像头、I2C 引脚、推理加速、音频设备、蓝牙平台识别需要适配。

### 泰山派硬件要点

| 项 | 说明 |
|---|---|
| 供电 | 外接座支持 **5～18 V** 入，4S（约 14.8～16.8 V）可直供主控，省主控侧 12 V 稳压；关节电源仍建议独立开关 |
| 音频 | 板载 **RK809**：麦克风 + Class-D 功放（约 **1.3 W @ 8 Ω**），可接 **8 Ω / 2 W** 喇叭；桌面近距离可先省外置音频板 |
| 摄像头 | 板载 **30Pin / 0.5mm MIPI CSI**，**不能直插**树莓派 15Pin 相机；需 30P 模组或转接板 + 驱动 |
| 内存 | 全功能同开约 **1.5～2.2 GB** → **建议 4GB**；2GB 仅能降画质/砍视觉硬撑 |
| 推理 | 当前约 0.76 MB、`obs[61]→14` 小策略，A55 上 **50 Hz 通常可行**（ORT CPU）；无 QNN HTP |

### 模块适配表

| 模块 | 能否直接用 | 要做什么 |
|---|---|---|
| `motor/ht_can.py`、姿态、轮驱 | 基本可以 | PCAN → `can0`，继续 `setup_can0.sh` |
| `locomotion` 策略 ONNX / URDF | 可复用 | 去掉 QNN，改 ORT CPU（或 RKNN）；重新编译 |
| `Camera/` | 否 | `libcamerasrc` → `v4l2src` / rkisp；换 30P 相机 |
| `IMU/` `TOF/` | 改引脚或总线 | 推荐改硬件 `/dev/i2c-x`；或改 `i2c_gpio.c` 的 chip/line |
| `speaker/` | 改设备名 | 指向板载 RK809 声卡（`/proc/asound/cards`） |
| `BT/` | 部分可复用 | `platform_detect.py` 增加 RK3566 / 泰山派识别 |
| `touch/` `LED_Ring/` | 改 GPIO | 按泰山派 gpiochip 与引脚重映射 |

### 建议移植顺序

1. 系统（Ubuntu/Debian）+ PCAN `can0` → `scan_motors.py`
2. 板载喇叭 → `test_speaker.py`
3. 硬件 I2C：IMU → TOF
4. 摄像头 `v4l2` 出图 → FaceLocate / 推流（建议先 640/720p）
5. 编译 `duck_locomotion`（CPU 推理）验证 50 Hz
6. BLE / 触摸 / 灯环
7. 全功能同开联调

### 树莓派 15Pin 摄像头

不要硬插。需 [CSI 转接板](https://oshwhub.com/wakkk3161554058/tai-shan-pai-csi-shuang-mu-dan-mu-zhuan-jie-ban)（如 OV5647）并适配设备树驱动；更省事是直接选官方测过的 **30P** 模组（如 OV5695）。

---

## 安全提示

- 关节上电前确认 CAN 终端电阻、H/L 极性、力矩限幅。
- 运控默认干跑；`--apply` 会驱动电机，退出前扶稳机体。
- 上电顺序建议：先主控 → `can0` UP → 再开关节电源；下电反序。

---

## 开源与使命

开源 URDF 与强化学习相关算法部分，降低桌面机器人与 RL 入门门槛。

**作品使命：** 一台会成长的创意平台。鸭子的能力随开发者不断进步——它是赛博宠物，也是如何被你定义的那只鸭。
