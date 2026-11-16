# FEETECH HD1910M 独立驱动

这是 CyberDuck 的 HD1910M 磁编码 HLS 舵机目录。目录里有一条独立 Python 调试链路，以及一条已经接入 `duck_locomotion` 的 C++ 运行链路。

## 当前链路

### 独立调试链路

用于扫描、读取、单个舵机点动和机械 HOME 校准，不会启动行走策略：

```text
test_servo.py
    -> servo_controller.py
    -> bus_servo.py
    -> pyserial
    -> /dev/ttyUSB0
    -> CH340/USB-TTL
    -> HD1910M FT-SCS 总线
```

常用命令：

```bash
cd /home/sunteng/project/cyberDuck-with-git/cyberDuck/servo
python test_servo.py --scan
python test_servo.py --id 1 --read
python test_servo.py --id 1 --raw 2048 --speed 10 --torque-limit 100 --move
```

真实运动必须显式带 `--move`。

### 行走策略集成链路

行走程序的入口是 `locomotion/src/main.cpp`，C++ 通过 `servo/cpp` 控制三个头部舵机：

```text
locomotion/src/main.cpp
    -> servo::Bus
    -> servo/cpp/src/bus.cpp
    -> FT-SCS packet: 0xFF 0xFF + ID + length + instruction + params + checksum
    -> /dev/ttyUSB0
    -> CH340/USB-TTL
    -> HD1910M
```

头部执行器映射为：

| 策略下标 | 关节 | 总线 ID | C++ 控制方式 |
| ---: | --- | ---: | --- |
| 11 | `head_pitch` | 1 | 位置读写 |
| 12 | `head_yaw` | 2 | 位置读写 |
| 13 | `head_roll` | 3 | 位置读写 |

腿部 0～9 和 `neck_pitch` 10 仍然走 CAN；舵机不经过 CAN。

每个 50 Hz 控制周期的舵机流程是：

1. 从 `0x38` 读取当前位置，从 `0x3A` 读取速度。
2. 将舵机 raw 位置换算成策略仿真角 `q_sim`。
3. 将 14 维 `obs` 送入 ONNX，得到 14 维动作。
4. 用 `HOME + 0.25 * action` 计算头部目标角。
5. 按 `raw_HOME` 和 `direction` 将目标角换算成 0～4095 的舵机位置。
6. 向三个舵机写入目标位置。

舵机故障或反馈超时后，控制环会：

- 禁止策略进入 Track 状态；
- 关闭三个舵机的扭矩；
- `--apply` 只有在 CAN、舵机总线和校准状态都满足时才允许启动。

## 接线

- HD1910M 使用 TTL 半双工单总线。
- USB-TTL 转换器必须支持半双工总线方向切换，或使用飞特兼容的单总线接口板。
- 舵机供电使用符合规格的独立电源，USB-TTL 与舵机电源共地。
- 默认串口：1 Mbps、8N1、舵机 ID 1。

不要把普通的两根 TX/RX 全双工串口直接并接到舵机数据线上，除非转换器明确支持半双工单线模式。

## 使用已有 Miniconda 环境

当前机器上已经有可用的 `hi_env` 环境，不需要再创建 `cyberduck-servo`。该环境已验证包含：

```text
Python 3.8.20
pyserial 3.5
PyYAML 6.0.2
```

在 zsh 中启用环境：

```bash
source /home/sunteng/miniconda3/etc/profile.d/conda.sh
conda activate hi_env
```

检查依赖：

```bash
python -c "import serial, yaml; print('pyserial', serial.VERSION); print('PyYAML', yaml.__version__)"
```

如果当前 shell 已经初始化过 Conda，只需要：

```bash
conda activate hi_env
```

`environment.yml` 保留作为将来重新部署时的参考，但当前不要执行 `conda env create`。

## 独立调试配置

编辑 `servo_config.yaml`：

- `port`：USB-TTL 设备，例如 `/dev/ttyUSB0`
- `baudrate`：默认 `1000000`
- `id`：三个舵机的实际 ID
- `center_deg`：机械中位对应的逻辑角度
- `min_deg` / `max_deg`：机械安全限位
- `direction`：安装方向相反时设为 `-1`
- `center_raw`：机械中位对应的 HD1910M 原始位置，默认 `2048`
- `raw_min` / `raw_max`：原始位置安全范围，默认 `0` / `4095`
- `default_speed`：运行速度原始值，HD1910M 单位约为 `0.732 RPM/计数`，`0` 表示停止
- `torque_limit`：输出扭矩限制，范围 `0..1000`，单位为 `0.1%`

配置中的角度是逻辑角度。HD1910M 的位置范围按 0～4095 对应约 0～360°，逻辑中心通过 `center_raw` 映射。

## 独立测试

先只读扫描：

```bash
cd servo
python test_servo.py --scan
python test_servo.py --id 1 --read
```

真实运动必须显式带 `--move`：

```bash
python test_servo.py --id 1 --center --move
python test_servo.py --id 1 --angle 180 --duration 2 --move
python test_servo.py --all-center --move
```

首次更换电机型号时，先读取当前位置，再以当前位置为基准做小范围偏移。不要直接假设机械中位就是 `2048`：

```bash
python test_servo.py --id 1 --read
# 例如当前 position_raw=2100，再做约 10 度的小幅测试
python test_servo.py --id 1 --raw 2214 --speed 10 --torque-limit 100 --move
```

## 协议实现范围

- PING
- READ
- WRITE
- HD1910M 内存表小端 16 位读写
- 位置、速度、扭矩限制、加速度、扭矩开关
- 当前位置、速度、负载、电压、温度、状态、移动标志、电流和电流偏置读取
- ID 扫描

第一阶段不自动修改 EEPROM 参数，避免误改 ID、波特率或保护配置。修改 ID、波特率前应先备份当前配置，并确保总线上只有一个目标舵机。

## 行走策略配置

策略控制环不读取 `servo_config.yaml`，而是读取：

```text
locomotion/config/locomotion.cfg
```

其中：

```text
servo_port /dev/ttyUSB0
servo_baudrate 1000000
servo_speed 10
servo_acceleration 10
servo_torque_limit 100
servo_calibrated false

servo_joint head_pitch 1 2048 1 0.0
servo_joint head_yaw   2 2048 1 0.0
servo_joint head_roll  3 2048 1 0.0
```

`servo_joint` 格式为：

```text
servo_joint <name> <id> <raw_HOME> <direction> <HOME_rad>
```

这里的 `2048` 和方向 `1` 只是占位值，必须根据装配后的实际机械 HOME 校准。校准完成后才把：

```text
servo_calibrated true
```

并使用：

```bash
cd /home/sunteng/project/cyberDuck-with-git/cyberDuck/locomotion
../build-new/locomotion/duck_locomotion --apply
```

## 注意

1. HD1910M 默认地址、波特率和相位必须与配置一致；不要直接沿用 STS3215 的相位设置。
2. 首次测试把角度限位设窄，先单舵机、低速、低加速度验证方向。
3. 三个舵机不能共用不足以承受堵转电流的 USB 或主控 5 V 供电。
4. Python 调试配置和 C++ 行走配置是两套配置；修改 `servo_config.yaml` 不会自动修改 `locomotion.cfg`。
5. C++ 集成驱动当前读取位置和速度作为闭环反馈；温度、电压、状态和电流读取仍由 Python 工具提供，后续可再加入 C++ 安全监控。

