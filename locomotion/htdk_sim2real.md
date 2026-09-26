# HT-DK_V1 行走策略 sim2real 部署说明

这份说明交给机器人端的实现者。目标是把 `cyberDuck_RL` 里已经训好的行走策略，按 50 Hz 跑在 HT-DK_V1 实机上。软件结构参考 [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck)，硬件映射、关节顺序、零位和力矩上限以本文为准。不要把官方 MicroDuck 的关节角、HOME、XL330/BAM 电机模型抄过来。

当前策略只让双腿走路。脖子和头保持站立零位，不跟策略输出。第一次上电，所有参与运动的关节力矩先限制在 **0.5 N·m**。

## 0. 给实现者的边界

做这些事：

1. 用本文的观测公式组 61 维输入，用 ONNX 推出 14 维动作。
2. 14 维里只有 10 个腿关节下发到电机。脖子 4 个关节的网络输出丢弃，电机目标锁在零位。
3. 行走指令来自北通手柄的 2.4G 接收器，当成 Linux 手柄读，不要走官方鸭子的蓝牙配对流程。
4. 先跑 `scripts/calibrate_imu_mount.py`，把 IMU 轴映射到仿真 `base_link`。对不齐就不要闭合策略。
5. 力矩上限从 0.5 N·m 起步。方向核对通过之前不要提高。

不要做这些事：

- 不要加载官方 MicroDuck 的 `policy.onnx`、关节表、`DEFAULT_POSE`。官方髋 pitch 左右反号，这台机器人两条腿同号。
- 不要把策略的脖子输出接到电机上。
- 不要在实机上再做一次观测归一化。导出的 ONNX 里已经包含了。
- 不要用行走过程中的原始加速度代替 `projected_gravity`。
- 不要把官方 `robotd` 里的 XL330 电流环、BAM 参数写进这台机器人的电机。
- 不要在第一次上电使用训练时的 3.7 N·m 力矩上限。

## 1. 参考软件，以及要换掉的部分

官方仓库是鸭子的运行时，不是这台机器人的模型。值得沿用的只有结构：

| 官方进程 | 职责 | HT-DK 上怎么做 |
|---|---|---|
| `robotd` | 50 Hz 控制环，读传感器，跑 ONNX，写电机 | 自己写一条同样节拍的环。关节名、零位、PD、力矩上限用本文 |
| `padd` | 读手柄，把意图送给控制环 | 读北通 2.4G 接收器对应的 `/dev/input/js*`，不使用官方蓝牙手柄绑定 |
| ONNX 策略 | `obs[1,61] -> actions[1,14]`，归一化在模型内部 | 用本文第 3 节的导出命令，从本次训练的检查点导出 |

官方文档入口：

- 总结构：<https://github.com/pollen-robotics/microduck/blob/main/docs/design/architecture.md>
- 运行时说明：<https://github.com/pollen-robotics/microduck/blob/main/README.md>

控制环和手柄读取可以分成两个线程，中间只传三个速度指令和一个使能位。电机总线由控制环独占。

## 2. 实机和仿真必须一致的物理约定

### 2.1 坐标系

躯干坐标系就是 URDF 的 `base_link`，也是 `robot_walk.xml` 里 IMU site 的坐标系。site 名叫 `imu`，位置在躯干原点，没有额外旋转。

- +X：朝前，脚尖方向
- +Y：朝左。左髋安装点在 y = +0.0275 m，右髋在 y = -0.0275 m
- +Z：朝上。站立时躯干原点高度约 0.1648 m
- 角速度用右手定则

策略看到的躯干角速度顺序是 roll、pitch、yaw，也就是绕 +X、+Y、+Z。

符号核对，部署前用手动慢转确认，不要靠猜：

| 动作 | 机体角速度 |
|---|---|
| 右肩向下 | wx > 0 |
| 低头（前倾） | wy > 0 |
| 从上往下看逆时针 | wz > 0 |

直立静止时，`projected_gravity` 必须接近 `(0, 0, -1)`。这是重力加速度方向在机体系里的单位向量，不是加速度计读数。加速度计静止时测到的比力大约是 `(0, 0, +9.81)`。

### 2.2 时间和网络

- 策略周期 50 Hz，周期 0.02 s。仿真里物理步是 0.005 s，每 4 步决策一次，实机只需要保证策略 50 Hz。
- 两次策略更新之间，位置目标保持不变。
- 网络是 MLP，没有隐状态。输入名 `obs`，形状 `[1, 61]`，float32。输出名 `actions`，形状 `[1, 14]`，是高斯分布的均值，不是采样。
- 隐藏层 512、256、128，激活 ELU。这些已经在 ONNX 里，实机不要重写网络。
- 观测归一化是训练时的 `EmpiricalNormalization`，导出时烤进 ONNX。实机把原始物理量送进去。
- 关节速度观测在仿真里滞后 1 个策略周期（20 ms）。实机用上一拍的关节速度，不要用当前拍。
- 陀螺仪在仿真里随机滞后 0 或 1 拍。实机用最新一拍即可。
- 仿真动作延迟大约 5–15 ms，小于一个策略周期。实机在本周期算出目标后尽快发给电机，不要再人为延后 20 ms。

### 2.3 关节顺序

下面 14 个下标同时用于：关节角观测、关节速度观测、上一拍动作、网络输出。嘴不在其中。

| 下标 | 关节 | 轴向 | URDF 范围 (rad) | 站立零位 (rad) | 动作缩放 | 训练力矩上限 | 首次上电力矩上限 |
|---|---|---|---|---|---|---|---|
| 0 | `l_hip_yaw_joint` | +Z | -0.436 ~ 0.176 | 0 | 0.25 | 3.7 | 0.5 |
| 1 | `l_hip_roll_joint` | +X | -0.384 ~ 0.080 | 0 | 0.10 | 3.7 | 0.5 |
| 2 | `l_hip_pitch_joint` | +Y | -1.571 ~ 1.571 | +0.175 | 0.50 | 3.7 | 0.5 |
| 3 | `l_calf_joint` | +Y | -1.571 ~ 1.571 | 0 | 0.50 | 3.7 | 0.5 |
| 4 | `l_ankle_joint` | +Y | -1.571 ~ 1.571 | -0.175 | 0.40 | 3.7 | 0.5 |
| 5 | `neck_pitch_joint` | +Y | -1.200 ~ 1.571 | 0 | 0.40 | 3.7 | 保持，不跟策略 |
| 6 | `head_pitch_joint` | +Y | -1.240 ~ 1.571 | 0 | 0.40 | 3.7 | 保持，不跟策略 |
| 7 | `head_yaw_joint` | +Z | -2.967 ~ 2.967 | 0 | 0.50 | 0.3 | 保持，不跟策略 |
| 8 | `head_roll_joint` | 约 +X | -0.436 ~ 0.436 | 0 | 0.20 | 0.3 | 保持，不跟策略 |
| 9 | `r_hip_yaw_joint` | +Z | -0.176 ~ 0.436 | 0 | 0.25 | 3.7 | 0.5 |
| 10 | `r_hip_roll_joint` | +X | -0.080 ~ 0.384 | 0 | 0.10 | 3.7 | 0.5 |
| 11 | `r_hip_pitch_joint` | +Y | -1.571 ~ 1.571 | +0.175 | 0.50 | 3.7 | 0.5 |
| 12 | `r_calf_joint` | +Y | -1.571 ~ 1.571 | 0 | 0.50 | 3.7 | 0.5 |
| 13 | `r_ankle_joint` | +Y | -1.571 ~ 1.571 | -0.175 | 0.40 | 3.7 | 0.5 |

左右髋 pitch、左右踝的零位同号。正方向是父连杆坐标系下绕上表轴向的右手定则，零点是 URDF 的机械零点，不是站立姿势。

腿的位置目标：

```text
q_des[i] = HOME[i] + SCALE[i] * action[i]     # i 属于双腿下标 0..4 和 9..13
```

脖子和头：

```text
q_des[5] = q_des[6] = q_des[7] = q_des[8] = 0
```

`action[5:9]` 仍然写进下一拍的 `last_action`，这样策略看到的动作历史和训练一致，只是电机不执行。

`q_des` 再夹到该关节的 URDF 硬限位内。限位外的目标一律裁掉。

站立零位数组，下标与上表一致：

```text
HOME = [0, 0, 0.175, 0, -0.175,  0, 0, 0, 0,  0, 0, 0.175, 0, -0.175]
SCALE = [0.25, 0.10, 0.50, 0.50, 0.40,  0.40, 0.40, 0.50, 0.20,  0.25, 0.10, 0.50, 0.50, 0.40]
```

### 2.4 力矩

仿真执行器是 MuJoCo 位置伺服，不是电流环辨识模型：

```text
tau = clamp(kp * (q_des - q) - kv * q_dot,  -tau_max, +tau_max)
```

| 关节 | kp | kv | 首次 tau_max | 以后允许提高到 |
|---|---|---|---|---|
| 10 个腿关节 | 60 | 2.0 | 0.5 N·m | 不超过 3.7 N·m |
| `neck_pitch`、`head_pitch` | 20 | 0.8 | 保持零位，同样先用 0.5 | 不超过 3.7 |
| `head_yaw`、`head_roll` | 8 | 0.3 | 保持零位 | 不超过 0.3。不要写成 0.5，额定只有 0.3 |

0.5 N·m 配合腿的 kp=60，大约 0.008 rad（0.5°）的位置误差就会饱和。第一次上电时期望的现象是：机器人几乎走不动，只是顺着指令轻轻挣。这是对的。方向和 IMU 核对完成之前，不要把 0.5 调大。

若电机驱动是位置模式而不是力矩模式：把驱动器的力矩/电流限制设成上表的 `tau_max`，位置增益尽量靠近 kp、kv。软件侧仍然要限制 `|q_des - q|`，腿关节第一次上电不超过 `0.5/60 ≈ 0.0083 rad`，避免驱动器内部增益未知时一下顶满。

力矩单位是 N·m。用户口头说的「0.5 N」在这里按 0.5 N·m 执行。

## 3. 导出 ONNX

在训练工程里执行，不要手写 `state_dict` 转 ONNX。`play` 能看不代表实机文件里带了归一化。

```bash
cd cyberDuck_RL
uv run scripts/export.py Mjlab-Velocity-Flat-MicroDuck \
  --checkpoint-file logs/rsl_rl/velocity/2026-09-27_01-50-18_velocity/model_3250.pt \
  --onnx-file htdk_velocity.onnx
```

把检查点换成实际要部署的那一个。导出后在机器人上确认：

- 输入 `obs`，`[1, 61]`
- 输出 `actions`，`[1, 14]`
- 全零观测的输出是有限值，没有 NaN

推理只用这个 ONNX。critic 不部署。

## 4. 61 维观测，按这个顺序填

每一项都是原始物理量。不要减均值、不要除标准差、不要乘训练时的噪声。

| 下标 | 名称 | 单位 | 怎么填 |
|---|---|---|---|
| 0:3 | `base_ang_vel` | rad/s | `R @ gyro`。顺序 wx, wy, wz。用最新一拍 |
| 3:6 | `projected_gravity` | 无量纲 | 融合姿态转到机体系后的重力方向。直立静止约为 `(0, 0, -1)` |
| 6:20 | `joint_pos` | rad | `q_urdf[i] - HOME[i]`，14 个关节，含脖子。脖子锁在零位时这 4 维应接近 0 |
| 20:34 | `joint_vel` | rad/s | 上一拍的 `q_dot_urdf`，不是当前拍。第一拍填 0 |
| 34:48 | `last_action` | 无量纲 | 上一拍网络的 14 维原始输出。第一拍填 0。这里是缩放前的数 |
| 48:51 | `twist` | m/s, m/s, rad/s | 手柄指令 `lin_vel_x, lin_vel_y, ang_vel_z` |
| 51:55 | `head_command` | rad | 固定 0, 0, 0, 0。对应 neck_pitch, head_pitch, head_yaw, head_roll |
| 55:61 | `body_command` | m / rad | 固定 0。顺序 x, y, z, roll, pitch, yaw |

关节角必须先换成 URDF 角度再减 HOME。如果编码器在站立姿势标零，而不是在 URDF 机械零点标零，则：

```text
q_urdf = q_from_standing_zero + HOME
joint_pos_obs = q_from_standing_zero
```

两种标零最后得到的 `joint_pos_obs` 相同。推荐编码器读数直接是 URDF 角度，这样限位判断不用再换算。正方向必须和 URDF 轴一致，见第 8 节的单关节点动。

`projected_gravity` 的算法，四元数与 MuJoCo 一样是 wxyz，`R_body_from_world` 由融合姿态给出：

```text
projected_gravity = R_body_from_world @ (0, 0, -1)
```

融合所用的陀螺仪和加速度，先乘第 6 节求出的 `R`，再送进滤波器。行走时不要用 `-normalize(accel)` 代替这一项，脚落地时的加速度冲击会把策略打偏。静止核对时两者应该接近。

`lin_vel_x` 为正表示沿机头方向走。`lin_vel_y` 为正表示向机器人左侧走。`ang_vel_z` 为正表示从上往下看逆时针转。

训练时指令范围是 vx ∈ [-0.4, 0.4]，vy ∈ [-0.3, 0.3]，wz ∈ [-1.0, 1.0]。实机手柄满杆不要超过这个范围。第一次上电再乘 0.25，也就是满杆大约 vx 0.10、vy 0.075、wz 0.25。

头颈指令和躯干位姿指令在本次部署里永远为 0。训练后期头颈指令范围很大，但那是仿真课程；实机明确不跟头。

## 5. 50 Hz 控制环

伪代码里的单位都是 rad、rad/s、N·m。`policy` 是 ONNX 推理。

```text
tau_max_leg = 0.5          # 方向核对通过前不许改
cmd_scale = 0.25           # 同上
last_action = zeros(14)
q_dot_prev = zeros(14)
q_des_hold = copy(HOME)

every 0.02 s:
    q = read_urdf_joint_positions()          # 14 维，含脖子
    q_dot = read_urdf_joint_velocities()     # 没有速度反馈就用 (q - q_prev) / 0.02，再整体延后一拍
    gyro_b, quat_b = imu_in_base_link()      # 已经乘过安装旋转 R

    if not enabled or estop or loop_late:
        # 松开使能：停在当前角度，不追策略，也不突然扯回 HOME
        q_des = q
        send_position_with_torque_limit(q_des, tau_max=0 on estop else tau_max_leg)
        q_dot_prev = q_dot
        q_prev = q
        continue

    obs[0:3] = gyro_b
    obs[3:6] = rotate_gravity(quat_b)        # (0,0,-1) 转到机体系
    obs[6:20] = q - HOME
    obs[20:34] = q_dot_prev
    obs[34:48] = last_action
    obs[48:51] = clip(gamepad_twist * cmd_scale, training_limits * cmd_scale)
    obs[51:61] = 0

    action = policy(obs)                     # shape (14,), 有限值
    last_action = action

    q_des = HOME.copy()
    for i in [0,1,2,3,4, 9,10,11,12,13]:
        q_des[i] = clamp(HOME[i] + SCALE[i] * action[i], URDF_LO[i], URDF_HI[i])
    # 5,6,7,8 保持 HOME，也就是 0

    send legs with kp=60, kv=2, tau_max=0.5
    send neck/head hold at 0 with their own kp/kv and tau_max

    q_dot_prev = q_dot
    q_prev = q
```

使能条件同时满足才把 `enabled` 置真：

1. 急停没有按下。
2. 北通的使能键正按住（死人开关）。松开的那一拍立刻停止跟随策略。
3. 每个关节的 `|q - HOME|` 小于 0.35 rad。有关节差得更多就拒绝使能，先用手动摆回站立附近。
4. IMU 校准文件已加载，并且最近 0.5 秒直立静止时 `projected_gravity` 与 `(0, 0, -1)` 的夹角小于约 8°。没做这项检查就不要使能。
5. 控制周期没有连续两拍超过 30 ms。超时则目标冻结在当前角度，力矩保持但不再跑策略；超过 100 ms 则力矩清零。

上电顺序：

1. 力矩为 0，只读编码器、IMU、手柄。
2. 打印 14 个 `q - HOME`、重力向量、手柄三轴。人眼确认符号。
3. 用 2 秒把腿和脖子的力矩上限从 0 升到 0.5，目标一直是 HOME，不跑策略。脖子应停在初始姿态。
4. 按住使能键后才进入上面的策略环，并且 `cmd_scale = 0.25`。
5. 单关节点动和 IMU 三项慢转都通过之后，才允许人把 `tau_max_leg` 提到 1.0，然后再到 2.0。3.7 是训练上限，不是第一天的目标。`head_yaw` 和 `head_roll` 任何时候都不超过 0.3。

## 6. IMU 安装校准

IMU 装在躯干上，尽量靠近 `base_link` 原点，不要装在头上。轴向尽量和躯干平行。剩下的安装误差用脚本求一个常数旋转矩阵，不要在策略里临时改符号。

脚本：`cyberDuck_RL/scripts/calibrate_imu_mount.py`。

它求的 `R` 满足 `R @ v_sensor = v_body`。静止时：

```text
projected_gravity = -normalize(R @ accel)
base_ang_vel = R @ gyro
```

行走时重力改由融合四元数计算，但陀螺仪和加速度在进入滤波器之前都要先乘这个 `R`。

### 6.1 先确认脚本本身

不接硬件：

```bash
python scripts/calibrate_imu_mount.py --self-test
```

应打印 `self-test 通过`。

### 6.2 采集

机器人放在站立零位附近，电机可以力矩为 0，用人扶稳。串口每行六个数 `ax ay az gx gy gz`。加速度 m/s² 或 g 都可以。陀螺仪默认 rad/s；若输出是 deg/s，加 `--gyro-unit deg`。

```bash
python scripts/calibrate_imu_mount.py \
  --serial /dev/ttyUSB0 --baud 115200 \
  --seconds 2 --out imu_mount.json
```

按提示采六段：

1. `upright`：站直放稳。
2. `nose_down`：前倾大约 20° 到 40°，稳住。
3. `left_down`：机器人自己的左侧降低大约 20° 到 40°，稳住。
4. `roll_right_down`：缓慢地把右肩往下转。
5. `pitch_nose_down`：缓慢低头。
6. `yaw_ccw`：从上往下看缓慢逆时针转。

也可以先把数据存成 CSV，列名 `pose,ax,ay,az,gx,gy,gz`，pose 用上面六个名字，再：

```bash
python scripts/calibrate_imu_mount.py --csv imu_log.csv --out imu_mount.json
```

通过时会写下 `imu_mount.json`。直立重力应接近 `[0, 0, -1]`。三轴慢转必须各自在对应分量上为正。有一项失败脚本会拒绝保存。失败时重采，不要手工翻转某一列来凑结果。芯片坐标系和躯干坐标系差一个刚体旋转时，一个 `R` 就够；如果陀螺仪和加速度计不是同一套轴向，先把传感器固件的轴定义查清再校准。

### 6.3 部署时怎么用

```python
R = np.array(json.load(open("imu_mount.json"))["R"])  # 3x3
gyro_b = R @ gyro_sensor
accel_b = R @ accel_sensor
# 把 gyro_b、accel_b 送进姿态融合。融合四元数表达的是 base_link，不是传感器外壳。
# obs[0:3] = gyro_b
# obs[3:6] = R_body_from_world @ [0, 0, -1]
```

训练时还有最多 6° 的 IMU 安装随机，所以校准残差在大约 5° 以内可以接受。超过 8° 不要使能策略。

## 7. 北通手柄（2.4G 接收器）

接收器插上后应出现为 Linux 操纵杆，常见节点 `/dev/input/js0`。先确认设备，再填轴号。不同型号的北通轴号不一样，语义以本节为准，轴号以实机探测为准。

```bash
lsusb
grep -A 8 -i -E 'betop|beitong|北通' /proc/bus/input/devices || true
sudo apt install joystick
jstest /dev/input/js0
```

`jstest` 里推杆，记下编号。需要的语义：

| 操作 | 指令 | 训练满杆（第一次上电再乘 0.25） |
|---|---|---|
| 左摇杆向前 | `lin_vel_x > 0` | 0.40 m/s |
| 左摇杆向左（机器人左侧，不是操作者左侧） | `lin_vel_y > 0` | 0.30 m/s |
| 右摇杆向左 | `ang_vel_z > 0`（逆时针） | 1.0 rad/s |
| 按住指定肩键或扳机 | 策略使能 | 松开即停止跟随 |
| 另一个键 | 急停，全部力矩立即为 0 | |

摇杆死区建议 0.12，死区以内指令为 0。滤波用一阶低通即可，时间常数大约 0.1 s，避免指令阶跃。

不要使用官方仓库 `padd` 的蓝牙绑定流程，除非这块北通接收器恰好枚举成它们写死的设备。读 Linux joystick 事件即可。内核 `js_event` 是 8 字节：`uint32 time`、`int16 value`、`uint8 type`、`uint8 number`。轴值 int16 满量程对应 ±1，再乘上面的速度上限和 `cmd_scale`。

使能键没有按住时，速度指令即使摇杆偏了也必须当成 0，并且不跑策略。急停优先于使能。

## 8. 上电前的单关节符号核对

力矩上限 0.5 N·m，一次只动一个腿关节，目标相对当前角度 +0.05 rad，其余关节保持。人眼看的方向：

| 关节 | 正方向应看到的运动 |
|---|---|
| 髋 yaw | 从上往下看，该腿绕竖直轴逆时针 |
| 髋 roll | 绕朝前的轴，按右手定则。左腿正方向幅度很小，外侧限位只有 +0.08 rad，点动用 +0.03 和 -0.03 各一次 |
| 髋 pitch | 绕向左的轴。正角度在零位基础上是站立那 +0.175 的方向 |
| 小腿 | 绕向左的轴 |
| 踝 | 绕向左的轴。站立零位是 -0.175，点动不要打到限位 |

右腿轴向和左腿相同，不是镜像反号。某一轴方向反了，就把该电机的编码器符号反过来，然后重新做这一轴的点动。不要在观测里对单个关节打补丁的同时又在电机侧打一次，那样会打两次。

脖子四个关节只做保持，不做策略点动。确认它们在力矩从 0 升到 0.5 的过程中停在初始外观，没有自己点头或转头。

## 9. 和仿真不一致时会怎样

| 现象 | 先查 |
|---|---|
| 一使能就猛地一抽 | 力矩是否仍是 0.5；`q_des` 是否用了 `HOME + scale * action`；编码器零点是不是把站立姿势当成了 0 却没有加 HOME |
| 指令向前，机器人向后或向侧 | 手柄符号，以及 IMU 的 yaw 是否反了 |
| 站着不动但腿抖 | `projected_gravity` 在直立时不是 `(0,0,-1)`，或关节速度没有延后一拍、符号反了 |
| 头跟着走路晃 | 脖子目标被接上了策略输出。应锁 0 |
| 输出数值正常但电机几乎不动 | 0.5 N·m 饱和，这是第一次上电的预期。确认方向后再提力矩 |
| 仿真里能走、实机一加载就姿态怪异 | ONNX 是否未带归一化；或者用了官方鸭子的关节顺序和反号 HOME |

## 10. 建议的实机文件

机器人端至少留下这些，方便下次换检查点：

- `htdk_velocity.onnx`：第 3 节导出的文件
- `imu_mount.json`：校准脚本的输出
- `gamepad_map.json`：`jstest` 记下的轴号、使能键、急停键
- 一份启动日志：14 个编码器角度、重力向量、力矩上限、检查点文件名

力矩上限和 `cmd_scale` 放在启动参数里，默认 `0.5` 和 `0.25`。不要把提高后的值写死成代码默认值。
