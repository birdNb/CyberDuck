# HT-DK 行走策略的输出关节顺序

网络输出是 **14 维**，名字 `actions`，形状 `[1, 14]`。顺序是 **左腿 5、头颈 4、右腿 5**。嘴不在这 14 维里。

观测里的关节角、关节速度、上一拍动作，用的是同一套下标。不要按电机 ID、不要按 URDF 文件里关节出现的先后（URDF 先写的是右腿），也不要按官方 MicroDuck 的左右反号零位来排。

## 下标

| 下标 | 关节 | 部位 | 正方向轴 | 站立零位 HOME (rad) | 缩放 SCALE |
|---|---|---|---|---|---|
| 0 | `l_hip_yaw_joint` | 左腿 | +Z | 0 | 0.25 |
| 1 | `l_hip_roll_joint` | 左腿 | +X | 0 | 0.10 |
| 2 | `l_hip_pitch_joint` | 左腿 | +Y | +0.68 | 0.50 |
| 3 | `l_calf_joint` | 左腿 | +Y | −0.88 | 0.50 |
| 4 | `l_ankle_joint` | 左腿 | +Y | +0.21 | 0.40 |
| 5 | `neck_pitch_joint` | 头颈 | +Y | +0.52 | 0.40 |
| 6 | `head_pitch_joint` | 头颈 | +Y | −0.50 | 0.40 |
| 7 | `head_yaw_joint` | 头颈 | +Z | 0 | 0.50 |
| 8 | `head_roll_joint` | 头颈 | 约 +X | 0 | 0.20 |
| 9 | `r_hip_yaw_joint` | 右腿 | +Z | 0 | 0.25 |
| 10 | `r_hip_roll_joint` | 右腿 | +X | 0 | 0.10 |
| 11 | `r_hip_pitch_joint` | 右腿 | +Y | +0.68 | 0.50 |
| 12 | `r_calf_joint` | 右腿 | +Y | −0.88 | 0.50 |
| 13 | `r_ankle_joint` | 右腿 | +Y | +0.21 | 0.40 |

轴是该关节在 URDF / `robot_walk.xml` 里的转轴。正角度是绕这个轴的右手定则。左右腿站立零位对称：髋 pitch 都是 +0.68，小腿都是 −0.88，踝都是 +0.21。

`head_roll_joint` 的轴是 `1 -0.0029596 0`，几乎就是朝前的 +X。

可以直接抄的数组，下标和上表一致：

```text
JOINTS = [
    "l_hip_yaw_joint", "l_hip_roll_joint", "l_hip_pitch_joint", "l_calf_joint", "l_ankle_joint",
    "neck_pitch_joint", "head_pitch_joint", "head_yaw_joint", "head_roll_joint",
    "r_hip_yaw_joint", "r_hip_roll_joint", "r_hip_pitch_joint", "r_calf_joint", "r_ankle_joint",
]
HOME  = [0, 0, 0.68, -0.88, 0.21,  0.52, -0.50, 0, 0,  0, 0, 0.68, -0.88, 0.21]
SCALE = [0.25, 0.10, 0.50, 0.50, 0.40,  0.40, 0.40, 0.50, 0.20,  0.25, 0.10, 0.50, 0.50, 0.40]
```

## 输出怎么变成关节目标

每一维是无量纲的原始动作，不是弧度，也不是力矩。位置目标是：

```text
q_des[i] = HOME[i] + SCALE[i] * action[i]
```

单位是弧度，零点是 URDF 机械零点，不是站立姿势。站着不动、动作为 0 时，各关节应停在上表的 HOME。

本次实机部署只把腿交给策略。下标 **0–4 和 9–13** 按上面的公式下发。下标 **5–8** 电机目标固定为 HOME 里的站立角，不使用 `action[5]` 到 `action[8]`。这 4 个数仍然要原样放进下一拍观测的 `last_action`，不能改成 0。

`mouth_joint` 在模型里是第 15 个执行器，策略输出没有它。嘴保持闭合，不要在 14 维后面再接一维。

## 和观测的对应关系

61 维输入里，下面三块和输出共用这 14 个下标：

| 观测 | 下标 | 含义 |
|---|---|---|
| `joint_pos` | 6:20 | `q_urdf - HOME` |
| `joint_vel` | 20:34 | 关节角速度，用上一拍 |
| `last_action` | 34:48 | 上一拍网络输出的 14 个数，缩放之前 |

例如输出第 2 维是左髋 pitch，观测里它的关节角在第 `6+2=8` 维，关节速度在第 `20+2=22` 维，上一拍动作在第 `34+2=36` 维。右腿从输出下标 9 开始，对应观测 `6+9=15`、`20+9=29`、`34+9=43`。

## 不要用的顺序

- URDF 文件里右腿写在左腿前面。策略不是这个顺序。
- 官方 MicroDuck 的关节名叫 `left_knee` / `right_knee`，而且左右髋 pitch 零位反号。这台机器人膝关节叫 `l_calf_joint` / `r_calf_joint`，左右同号。
- 电机总线扫描到的 ID 顺序不一定等于上表。上电后按关节名映射到这 14 个下标，再发目标。
