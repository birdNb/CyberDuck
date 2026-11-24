倒地起身策略部署说明（model_1999.pt）

这份目录是 model_1999.pt、env.yaml、agent.yaml。
权重契约是 cyberduck_standup_obs46_v3_face_down_locked_wheels，任务 CyberDuckV2-Flat-StandUp，迭代 1999。
运行名 standup_face_down_symmetric_v4。actor 是 MLP（512-256-128，ELU），带观测归一化，没有 LSTM。
控制周期 0.02 s。除脖子 latent -0.089 外，位置关节 latent 为 0。轮子动作 scale 为 0。
没按 LB 时每拍推理但不执行。重力 z 高于 -0.7 并持续 0.3 s 算倒地，这时按 LB 才把关节目标发出去。再按 LB 退出。

当前 sport_ctrl/wheel_run/getup_policy.py 还在找 onnx，并按行走策略拼 61 维观测。
它不会加载这份 pt，倒地后按 LB 也不会用这份权重动关节。下面是这份权重自己的约定，和实机差在哪里。


一、actor 观测 46 维，按这个顺序拼，再做 (obs-mean)/std

下标    内容                         维数
0:3     base_ang_vel                 3    机体角速度 rad/s，xyz
3:6     projected_gravity            3    投影重力，直立约 (0, 0, -1)
6:18    joint_pos                    12   见第二节，不含两个轮子
18:32   joint_vel                    14   上面 12 个再加右轮、左轮
32:46   actions                      14   上一拍原始动作：12 个位置 + 2 个轮速

critic 53 维只在训练用，实机不喂。多出来的是 base_lin_vel 3 维和 wheel_contact。
actor 没有线速度，也没有摇杆指令。

训练时这两项 IMU 用的是 base_ang_vel_imu_misaligned / projected_gravity_imu_misaligned，
max_angle_deg 6°，再加角速度噪声 ±0.03、重力噪声 ±0.01。
这是训练时的随机安装误差，部署不要再人为转 6°，也不要加噪声。
部署要的是已经变到 base_link 的名义值。

实机 IMU（IMU/lsm6dsv16x.py，LSM6DSV16X）：
  gyro 单位 rad/s，重力是 -normalize(加速度)，再乘安装矩阵 R。
  约定 +X 前、+Y 左、+Z 上。直立重力 (0, 0, -1)。前倾 gx>0，低头 gyro_y>0。
  和 MuJoCo 的 projected_gravity 同向，也和行走策略 watch_policy 用的是同一路读数。
  所以 0:6 的 xyz 顺序和实机一致。
  安装矩阵应读 sport_ctrl/locomotion/config/imu_mount.json（相对单位阵约 2.3°）。
  现在加载路径仍是 cyberDuck/locomotion/config/imu_mount.json，这个文件已经不在了，
  读不到就退回单位阵。差大约 2°，轴向没有反。


二、位置关节顺序：右腿在前，不含轮子

观测 joint_pos 和动作 joint_pos 都是这 12 个，preserve_order：

序号  仿真关节            实机                         行走策略里的 SIGN
0     r_hip_yaw_joint     CAN 5  右髋 yaw             -1
1     r_hip_roll_joint    CAN 4  右髋 roll            -1
2     r_hip_pitch_joint   CAN 3  右髋 pitch           -1
3     r_calf_joint        CAN 2  右膝                 +1
4     l_hip_yaw_joint     CAN 10 左髋 yaw             -1
5     l_hip_roll_joint    CAN 9  左髋 roll            +1
6     l_hip_pitch_joint   CAN 8  左髋 pitch           +1
7     l_calf_joint        CAN 7  左膝                 -1
8     neck_pitch_joint    CAN 12 脖子俯仰             +1
9     head_pitch_joint    CAN 11 头俯仰               +1
10    head_yaw_joint      舵机 13 头 yaw              +1
11    head_roll_joint     舵机 14 头 roll             +1

和现在行走策略的 14 关节顺序不一致。行走是左腿五个（含左轮）→ 脖子 → 头俯仰 →
头 yaw → 头 roll → 右腿五个（含右轮）。不能按那个下标去套这份权重。

头 yaw / roll 在这台轮式机上是舵机 13、14，不是行走表里的 CAN 14、13。
那张表把 yaw 写成 14、roll 写成 13，和 pose_config.yaml 的舵机对调。
嘴（舵机 15）不在策略里。

电机角到仿真角只乘这一次 SIGN。编码器 0 对仿真 0（腿伸直是 0°）：
  q_sim = SIGN × q_motor
行走 HOME 只属于行走策略，不起身不用那组 0.68 / -0.88。
轮子速度同号：
右轮 CAN 1 装反，wheel_sign=-1，和 r_ankle 的 SIGN=-1 相同；
左轮 CAN 6 为 +1。仿真轮速 = SIGN * 电机速度，也就是驾驶日志里“前进为正”的那个速度。


三、joint_pos 不是“弧度减去默认角”

yaml 里 joint_pos_rel、biased: true，看起来像 q - default。
权重契约写的是 mapping=joint_range_tanh_home_offset，
encoder_bias_order=map_then_subtract_then_bound。
latent_offset 与 artanh((q_default-中点)/半宽) 一致，误差小于 1e-7，按这个编码：

  mid = (lo+hi)/2
  half = (hi-lo)/2
  z = artanh( clip((q_sim-mid)/half, ±0.999999) )
  obs = z - latent_offset

默认姿态上这 12 维是 0。直接喂 q_sim-default（弧度）和训练分布不符。

默认角 offset、限位、latent_offset（都是仿真弧度）：

关节            offset     限位                     latent_offset
r_hip_yaw       0          ±0.436                   0
r_hip_roll      0          ±0.384                   0
r_hip_pitch     0.490      ±1.571                   0.322652
r_calf         -0.444      ±1.571                  -0.290530
l_hip_yaw       0          ±0.436                   0
l_hip_roll      0          ±0.384                   0
l_hip_pitch     0.490      ±1.571                   0.322652
l_calf         -0.444      ±1.571                  -0.290530
neck_pitch     -0.360      -1.571 .. +1.877        -0.306845
head_pitch      0.405      ±1.571                   0.263748
head_yaw        0          ±2.967                   0
head_roll       0.011      ±0.208                   0.052934

行走 HOME 与这里不同：髋 pitch 0.68 对 0.49，膝 -0.88 对 -0.444，
脖子 0.52 对 -0.36，头俯仰 -0.50 对 0.405。不能把行走的 scale（0.25/0.80/1.0）拿来用。


四、joint_vel 14 维，仿真 rad/s，不做 tanh

顺序：第二节的 12 个位置关节，然后

12    r_ankle_joint    CAN 1 右轮
13    l_ankle_joint    CAN 6 左轮

训练噪声 ±0.25 rad/s，关节速度还有 1 拍延迟。部署喂当前拍的仿真速度，不要做归一化以外的缩放。
位置观测里没有轮子角度。


五、动作 14 维

前 12 维 joint_pos，scale 全是 1，raw_action_limit 空，bound_targets 真。
动作 0 对应默认姿态，不是关节限位中点：

  q_sim = mid + half * tanh(a + latent_offset)
  然后再夹进上表限位

后 2 维 joint_vel，顺序右轮、左轮：

  v_sim = clip(a, ±2) * 10
  再夹到 ±20 rad/s

轮行进程发出去之前还会乘 wheel_sign。因为轮子的 SIGN 和 wheel_sign 相同，
交给轮速进程的左右指令就是 v_sim 本身（前进为正）。
上一拍动作回灌的是这 14 维原始输出；轮速维先夹到 ±2，位置维没有 wrapper 裁剪
（wrapper_clip_actions 是空的）。

仿真里位置关节刚度 10、阻尼 0.4、力矩上限 2 N·m；轮子刚度 0、阻尼 0.4、力矩上限 2 N·m。
轮行现在位置环力矩约 1 N·m，轮子走速度环。PD 和训练不一致，接上之前要单独定力矩。


六、轮行里已经按上面接上

getup_policy.py 直接读这份 pt（不经过 onnx，也不走行走的 61 维）。
位置观测和位置动作用第三节、第五节的 tanh 编码。腿的力矩上限 2 N·m。脖子 12 和头俯仰 11 用轮行平时的力矩（约 1 N·m），并排在腿前面发。
脖子 12 不把策略绝对角直接发出去。这台正角度越大越缩，缩着压不到地。按 LB 后先收到 0° 伸直；测到 8° 以内或 1.5 秒后，电机目标改为 0° 加上（本拍策略绝对角 − 伸直那一拍的策略绝对角），并夹在脖子限位里。头俯仰仍用绝对角。
左右轮的速度输出已屏蔽：动作仍先夹到 ±2 并回灌上一拍，但不写到 CAN 1 和 CAN 6。起身时两轮只发速度 0。
头 yaw、roll 下到舵机 13、14。推理从启动就跑，关节指令在倒地后按 LB 才发出；再按 LB 退出，腿和头回到轮行默认姿态。自平衡默认关。
