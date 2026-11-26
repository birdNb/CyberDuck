# 轮行（C++）

站立后用手柄开轮子。电机指令只从本程序的 CAN 线程发出。头部张望、嘴、灯环、音量、倒地起身和前左翻滚用原来的 Python 实现，跟在旁边跑，把关节角交回这条 CAN 线程。

不要和 `sport_ctrl/wheel_run/wheel_drive.py` 同时开。两套程序会抢 `can0`。

## 编译和启动

在仓库根目录：

```bash
cmake -S /home/aidlux/cyberDuck -B /home/aidlux/cyberDuck/build-wheel
cmake --build /home/aidlux/cyberDuck/build-wheel --target wheel_drive
sudo /home/aidlux/cyberDuck/build-wheel/sport_ctrl/wheel_run/cpp/wheel_drive
```

需要 root，实时线程才能用 `SCHED_FIFO`。`can0` 如果是 DOWN，程序会自己执行 `motor/setup_can0.sh` 把它拉起来。

开机由 `wheel-drive.service` 拉起，先重试 `motor/setup_can0.sh`，再运行上面的程序。查看和停止：

```bash
systemctl status wheel-drive
journalctl -u wheel-drive -f
sudo systemctl stop wheel-drive
```

自检不碰电机：

```bash
/home/aidlux/cyberDuck/build-wheel/sport_ctrl/wheel_run/cpp/wheel_drive --self-test
```

常用参数：

```bash
sudo /home/aidlux/cyberDuck/build-wheel/sport_ctrl/wheel_run/cpp/wheel_drive --vmax 10 --no-balance
sudo /home/aidlux/cyberDuck/build-wheel/sport_ctrl/wheel_run/cpp/wheel_drive --swap-wheels
```

`--help` 能列出全部参数。姿态文件默认是 `sport_ctrl/pose_contrl/pose_config.yaml`。

## 怎么开

上电后关节是卸力的。长按 Y 约 1 秒，腿才柔和进 `drive` 姿态。进驾驶后的前 3 秒轮子只保持当前位置，之后才跟摇杆。

再按一次 Ctrl+C 才退出。只按一次会先卸力，停在等待。

## 手柄

和 Python 版同一套北通键位。左摇杆向左时左轮变慢、右轮变快，车向左转。

| 操作 | 作用 |
| --- | --- |
| 左摇杆上下 | 前进 / 后退。松杆和起步都有斜坡 |
| 左摇杆左右 | 行进中差速转弯；只有横向时内侧轮停、外侧轮转 |
| 右摇杆左右 | 原地自转。向左推，车向左自转 |
| 右摇杆上下 | 只升降两条腿，在 `drive` 和 `extended` 之间插值 |
| LT | 半速变全速。不按约 ±10 rad/s，按住约 ±20 rad/s。侧倾也从半幅变满幅 |
| Y 短按 | 开关自平衡。进驾驶时默认关，只用两侧髋 |
| Y 长按 | 卸力等待时，按住约 1 秒再进入轮行 |
| B | 轮速归零，腿走到卸力姿态，到位后才卸力。没到位时再按一次 B，就在当前位置卸力。程序不退出 |
| RB | 开关头部张望。默认关。卸力等待时也能开，腿保持卸力 |
| 右扳机 | 张嘴并短叫。按住满 3 秒改长鸣。张望开着时扳机不抢嘴 |
| X | 切换灯环灯效 |
| 十字键上下 | 喇叭音量，一格 5% |
| 十字键左右 | 左右侧倾。默认半幅，按住 LT 到满幅。侧出和回中约 24°/s |
| LB | 倒地后才启用起身。推理在 C++ 里一直跑，不带动关节。再按 LB 退出。起身时轮子锁定 |
| A | 前左翻滚。再按 A，或满 5 秒，退出。翻滚时轮子仍跟摇杆 |

摇杆有两根以上同时卡在 ±0.5 附近时，这一帧摇杆和按键都丢掉，只保留 B。

## 线程

| 线程 | 优先级 | 周期 | 做什么 |
| --- | --- | --- | --- |
| 看门 | 70 | 20 ms | 心跳断了就切断两轮力矩 |
| CAN | 60 | 200 Hz | 唯一发 CAN 的线程。轮子运动帧在每拍最后 |
| 轮子 | 50 | 200 Hz | 斜坡、低通、位置追速 |
| 身体 | 40 | 100 Hz | 升降、侧倾、自平衡、站立保持 |
| IMU | 30 | 200 Hz | 读原始角速度和加速度 |
| 手柄 | 20 | 4 ms | 读 `/dev/input/js*` |
| 表演 | 普通 | 20 ms | 跟 Python 伴随程序交换头部和策略角度 |
| 主线程 | 普通 | | 状态切换和日志 |

轮子用速度模式 `0x1C`，只发速度，不发圈数；**CAN 上轮速指令封顶 10 Hz**。腿/脖子 `0x1F` 查询码一律 `0x00`（不随位置回包）；状态用稀疏 `0x0E`（翻滚时每拍约 3 路）。曾用全关节 `0x1F+0x0E` 把 PCAN-USB 灌爆，观测冻住后翻滚会站直僵住。停止模式只在卸力姿态到位之后发出。不重设零位。

## 伴随程序

`show_bridge.py` 由 `wheel_drive` 自己拉起，一般不用手开。它复用 Python 里的 `HeadLook`、`Mouth`、`LedRing`、`DuckVolume`、`GetupPolicy`、`RouladePolicy`。

舵机走原来的串口，灯环由本程序拉起 `LED_Ring/white_flow.py`，手柄 X 在这里切灯效。开机服务 `led-boot` 保持关闭，运控退出后也不再把它拉起来。灯的 5V 仍由 `led-5v` 供电。叫声走原来的喇叭。头俯仰（11）、脖子（12）和起身/翻滚的腿，角度经 `/tmp/cyberduck-wheel-show.sock` 交给 C++。

脖子和头俯仰用 1 N·m。起身和翻滚的腿用 2 N·m。12 号在 2 N·m 时跟不住。

## 日志

驾驶段写在：

`sport_ctrl/wheel_run/logs/drive_cpp_年月日_时分秒.csv`

混合轮速(`0x1C`)+身体位置(`0x1F`)+状态查询(`0x0E`) 的逐帧 CAN 报文默认写入  
`sport_ctrl/wheel_run/logs/can_mix_年月日_时分秒.csv`（`--no-can-log` 可关）。列：`t_s,dir,id,arb,dlc,hex,kind,query,pos_rad,vel_rad_s,tqe_nm,fault`。

终端大约每 0.5 秒打一行。行尾的 CAN 数字是这一拍的耗时，单位是纳秒，不是微秒。
