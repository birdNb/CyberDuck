# 运控闭环

50 Hz 运控进程。策略文件还没训练好时，推理输出全零，其余链路已经接上：手柄速度指令、IMU、关节状态、观测向量、电机位置指令。

观测布局与 microduck `duck-control` 相同，61 维进、14 维出。嘴不进策略。

```
0..3     gyro，trunk，rad/s
3..6     projected gravity，trunk
6..20    关节位置 − 训练零位
20..34   关节速度
34..48   上一拍动作
48..51   vx vy vyaw
51..61   头和身体指令，目前为 0
```

## 目录

| 路径 | 职责 |
| --- | --- |
| `motor/cpp` | SocketCAN 上的 HighTorque 电机读写 |
| `IMU/cpp` | LSM6DSV16X，输出策略用的 gyro 和 gravity |
| `locomotion` | 手柄、观测、推理引擎、50 Hz 循环 |
| `locomotion/config/locomotion.cfg` | 频率、限幅、关节 CAN ID 和训练零位 |

## 编译

```bash
cd ~/cyberDuck
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/locomotion/duck_locomotion --self-test
```

## 运行

先干跑，只打印，不给电机上力矩：

```bash
cd ~/cyberDuck/locomotion
../build/locomotion/duck_locomotion
```

手柄左摇杆前后是 `vx`，左右是 `vy`，右摇杆左右是 `vyaw`。死区 0.12。没插手柄时速度指令为 0。

确认 `config/locomotion.cfg` 里的 CAN ID 和真实关节一致之后，才允许出力：

```bash
../build/locomotion/duck_locomotion --apply
```

`--apply` 会把电机设成位置模式，通信超时 100 ms（断连后电机自己停），Ctrl+C 卸力。前 2 秒从当前角度平滑到目标。

## 推理引擎

这块板是 Radxa Dragon Q6A，SoC 为 QCS6490：Kryo（A78 + A55）、Adreno、Hexagon HTP v68。

默认 `engine auto`：

- 没有 `model` 文件：`hold`，动作全零。
- 有 onnx：ONNX Runtime CPU，图优化全开，算子内线程数 1。61→14 的小网络在 A78 上远低于 20 ms，50 Hz 预算花在 CAN 和 IMU 上，不花在线程池上。
- `engine qnn`：走 ONNX Runtime 的 QNN Execution Provider，后端 `libQnnHtp.so`，`htp_arch=68`，`soc_model=6490`，性能模式 `burst`。QNN 不可用时自动退回 CPU。

CPU 库目前用本机已安装的 ONNX Runtime 1.29：

```
/home/radxa/miniconda3/envs/ros_jazzy/lib/python3.12/site-packages/onnxruntime/capi/libonnxruntime.so.1.29.0
```

也可以设 `ORT_DYLIB_PATH`，或在配置里写 `ort` 路径。

NPU 需要官方 QAIRT，并且模型是给 HTP 准备的量化 onnx。运行前：

```bash
export PRODUCT_SOC=6490
export DSP_ARCH=68
export ADSP_LIBRARY_PATH=$QNN_SDK_ROOT/lib/hexagon-v68/unsigned
```

策略导出约定（与 microduck 相同）：

- 前馈：输入 `obs` `[1,61]` float32，输出 14 维动作
- LSTM：输入 `obs`、`h_in`、`c_in`，输出 `actions`、`h_out`、`c_out`

把文件放到例如 `locomotion/policies/walk.onnx`，配置改成：

```
model policies/walk.onnx
engine auto
```

`action_scale` 默认 0.25。目标角 = 训练零位 + scale × 动作。动作先夹到 ±1。

## 安全

干跑是默认。`--apply` 会动电机。关节表没对上之前不要加这个参数。
