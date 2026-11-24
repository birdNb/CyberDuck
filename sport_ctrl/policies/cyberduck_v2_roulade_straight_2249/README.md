# CyberDuck V2 单前翻滚部署包（2249）

任务：`CyberDuckV2-Flat-Roulade-SingleForward`

本包保存从 `straight_ft_seed0` 取出的 **iteration 2249** 检查点，作为“身体最贴矢状面”的候选。

## 接口

- actor 观测：`48` 维（`base_ang_vel, projected_gravity, joint_pos(12), joint_vel(14), actions(14), roulade_phase(2)`）
- critic 观测：`55` 维
- 动作：`14` 维（12 路位置 + 2 路踝速度）
- 控制频率：`50 Hz`，回合 `5 s`
- 检查点契约与 `CyberDuckV2-Flat-Roulade-SingleForward` 兼容

## 评估

固定 yaw 0、32 环境：

- 成功：**32/32**
- 平均前滚角：**364.8°**
- 头触地：**32/32**
- 横向均值：**0.088 m**，最大 **0.175 m**
- 最终 yaw：**+7.7°**
- 平均/最大 `|横轴 z|`：**0.067 / 0.766**

注意：首次头触地时 `head_yaw ≈ -1.05`，头部仍然偏；2249 的优点是身体横向/矢状面，而不是头部 yaw。

## 与 750 base 的取舍

- **750 base**：前进 0.635 m、最终 yaw +1.5°、首次 head_yaw -0.16；但过程中最大 `|横轴 z|=0.957`，有一次明显侧偏。
- **2249**：横向 0.088 m、最终 yaw 7.7°、平均 `|横轴 z|` 0.067；但 head_yaw -1.05。

按“前向距离/最终朝向”选 750；按“身体留在矢状面”选 2249。

## 仿真回放

```bash
uv run --no-sync duck-play CyberDuckV2-Flat-Roulade-SingleForward \
  --agent trained --checkpoint-file deploy/cyberduck_v2_roulade_straight_2249/model.pt \
  --num-envs 1 --viewer viser
```

有图形桌面时可用 `--viewer native`。

## 目录

- `model.pt`：2249 检查点（`model_2249.pt` 的副本）
- `params/agent.yaml`、`params/env.yaml`：原训练配置，仅用于追溯
- `videos/straight2249_full_yaw0.mp4`、`videos/straight2249_head_yaw0.mp4`
- `manifest.json`：接口、评估与来源

## 注意

- 这是**仿真检查点**，不是 ONNX，也未做实机标定；不能直接当作实机部署模型。
- 实机部署前需要：ONNX 导出、观测/动作 ABI 对接、head_yaw 限位/安全约束、实机标定。
- 2249 的 head_yaw 仍为 -1.05；如果实机不允许偏头，需要额外的 head_yaw 约束/限位，或使用 750 base 并重新评估其侧偏风险。
- 继续训练到 5000 会创建新的 run，不会覆盖本包。
