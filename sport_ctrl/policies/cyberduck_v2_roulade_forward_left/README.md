# CyberDuck V2 前左翻滚部署包

任务：`CyberDuckV2-Flat-Roulade-ForwardLeft`

本目录包含两个可在仿真中回放/继续评估的检查点：

| 文件 | 说明 |
|---|---|
| `model.pt` | 主模型（model_5499）：100% 成功，前向位移 0.32–0.41 m，横向漂移 0.20–0.33 m |
| `model_5000_baseline.pt` | 原始左滚基线（model_5000）：100% 成功，前向 0.17–0.24 m，横向 0.28–0.37 m |

两者都是“前左翻滚”：会向前翻过头顶，同时保留明显的左肩侧滚和翻后转身；不是纯矢状面单前滚。若需要纯前向单滚，请使用后续的 `CyberDuckV2-Flat-Roulade-SingleForward` 任务（尚未训练）。

## 接口

- actor 观测：`48` 维（`base_ang_vel, projected_gravity, joint_pos(12), joint_vel(14), actions(14), roulade_phase(2)`）
- critic 观测：`55` 维
- 动作：`14` 维（12 路位置 + 2 路踝速度）
- 控制频率：`50 Hz`，回合 `5 s`
- 动作/观测契约与 `CyberDuckV2-Flat-Roulade-Recovery` 兼容

## 仿真回放

```bash
uv run --no-sync duck-play CyberDuckV2-Flat-Roulade-ForwardLeft \
  --agent trained --checkpoint-file deploy/cyberduck_v2_roulade_forward_left/model.pt \
  --num-envs 1 --viewer viser
```

有图形桌面时可用 `--viewer native`。回放默认从站立开始；正常训练/回放的出生朝向是随机的，所以画面里的左右方向会随出生 yaw 变化。

固定朝向 0° 的预览视频：

- `videos/model_5499_fixed_yaw0.mp4`
- `videos/model_5000_fixed_yaw0.mp4`

## 评估

```bash
uv run --no-sync python -B tests/test_cyberduck_roulade_training.py eval \
  --task CyberDuckV2-Flat-Roulade-ForwardLeft \
  --checkpoint deploy/cyberduck_v2_roulade_forward_left/model.pt \
  --num-envs 8 --steps 260 --seed 0
```

最近一次 8 环境验证：`8/8` 成功。

## 注意

- 这是**仿真检查点**，不是 ONNX，也未做实机标定；导出与实机部署链路尚未提供。
- 不要使用大训练的最终检查点 `model_21248.pt`：7000 轮后策略退化到 0% 成功。
- `params/` 保存原始训练配置；`env.yaml` 里的奖励权重仅用于追溯，推理不依赖奖励。
