# CyberDuck V2 Roulade Hard150 2997

## 任务

`CyberDuckV2-Flat-Roulade-Recovery-Speed150`

从 `head_tuck` 的 `model_1998` 继续训练，加入 150 RPM 仿真硬裁剪、`head_pitch` 收头奖励、嘴部接触惩罚和动作幅度惩罚。

## 接口

- actor：`48`
- critic：`55`
- 动作：`14`（12 位置 + 2 踝速度）
- 控制：`50 Hz`，回合 `5 s`
- 契约：`cyberduck_roulade_recovery_obs48_v4`

## 评估

32 环境、固定 yaw 0、260 控制步：

- 成功：**32/32**
- 平均最大前滚角：`352.70°`（范围 `345.05°–368.10°`）
- 头触地：`32/32`
- 平均最大倾角：`150.52°`
- 平均最低根高：`0.0834 m`
- 平均最终根高：`0.1594 m`

## 速度

实际裁剪后的仿真速度：

- 峰值：**149.999997 RPM**
- 峰值关节：`l_hip_pitch_joint`
- 超过 150 RPM：**0/250** 个控制步

带观测延迟/噪声的 CSV 观测速度：

- 峰值：`152.30 RPM`
- 超过 150 RPM：`54/250` 个控制步

注意：150 RPM 是环境里的 `qvel` 硬裁剪，不是电机驱动器模型；实机仍需驱动器限速。

## 力矩和动作幅度

- 仿真力矩峰值：`2.0 N·m`
- 任一关节超过连续额定 `0.6 N·m`：`249/250` 个控制步，持续约 `4.98 s`
- 速度超过 60 RPM：`135/250` 步，约 `2.70 s`
- 超过空载 390 RPM：`0` 步
- 原始动作均值 `|action|`：`0.714`
- 原始动作最大 `|action|`：`5.664`

## 结论

- **成功率 32/32，没有下降。**
- **仿真实际速度峰值已裁到 150 RPM**，超出步数为 0。
- **动作幅度仍偏大**，最大动作 5.664；不能称为动作幅度已受控。
- 力矩仍在 2.0 N·m 限幅，但长时间超过连续额定 0.6 N·m，热/电流没有验证。
- 硬裁剪是非物理的仿真安全措施，实机必须用驱动器速度/电流限幅；这个包不能直接作为实机安全认证结果。

## 回放

```bash
uv run --no-sync duck-play CyberDuckV2-Flat-Roulade-Recovery-Speed150 \
  --agent trained \
  --checkpoint-file deploy/cyberduck_v2_roulade_hard150_2997/model.pt \
  --num-envs 1 --viewer native
```

## 目录

```text
model.pt                 # model_2997
params/                  # 训练时 agent/env 快照
metrics/eval_yaw0_32env.json
metrics/rollout_250steps.csv
metrics/actuator_safety.json
metrics/actuator_trace.csv
videos/recovery_speed150_hard150_model2997.mp4
manifest.json
README.md
```

## 来源

```text
logs/rsl_rl/cyberduck_v2_roulade_recovery_speed150/
  2026-11-26_15-31-22_recovery_speed150_hard150_from1998/model_2997.pt
```

模型 SHA256：

```text
78aea84f890f840a0267b8112514da2e4637e6e02ab7d559acc264a86435c548
```
