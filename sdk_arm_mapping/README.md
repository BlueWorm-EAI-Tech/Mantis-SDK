# Mantis SDK 双臂关节方向映射工具

这个工具用于建立 Mantis SDK 双臂关节参数与实机运动方向之间的映射关系，帮助我们明确“某个关节角度增大/减小时，机械臂到底向哪里动、如何旋转”。它的直接目标是服务于 `coffee.py` 的理解，以及后续咖啡拉花轨迹的调试与安全验证。

当前阶段只测试：

- `left_arm`
- `right_arm`
- 每条手臂 7 个关节：
  - `shoulder_pitch`
  - `shoulder_yaw`
  - `shoulder_roll`
  - `elbow_pitch`
  - `wrist_roll`
  - `wrist_pitch`
  - `wrist_yaw`

当前不测试：

- `gripper`
- `head`
- `waist`
- `chassis`
- `IK`

## 推荐测试顺序

建议按下面顺序逐步推进，不要一开始就整套联动：

1. 先做 dry-run，确认测试计划和动作序列。
2. 先测 `left wrist_roll`。
3. 再测 `left wrist_pitch` / `left wrist_yaw`。
4. 再测 `left elbow_pitch`。
5. 再测 `left shoulder_pitch` / `left shoulder_roll` / `left shoulder_yaw`。
6. 然后按同样顺序测试 `right arm`。
7. 不建议直接用 `joint=all` 在实机跑完整流程，除非前面的单关节测试已经确认安全。

## 实机测试前检查

- 机器人两臂周围无人。
- 桌面、杯子、咖啡机、奶壶全部移开。
- 急停可用。
- 单次只测一个关节。
- 不要夹持工具或液体。
- 旁边有人看护。
- 每一步动作前先看清终端提示，再按 Enter。

## 示例命令

dry-run：

```bash
python3 sdk_arm_mapping/scripts/map_arm_joint_directions.py \
  --config sdk_arm_mapping/config/arm_mapping.yaml \
  --side left \
  --joint shoulder_pitch
```

`left wrist_roll` 实机测试：

```bash
python3 sdk_arm_mapping/scripts/map_arm_joint_directions.py \
  --config sdk_arm_mapping/config/arm_mapping.yaml \
  --side left \
  --joint wrist_roll \
  --execute \
  --i-understand-real-robot-risk
```

`left shoulder_pitch` 实机测试：

```bash
python3 sdk_arm_mapping/scripts/map_arm_joint_directions.py \
  --config sdk_arm_mapping/config/arm_mapping.yaml \
  --side left \
  --joint shoulder_pitch \
  --execute \
  --i-understand-real-robot-risk
```

`right wrist_roll` 实机测试：

```bash
python3 sdk_arm_mapping/scripts/map_arm_joint_directions.py \
  --config sdk_arm_mapping/config/arm_mapping.yaml \
  --side right \
  --joint wrist_roll \
  --execute \
  --i-understand-real-robot-risk
```

生成 Markdown 报告：

```bash
python3 sdk_arm_mapping/scripts/generate_arm_mapping_report.py
```

## 说明

- 工具不会自动判断“向前 / 向后 / 向上 / 向下”等语义，方向结论必须由人工观察后录入。
- 每次实机测试会自动写入 CSV，后续可以再汇总生成 Markdown 报告。
- 如果方向不确定，请选择“方向不确定”，并在备注里写清观察角度，例如“从机器人正前方观察”。
- 默认是 dry-run，不会连接机器人，也不会执行任何实机动作。
- 只有同时传入 `--execute` 和 `--i-understand-real-robot-risk` 才允许真正连接机器人并执行动作。
