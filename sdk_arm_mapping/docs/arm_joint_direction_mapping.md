# 双臂关节方向映射报告

## 1. 测试说明

- 报告生成时间：`2026-05-18T10:27:01`
- 已读取 CSV 记录数：`31`
- 当前范围只覆盖 `left_arm` 与 `right_arm` 的 7 个关节，不包含 gripper、head、waist、chassis 和 IK。
- 报告不会写入原始 SN / IP；如需附带连接目标，请使用脱敏形式，例如 `BW_****`、`192.168.*.*`。

## 2. 安全提醒

- 默认应该先 dry-run，再做实机单关节小幅测试。
- 建议顺序：先 wrist，再 elbow，最后 shoulder。
- shoulder 关节会带动整条手臂，风险最高，必须保证周围无障碍物。
- 不要夹持杯子、奶壶或其他工具做方向映射。

## 3. 左臂关节方向映射表

| side | joint | direction_type | input target | observed motion | note | status |
| --- | --- | --- | --- | --- | --- | --- |
| left | wrist_roll | positive_delta | 0.250000 rad / 14.32 deg | 向身体内侧 | 左手小臂向内侧旋转 | ok |
| left | wrist_roll | negative_delta | -0.250000 rad / -14.32 deg | 向身体外侧 | 左手小臂向外侧旋转 | ok |
| left | wrist_pitch | positive_delta | 0.150000 rad / 8.59 deg | 向上 | 左手腕部向上偏转 | ok |
| left | wrist_pitch | negative_delta | -0.150000 rad / -8.59 deg | 向下 | 左手小腕向下偏转 | ok |
| left | wrist_yaw | positive_delta | 0.250000 rad / 14.32 deg | 向身体外侧 | 左手小腕向身体外侧偏转 | ok |
| left | wrist_yaw | negative_delta | -0.250000 rad / -14.32 deg | 向身体内侧 | 左手小腕向内侧偏转 | ok |
| left | elbow_pitch | positive_delta | 0.150000 rad / 8.59 deg | 向下 | 左手向下偏转 | ok |
| left | elbow_pitch | negative_delta | -0.150000 rad / -8.59 deg | 向上 | 左手小臂向上偏转，大臂小臂夹角变小 | ok |
| left | shoulder_pitch | positive_delta | 0.150000 rad / 8.59 deg | 向上 | - | ok |
| left | shoulder_pitch | negative_delta | -0.150000 rad / -8.59 deg | 向下 | - | ok |
| left | shoulder_roll | positive_delta | 0.150000 rad / 8.59 deg | 向身体内侧 | 左肩关节向内偏转 | ok |
| left | shoulder_roll | negative_delta | -0.150000 rad / -8.59 deg | 向身体外侧 | 左肩向外偏转 | ok |
| left | shoulder_yaw | positive_delta | 0.120000 rad / 6.88 deg | 向身体外侧 | 左大臂远离身体，向外打开 | ok |
| left | shoulder_yaw | negative_delta | -0.120000 rad / -6.88 deg | 向身体内侧 | 左边大臂向内偏转，靠近身体 | ok |

## 4. 右臂关节方向映射表

| side | joint | direction_type | input target | observed motion | note | status |
| --- | --- | --- | --- | --- | --- | --- |
| right | wrist_roll | positive_delta | 0.250000 rad / 14.32 deg | 向身体内侧 | 从机器人正前方观察：右手腕向身体内侧旋转 | ok |
| right | wrist_roll | negative_delta | -0.250000 rad / -14.32 deg | 向身体外侧 | 从机器人正前方观察：右手腕向身体外侧旋转 | ok |
| right | wrist_pitch | positive_delta | 0.150000 rad / 8.59 deg | 向上 | 右手手腕向抬起 | ok |
| right | wrist_pitch | negative_delta | -0.150000 rad / -8.59 deg | 向下 | 右手手腕向下偏转 | ok |
| right | wrist_yaw | positive_delta | 0.250000 rad / 14.32 deg | 向身体外侧 | 向外偏转 | ok |
| right | wrist_yaw | negative_delta | -0.250000 rad / -14.32 deg | 向身体内侧 | 右手手腕向内偏转 | ok |
| right | elbow_pitch | positive_delta | 0.150000 rad / 8.59 deg | 向下 | 右小臂向下，肘部夹角变大 | ok |
| right | elbow_pitch | negative_delta | -0.150000 rad / -8.59 deg | 向上 | 右小臂向上，肘部夹角变小 | ok |
| right | shoulder_pitch | positive_delta | 0.150000 rad / 8.59 deg | 向上 | +delta -,向前 | ok |
| right | shoulder_pitch | negative_delta | -0.150000 rad / -8.59 deg | 向下 | -delta -> 右臂向下/向后 | ok |
| right | shoulder_roll | positive_delta | 0.150000 rad / 8.59 deg | 向身体内侧 | 从机器人正前方观察：右臂向身体内侧收拢 | ok |
| right | shoulder_roll | negative_delta | -0.150000 rad / -8.59 deg | 向身体外侧 | 从机器人正前方观察：右臂向身体外侧展开 | ok |
| right | shoulder_yaw | positive_delta | 0.120000 rad / 6.88 deg | 向身体外侧 | 向身体外侧 | ok |
| right | shoulder_yaw | negative_delta | -0.120000 rad / -6.88 deg | 向身体内侧 | 向内侧 | ok |

## 5. 异常/中止记录

当前没有异常或中止记录。

## 6. 未测试项

全部 28 个方向项都已有成功记录。

## 7. 对 coffee.py / 咖啡拉花调试最关键的结论

- `coffee.py` 里优先受影响的关节通常是 `shoulder_pitch`、`shoulder_roll`、`elbow_pitch`、`wrist_roll`；这些方向没确认前，不建议直接调大幅拉花动作。
- `wrist_pitch` 和 `wrist_yaw` 更直接影响末端姿态、倾倒角和杯口朝向；它们的正负方向应该先通过单关节测试确认，再进入倾倒轨迹微调。
- 左右臂不要默认按“镜像”理解，应该分别记录；如果现场观察到镜像关系，也建议明确写进 CSV 备注。
- 已记录方向样本：`left shoulder_pitch`，+delta => 向上，-delta => 向下。
- 已记录方向样本：`left shoulder_roll`，+delta => 向身体内侧，-delta => 向身体外侧。
- 已记录方向样本：`left elbow_pitch`，+delta => 向下，-delta => 向上。
- 已记录方向样本：`left wrist_roll`，+delta => 向身体内侧，-delta => 向身体外侧。
- 已记录方向样本：`left wrist_pitch`，+delta => 向上，-delta => 向下。
- 已记录方向样本：`left wrist_yaw`，+delta => 向身体外侧，-delta => 向身体内侧。
- 已记录方向样本：`left shoulder_yaw`，+delta => 向身体外侧，-delta => 向身体内侧。
- 已记录方向样本：`right shoulder_pitch`，+delta => 向上，-delta => 向下。
