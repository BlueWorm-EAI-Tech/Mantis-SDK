# 空壶壶嘴对杯口标定

这个目录只用于倒奶/拉花前的空壶壶嘴对杯口标定，不接入正式 coffee 流程，也不执行少量水测试自动流程。

推荐流程：

1. 先固定右手杯子位置，确认杯子稳定、周围无遮挡。
2. 用左手夹住空奶壶，建议先执行 `grip`，默认 `left_gripper.set_position(0.70)`。
3. 用 `x+` / `x-` / `y+` / `y-` / `z+` / `z-` 做单步 relative IK 对位。
4. 空壶壶嘴接近杯口后，再用 `roll03`、`roll05`、`roll07` 分步验证倾斜姿态。
5. 少量水测试前，必须先通过空壶 `roll07` 对杯口验证，并确认没有碰撞、卡住、打滑或杯口干涉。

运行方式：

```zsh
cd /home/lanchong/BlueWorm_ws/Mantis-SDK-github
python3 pour_alignment_calib/pour_align_calib.py --dry-run
```

真实机器人执行必须显式传入：

```zsh
python3 pour_alignment_calib/pour_align_calib.py --execute --i-understand-real-robot-risk
```

安全注意事项：

- 每个真实动作执行前都需要人工输入 `y` 确认。
- 本工具不使用 README 示例 absolute IK 点，不做大范围 absolute IK。
- 左臂平移动作默认使用 `arm.ik(dx, dy, dz, 0, 0, 0, block=True, abs=False)`。
- `obs unsafe` 或 `obs near_collision` 会在 CSV 中标记 `risk_detected=True`；出现这两种观察时建议停止本轮调试。
- 调试时必须有人看护急停，空壶、杯子和周边障碍物的位置变化后要重新确认安全间隙。

日志会自动写入：

```zsh
pour_alignment_calib/logs/
```
