# 空壶壶嘴对杯口标定

这个目录只用于倒奶/拉花前的右手取杯与空壶壶嘴对杯口标定，不接入正式 coffee 流程，也不执行少量水测试自动流程。现在右手杯子标定不再只依赖 `right_cup_pose`，应先把右手从桌面取杯、抬杯、转移到倒奶前杯口位姿的路径分阶段调稳定，再让左手空壶对杯口。

左右夹爪默认值：

- `left_grip` 默认执行 `robot.left_gripper.set_position(0.70, block=True)`，用于夹空奶壶。
- `right_grip` 默认执行 `robot.right_gripper.set_position(0.80, block=True)`，用于夹杯子。
- `grip` 是 `left_grip` 的兼容别名；新标定记录中建议优先使用 `left_grip`。
- 推荐通过 `--left-gripper-pitcher-position` 和 `--right-gripper-cup-position` 显式记录本轮标定值。

推荐标定顺序：

1. `right_open` 打开右夹爪。
2. `right_table_pregrasp` 让右臂到桌面杯子附近的预抓取位姿。
3. `right_table_grasp_pose` 让右臂进入桌面抓杯位姿。
4. 人工放杯/确认夹持位置，确认杯子、桌面、手指和周边间隙安全。
5. `right_grip` 夹杯，默认 `robot.right_gripper.set_position(0.80, block=True)`。
6. `right_lift_cup` 夹杯后抬离桌面。
7. `right_transfer_cup` 将右手杯子转移到后续倒奶前区域的第一段。
8. `right_transfer_cup_b` 完成右手杯子转移的第二段。
9. `right_pour_ready` 将右手杯子移动到左手空壶对杯口时使用的杯口目标位姿。
10. 使用 `left_grip` 或兼容别名 `grip` 夹住空奶壶。
11. 用 `x+` / `x-` / `y+` / `y-` / `z+` / `z-` 做左手单步 relative IK，对准杯口。
12. 空壶壶嘴接近杯口后，再用 `roll03`、`roll05`、`roll07` 分步做空壶倾斜验证。
13. 少量水测试前，必须先通过空壶 `roll07` 对杯口验证，并确认没有碰撞、卡住、打滑或杯口干涉。

右手取杯阶段命令：

- `right_table_pregrasp` 只移动右臂到桌面杯子附近的预抓取位姿，不控制右夹爪。
- `right_table_grasp_pose` 只移动右臂进入桌面抓杯位姿，不控制右夹爪。
- `right_lift_cup` 用于 `right_grip` 后抬杯离开桌面。
- `right_transfer_cup` 用于将右手杯子转移到后续倒奶前区域的第一段。
- `right_transfer_cup_b` 用于完成右手杯子转移的第二段；如果第一段后出现风险，不要继续执行。
- `right_pour_ready` 用于进入后续左手空壶对杯口时使用的右手杯口目标位姿。
- `right_cup_pose` 保留为兼容旧记录的命令，是 `right_pour_ready` 的 alias/legacy command。

`right_pour_ready` / `right_cup_pose` 注意事项：

- 它不是完整取杯流程，只设置倒奶前右手杯口姿态相关的部分右臂关节。
- 它要求右手已经稳定夹杯。
- 它最好在 `right_table_pregrasp` -> `right_table_grasp_pose` -> `right_grip` -> `right_lift_cup` -> `right_transfer_cup` -> `right_transfer_cup_b` 之后使用。
- 该命令只在本标定脚本内发送右臂关节目标，不会自动调用完整 `coffee.py` 流程。
- 该命令不会修改或调用 `coffee_replay_safe.py`，也不会接入 `left_hand_pour_milk`。
- 该位姿必须现场确认安全，不能默认认为适合所有初始姿态、杯子尺寸或夹持状态。

左夹爪命令：

- `left_open` 使用 `robot.left_gripper.set_position(1.00, block=True)` 打开左夹爪。
- `left_grip` 使用 `robot.left_gripper.set_position(0.70, block=True)` 夹空奶壶。
- `grip` 是 `left_grip` 的兼容别名。
- `left_loose` / `left_tight` 基于本脚本内部记录的左夹爪估计值按 `--left-gripper-step` 微调，并裁剪到 `[0.0, 1.0]`。
- 这些命令都需要人工输入 `y` 才会在 execute 模式执行，并且都会写入 CSV。

右夹爪命令：

- `right_open` 使用 `robot.right_gripper.set_position(1.00, block=True)` 打开右夹爪，不调用 SDK `open()`。
- `right_grip` 使用 `robot.right_gripper.set_position(0.80, block=True)` 夹杯。
- `right_loose` / `right_tight` 基于本脚本内部记录的右夹爪估计值按 `--right-gripper-step` 微调，并裁剪到 `[0.0, 1.0]`。
- 这些命令都需要人工输入 `y` 才会在 execute 模式执行，并且都会写入 CSV。

所有夹爪命令均使用 `set_position()`，不调用 SDK `open()`。

运行方式：

```zsh
cd /home/lanchong/BlueWorm_ws/Mantis-SDK-github
python3 pour_alignment_calib/pour_align_calib.py --dry-run
```

显式记录左右夹爪目标值：

```zsh
python3 pour_alignment_calib/pour_align_calib.py --dry-run \
  --left-gripper-pitcher-position 0.70 \
  --right-gripper-cup-position 0.80
```

真实机器人执行必须显式传入：

```zsh
python3 pour_alignment_calib/pour_align_calib.py --execute --i-understand-real-robot-risk
```

安全注意事项：

- 每个真实动作执行前都需要人工输入 `y` 确认。
- 每个右臂阶段命令执行完后会优先尝试 `robot.right_arm.wait()`，不支持时再尝试 `robot.wait(robot.right_arm.joint_names)`。
- `right_table_pregrasp`、`right_table_grasp_pose`、`right_lift_cup`、`right_transfer_cup`、`right_transfer_cup_b`、`right_pour_ready` 和 `right_cup_pose` 都只移动右臂，不控制任何夹爪。
- `right_pour_ready` / `right_cup_pose` 不会自动调用 `robot.home()`，执行前必须人工确认当前右臂初始姿态是否适合直接切到该位姿。
- 本工具不会自动调用 `robot.home()`。
- 本工具不会调用 `right_arm.home()` 或 `left_arm.home()`。
- 本工具不会自动跑完整 `coffee.py` 流程。
- 本工具不会自动接入或调用 `coffee_replay_safe.py`。
- 本工具不会自动做少量水测试。
- 本工具不会自动运行完整拿杯序列；每一步都需要手动输入命令。
- 本工具不使用 SDK `open()`，夹爪打开也使用 `set_position()`。
- 本工具不使用 README 示例 absolute IK 点，不做大范围 absolute IK。
- 左臂平移动作默认使用 `arm.ik(dx, dy, dz, 0, 0, 0, block=True, abs=False)`。
- `obs unsafe` 或 `obs near_collision` 会在 CSV 中标记 `risk_detected=True`；出现这两种观察时建议停止本轮调试。
- 如果任一步出现杯子滑动、杯口倾斜过大、靠近身体、碰撞风险，立即停止，不进入下一阶段。
- 调试时必须有人看护急停，空壶、杯子和周边障碍物的位置变化后要重新确认安全间隙。
- 如果执行 `right_open` 或救援控制台菜单 5 后出现双夹爪同时动作或回 home，必须立即停止本轮标定，检查是否有多个控制脚本同时运行，并检查日志，不要继续夹杯/倒水。

日志会自动写入：

```zsh
pour_alignment_calib/logs/
```
