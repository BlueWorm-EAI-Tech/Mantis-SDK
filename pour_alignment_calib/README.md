# 空壶壶嘴对杯口标定

这个目录只用于倒奶/拉花前的右手取杯与空壶壶嘴对杯口标定，不接入正式 coffee 流程，也不执行少量水测试自动流程。右手杯子位姿必须从 `coffee_replay_safe.py` 的真实阶段链路复现，不再建议单独使用旧的猜测版 `right_table_*` 命令。

左右夹爪默认值：

- `left_grip` 默认执行 `robot.left_gripper.set_position(0.70, block=True)`，用于夹空奶壶。
- `right_grip` 默认执行 `robot.right_gripper.set_position(0.80, block=True)`，用于夹杯子。
- `replay_right_*` 动作以 `coffee_replay_safe.py` 为参考；右臂关节动作顺序、目标值、`block` 参数、`sleep` 和 `robot.wait()` 尽量严格对齐。
- `replay_right_grasp_cup` 中右夹爪初始化闭合默认使用 `--right-gripper-closed-position 0.00`；该值可按 SDK/实机实际闭合定义调整。
- `replay_right_grasp_cup` 默认不执行左夹爪初始化；只有显式传入 `--include-left-gripper-init` 才会使用 `--left-gripper-closed-position` 初始化左夹爪。
- `grip` 是 `left_grip` 的兼容别名；新标定记录中建议优先使用 `left_grip`。
- 推荐通过 `--left-gripper-pitcher-position` 和 `--right-gripper-cup-position` 显式记录本轮标定值。
- `right_grip` / `replay_right_grasp_cup` 抓杯默认 0.80 是当前 `pour_alignment_calib` 标定脚本的现场覆盖值；原始 `coffee_replay_safe.py` 中抓杯位置为 `robot.right_gripper.set_position(0.6)`。
- 若要严格复现 `coffee_replay_safe.py` 的 0.6，请运行时显式传 `--right-gripper-cup-position 0.6`。

推荐标定顺序：

1. 先右手空载复现：`replay_right_grasp_cup` -> `replay_right_move_to_coffee_machine` -> `replay_right_retreat_after_coffee` -> `replay_right_pour_ready`。
2. 再右手夹空杯复现同一条阶段链路，逐阶段确认杯子、桌面、手指、身体和周边间隙安全。
3. 右手链路稳定后，使用 `left_grip` 或兼容别名 `grip` 夹住空奶壶。
4. 用 `x+` / `x-` / `y+` / `y-` / `z+` / `z-` 做左手单步 relative IK，对准杯口。
5. 空壶壶嘴接近杯口后，再用 `roll03`、`roll05`、`roll07` 分步做空壶倾斜验证。
6. 少量水测试前，必须先通过空壶 `roll07` 对杯口验证，并确认没有碰撞、卡住、打滑或杯口干涉。

右手 replay 阶段命令：

- `replay_right_grasp_cup` 复现 `coffee_replay_safe.py::right_hand_grasp_cup`，用于右手从桌面抓杯并抬离桌面。
- `replay_right_grasp_cup` 中右臂关节动作、`block`、`sleep`、`robot.wait()` 与源流程对齐；唯一夹杯例外是 `right_gripper.set_position` 实际执行值来自 `--right-gripper-cup-position`，默认 0.80。
- `replay_right_move_to_coffee_machine` 复现 `coffee_replay_safe.py::right_hand_move_to_coffee_machine`。
- `replay_right_retreat_after_coffee` 复现 `coffee_replay_safe.py::right_hand_retreat_after_coffee`，包含 `robot.right_arm.home()`，必须重点确认路径安全。
- `replay_right_pour_ready` 只复现 `coffee_replay_safe.py::left_hand_move_to_pour_pose` 中右手相关动作，不执行左手动作；它不是完整 `left_hand_move_to_pour_pose`。
- `right_pour_ready` 和 `right_cup_pose` 保留为兼容旧记录的命令，都是 `replay_right_pour_ready` 的 alias。
- `right_table_pregrasp`、`right_table_grasp_pose`、`right_lift_cup`、`right_transfer_cup`、`right_transfer_cup_b` 已标记 deprecated，不再执行旧的猜测版动作。

`right_pour_ready` / `right_cup_pose` 注意事项：

- 它不是完整取杯流程，只设置倒奶前右手杯口姿态相关的部分右臂关节。
- 它不是完整 `left_hand_move_to_pour_pose`，只复现其中右手接奶位动作子集，不会加入左臂动作。
- `replay_right_pour_ready` / `right_pour_ready` / `right_cup_pose` 使用 `--right-pour-ready-wrist-yaw -0.70`、`--right-pour-ready-wrist-pitch -0.50`、`--right-pour-ready-wrist-roll 0.30`、`--right-pour-ready-shoulder-roll 0.70` 作为默认目标，可在启动脚本时覆盖。
- 它要求右手已经稳定夹杯。
- 它最好在 `replay_right_grasp_cup` -> `replay_right_move_to_coffee_machine` -> `replay_right_retreat_after_coffee` 之后使用。
- 该命令只在本标定脚本内发送右臂关节目标，不会自动调用完整 `coffee.py` 流程。
- 该命令不会修改或调用 `coffee_replay_safe.py`，也不会接入 `left_hand_pour_milk`。
- 该位姿必须现场确认安全，不能默认认为适合所有初始姿态、杯子尺寸或夹持状态。
- 到达接奶位后，如果杯子左右歪，优先用 `right_roll+` / `right_roll-` 微调右腕 `wrist_roll`。
- 如果杯子前后俯仰过大，优先用 `right_pitch+` / `right_pitch-` 微调右腕 `wrist_pitch`。
- 如果杯口方向不朝左手奶壶，再用 `right_yaw+` / `right_yaw-` 微调右腕 `wrist_yaw`。
- 推荐每次只改 `0.05 rad`，不要同时改多个关节；步长可用 `--right-wrist-step` 调整。
- 不要优先用 `shoulder_roll` 修杯口倾角；`shoulder_roll` 主要用于调整空间位置。

右手接奶位手腕微调命令：

- `right_roll+` / `right_roll-` 基于当前记录或 `--right-pour-ready-wrist-roll` 按 `--right-wrist-step` 微调右腕 roll。
- `right_pitch+` / `right_pitch-` 基于当前记录或 `--right-pour-ready-wrist-pitch` 按 `--right-wrist-step` 微调右腕 pitch。
- `right_yaw+` / `right_yaw-` 基于当前记录或 `--right-pour-ready-wrist-yaw` 按 `--right-wrist-step` 微调右腕 yaw。
- `right_set_roll <value>`、`right_set_pitch <value>`、`right_set_yaw <value>` 直接设置对应右腕目标。
- 这些命令只控制右臂对应手腕关节，不控制夹爪，不移动左臂，不调用 home；真实执行前都需要人工输入 `y` 确认。

左夹爪命令：

- `left_open` 使用 `robot.left_gripper.set_position(1.00, block=True)` 打开左夹爪。
- `left_grip` 使用 `robot.left_gripper.set_position(0.70, block=True)` 夹空奶壶。
- `grip` 是 `left_grip` 的兼容别名。
- `left_loose` / `left_tight` 基于本脚本内部记录的左夹爪估计值按 `--left-gripper-step` 微调，并裁剪到 `[0.0, 1.0]`。
- 这些命令都需要人工输入 `y` 才会在 execute 模式执行，并且都会写入 CSV。

右夹爪命令：

- `right_open` 使用 `robot.right_gripper.set_position(1.00, block=True)` 打开右夹爪，不调用 SDK `open()`。
- `right_grip` 使用 `robot.right_gripper.set_position(0.80, block=True)` 夹杯。
- `replay_right_grasp_cup` 的源调用记录仍保留 `robot.right_gripper.set_position(0.6)`，但实际执行值来自 `--right-gripper-cup-position`；默认保持现场标定覆盖值 0.80。
- `right_loose` / `right_tight` 基于本脚本内部记录的右夹爪估计值按 `--right-gripper-step` 微调，并裁剪到 `[0.0, 1.0]`。
- 这些命令都需要人工输入 `y` 才会在 execute 模式执行，并且都会写入 CSV。

所有夹爪命令和 replay stage 内的夹爪动作均使用 `set_position(..., block=True)`，不调用 SDK `open()` 或 `close()`。

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
- 每个 replay stage 执行前都需要人工输入 `y` 确认，stage 内部按 `coffee_replay_safe.py` 原顺序连续执行。
- 每个 replay stage 末尾都会显式执行 `robot.wait()` 和 `sleep(0.5)`。
- `replay_right_retreat_after_coffee` 包含 `right_arm.home()`，风险比普通小步标定更高，必须重点确认路径安全。
- 除 `replay_right_retreat_after_coffee` 为严格复现会调用 `right_arm.home()` 外，本工具不会自动调用 `robot.home()`、`right_arm.home()` 或 `left_arm.home()`。
- 本工具不会自动跑完整 `coffee.py` 流程。
- 本工具不会自动接入或调用 `coffee_replay_safe.py`。
- 本工具不会自动做少量水测试。
- 本工具不会自动连续执行多个 replay stage；每个阶段都需要手动输入命令。
- 本工具不会自动执行左手倒奶动作。
- 本工具不使用 SDK `open()` 或 `close()`，夹爪打开/闭合也使用 `set_position()`。
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
