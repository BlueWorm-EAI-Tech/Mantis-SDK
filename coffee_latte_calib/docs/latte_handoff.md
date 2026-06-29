# 咖啡拉花任务离职交接

本文档用于交接 `feat/latte-pour-real-tune` 分支上的咖啡拉花/倒奶实机调试工作。接手同事优先按本文复现当前成果，再参考已有 README 和日志继续开发。

## 当前结论

- 当前目标已经从“完整复杂拉花轨迹”收敛为“右手持杯接奶位 + 左手空壶壶嘴对杯口”的最小可复现 demo。
- 已放弃用 SDK IK 手动逐点生成复杂拉花轨迹；后续应优先使用候选位姿和保守小动作。
- 当前主要入口是 `coffee_latte_calib/scripts/pour_align_calib.py`。
- 历史实机记录主要在 `pour_alignment_calib/logs/`；新整理后的脚本默认把日志写入 `coffee_latte_calib/logs/`。
- 当前最有价值的候选位姿是 `left_right_alignment_candidate_02`，已整理到 `coffee_latte_calib/docs/candidate_poses.md`。

## 分支和文件地图

先确认你在正确仓库和分支：

```zsh
cd /home/lanchong/BlueWorm_ws/Mantis-SDK-github
git branch --show-current
git status --short
```

期望分支：

```text
feat/latte-pour-real-tune
```

核心文件：

| 路径 | 用途 |
| --- | --- |
| `coffee_latte_calib/scripts/pour_align_calib.py` | 当前推荐主入口，用于右手接奶位复现、左手空壶对杯口、候选位姿保存 |
| `coffee_latte_calib/scripts/coffee_replay_safe.py` | 从原 `coffee.py` 拆出的安全复现脚本，可按阶段 dry-run 或实机执行 |
| `coffee_latte_calib/scripts/ik_tune_console.py` | 早期 IK、夹爪、手腕单项调试控制台 |
| `coffee_latte_calib/scripts/latte_pour_tune.py` | 早期倒奶/拉花段参数调试脚本 |
| `coffee_latte_calib/README.md` | 当前脚本命令总说明 |
| `coffee_latte_calib/docs/candidate_poses.md` | 已整理的候选位姿 |
| `coffee_latte_calib/docs/safety_notes.md` | 简版安全注意事项 |
| `coffee_latte_calib/docs/tuning_summary.md` | 当前调试结论摘要 |
| `pour_alignment_calib/logs/` | 旧目录中的历史 CSV/JSONL 实机调试记录 |

根目录保留了轻量 wrapper，兼容旧习惯：

```zsh
python3 pour_align_calib.py --dry-run
python3 coffee_replay_safe.py --dry-run
python3 ik_tune_console.py --dry-run
python3 latte_pour_tune.py --help
```

新开发优先直接使用 `coffee_latte_calib/scripts/` 下的脚本，避免误以为旧目录还是主入口。

## 实机信息和安全边界

历史日志中的实机信息如下，现场复现前必须确认是否变化：

- 历史 IP：`192.168.1.151`
- 历史 SN：`BW_3N5CRT22`
- 默认机器人版本：`3.0`

连接参数来自 `connection_selector.py`。如果现场 IP 或 SN 变化，启动脚本时显式传入：

```zsh
python3 coffee_latte_calib/scripts/pour_align_calib.py --dry-run --real-ip 192.168.1.151
python3 coffee_latte_calib/scripts/pour_align_calib.py --dry-run --sn BW_3N5CRT22 --conn-profile real-sn
```

实机风险边界：

- 第一次必须跑 `--dry-run`，不要直接 execute。
- 不要直接跑完整 `coffee.py`。
- 不要自动执行 `left_hand_pour_milk`。
- 不要在未验证空壶时加水。
- 不要把 `right_x+ 28` 这类 repeat 小步合并成一次大 IK。
- 出现 `near_collision`、`unsafe`、抖动、异响、杯子打滑、壶嘴卡住，立即停止。
- 夹爪正夹着杯子或奶壶时，不要无条件 `left_open` 或 `right_open`；必须确认物体由桌面或人工支撑。

实机模式必须显式传入双重确认参数：

```zsh
python3 coffee_latte_calib/scripts/pour_align_calib.py --execute --i-understand-real-robot-risk
```

## 我做了什么

主要工作分为五类：

1. 整理任务目录

把近期咖啡拉花/倒奶相关脚本集中到 `coffee_latte_calib/`，并保留根目录 wrapper 兼容旧命令。旧历史日志没有搬走，仍保留在 `pour_alignment_calib/logs/`。

2. 拆安全复现流程

把原始 `coffee.py` 中高风险的完整流程拆成可人工确认的阶段。当前不建议直接跑完整 `coffee.py`，而是用 `coffee_replay_safe.py` 或 `pour_align_calib.py` 分阶段复现。

3. 建右手接奶位链路

在 `pour_align_calib.py` 中加入 `replay_right_*` 阶段命令，按 `coffee_replay_safe.py` 的右手阶段复现右手从桌面取杯、送到咖啡机、回撤、进入倒奶前接奶位的动作。

4. 做左右手小步标定

加入右手和左手 relative IK 小步命令、手腕微调命令、夹爪命令和批量 repeat 命令。repeat 命令内部仍逐步执行，每一步成功后才累计 offset。

5. 保存候选位姿

加入 `show_state`、`save`、`save_pose`、`list_candidates`。其中 `save_pose` 会保存可复现候选位姿，并生成 `suggested_replay_commands`。

## 当前成果

当前最佳候选是 `left_right_alignment_candidate_02`。

右手候选状态：

```text
dx=+0.140
dy=+0.000
dz=-0.050
wrist_yaw=-0.700
wrist_pitch=+0.100
wrist_roll=+0.200
elbow_pitch=+0.250
shoulder_pitch=+0.000
shoulder_roll=+0.700
```

左手候选状态：

```text
dx=-0.075
dy=-0.120
dz=+0.235
wrist_yaw=-0.300
wrist_pitch=-0.400
wrist_roll=-0.200
elbow_pitch=-0.420
shoulder_pitch=-0.350
shoulder_roll=+0.500
left_gripper=0.700
right_gripper=0.800
```

历史候选记录位置：

```text
pour_alignment_calib/logs/pour_align_candidates.jsonl
```

最后一条关键 CSV 记录位置：

```text
pour_alignment_calib/logs/pour_align_trials_20260521_105308.csv
```

该候选的现场观察备注是 `spout_in_cup`，即空壶壶嘴已对到杯口附近。它不是带水倒奶的验证结果。

## 复现流程

### 1. Dry-run 检查

先确认脚本能启动，并查看菜单和命令：

```zsh
cd /home/lanchong/BlueWorm_ws/Mantis-SDK-github
python3 coffee_latte_calib/scripts/pour_align_calib.py --dry-run
```

进入控制台后可输入：

```text
help
help all
help right
help left
show_state
list_candidates
q
```

如果只想确认连接配置，不连接机器人：

```zsh
python3 coffee_latte_calib/scripts/pour_align_calib.py --dry-run --print-connection-config
```

### 2. 实机启动

确认机器人端 SDK bridge 已启动、周围无障碍、有人看护急停后，再进入 execute：

```zsh
python3 coffee_latte_calib/scripts/pour_align_calib.py --execute --i-understand-real-robot-risk
```

如需固定 IP：

```zsh
python3 coffee_latte_calib/scripts/pour_align_calib.py --execute --i-understand-real-robot-risk --conn-profile real-ip --real-ip 192.168.1.151
```

每个真实动作前脚本仍会要求人工输入 `y` 确认。不要用习惯性回车跳过现场观察。

### 3. 右手取杯到接奶位

推荐先空载复现，再夹空杯复现。

```text
replay_right_grasp_cup
replay_right_move_to_coffee_machine
replay_right_retreat_after_coffee
replay_right_pour_ready
```

当前候选需要继续加右手 offset：

```text
right_x+ 28
right_z- 10
```

含义：

- `right_x+ 28` 是连续 28 次 `0.005 m` 小步，累计 `+0.140 m`。
- `right_z- 10` 是连续 10 次 `0.005 m` 小步，累计 `-0.050 m`。
- 这两个命令不是一次性大位移 IK。

右手夹杯默认值：

```text
right_grip -> robot.right_gripper.set_position(0.80, block=True)
```

如需严格复现旧 `coffee_replay_safe.py` 的抓杯值，可启动时传：

```zsh
python3 coffee_latte_calib/scripts/pour_align_calib.py --execute --i-understand-real-robot-risk --right-gripper-cup-position 0.6
```

但当前现场经验值是 `0.80`。

### 4. 左手空壶对杯口

右手稳定后，再让左手夹空壶并进入倒奶预备框架：

```text
left_grip
replay_left_move_to_pour_pose_left_only
replay_left_pour_prep_frame
```

复现当前候选 offset：

```text
left_x- 15
left_y- 40
left_z+ 47
```

含义：

- `left_x- 15` 累计 `-0.075 m`。
- `left_y- 40` 按小步累计 `-0.120 m`。
- `left_z+ 47` 累计 `+0.235 m`。

左手夹空奶壶默认值：

```text
left_grip -> robot.left_gripper.set_position(0.70, block=True)
```

壶嘴方向微调后的候选值：

```text
left_yaw=-0.300
left_pitch=-0.400
left_roll=-0.200
```

如果复现后方向不对，优先用小步命令：

```text
left_yaw+
left_yaw-
left_pitch+
left_pitch-
left_roll+
left_roll-
```

不要一开始直接大角度 `wrist_roll`。

### 5. 保存复现结果

复现或微调后，必须保存现场状态：

```text
show_state
obs spout_in_cup_recheck
save current_recheck_note
save_pose left_right_alignment_candidate_02_recheck
```

新脚本默认写入：

```text
coffee_latte_calib/logs/pour_align_candidates.jsonl
```

历史记录仍在：

```text
pour_alignment_calib/logs/pour_align_candidates.jsonl
```

## 常用命令速查

右手 replay：

```text
replay_right_grasp_cup
replay_right_move_to_coffee_machine
replay_right_retreat_after_coffee
replay_right_pour_ready
```

右手接奶位微调：

```text
right_x+
right_x-
right_y+
right_y-
right_z+
right_z-
right_roll+
right_roll-
right_pitch+
right_pitch-
right_yaw+
right_yaw-
right_elbow+
right_elbow-
right_shoulder_pitch+
right_shoulder_pitch-
```

左手空壶对杯口：

```text
left_grip
replay_left_move_to_pour_pose_left_only
replay_left_pour_prep_frame
left_x+
left_x-
left_y+
left_y-
left_z+
left_z-
left_yaw+
left_yaw-
left_pitch+
left_pitch-
left_roll+
left_roll-
roll0
roll03
roll05
roll07
```

记录：

```text
show_state
obs <note>
save <note>
save_pose <candidate_name>
list_candidates
```

## 后续开发建议

1. 先复现 `left_right_alignment_candidate_02`，不要直接改大动作参数。
2. 若要少量清水测试，先空壶连续通过 `roll07`，确认没有碰撞、卡住、打滑或杯口干涉。
3. 水测试建议先只做人工触发的小步倾斜，不要接入完整自动流程。
4. 如果要把候选位姿接入正式流程，先把 replay 命令固化为显式阶段，不要直接调用历史 `coffee.py`。
5. 后续可以实现 `apply_candidate <name>`，从 JSONL 读取候选并逐步回放 `suggested_replay_commands`。
6. 若现场更换杯子、奶壶、桌面高度或夹持方式，原候选只能作为起点，不能当作安全保证。

## 最小验证方式

本文档是交接文档，不要求构建。

文档修改后建议做静态检查：

```zsh
git diff -- coffee_latte_calib/docs/latte_handoff.md coffee_latte_calib/README.md
python3 coffee_latte_calib/scripts/pour_align_calib.py --dry-run --print-connection-config
```

第二条命令只打印连接配置，不连接机器人，不执行动作。
