# Mantis SDK 示例

请在仓库根目录使用 `python -m examples.<分类>.<脚本>` 运行示例。
会让机器人运动的示例都需要显式传入 `--ip` 或 `--sn`；示例脚本不会写死机器人目标。

## 通用参数

```bash
python -m examples.basic.connection_example --ip 192.168.1.100
python -m examples.basic.connection_example --sn BW_XXXXXXX --robot-version 3.0
```

- `--ip`: 机器人 IP 地址
- `--sn`: 机器人序列号
- `--port`: Zenoh 端口，默认 `7447`
- `--robot-version`: 机器人版本，支持 `2.0` 或 `3.0`
- `--verify / --no-verify`: 连接时是否等待机器人状态验证

## 功能覆盖表

| 功能 | 示例 |
| --- | --- |
| IP / SN 连接、`robot_ip`、`robot_sn`、`system_status` | `basic/connection_example.py` |
| `subscribe_status()` 状态订阅和状态频率测量 | `basic/status_subscription_example.py` |
| RViz 预览风格的手臂动作 | `basic/rviz_preview_example.py` |
| 机器人发现辅助函数和 `RobotDiscovery` 回调 | `discovery/discovery_example.py` |
| 原始 Zenoh `sn` 话题诊断 | `discovery/sn_topic_diagnostic.py` |
| 手臂关节控制、限位、速度、`home`、`set_joint`、`set_joints`、阻塞/非阻塞等待 | `arm/joint_control_example.py` |
| 机器人端 IK 绝对/相对末端位姿命令，左/右/双臂，`block=True` 反馈闭环和 `block=False + robot.wait()` | `arm/ik_pose_example.py` |
| 3.0 手动命名关节姿态实验 | `arm/manual_joint_pose_example.py` |
| 夹爪 open、close、half_open、set_position、速度 | `gripper/gripper_example.py` |
| 头部 look、set_pose、set_pitch、set_yaw、center | `head/head_example.py` |
| 腰部高度、up、down、move、home | `waist/waist_height_example.py` |
| 3.0 腰部弯腰、弯腰速度、bend_forward、bend_backward | `waist/waist_bend_3_0_example.py` |
| 底盘 forward、backward、strafe、turn、move、stop、摩擦补偿 | `chassis/chassis_example.py` |
| 并行/非阻塞运动和 `robot.wait()` | `workflows/parallel_motion_example.py` |
| 多步骤 workflow，组合手臂、夹爪、头部和 home | `workflows/coffee_workflow_example.py` |

## 手臂 IK 验收示例

```bash
python -m examples.arm.ik_pose_example --ip 192.168.1.100 --robot-version 3.0 --side left --mode abs
python -m examples.arm.ik_pose_example --ip 192.168.1.100 --robot-version 3.0 --side right --mode rel
python -m examples.arm.ik_pose_example --ip 192.168.1.100 --robot-version 3.0 --side both --mode both --non-blocking
python -m examples.arm.ik_pose_example --ip 192.168.1.100 --robot-version 3.0 --side left --mode abs --max-velocity 2.0 --max-acceleration 4.0 --max-jerk 8.0
```

`--non-blocking` 会先以 `block=False` 连续发送命令，再调用 `robot.wait()` 等待已发出的手臂 `command_id` 完成。该示例用于验证 SDK 端无本地 IK 依赖、ROS 端统一解算、反馈闭环阻塞等待这三件事。

`--max-velocity`、`--max-acceleration`、`--max-jerk` 会随 SDK 手臂命令发送到 ROS 侧，最终作为 `ctrl/selected_motion_profile` 控制 smoother 的 Ruckig 速度、加速度和 jerk；示例末尾的手臂回零也会沿用这组参数，避免日志里混入 SDK 默认 profile。不传时使用机器人 ROS 侧 SDK 默认 profile。

历史 notebook 实验已归档到 `archive/notebooks/`，不作为维护中的可运行示例。
