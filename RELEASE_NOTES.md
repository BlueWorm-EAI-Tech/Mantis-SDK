# Mantis SDK V1.0.4 Release Notes

📅 2026-1-4

## 概述

Mantis SDK V1.0 是 Mantis 机器人的 Python 控制接口，**无需安装 ROS2**，通过 Zenoh 协议直接与机器人通信。适用于客户二次开发、快速原型验证、教学演示等场景。

## 主要特性

### 🎮 简洁的 API 设计

```python
from mantis import Mantis

with Mantis(ip="192.168.1.100") as robot:
    robot.left_arm.set_shoulder_pitch(-0.5)
    robot.head.look_left()
    robot.left_gripper.open()
```

### 🦾 完整的机器人控制


| 模块        | 功能                                                                           |
| ----------- | ------------------------------------------------------------------------------ |
| **Arm**     | 双臂 7 自由度控制 (shoulder_pitch/yaw/roll, elbow_pitch, wrist_roll/pitch/yaw) |
| **Gripper** | 左右夹爪开合控制 (0.0 ~ 1.0)                                                   |
| **Head**    | 头部俯仰/偏航控制                                                              |
| **Chassis** | 全向底盘移动控制 (vx, vy, omega)                                               |

### 🛡️ 安全保护

- **关节限位保护**: 自动限制在 URDF 定义范围内
- **软停止**: `stop()` 方法立即停止所有运动
- **异常处理**: 连接断开自动清理资源

### 🖥️ 仿真预览

```python
# 仿真模式 - 在 RViz 中预览，不连接实机
with Mantis(sim=True) as robot:
    robot.left_arm.set_joints([0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0])
```

- 需配合 `bw_motion_ws` 的 `sdk_sim.launch.py` 启动
- EMA 平滑算法，运动流畅

### 📊 关节反馈

```python
def on_feedback(joint_names, positions):
    print(f"关节位置: {dict(zip(joint_names, positions))}")

robot.subscribe_feedback(on_feedback)
```

## API 参考

### Mantis 主类


| 方法                           | 说明             |
| ------------------------------ | ---------------- |
| `connect()`                    | 建立连接         |
| `disconnect()`                 | 断开连接         |
| `home()`                       | 所有关节回零位   |
| `stop()`                       | 停止所有运动     |
| `set_smoothing(alpha, rate)`   | 设置仿真平滑参数 |
| `subscribe_feedback(callback)` | 订阅关节反馈     |

### Arm 手臂控制


| 方法                         | 说明              |
| ---------------------------- | ----------------- |
| `set_joints(positions)`      | 批量设置 7 个关节 |
| `set_joint(index, position)` | 设置单个关节      |
| `set_shoulder_pitch(value)`  | 设置肩部俯仰      |
| `set_elbow_pitch(value)`     | 设置肘部俯仰      |
| ...                          | 其他关节类似      |

### Gripper 夹爪控制


| 方法                  | 说明                 |
| --------------------- | -------------------- |
| `set_position(value)` | 设置位置 (0.0 ~ 1.0) |
| `open()`              | 完全张开             |
| `close()`             | 完全闭合             |
| `half_open()`         | 半开状态             |

### Head 头部控制


| 方法                   | 说明          |
| ---------------------- | ------------- |
| `set_pose(pitch, yaw)` | 设置俯仰/偏航 |
| `look_left()`          | 看向左边      |
| `look_right()`         | 看向右边      |
| `look_up()`            | 抬头          |
| `look_down()`          | 低头          |
| `center()`             | 回中          |

### Chassis 底盘控制


| 方法                          | 说明     |
| ----------------------------- | -------- |
| `set_velocity(vx, vy, omega)` | 设置速度 |
| `forward(speed)`              | 前进     |
| `backward(speed)`             | 后退     |
| `strafe_left(speed)`          | 左平移   |
| `strafe_right(speed)`         | 右平移   |
| `turn_left(speed)`            | 左转     |
| `turn_right(speed)`           | 右转     |
| `stop()`                      | 停止     |

## 安装

### 依赖安装

```bash
pip install eclipse-zenoh
```

### SDK 安装

```bash
cd mantis
pip install -e .
```

## 仿真环境设置

SDK 仿真模式需要配合 ROS2 环境：

```bash
# 终端 1: 启动仿真环境
cd ~/bw_motion_ws
source install/setup.bash
ros2 launch bw_sim2real sdk_sim.launch.py

# 终端 2: 启动 Zenoh 桥接
zenoh-bridge-ros2dds -d 99

# 终端 3: 运行 SDK
python your_script.py
```

> **注意**: 仿真环境由 [Mantis Sim2Real](../bw_motion_ws/) 项目提供

## 系统要求

- **Python**: 3.8+
- **依赖**: eclipse-zenoh
- **仿真模式额外要求**: ROS2 Humble + zenoh-bridge-ros2dds

## 已知限制

- 底盘控制在仿真模式下暂不支持预览
- 仿真模式需要先启动 ROS2 环境和 Zenoh 桥接

---

## Changelog

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

### [1.0.4] - 2026-01-04

**修复**

- 修复夹爪控制时关节位置计算错误导致夹爪移动方向异常的问题
- 修正夹爪归一化值到实际关节位置的单位转换 (0.0-1.0 → 0.0-0.04m)

### [1.0.2] - 2025-12-30

优化代码框架

**问题反馈**: [GitHub Issues](https://github.com/BlueWorm-EAI-Tech/mantis-sdk/issues)

**许可证**: MIT License

© 2025 BlueWorm-EAI-Tech. All rights reserved.
