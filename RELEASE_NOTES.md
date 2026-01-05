# Mantis SDK Release Notes

📅 最新版本: V1.2.0 (2026-01-05)

## Changelog

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

### [1.2.0] - 2026-01-05

**重构**

- 🔧 **移除 zenoh-bridge-ros2dds 依赖**: 改用纯 Python Zenoh + JSON 格式通讯，解决桥接通讯不稳定问题
- 🔧 **删除 CDR 编解码模块**: 不再需要 ROS2 消息序列化，代码更简洁
- 🔧 **统一 sim/real 模式**: SDK 不再区分仿真和实机，统一发布到 `sdk/joint_states` 和 `sdk/chassis` 话题

**新增**

- 🚗 底盘新增 `set_friction(linear, angular)` 方法，设置摩擦补偿系数
- 🚗 摩擦系数越大，运动时间越长，用于补偿地面摩擦力导致的距离损失

**修复**

- 🔧 修复夹爪开合方向映射问题（实机 0=打开, 1=关闭，已在 Bridge 端反转）
- 🔧 修复机器人连接时在未建立连接前调用 `home()` 导致的异常
- 🔧 修复关节方向修正影响 RViz 和实机同步的问题（方向修正移至 Bridge 端）

**优化**

- 🚀 提高底盘默认线速度: 0.1 m/s → 1.0 m/s
- 🚀 调整底盘默认角速度: 0.5 rad/s → 0.3 rad/s
- 🚀 提高底盘最大速度限制: 线速度 3.0 m/s，角速度 2.0 rad/s

**问题反馈**: [GitHub Issues](https://github.com/BlueWorm-EAI-Tech/mantis-sdk/issues)

**许可证**: MIT License

© 2025-2026 BlueWorm-EAI-Tech. All rights reserved.
