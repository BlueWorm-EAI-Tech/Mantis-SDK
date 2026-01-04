# Mantis SDK Release Notes

📅 最新版本: V1.1.0 (2026-01-04)


## Changelog

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

### [1.1.0] - 2026-01-04

**新增**

- ⚡ 所有运动方法添加 `block` 参数，支持阻塞/非阻塞模式
- ⚡ 新增 `wait()` 方法，等待所有部件运动完成
- ⚡ 新增 `is_moving` 属性，检查部件是否在运动中
- 🦿 新增 Waist（腰部）控制模块
- 🚗 底盘支持 Gazebo 物理仿真，可在仿真中预览底盘移动
- 🚗 底盘 API 重构为基于距离/角度的安全控制模式

**修复**

- 🔧 修复双臂 `shoulder_yaw` 关节零点限位错误（旧: 0.08~1.04，新: -0.213~2.029）
- 🔧 修复夹爪控制时关节位置计算错误导致夹爪移动方向异常的问题
- 🔧 修正夹爪归一化值到实际关节位置的单位转换 (0.0-1.0 → 0.0-0.04m)

**改进**

- 📦 合并 RViz/Gazebo 仿真 launch 文件为统一的 `sdk_sim.launch.py`
- 📖 更新 README 文档，添加并行运动示例

### [1.0.2] - 2025-12-30

- 优化代码框架

### [1.0.1] - 2025-12-29

- 初始发布

---

**问题反馈**: [GitHub Issues](https://github.com/BlueWorm-EAI-Tech/mantis-sdk/issues)

**许可证**: MIT License

© 2025-2026 BlueWorm-EAI-Tech. All rights reserved.
