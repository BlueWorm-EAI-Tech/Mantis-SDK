# Mantis SDK Release Notes

📅 最新版本: V1.3.3 (2026-02-05)

## Changelog

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

### [1.3.3] - 2026-02-05

**修复**

- 修复混合使用单关节控制（如 `set_shoulder_pitch`）与 IK 增量控制时，IK 目标点未更新的问题。
  - 现在所有单关节设置方法 (`set_joint`, `set_shoulder_pitch` 等) 都会自动同步 IK 求解器的内部目标状态。
  - 解决了先手动调整关节，再调用 `ik(abs=False)` 时发生位置回跳的问题。

- 修复 IK 增量控制 (`abs=False`) 在连续调用时无法正确累积的问题。
- 优化内部目标位姿维护逻辑：
  - `set_joints`/`home` 等绝对控制指令会自动同步 IK 目标点。
  - `ik(abs=False)` 现在基于内部维护的目标点进行累加，确保连续增量运动的连贯性。

**问题反馈**: [GitHub Issues](https://github.com/BlueWorm-EAI-Tech/mantis-sdk/issues)

**许可证**: MIT License

© 2025-2026 BlueWorm-EAI-Tech. All rights reserved.
