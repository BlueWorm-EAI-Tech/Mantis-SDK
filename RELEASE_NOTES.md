# Mantis SDK Release Notes

📅 最新版本: V1.3.10 (2026-06-16)

## Changelog

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

### [1.3.10] - 2026-06-16

**清理**

- 移除 SDK 包内不再使用的本地 IK URDF / mesh 资源，IK 模型由机器人 ROS 侧维护。
- 移除 `MANIFEST.in` 中对 `mantis/model` 的打包规则，SDK 客户端安装包只保留运行时需要的 Python 代码。

**测试**

- 新增回归测试，防止 SDK 重新 vendor 本地 IK 模型资源或本地 IK 求解依赖。

### [1.3.9] - 2026-05-14

**修复**

- 修复 `ik(abs=False)` 在连续调用时内部增量目标被重置的问题，恢复相对 IK 与绝对 IK 的预期语义差异。
- 保留手动 `set_joint` / `set_joints` 对 IK 目标点的同步重置逻辑，避免手动调姿后相对 IK 基于过期目标继续累积。

**对齐**

- SDK 双臂 IK 模型切换为与 VR 共用的 `mantis_2_0_ik.urdf`，统一 2.0 / 3.0 的 IK 解算模型语义。
- IK 模型缺失时改为显式报错，不再静默回退到旧版本地模型。

**测试**

- 新增 `tests/test_ik_model_alignment.py`，锁定 SDK 与 VR 使用同一份 IK 模型。
- 补充 IK 相对模式目标保持与手动关节控制重置逻辑的回归测试。

### [1.3.8] - 2026-05-14

**修复**

- 2.0关节映射

### [1.3.7] - 2026-05-11

**修复**

- 同步 SDK 关节方向修正表到当前 bridge/runtime 约定，修复 SDK 模式下部分肩部与腕部关节方向与现行 VR/实机表现不一致的问题。
- 新增 `tests/test_joint_direction_map.py` 回归测试，锁定 14 个手臂关节的方向映射，避免后续版本回退到旧方向约定。

**兼容性**

- `v1.3.7` 仅支持机器人端 `26.5.11.1` 及以上版本。
- 该版本依赖 2026-05-11 之后的关节方向约定；如果机器人端仍是旧方向定义，请继续使用 `v1.3.5`。

### [1.3.5] - 2026-03-06

**新增**

- 新增局域网机器人发现能力（`/sn`）：
  - 增加 `RobotDiscovery` 独立发现模块。
  - 支持增量维护在线机器人列表（上线加入、离线超时剔除）。
  - 支持无需实例化的函数接口：
    - `start_robot_discovery()`
    - `list_discovered_robots()`
    - `stop_robot_discovery()`

**变更**

- SDK 连接流程升级为“身份解析 + 状态校验”双重验证：
  - `Mantis.connect` 支持按 `ip` 或 `sn` 连接。
  - 连接时先通过发现话题解析目标机器人身份，再通过状态话题完成校验。
- 多机控制隔离增强：
  - SDK 控制发布改为带 SN 前缀的话题（`<SN>/sdk/joint_states`、`<SN>/sdk/chassis`）。
  - SDK 状态订阅改为 `<SN>/sdk/system_status`。
  - 避免同一局域网多机器人被同一控制流串控。
- 文档更新：
  - 更新版本兼容矩阵：
    - `v1.3.2 以下` ↔ `< 26.2.3.1`
    - `v1.3.2 ~ v1.3.4` ↔ `26.2.3.1 ~ 26.3.3.1`
    - `v1.3.5` ↔ `26.3.6.2 ~ < 26.5.11.1`
    - `v1.3.6 及以上` ↔ `>= 26.5.11.1`

**工具**

- 新增 `test_sn_read.py` 诊断脚本，用于直接验证 Zenoh `sn` 话题是否可读、数据格式是否正确。

### [1.3.4] - 2026-02-07

**修复**

- 修复混合使用单关节控制（如 `set_shoulder_pitch`）与 IK 增量控制时，IK 目标点未更新的问题。
  - 现在所有单关节设置方法 (`set_joint`, `set_shoulder_pitch` 等) 都会自动同步 IK 求解器的内部目标状态。
  - 解决了先手动调整关节，再调用 `ik(abs=False)` 时发生位置回跳的问题。
- 修复 IK 增量控制 (`abs=False`) 在连续调用时无法正确累积的问题。
- 优化内部目标位姿维护逻辑：
  - `set_joints`/`home` 等绝对控制指令会自动同步 IK 目标点。
  - `ik(abs=False)` 现在基于内部维护的目标点进行累加，确保连续增量运动的连贯性。
- 修复 pip 下载缺少模型文件的问题。

**问题反馈**: [GitHub Issues](https://github.com/BlueWorm-EAI-Tech/mantis-sdk/issues)

**许可证**: MIT License

© 2025-2026 BlueWorm-EAI-Tech. All rights reserved.
