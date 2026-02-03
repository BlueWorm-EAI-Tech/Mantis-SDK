# Mantis SDK Release Notes

📅 最新版本: V1.3.2 (2026-01-27)

## Changelog

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

### [1.3.2] - 2026-02-03

**文档**

- 更新机械臂的运动检测功能，使用上位机实时返回的关节状态作为判断关节运动的依据，避免在运动过程中因为小的误差导致判断错误
- 删除sdk中的平滑滤波，所有的运动规划交给上位机
- 注意！！！：使用本sdk必须将上位机代码、VR程序更新到最新版本，本sdk只适配26.2.3.1以上的机器人代码，请更新sdk后务必升级机器人代码



**问题反馈**: [GitHub Issues](https://github.com/BlueWorm-EAI-Tech/mantis-sdk/issues)

**许可证**: MIT License

© 2025-2026 BlueWorm-EAI-Tech. All rights reserved.
