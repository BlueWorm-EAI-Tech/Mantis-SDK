# 双臂关节方向映射报告

当前还没有汇总日志。请先运行 `map_arm_joint_directions.py` 进行 dry-run 或实机单关节测试，再执行：

```bash
python3 sdk_arm_mapping/scripts/generate_arm_mapping_report.py
```

生成后的报告会覆盖当前文件，并补齐以下内容：

1. 测试说明
2. 安全提醒
3. 左臂关节方向映射表
4. 右臂关节方向映射表
5. 异常/中止记录
6. 未测试项
7. 对 `coffee.py` / 咖啡拉花调试最关键的结论

说明：

- 报告不会明文写入原始 SN / IP。
- 如需附带连接目标，请先手工脱敏，例如 `BW_****`、`192.168.*.*`。
