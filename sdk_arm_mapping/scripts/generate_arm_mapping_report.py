#!/usr/bin/env python3
"""Generate a Markdown report from arm joint direction mapping CSV logs."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LOG_DIR = REPO_ROOT / "sdk_arm_mapping" / "logs"
DEFAULT_OUTPUT = REPO_ROOT / "sdk_arm_mapping" / "docs" / "arm_joint_direction_mapping.md"

JOINT_SEQUENCE = [
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
    "elbow_pitch",
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
]
DIRECTION_SEQUENCE = ["positive_delta", "negative_delta"]
PRIORITY_JOINTS = [
    "shoulder_pitch",
    "shoulder_roll",
    "elbow_pitch",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
    "shoulder_yaw",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="根据 CSV 日志生成双臂关节方向映射 Markdown 报告。")
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=DEFAULT_LOG_DIR,
        help=f"CSV 日志目录，默认: {DEFAULT_LOG_DIR}",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Markdown 输出路径，默认: {DEFAULT_OUTPUT}",
    )
    return parser.parse_args()


def markdown_escape(value: str) -> str:
    text = (value or "").strip()
    if not text:
        return "-"
    return text.replace("|", r"\|").replace("\n", "<br>")


def load_rows(log_dir: Path) -> List[Dict[str, str]]:
    if not log_dir.exists():
        return []

    rows: List[Dict[str, str]] = []
    for csv_file in sorted(log_dir.glob("*.csv")):
        with csv_file.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            for index, row in enumerate(reader):
                normalized = dict(row)
                normalized["_source_file"] = csv_file.name
                normalized["_source_index"] = str(index)
                rows.append(normalized)
    return rows


def latest_rows_by_key(rows: Sequence[Dict[str, str]]) -> Dict[Tuple[str, str, str], Dict[str, str]]:
    latest: Dict[Tuple[str, str, str], Dict[str, str]] = {}
    for row in rows:
        key = (row.get("side", ""), row.get("joint", ""), row.get("direction_type", ""))
        latest[key] = row
    return latest


def latest_success_rows(rows: Sequence[Dict[str, str]]) -> Dict[Tuple[str, str, str], Dict[str, str]]:
    latest: Dict[Tuple[str, str, str], Dict[str, str]] = {}
    for row in rows:
        if row.get("status") != "ok":
            continue
        key = (row.get("side", ""), row.get("joint", ""), row.get("direction_type", ""))
        latest[key] = row
    return latest


def format_input_target(row: Dict[str, str]) -> str:
    rad = row.get("target_value_rad", "").strip()
    deg = row.get("target_value_deg", "").strip()
    if not rad and not deg:
        return "-"
    if rad and deg:
        return f"{rad} rad / {deg} deg"
    return rad or deg


def render_mapping_table(
    side: str,
    latest_rows: Dict[Tuple[str, str, str], Dict[str, str]],
) -> List[str]:
    title = "左臂" if side == "left" else "右臂"
    lines = [f"## {'3' if side == 'left' else '4'}. {title}关节方向映射表", ""]
    lines.append("| side | joint | direction_type | input target | observed motion | note | status |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- |")

    for joint in JOINT_SEQUENCE:
        for direction_type in DIRECTION_SEQUENCE:
            row = latest_rows.get((side, joint, direction_type))
            if row is None:
                lines.append(
                    f"| {side} | {joint} | {direction_type} | - | 未测试 | - | not_tested |"
                )
                continue
            lines.append(
                "| "
                + " | ".join(
                    [
                        side,
                        joint,
                        direction_type,
                        markdown_escape(format_input_target(row)),
                        markdown_escape(row.get("observation", "")),
                        markdown_escape(row.get("note", "")),
                        markdown_escape(row.get("status", "")),
                    ]
                )
                + " |"
            )
    lines.append("")
    return lines


def render_abnormal_records(rows: Sequence[Dict[str, str]]) -> List[str]:
    lines = ["## 5. 异常/中止记录", ""]
    abnormal_rows = [row for row in rows if row.get("status") in {"aborted", "error"}]
    if not abnormal_rows:
        lines.append("当前没有异常或中止记录。")
        lines.append("")
        return lines

    lines.append(
        "| timestamp | side | joint | direction_type | observed motion | note | status | error_message |"
    )
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for row in abnormal_rows:
        lines.append(
            "| "
            + " | ".join(
                [
                    markdown_escape(row.get("timestamp", "")),
                    markdown_escape(row.get("side", "")),
                    markdown_escape(row.get("joint", "")),
                    markdown_escape(row.get("direction_type", "")),
                    markdown_escape(row.get("observation", "")),
                    markdown_escape(row.get("note", "")),
                    markdown_escape(row.get("status", "")),
                    markdown_escape(row.get("error_message", "")),
                ]
            )
            + " |"
        )
    lines.append("")
    return lines


def render_untested_items(
    success_rows: Dict[Tuple[str, str, str], Dict[str, str]],
    latest_rows: Dict[Tuple[str, str, str], Dict[str, str]],
) -> List[str]:
    lines = ["## 6. 未测试项", ""]
    untested: List[str] = []
    for side in ("left", "right"):
        for joint in JOINT_SEQUENCE:
            for direction_type in DIRECTION_SEQUENCE:
                key = (side, joint, direction_type)
                if key in success_rows:
                    continue
                latest_status = latest_rows.get(key, {}).get("status")
                if latest_status:
                    untested.append(f"- `{side}` / `{joint}` / `{direction_type}`: 当前无成功记录，最新状态为 `{latest_status}`。")
                else:
                    untested.append(f"- `{side}` / `{joint}` / `{direction_type}`: 尚无任何记录。")

    if not untested:
        lines.append("全部 28 个方向项都已有成功记录。")
    else:
        lines.extend(untested)
    lines.append("")
    return lines


def build_joint_summary(
    side: str,
    joint: str,
    success_rows: Dict[Tuple[str, str, str], Dict[str, str]],
) -> str:
    positive = success_rows.get((side, joint, "positive_delta"))
    negative = success_rows.get((side, joint, "negative_delta"))
    if positive is None and negative is None:
        return ""

    parts: List[str] = [f"`{side} {joint}`"]
    if positive is not None:
        parts.append(f"+delta => {positive.get('observation', '未填写')}")
    if negative is not None:
        parts.append(f"-delta => {negative.get('observation', '未填写')}")
    return "，".join(parts)


def render_key_conclusions(
    success_rows: Dict[Tuple[str, str, str], Dict[str, str]],
) -> List[str]:
    lines = ["## 7. 对 coffee.py / 咖啡拉花调试最关键的结论", ""]
    lines.append(
        "- `coffee.py` 里优先受影响的关节通常是 `shoulder_pitch`、`shoulder_roll`、`elbow_pitch`、`wrist_roll`；这些方向没确认前，不建议直接调大幅拉花动作。"
    )
    lines.append(
        "- `wrist_pitch` 和 `wrist_yaw` 更直接影响末端姿态、倾倒角和杯口朝向；它们的正负方向应该先通过单关节测试确认，再进入倾倒轨迹微调。"
    )
    lines.append(
        "- 左右臂不要默认按“镜像”理解，应该分别记录；如果现场观察到镜像关系，也建议明确写进 CSV 备注。"
    )

    summaries: List[str] = []
    for side in ("left", "right"):
        for joint in PRIORITY_JOINTS:
            summary = build_joint_summary(side, joint, success_rows)
            if summary:
                summaries.append(f"- 已记录方向样本：{summary}。")

    if summaries:
        lines.extend(summaries[:8])
    else:
        lines.append("- 目前还没有成功日志，因此还不能从真实观察结果反推 `coffee.py` 的动作语义。")

    lines.append("")
    return lines


def render_report(rows: Sequence[Dict[str, str]]) -> str:
    latest_rows = latest_rows_by_key(rows)
    success_rows = latest_success_rows(rows)

    lines: List[str] = ["# 双臂关节方向映射报告", ""]
    lines.append("## 1. 测试说明")
    lines.append("")
    lines.append(f"- 报告生成时间：`{datetime.now().isoformat(timespec='seconds')}`")
    lines.append(f"- 已读取 CSV 记录数：`{len(rows)}`")
    lines.append("- 当前范围只覆盖 `left_arm` 与 `right_arm` 的 7 个关节，不包含 gripper、head、waist、chassis 和 IK。")
    lines.append("- 报告不会写入原始 SN / IP；如需附带连接目标，请使用脱敏形式，例如 `BW_****`、`192.168.*.*`。")
    lines.append("")

    lines.append("## 2. 安全提醒")
    lines.append("")
    lines.append("- 默认应该先 dry-run，再做实机单关节小幅测试。")
    lines.append("- 建议顺序：先 wrist，再 elbow，最后 shoulder。")
    lines.append("- shoulder 关节会带动整条手臂，风险最高，必须保证周围无障碍物。")
    lines.append("- 不要夹持杯子、奶壶或其他工具做方向映射。")
    lines.append("")

    lines.extend(render_mapping_table("left", latest_rows))
    lines.extend(render_mapping_table("right", latest_rows))
    lines.extend(render_abnormal_records(rows))
    lines.extend(render_untested_items(success_rows, latest_rows))
    lines.extend(render_key_conclusions(success_rows))

    return "\n".join(lines).rstrip() + "\n"


def main() -> int:
    args = parse_args()
    rows = load_rows(args.log_dir)
    report = render_report(rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(report, encoding="utf-8")
    print(f"已生成报告: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
