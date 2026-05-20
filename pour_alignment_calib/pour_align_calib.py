"""
Empty pitcher spout-to-cup alignment calibration console.

This script is intentionally independent from the coffee replay pipeline.
It only supports one confirmed operator action at a time.
"""

from __future__ import annotations

import argparse
import csv
import json
import shlex
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from connection_selector import add_connection_args, connect_robot_with_selector  # noqa: E402


DEFAULT_ROBOT_VERSION = "3.0"
DEFAULT_LINEAR_STEP = 0.005
DEFAULT_LINEAR_STEP_SMALL = 0.003
DEFAULT_ROT_STEP = 0.05
DEFAULT_GRIPPER_POSITION = 0.70
DEFAULT_LEFT_GRIPPER_OPEN_POSITION = 1.00
DEFAULT_LEFT_GRIPPER_CLOSED_POSITION = 0.00
DEFAULT_LEFT_GRIPPER_PITCHER_POSITION = 0.70
DEFAULT_LEFT_GRIPPER_STEP = 0.05
DEFAULT_RIGHT_GRIPPER_OPEN_POSITION = 1.00
DEFAULT_RIGHT_GRIPPER_CLOSED_POSITION = 0.00
DEFAULT_RIGHT_GRIPPER_CUP_POSITION = 0.80
DEFAULT_RIGHT_GRIPPER_STEP = 0.05
DEFAULT_MAX_WRIST_ROLL = 0.70
GRIPPER_MIN = 0.0
GRIPPER_MAX = 1.0
LOG_DIR = Path(__file__).resolve().parent / "logs"

# Source: coffee_replay_safe.py right-hand replay stages only.
# Do not import or call coffee_replay_safe.py; these are explicit SDK calls
# copied from the named source stages for calibration use.
RIGHT_REPLAY_STAGES = {
    "replay_right_grasp_cup": [
        {
            "kind": "gripper",
            "target": "right_gripper",
            "method": "set_position",
            "value_arg": "right_gripper_closed_position",
            "block": True,
            "description": "右夹爪初始化为闭合",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "004",
            "source_sdk_call": "robot.right_gripper.close()",
        },
        {
            "kind": "gripper",
            "target": "left_gripper",
            "method": "set_position",
            "value_arg": "left_gripper_closed_position",
            "block": True,
            "description": "左夹爪初始化为闭合，仅 --include-left-gripper-init 时执行",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "005",
            "source_sdk_call": "robot.left_gripper.close()",
            "enabled_arg": "include_left_gripper_init",
        },
        {
            "kind": "gripper",
            "target": "right_gripper",
            "method": "set_position",
            "value_arg": "right_gripper_open_position",
            "block": True,
            "description": "右夹爪打开准备抓杯",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "006",
            "source_sdk_call": "robot.right_gripper.open()",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_shoulder_pitch",
            "value": 0.7,
            "block": False,
            "description": "右肩俯仰到抓杯预备位",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "007",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_shoulder_roll",
            "value": -0.42,
            "block": False,
            "description": "右肩翻滚到抓杯预备位",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "008",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_wrist_roll",
            "value": 0.1,
            "block": True,
            "description": "右腕翻滚对齐杯把方向",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "009",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_elbow_pitch",
            "value": 1.0,
            "block": False,
            "description": "右肘下探接近杯子",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "010",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_wrist_pitch",
            "value": 0.1,
            "block": True,
            "description": "右腕俯仰微调抓取姿态",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "011",
        },
        {
            "kind": "sleep",
            "target": "robot",
            "method": "sleep",
            "value": 1.0,
            "block": True,
            "description": "保留原始抓杯前等待",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "012",
        },
        {
            "kind": "gripper",
            "target": "right_gripper",
            "method": "set_position",
            "value_arg": "right_gripper_cup_position",
            "block": True,
            "description": "右夹爪收至抓杯位置，标定默认 0.80",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "013",
            "source_sdk_call": "robot.right_gripper.set_position(0.6)",
        },
        {
            "kind": "sleep",
            "target": "robot",
            "method": "sleep",
            "value": 1.0,
            "block": True,
            "description": "保留原始抓杯后等待",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "014",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_elbow_pitch",
            "value": 0.6,
            "block": False,
            "description": "右肘抬起准备离开杯架",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "015",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_shoulder_pitch",
            "value": 0.6,
            "block": True,
            "description": "右肩俯仰抬杯离桌",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "016",
        },
        {
            "kind": "wait",
            "target": "robot",
            "method": "wait",
            "value": None,
            "block": True,
            "description": "阶段结束统一等待，兜底处理 block=False 动作",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "017",
        },
        {
            "kind": "sleep",
            "target": "robot",
            "method": "sleep",
            "value": 0.5,
            "block": True,
            "description": "阶段结束后观察停稳",
            "source_stage": "right_hand_grasp_cup",
            "source_action_id": "018",
        },
    ],
    "replay_right_move_to_coffee_machine": [
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_shoulder_roll",
            "value": 0.3,
            "block": None,
            "description": "右肩翻滚离开抓杯位",
            "source_stage": "right_hand_move_to_coffee_machine",
            "source_action_id": "019",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_shoulder_pitch",
            "value": 0.7,
            "block": False,
            "description": "右肩俯仰朝向咖啡机",
            "source_stage": "right_hand_move_to_coffee_machine",
            "source_action_id": "020",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_shoulder_roll",
            "value": 0.65,
            "block": False,
            "description": "右肩翻滚横向送杯到咖啡机",
            "source_stage": "right_hand_move_to_coffee_machine",
            "source_action_id": "021",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_wrist_roll",
            "value": -0.3,
            "block": None,
            "description": "右腕翻滚调整接咖啡角度",
            "source_stage": "right_hand_move_to_coffee_machine",
            "source_action_id": "022",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_shoulder_pitch",
            "value": 0.98,
            "block": False,
            "description": "右肩俯仰继续送杯到咖啡出口",
            "source_stage": "right_hand_move_to_coffee_machine",
            "source_action_id": "023",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_elbow_pitch",
            "value": 0.98,
            "block": False,
            "description": "右肘俯仰配合送杯到最终接咖啡位",
            "source_stage": "right_hand_move_to_coffee_machine",
            "source_action_id": "024",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_wrist_roll",
            "value": -0.68,
            "block": False,
            "description": "右腕翻滚微调接咖啡姿态",
            "source_stage": "right_hand_move_to_coffee_machine",
            "source_action_id": "025",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_wrist_pitch",
            "value": 0.0,
            "block": True,
            "description": "右腕俯仰归零以对齐杯口",
            "source_stage": "right_hand_move_to_coffee_machine",
            "source_action_id": "026",
        },
        {
            "kind": "wait",
            "target": "robot",
            "method": "wait",
            "value": None,
            "block": True,
            "description": "阶段结束统一等待",
            "source_stage": "right_hand_move_to_coffee_machine",
            "source_action_id": "027",
        },
        {
            "kind": "sleep",
            "target": "robot",
            "method": "sleep",
            "value": 0.5,
            "block": True,
            "description": "阶段结束后观察停稳",
            "source_stage": "right_hand_move_to_coffee_machine",
            "source_action_id": "028",
        },
    ],
    "replay_right_retreat_after_coffee": [
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "home",
            "value": None,
            "block": None,
            "description": "右臂从咖啡机位置回撤到 home",
            "source_stage": "right_hand_retreat_after_coffee",
            "source_action_id": "037",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_shoulder_roll",
            "value": 0.6,
            "block": False,
            "description": "右肩翻滚切到后续接奶中间姿态",
            "source_stage": "right_hand_retreat_after_coffee",
            "source_action_id": "038",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_wrist_pitch",
            "value": -0.3,
            "block": True,
            "description": "右腕俯仰切到后续接奶中间姿态",
            "source_stage": "right_hand_retreat_after_coffee",
            "source_action_id": "039",
        },
        {
            "kind": "wait",
            "target": "robot",
            "method": "wait",
            "value": None,
            "block": True,
            "description": "阶段结束统一等待",
            "source_stage": "right_hand_retreat_after_coffee",
            "source_action_id": "040",
        },
        {
            "kind": "sleep",
            "target": "robot",
            "method": "sleep",
            "value": 0.5,
            "block": True,
            "description": "阶段结束后观察停稳",
            "source_stage": "right_hand_retreat_after_coffee",
            "source_action_id": "041",
        },
    ],
    "replay_right_pour_ready": [
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_wrist_yaw",
            "value": -0.7,
            "block": False,
            "description": "右腕偏航对准接奶方向",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "061",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_wrist_pitch",
            "value": -0.5,
            "block": False,
            "description": "右腕俯仰切到接奶角度",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "062",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_wrist_roll",
            "value": 0.3,
            "block": False,
            "description": "右腕翻滚调整杯口姿态",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "063",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_shoulder_roll",
            "value": 0.7,
            "block": False,
            "description": "右肩翻滚切到接奶位",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "064",
        },
        {
            "kind": "sleep",
            "target": "robot",
            "method": "sleep",
            "value": 1.0,
            "block": True,
            "description": "保留原始倒奶前等待",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "065",
        },
        {
            "kind": "wait",
            "target": "robot",
            "method": "wait",
            "value": None,
            "block": True,
            "description": "阶段结束统一等待",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "066",
        },
        {
            "kind": "sleep",
            "target": "robot",
            "method": "sleep",
            "value": 0.5,
            "block": True,
            "description": "阶段结束后观察停稳",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "067",
        },
    ],
}
RIGHT_REPLAY_STAGE_ALIASES = {
    "right_pour_ready": "replay_right_pour_ready",
    "right_cup_pose": "replay_right_pour_ready",
}
DEPRECATED_RIGHT_STAGE_COMMANDS = {
    "right_table_pregrasp",
    "right_table_grasp_pose",
    "right_lift_cup",
    "right_transfer_cup",
    "right_transfer_cup_b",
}

OBSERVATION_CHOICES = {
    "spout_in_cup",
    "edge",
    "outside",
    "near_collision",
    "unsafe",
    "uncertain",
}
RISK_OBSERVATIONS = {"unsafe", "near_collision"}

CSV_FIELDNAMES = [
    "timestamp",
    "command",
    "command_type",
    "arm",
    "dx",
    "dy",
    "dz",
    "joint_targets",
    "wrist_roll_target",
    "wrist_yaw_delta_or_target",
    "wrist_pitch_delta_or_target",
    "gripper_position",
    "dry_run",
    "execute",
    "user_confirmed",
    "status",
    "observed_alignment",
    "user_observation",
    "risk_detected",
]


@dataclass
class SessionState:
    log_path: Path
    dry_run: bool
    execute: bool
    current_wrist_roll: Optional[float] = None
    current_wrist_yaw: Optional[float] = None
    current_wrist_pitch: Optional[float] = None
    current_left_gripper_position: Optional[float] = None
    current_right_gripper_position: Optional[float] = None
    last_observed_alignment: str = ""
    last_user_observation: str = ""
    risk_detected: bool = False


@dataclass(frozen=True)
class Action:
    command: str
    command_type: str
    arm: str = ""
    joint_targets: str = ""
    dx: Optional[float] = None
    dy: Optional[float] = None
    dz: Optional[float] = None
    wrist_roll_target: Optional[float] = None
    wrist_yaw_delta_or_target: Optional[float] = None
    wrist_pitch_delta_or_target: Optional[float] = None
    gripper_position: Optional[float] = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="空壶壶嘴对杯口标定控制台，仅做小步人工确认动作。"
    )
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印动作并写 CSV，不连接机器人。默认模式。",
    )
    mode_group.add_argument(
        "--execute",
        action="store_true",
        help="连接真实机器人并执行动作，必须同时传入风险确认参数。",
    )
    parser.add_argument(
        "--i-understand-real-robot-risk",
        action="store_true",
        help="确认已知晓实机风险；缺少该参数时禁止 execute。",
    )
    parser.add_argument(
        "--linear-step",
        type=float,
        default=DEFAULT_LINEAR_STEP,
        help=f"X/Z relative IK 步长，默认 {DEFAULT_LINEAR_STEP}",
    )
    parser.add_argument(
        "--linear-step-small",
        type=float,
        default=DEFAULT_LINEAR_STEP_SMALL,
        help=f"Y relative IK 小步长，默认 {DEFAULT_LINEAR_STEP_SMALL}",
    )
    parser.add_argument(
        "--rot-step",
        type=float,
        default=DEFAULT_ROT_STEP,
        help=f"wrist yaw/pitch 小步角度，默认 {DEFAULT_ROT_STEP}",
    )
    parser.add_argument(
        "--gripper-position",
        type=float,
        default=DEFAULT_GRIPPER_POSITION,
        help=(
            "兼容旧参数：左夹爪夹空奶壶目标位置，默认 "
            f"{DEFAULT_GRIPPER_POSITION}；建议改用 --left-gripper-pitcher-position"
        ),
    )
    parser.add_argument(
        "--left-gripper-open-position",
        type=float,
        default=DEFAULT_LEFT_GRIPPER_OPEN_POSITION,
        help=f"左夹爪打开目标位置，默认 {DEFAULT_LEFT_GRIPPER_OPEN_POSITION}",
    )
    parser.add_argument(
        "--left-gripper-closed-position",
        type=float,
        default=DEFAULT_LEFT_GRIPPER_CLOSED_POSITION,
        help=f"左夹爪闭合初始化目标位置，默认 {DEFAULT_LEFT_GRIPPER_CLOSED_POSITION}",
    )
    parser.add_argument(
        "--left-gripper-pitcher-position",
        type=float,
        default=DEFAULT_LEFT_GRIPPER_PITCHER_POSITION,
        help=f"左夹爪夹空奶壶目标位置，默认 {DEFAULT_LEFT_GRIPPER_PITCHER_POSITION}",
    )
    parser.add_argument(
        "--left-gripper-step",
        type=float,
        default=DEFAULT_LEFT_GRIPPER_STEP,
        help=f"左夹爪微调步长，默认 {DEFAULT_LEFT_GRIPPER_STEP}",
    )
    parser.add_argument(
        "--right-gripper-open-position",
        type=float,
        default=DEFAULT_RIGHT_GRIPPER_OPEN_POSITION,
        help=f"右夹爪打开目标位置，默认 {DEFAULT_RIGHT_GRIPPER_OPEN_POSITION}",
    )
    parser.add_argument(
        "--right-gripper-closed-position",
        type=float,
        default=DEFAULT_RIGHT_GRIPPER_CLOSED_POSITION,
        help=f"右夹爪闭合初始化目标位置，默认 {DEFAULT_RIGHT_GRIPPER_CLOSED_POSITION}",
    )
    parser.add_argument(
        "--right-gripper-cup-position",
        type=float,
        default=DEFAULT_RIGHT_GRIPPER_CUP_POSITION,
        help=f"右夹爪夹杯目标位置，默认 {DEFAULT_RIGHT_GRIPPER_CUP_POSITION}",
    )
    parser.add_argument(
        "--right-gripper-step",
        type=float,
        default=DEFAULT_RIGHT_GRIPPER_STEP,
        help=f"右夹爪微调步长，默认 {DEFAULT_RIGHT_GRIPPER_STEP}",
    )
    parser.add_argument(
        "--max-wrist-roll",
        type=float,
        default=DEFAULT_MAX_WRIST_ROLL,
        help=f"允许的最大 wrist_roll 目标，默认 {DEFAULT_MAX_WRIST_ROLL}",
    )
    parser.add_argument(
        "--log-file",
        default=None,
        help="CSV 日志路径；默认写入 pour_alignment_calib/logs/。",
    )
    parser.add_argument(
        "--include-left-gripper-init",
        action="store_true",
        help="replay_right_grasp_cup 中包含左夹爪闭合初始化；默认不执行左夹爪初始化。",
    )
    add_connection_args(parser, default_profile="interactive")
    parser.set_defaults(robot_version=DEFAULT_ROBOT_VERSION)
    args = parser.parse_args()
    argv = sys.argv[1:]
    old_gripper_arg_used = flag_present(argv, "--gripper-position")
    left_pitcher_arg_used = flag_present(argv, "--left-gripper-pitcher-position")
    if old_gripper_arg_used and not left_pitcher_arg_used:
        args.left_gripper_pitcher_position = args.gripper_position
    return args


def flag_present(argv: list[str], flag: str) -> bool:
    return any(item == flag or item.startswith(f"{flag}=") for item in argv)


def validate_args(args: argparse.Namespace) -> None:
    if not args.execute:
        args.dry_run = True
    if args.execute and not args.i_understand_real_robot_risk:
        raise SystemExit(
            "拒绝执行：execute 模式必须同时传入 "
            "--execute --i-understand-real-robot-risk。"
        )
    if args.linear_step <= 0.0:
        raise SystemExit("--linear-step 必须大于 0")
    if args.linear_step_small <= 0.0:
        raise SystemExit("--linear-step-small 必须大于 0")
    if args.rot_step <= 0.0:
        raise SystemExit("--rot-step 必须大于 0")
    if not 0.0 <= args.gripper_position <= 1.0:
        raise SystemExit("--gripper-position 必须在 0.0 到 1.0 之间")
    if not GRIPPER_MIN <= args.left_gripper_open_position <= GRIPPER_MAX:
        raise SystemExit("--left-gripper-open-position 必须在 0.0 到 1.0 之间")
    if not GRIPPER_MIN <= args.left_gripper_closed_position <= GRIPPER_MAX:
        raise SystemExit("--left-gripper-closed-position 必须在 0.0 到 1.0 之间")
    if not GRIPPER_MIN <= args.left_gripper_pitcher_position <= GRIPPER_MAX:
        raise SystemExit("--left-gripper-pitcher-position 必须在 0.0 到 1.0 之间")
    if args.left_gripper_step <= 0.0:
        raise SystemExit("--left-gripper-step 必须大于 0")
    if not GRIPPER_MIN <= args.right_gripper_open_position <= GRIPPER_MAX:
        raise SystemExit("--right-gripper-open-position 必须在 0.0 到 1.0 之间")
    if not GRIPPER_MIN <= args.right_gripper_closed_position <= GRIPPER_MAX:
        raise SystemExit("--right-gripper-closed-position 必须在 0.0 到 1.0 之间")
    if not GRIPPER_MIN <= args.right_gripper_cup_position <= GRIPPER_MAX:
        raise SystemExit("--right-gripper-cup-position 必须在 0.0 到 1.0 之间")
    if args.right_gripper_step <= 0.0:
        raise SystemExit("--right-gripper-step 必须大于 0")
    if args.max_wrist_roll < 0.0:
        raise SystemExit("--max-wrist-roll 必须大于等于 0")


def resolve_log_path(log_file: Optional[str]) -> Path:
    if log_file:
        path = Path(log_file).expanduser()
        if not path.is_absolute():
            path = PROJECT_ROOT / path
        return path
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return LOG_DIR / f"pour_align_trials_{timestamp}.csv"


def init_csv_log(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    if log_path.exists():
        return
    with log_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def fmt_bool(value: bool) -> str:
    return "true" if value else "false"


def fmt_float(value: Optional[float]) -> str:
    if value is None:
        return ""
    return f"{value:.6f}"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def append_log(
    state: SessionState,
    action: Action,
    *,
    user_confirmed: Optional[bool],
    status: str,
    observed_alignment: str = "",
    user_observation: str = "",
) -> None:
    row = {
        "timestamp": now_iso(),
        "command": action.command,
        "command_type": action.command_type,
        "arm": action.arm,
        "dx": fmt_float(action.dx),
        "dy": fmt_float(action.dy),
        "dz": fmt_float(action.dz),
        "joint_targets": action.joint_targets,
        "wrist_roll_target": fmt_float(action.wrist_roll_target),
        "wrist_yaw_delta_or_target": fmt_float(action.wrist_yaw_delta_or_target),
        "wrist_pitch_delta_or_target": fmt_float(action.wrist_pitch_delta_or_target),
        "gripper_position": fmt_float(action.gripper_position),
        "dry_run": fmt_bool(state.dry_run),
        "execute": fmt_bool(state.execute),
        "user_confirmed": "" if user_confirmed is None else fmt_bool(user_confirmed),
        "status": status,
        "observed_alignment": observed_alignment,
        "user_observation": user_observation,
        "risk_detected": fmt_bool(state.risk_detected),
    }
    with state.log_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDNAMES)
        writer.writerow(row)


def pretty_path(path: Path) -> str:
    try:
        return str(path.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def print_help() -> None:
    print(
        """
Commands:
  help                 show commands
  left_open            set left_gripper to configured open position
  left_grip            set left_gripper to configured pitcher position, default 0.70
  left_loose           loosen left_gripper by configured step
  left_tight           tighten left_gripper by configured step
  grip                 alias of left_grip
  right_open           set right_gripper to configured open position
  right_grip           set right_gripper to configured cup position, default 0.80
  right_loose          loosen right_gripper by configured step
  right_tight          tighten right_gripper by configured step
  replay_right_grasp_cup
                       replay coffee_replay_safe right_hand_grasp_cup stage for right cup grasp
  replay_right_move_to_coffee_machine
                       replay coffee_replay_safe right_hand_move_to_coffee_machine stage
  replay_right_retreat_after_coffee
                       replay coffee_replay_safe right_hand_retreat_after_coffee stage, contains right_arm.home()
  replay_right_pour_ready
                       replay right-hand actions inside left_hand_move_to_pour_pose
  right_pour_ready / right_cup_pose
                       alias of replay_right_pour_ready
  right_table_pregrasp / right_table_grasp_pose / right_lift_cup / right_transfer_cup
                       deprecated; no action, use replay_right_* commands
  x+ / x-              left_arm relative IK X +/- step
  y+ / y-              left_arm relative IK Y +/- small step
  z+ / z-              left_arm relative IK Z +/- step
  roll0                set left wrist_roll target to 0
  roll03               set left wrist_roll target to 0.3
  roll05               set left wrist_roll target to 0.5
  roll07               set left wrist_roll target to 0.7
  yaw+ / yaw-          small left wrist_yaw target step when SDK supports it
  pitch+ / pitch-      small left wrist_pitch target step when SDK supports it
  obs [value]          record observation: spout_in_cup / edge / outside / near_collision / unsafe / uncertain
  save [note]          save current calibration note
  quit                 exit
""".strip()
    )


def print_startup_banner(args: argparse.Namespace, state: SessionState) -> None:
    mode = "EXECUTE" if args.execute else "DRY-RUN"
    print("=" * 72)
    print(f"[Pour Align Calib] {mode}")
    print(f"log: {pretty_path(state.log_path)}")
    print("空壶标定：先分阶段确认右手取杯到倒奶前姿态，再用左手 relative IK 小步对位。")
    if args.execute:
        print("实机模式：每个真实动作都需要输入 y 二次确认。")
    else:
        print("dry-run：不会连接机器人，只打印动作计划并写日志。")
    print("输入 help 查看命令，quit 退出。")


def connect_robot(args: argparse.Namespace):
    if getattr(args, "print_connection_config", False):
        connect_robot_with_selector(args, script_name=__file__)
        return None
    if args.execute:
        return connect_robot_with_selector(args, script_name=__file__)
    return None


def safe_shutdown(robot) -> None:
    if robot is None:
        return
    try:
        robot.stop()
        print("finally: 已尝试调用 robot.stop()")
    except Exception as exc:  # pragma: no cover - real SDK/runtime only
        print(f"finally: robot.stop() 失败: {exc}")
    try:
        robot.disconnect()
        print("finally: 已尝试调用 robot.disconnect()")
    except Exception as exc:  # pragma: no cover - real SDK/runtime only
        print(f"finally: robot.disconnect() 失败: {exc}")


def replay_stage_name(command: str) -> str:
    return RIGHT_REPLAY_STAGE_ALIASES.get(command, command)


def replay_stage_steps(command: str) -> list[dict]:
    return RIGHT_REPLAY_STAGES[replay_stage_name(command)]


def step_enabled(step: dict, args: argparse.Namespace) -> bool:
    enabled_arg = step.get("enabled_arg")
    if enabled_arg is None:
        return True
    return bool(getattr(args, enabled_arg))


def resolve_step_value(step: dict, args: argparse.Namespace) -> Optional[float]:
    if "value_arg" in step:
        return getattr(args, step["value_arg"])
    return step.get("value")


def replay_step_payload(command: str, args: argparse.Namespace) -> str:
    payload = []
    for step in replay_stage_steps(command):
        value = resolve_step_value(step, args)
        payload.append(
            {
                "source_stage": step["source_stage"],
                "source_action_id": step["source_action_id"],
                "kind": step["kind"],
                "target": step["target"],
                "method": step["method"],
                "value": value,
                "block": step.get("block"),
                "description": step["description"],
                "enabled": step_enabled(step, args),
                "source_sdk_call": step.get("source_sdk_call", ""),
            }
        )
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def replay_stage_arm_label(command: str, args: argparse.Namespace) -> str:
    steps = replay_stage_steps(command)
    targets = {step["target"] for step in steps if step_enabled(step, args)}
    if "left_gripper" in targets:
        return "right/left_gripper_init"
    return "right"


def build_replay_stage_action(command: str, args: argparse.Namespace) -> Action:
    return Action(
        command=command,
        command_type="replay_stage",
        arm=replay_stage_arm_label(command, args),
        joint_targets=replay_step_payload(command, args),
    )


def format_replay_step(step: dict, args: argparse.Namespace) -> str:
    action_id = step["source_action_id"]
    prefix = f"{action_id} {step['description']}: "
    if not step_enabled(step, args):
        return prefix + "[skipped unless --include-left-gripper-init]"
    kind = step["kind"]
    target = step["target"]
    method = step["method"]
    value = resolve_step_value(step, args)
    block = step.get("block")
    if kind == "sleep":
        return prefix + f"time.sleep({value:.1f})"
    if kind == "wait":
        return prefix + "robot.wait()"
    if method == "home":
        return prefix + f"robot.{target}.home()"
    if block is None:
        return prefix + f"robot.{target}.{method}({value:.6f})"
    return prefix + f"robot.{target}.{method}({value:.6f}, block={block})"


def replay_stage_lines(command: str, args: argparse.Namespace) -> list[str]:
    calls = []
    stage_name = replay_stage_name(command)
    if command != stage_name:
        calls.append(f"[alias] {command} -> {stage_name}")
    for step in replay_stage_steps(command):
        calls.append(format_replay_step(step, args))
    return calls


def describe_action(action: Action, args: Optional[argparse.Namespace] = None) -> str:
    if action.command_type == "replay_stage":
        if args is None:
            return action.command
        return "\n  ".join(replay_stage_lines(action.command, args))
    if action.command_type == "relative_ik":
        return (
            "robot.left_arm.ik("
            f"{action.dx or 0.0:.6f}, {action.dy or 0.0:.6f}, {action.dz or 0.0:.6f}, "
            "0, 0, 0, block=True, abs=False)"
        )
    if action.command_type == "wrist_roll":
        return f"robot.left_arm.set_wrist_roll({action.wrist_roll_target:.6f}, block=True)"
    if action.command_type == "wrist_yaw":
        return f"robot.left_arm.set_wrist_yaw({action.wrist_yaw_delta_or_target:.6f}, block=True)"
    if action.command_type == "wrist_pitch":
        return f"robot.left_arm.set_wrist_pitch({action.wrist_pitch_delta_or_target:.6f}, block=True)"
    if action.command_type == "gripper":
        if action.arm == "left":
            gripper_name = "left_gripper"
        elif action.arm == "right":
            gripper_name = "right_gripper"
        else:
            return f"unsupported gripper arm: {action.arm}"
        return f"robot.{gripper_name}.set_position({action.gripper_position:.6f}, block=True)"
    return action.command


def confirm_real_action(action: Action) -> bool:
    print(f"[confirm] {describe_action(action)}")
    user_input = input("输入 y 执行该真实动作，其他任意输入跳过：").strip().lower()
    return user_input == "y"


def confirm_replay_stage(action: Action, args: argparse.Namespace) -> bool:
    if action.command == "replay_right_retreat_after_coffee":
        print("[warning] replay_right_retreat_after_coffee 包含 robot.right_arm.home()，请重点确认路径安全。")
    print(f"[confirm] {describe_action(action, args)}")
    user_input = input("输入 y 执行该 replay stage，其他任意输入跳过：").strip().lower()
    return user_input == "y"


def execute_action(robot, action: Action, args: argparse.Namespace, state: SessionState) -> None:
    print(f"[plan] {describe_action(action)}")

    if state.dry_run:
        append_log(state, action, user_confirmed=None, status="dry_run")
        print("[dry-run] 已记录，不连接机器人。")
        update_tracked_state(action, state)
        return

    confirmed = confirm_real_action(action)
    if not confirmed:
        append_log(state, action, user_confirmed=False, status="skipped_by_user")
        print("[skip] 用户未确认，动作已跳过。")
        return

    status = "ok"
    try:
        start_time = time.perf_counter()
        if action.command_type == "relative_ik":
            robot.left_arm.ik(
                action.dx or 0.0,
                action.dy or 0.0,
                action.dz or 0.0,
                0,
                0,
                0,
                block=True,
                abs=False,
            )
        elif action.command_type == "wrist_roll":
            robot.left_arm.set_wrist_roll(action.wrist_roll_target, block=True)
        elif action.command_type == "wrist_yaw":
            robot.left_arm.set_wrist_yaw(action.wrist_yaw_delta_or_target, block=True)
        elif action.command_type == "wrist_pitch":
            robot.left_arm.set_wrist_pitch(action.wrist_pitch_delta_or_target, block=True)
        elif action.command_type == "gripper":
            if action.arm == "left":
                robot.left_gripper.set_position(action.gripper_position, block=True)
            elif action.arm == "right":
                robot.right_gripper.set_position(action.gripper_position, block=True)
            else:
                status = "unsupported"
        else:
            status = "unsupported"
        duration_s = time.perf_counter() - start_time
        if status == "ok":
            print(f"[ok] 动作完成，用时 {duration_s:.3f}s。")
            update_tracked_state(action, state)
    except Exception as exc:  # pragma: no cover - real SDK/runtime only
        status = f"error: {exc}"
        print(f"[error] {exc}")

    append_log(state, action, user_confirmed=True, status=status)


def execute_replay_stage(
    command: str,
    robot,
    args: argparse.Namespace,
    state: SessionState,
) -> None:
    action = build_replay_stage_action(command, args)
    stage_name = replay_stage_name(command)
    print(f"[plan] {command} will run:")
    print(f"  {describe_action(action, args)}")

    if state.dry_run:
        append_log(state, action, user_confirmed=None, status="dry_run")
        print("[dry-run] 已记录，不连接机器人，不执行 replay stage。")
        return

    confirmed = confirm_replay_stage(action, args)
    if not confirmed:
        append_log(state, action, user_confirmed=False, status="skipped_by_user")
        print(f"[skip] 用户未确认，{command} 已跳过。")
        return

    status = "ok"
    user_observation = ""
    try:
        start_time = time.perf_counter()
        for step in replay_stage_steps(command):
            if not step_enabled(step, args):
                continue
            execute_replay_step(robot, step, args)
        wait_status = "ok"
        duration_s = time.perf_counter() - start_time
        if wait_status == "ok":
            print(f"[ok] {command} 已完成，用时 {duration_s:.3f}s。")
        else:
            status = wait_status
            user_observation = "wait_unsupported"
            print(
                f"[warning] {command} 关节命令已发送，但 SDK 不支持已知等待接口；"
                "CSV 已记录 wait_unsupported。"
            )
    except Exception as exc:  # pragma: no cover - real SDK/runtime only
        status = f"error: {exc}"
        print(f"[error] {exc}")

    append_log(
        state,
        action,
        user_confirmed=True,
        status=status,
        user_observation=user_observation,
    )


def execute_replay_step(robot, step: dict, args: argparse.Namespace) -> None:
    kind = step["kind"]
    target_name = step["target"]
    method_name = step["method"]
    value = resolve_step_value(step, args)
    block = step.get("block")

    if kind == "sleep":
        time.sleep(value)
        return
    if kind == "wait":
        robot.wait()
        return

    target = getattr(robot, target_name)
    method = getattr(target, method_name)
    if method_name == "home":
        method()
    elif block is None:
        method(value)
    else:
        method(value, block=block)


def wait_for_right_arm(robot) -> str:
    wait_method = getattr(robot.right_arm, "wait", None)
    if callable(wait_method):
        try:
            wait_method()
            return "ok"
        except (AttributeError, TypeError):
            pass

    robot_wait = getattr(robot, "wait", None)
    joint_names = getattr(robot.right_arm, "joint_names", None)
    if callable(robot_wait) and joint_names is not None:
        try:
            robot_wait(joint_names)
            return "ok"
        except (AttributeError, TypeError):
            pass

    return "wait_unsupported"


def update_tracked_state(action: Action, state: SessionState) -> None:
    if action.command_type == "wrist_roll":
        state.current_wrist_roll = action.wrist_roll_target
    elif action.command_type == "wrist_yaw":
        state.current_wrist_yaw = action.wrist_yaw_delta_or_target
    elif action.command_type == "wrist_pitch":
        state.current_wrist_pitch = action.wrist_pitch_delta_or_target
    elif action.command_type == "gripper":
        if action.arm == "left":
            state.current_left_gripper_position = action.gripper_position
        elif action.arm == "right":
            state.current_right_gripper_position = action.gripper_position


def build_linear_action(command: str, args: argparse.Namespace) -> Action:
    dx = dy = dz = 0.0
    if command == "x+":
        dx = args.linear_step
    elif command == "x-":
        dx = -args.linear_step
    elif command == "y+":
        dy = args.linear_step_small
    elif command == "y-":
        dy = -args.linear_step_small
    elif command == "z+":
        dz = args.linear_step
    elif command == "z-":
        dz = -args.linear_step
    else:
        raise ValueError(f"unsupported linear command: {command}")
    return Action(command=command, command_type="relative_ik", dx=dx, dy=dy, dz=dz)


def build_roll_action(command: str, args: argparse.Namespace) -> Action:
    targets = {
        "roll0": 0.0,
        "roll03": 0.3,
        "roll05": 0.5,
        "roll07": 0.7,
    }
    target = targets[command]
    if target > args.max_wrist_roll:
        raise ValueError(
            f"{command} target {target:.3f} exceeds --max-wrist-roll {args.max_wrist_roll:.3f}"
        )
    return Action(command=command, command_type="wrist_roll", wrist_roll_target=target)


def build_right_gripper_action(
    command: str,
    args: argparse.Namespace,
    state: SessionState,
) -> Action:
    if command == "right_open":
        target = args.right_gripper_open_position
    elif command == "right_grip":
        target = args.right_gripper_cup_position
    else:
        current = state.current_right_gripper_position
        if current is None:
            current = args.right_gripper_cup_position
            print(
                "[info] current_right_gripper_position 未知，"
                f"以 right_gripper_cup_position={current:.3f} 作为微调基准。"
            )
        sign = 1.0 if command == "right_loose" else -1.0
        target = clamp(
            current + sign * args.right_gripper_step,
            GRIPPER_MIN,
            GRIPPER_MAX,
        )

    return Action(
        command=command,
        command_type="gripper",
        arm="right",
        gripper_position=target,
    )


def build_left_gripper_action(
    command: str,
    args: argparse.Namespace,
    state: SessionState,
) -> Action:
    if command == "left_open":
        target = args.left_gripper_open_position
    elif command in {"left_grip", "grip"}:
        target = args.left_gripper_pitcher_position
    else:
        current = state.current_left_gripper_position
        if current is None:
            current = args.left_gripper_pitcher_position
            print(
                "[info] current_left_gripper_position 未知，"
                f"以 left_gripper_pitcher_position={current:.3f} 作为微调基准。"
            )
        sign = 1.0 if command == "left_loose" else -1.0
        target = clamp(
            current + sign * args.left_gripper_step,
            GRIPPER_MIN,
            GRIPPER_MAX,
        )

    return Action(
        command=command,
        command_type="gripper",
        arm="left",
        gripper_position=target,
    )


def get_current_joint_target(arm, state: SessionState, joint_name: str) -> Optional[float]:
    if joint_name == "wrist_yaw" and state.current_wrist_yaw is not None:
        return state.current_wrist_yaw
    if joint_name == "wrist_pitch" and state.current_wrist_pitch is not None:
        return state.current_wrist_pitch
    if arm is None:
        return 0.0
    try:
        positions = arm.positions
    except Exception:
        return None
    index = 6 if joint_name == "wrist_yaw" else 5
    if len(positions) <= index:
        return None
    return float(positions[index])


def build_wrist_step_action(command: str, robot, args: argparse.Namespace, state: SessionState) -> Action:
    is_yaw = command.startswith("yaw")
    joint_name = "wrist_yaw" if is_yaw else "wrist_pitch"
    method_name = "set_wrist_yaw" if is_yaw else "set_wrist_pitch"
    command_type = "wrist_yaw" if is_yaw else "wrist_pitch"
    value_field = "wrist_yaw_delta_or_target" if is_yaw else "wrist_pitch_delta_or_target"
    arm = None if robot is None else robot.left_arm

    if arm is not None and not hasattr(arm, method_name):
        kwargs = {value_field: None}
        return Action(command=command, command_type=command_type, **kwargs)

    current = get_current_joint_target(arm, state, joint_name)
    if current is None:
        kwargs = {value_field: None}
        return Action(command=command, command_type=command_type, **kwargs)

    sign = 1.0 if command.endswith("+") else -1.0
    target = current + sign * args.rot_step
    kwargs = {value_field: target}
    return Action(command=command, command_type=command_type, **kwargs)


def record_unsupported(command: str, command_type: str, state: SessionState) -> None:
    print(f"[unsupported] {command}: 没有可靠的 SDK 控制接口或当前目标不可读，未执行。")
    append_log(
        state,
        Action(command=command, command_type=command_type),
        user_confirmed=None,
        status="unsupported",
    )


def handle_deprecated_right_stage(command: str, state: SessionState) -> None:
    print(
        f"[deprecated] {command} 已禁用：上一版动作不是 coffee_replay_safe.py 的严格复现。"
    )
    print(
        "请改用 replay_right_grasp_cup / replay_right_move_to_coffee_machine / "
        "replay_right_retreat_after_coffee / replay_right_pour_ready。"
    )
    append_log(
        state,
        Action(command=command, command_type="deprecated_right_stage", arm="right"),
        user_confirmed=None,
        status="deprecated_no_action",
    )


def handle_observation(parts: list[str], state: SessionState) -> None:
    if len(parts) > 1:
        observation = " ".join(parts[1:]).strip()
    else:
        print("可选观察值: " + " / ".join(sorted(OBSERVATION_CHOICES)))
        observation = input("obs> ").strip()

    normalized = observation.strip().lower()
    observed_alignment = normalized if normalized in OBSERVATION_CHOICES else "uncertain"
    if normalized in RISK_OBSERVATIONS:
        state.risk_detected = True
        print("[risk] 已记录 risk_detected=True。建议停止本轮调试，检查杯口/壶嘴/周边间隙。")

    state.last_observed_alignment = observed_alignment
    state.last_user_observation = observation
    append_log(
        state,
        Action(command="obs", command_type="observation"),
        user_confirmed=None,
        status="recorded",
        observed_alignment=observed_alignment,
        user_observation=observation,
    )
    print(f"[obs] recorded: {observed_alignment}")


def handle_save(parts: list[str], state: SessionState) -> None:
    if len(parts) > 1:
        note = " ".join(parts[1:]).strip()
    else:
        note = input("save note> ").strip()
    append_log(
        state,
        Action(command="save", command_type="save"),
        user_confirmed=None,
        status="saved",
        observed_alignment=state.last_observed_alignment,
        user_observation=note,
    )
    print("[save] 当前标定备注已写入 CSV。")


def process_command(line: str, robot, args: argparse.Namespace, state: SessionState) -> bool:
    try:
        parts = shlex.split(line)
    except ValueError as exc:
        print(f"命令解析失败: {exc}")
        return True
    if not parts:
        return True

    command = parts[0].lower()
    if command == "help":
        print_help()
        return True
    if command in {"quit", "exit", "q"}:
        return False
    if command == "obs":
        handle_observation(parts, state)
        return True
    if command == "save":
        handle_save(parts, state)
        return True
    if command == "grip":
        print("[alias] grip 等价于 left_grip")
        execute_action(robot, build_left_gripper_action(command, args, state), args, state)
        return True
    if command in {"left_open", "left_grip", "left_loose", "left_tight"}:
        execute_action(robot, build_left_gripper_action(command, args, state), args, state)
        return True
    if command in {"right_open", "right_grip", "right_loose", "right_tight"}:
        execute_action(robot, build_right_gripper_action(command, args, state), args, state)
        return True
    if command in RIGHT_REPLAY_STAGES or command in RIGHT_REPLAY_STAGE_ALIASES:
        execute_replay_stage(command, robot, args, state)
        return True
    if command in DEPRECATED_RIGHT_STAGE_COMMANDS:
        handle_deprecated_right_stage(command, state)
        return True
    if command in {"x+", "x-", "y+", "y-", "z+", "z-"}:
        execute_action(robot, build_linear_action(command, args), args, state)
        return True
    if command in {"roll0", "roll03", "roll05", "roll07"}:
        try:
            action = build_roll_action(command, args)
        except ValueError as exc:
            print(f"[blocked] {exc}")
            append_log(
                state,
                Action(command=command, command_type="wrist_roll"),
                user_confirmed=None,
                status=f"blocked: {exc}",
            )
            return True
        execute_action(robot, action, args, state)
        return True
    if command in {"yaw+", "yaw-", "pitch+", "pitch-"}:
        command_type = "wrist_yaw" if command.startswith("yaw") else "wrist_pitch"
        action = build_wrist_step_action(command, robot, args, state)
        if (
            command_type == "wrist_yaw"
            and action.wrist_yaw_delta_or_target is None
        ) or (
            command_type == "wrist_pitch"
            and action.wrist_pitch_delta_or_target is None
        ):
            record_unsupported(command, command_type, state)
            return True
        execute_action(robot, action, args, state)
        return True

    print(f"未知命令: {command}；输入 help 查看命令。")
    append_log(
        state,
        Action(command=command, command_type="unknown"),
        user_confirmed=None,
        status="unknown_command",
    )
    return True


def repl_loop(robot, args: argparse.Namespace, state: SessionState) -> None:
    print_help()
    while True:
        try:
            raw_line = input("pour-align> ")
        except EOFError:
            print()
            break
        line = raw_line.strip()
        if not line:
            continue
        if not process_command(line, robot, args, state):
            break


def main() -> int:
    args = parse_args()
    validate_args(args)

    log_path = resolve_log_path(args.log_file)
    state = SessionState(
        log_path=log_path,
        dry_run=bool(args.dry_run),
        execute=bool(args.execute),
    )
    init_csv_log(state.log_path)
    print_startup_banner(args, state)

    robot = None
    try:
        robot = connect_robot(args)
        if getattr(args, "print_connection_config", False):
            return 0
        repl_loop(robot, args, state)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: 正在安全收尾。")
        append_log(
            state,
            Action(command="keyboard_interrupt", command_type="shutdown"),
            user_confirmed=None,
            status="keyboard_interrupt",
        )
    finally:
        safe_shutdown(robot)

    return 0


if __name__ == "__main__":
    sys.exit(main())
