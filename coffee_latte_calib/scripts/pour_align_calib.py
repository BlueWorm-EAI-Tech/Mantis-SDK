"""
Empty pitcher spout-to-cup alignment calibration console.

This script is intentionally independent from the coffee replay pipeline.
It only supports one confirmed operator action at a time.
"""

from __future__ import annotations

import argparse
import csv
import difflib
import json
import re
import shlex
import subprocess
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
DEFAULT_RIGHT_WRIST_STEP = 0.05
DEFAULT_LEFT_WRIST_STEP = 0.05
DEFAULT_RIGHT_ARM_CLEARANCE_STEP = 0.05
DEFAULT_RIGHT_LINEAR_STEP = 0.005
DEFAULT_RIGHT_LINEAR_STEP_SMALL = 0.003
DEFAULT_MAX_REPEAT_COUNT = 20
DEFAULT_RIGHT_POUR_READY_WRIST_YAW = -0.70
DEFAULT_RIGHT_POUR_READY_WRIST_PITCH = 0.10
DEFAULT_RIGHT_POUR_READY_WRIST_ROLL = 0.20
DEFAULT_RIGHT_POUR_READY_ELBOW_PITCH = 0.25
DEFAULT_RIGHT_POUR_READY_SHOULDER_ROLL = 0.70
DEFAULT_LEFT_POUR_READY_SHOULDER_PITCH = -0.35
DEFAULT_LEFT_POUR_READY_ELBOW_PITCH = -0.42
DEFAULT_LEFT_POUR_READY_SHOULDER_ROLL = 0.50
DEFAULT_LEFT_POUR_READY_WRIST_PITCH = -0.45
DEFAULT_LEFT_POUR_WRIST_ROLL_PREP = 1.05
DEFAULT_LEFT_ARM_POUR_ADJUST_STEP = 0.05
DEFAULT_MAX_WRIST_ROLL = 0.70
GRIPPER_MIN = 0.0
GRIPPER_MAX = 1.0
LOG_DIR = Path(__file__).resolve().parent / "logs"
CANDIDATE_FILE = LOG_DIR / "pour_align_candidates.jsonl"
CANDIDATE_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")

# Source: coffee_replay_safe.py right-hand replay stages only.
# Do not import or call coffee_replay_safe.py; these are explicit SDK calls
# copied from the named source stages for calibration use.  Joint order,
# target values, block flags, sleep calls, and robot.wait() are kept aligned
# with the source stages unless a step below explicitly documents a calibration
# override.
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
            "description": (
                "右夹爪收至抓杯位置；source_sdk_call 是原始 0.6，"
                "实际执行使用 --right-gripper-cup-position，标定默认 0.80"
            ),
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
            "value_arg": "right_pour_ready_wrist_yaw",
            "block": False,
            "description": "右腕偏航对准接奶方向",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "061",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_wrist_pitch",
            "value_arg": "right_pour_ready_wrist_pitch",
            "block": False,
            "description": "右腕俯仰切到接奶角度",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "062",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_wrist_roll",
            "value_arg": "right_pour_ready_wrist_roll",
            "block": False,
            "description": "右腕翻滚调整杯口姿态",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "063",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_shoulder_roll",
            "value_arg": "right_pour_ready_shoulder_roll",
            "block": False,
            "description": "右肩翻滚切到接奶位",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "064",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_elbow_pitch",
            "value_arg": "right_pour_ready_elbow_pitch",
            "block": True,
            "description": "可选：右肘俯仰微调接奶间隙，仅显式传参时执行",
            "source_stage": "pour_alignment_calib_optional_clearance",
            "source_action_id": "064a",
            "enabled_if_not_none": "right_pour_ready_elbow_pitch",
        },
        {
            "kind": "arm",
            "target": "right_arm",
            "method": "set_shoulder_pitch",
            "value_arg": "right_pour_ready_shoulder_pitch",
            "block": True,
            "description": "可选：右肩俯仰微调接奶间隙，仅显式传参时执行",
            "source_stage": "pour_alignment_calib_optional_clearance",
            "source_action_id": "064b",
            "enabled_if_not_none": "right_pour_ready_shoulder_pitch",
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
LEFT_REPLAY_STAGES = {
    "replay_left_move_to_pour_pose_left_only": [
        {
            "kind": "arm",
            "target": "left_arm",
            "method": "set_elbow_pitch",
            "value": 0.9,
            "block": True,
            "description": "左肘进入倒奶预备过渡位",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "068",
        },
        {
            "kind": "arm",
            "target": "left_arm",
            "method": "set_shoulder_pitch",
            "value": 0.2,
            "block": False,
            "description": "左肩俯仰进入倒奶预备区",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "069",
        },
        {
            "kind": "arm",
            "target": "left_arm",
            "method": "set_elbow_pitch",
            "value": 0.4,
            "block": True,
            "description": "左肘回到预备区中间姿态",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "070",
        },
        {
            "kind": "arm",
            "target": "left_arm",
            "method": "home",
            "value": None,
            "block": False,
            "description": "严格复现源阶段中的 left_arm.home(block=False)",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "071",
        },
        {
            "kind": "arm",
            "target": "left_arm",
            "method": "set_wrist_pitch",
            "value_arg": "left_pour_ready_wrist_pitch",
            "block": True,
            "description": "左腕俯仰接入倒奶预备位",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "072",
        },
        {
            "kind": "wait",
            "target": "robot",
            "method": "wait",
            "value": None,
            "block": True,
            "description": "阶段结束统一等待",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "073",
        },
        {
            "kind": "sleep",
            "target": "robot",
            "method": "sleep",
            "value": 0.5,
            "block": True,
            "description": "阶段结束后观察停稳",
            "source_stage": "left_hand_move_to_pour_pose",
            "source_action_id": "074",
        },
    ],
    "replay_left_pour_prep_frame": [
        {
            "kind": "arm",
            "target": "left_arm",
            "method": "set_shoulder_pitch",
            "value_arg": "left_pour_ready_shoulder_pitch",
            "block": False,
            "description": "左肩俯仰进入倒奶前姿态框架",
            "source_stage": "left_hand_pour_milk",
            "source_action_id": "075",
        },
        {
            "kind": "arm",
            "target": "left_arm",
            "method": "set_elbow_pitch",
            "value_arg": "left_pour_ready_elbow_pitch",
            "block": False,
            "description": "左肘进入倒奶前姿态框架",
            "source_stage": "left_hand_pour_milk",
            "source_action_id": "076",
        },
        {
            "kind": "arm",
            "target": "left_arm",
            "method": "set_shoulder_roll",
            "value_arg": "left_pour_ready_shoulder_roll",
            "block": False,
            "description": "左肩 roll 进入倒奶前姿态框架",
            "source_stage": "left_hand_pour_milk",
            "source_action_id": "077",
        },
        {
            "kind": "wait",
            "target": "robot",
            "method": "wait",
            "value": None,
            "block": True,
            "description": "阶段结束统一等待；不自动执行 wrist_roll=1.05",
            "source_stage": "left_hand_pour_milk",
            "source_action_id": "078",
        },
        {
            "kind": "sleep",
            "target": "robot",
            "method": "sleep",
            "value": 0.5,
            "block": True,
            "description": "阶段结束后观察停稳",
            "source_stage": "left_hand_pour_milk",
            "source_action_id": "079",
        },
    ],
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
    "cumulative_right_dx",
    "cumulative_right_dy",
    "cumulative_right_dz",
    "cumulative_left_dx",
    "cumulative_left_dy",
    "cumulative_left_dz",
    "repeat_command",
    "repeat_index",
    "repeat_total",
    "repeat_status",
    "completed_steps",
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
    "candidate_name",
    "candidate_file",
]

LINEAR_STEP_COMMANDS = {
    "x+",
    "x-",
    "y+",
    "y-",
    "z+",
    "z-",
    "left_x+",
    "left_x-",
    "left_y+",
    "left_y-",
    "left_z+",
    "left_z-",
    "right_x+",
    "right_x-",
    "right_y+",
    "right_y-",
    "right_z+",
    "right_z-",
}
REPEATABLE_SMALL_STEP_COMMANDS = LINEAR_STEP_COMMANDS | {
    "right_roll+",
    "right_roll-",
    "right_pitch+",
    "right_pitch-",
    "right_yaw+",
    "right_yaw-",
    "right_elbow+",
    "right_elbow-",
    "right_shoulder_pitch+",
    "right_shoulder_pitch-",
    "left_roll+",
    "left_roll-",
    "left_pitch+",
    "left_pitch-",
    "left_yaw+",
    "left_yaw-",
}


@dataclass
class SessionState:
    log_path: Path
    candidate_path: Path
    dry_run: bool
    execute: bool
    current_wrist_roll: Optional[float] = None
    current_wrist_yaw: Optional[float] = None
    current_wrist_pitch: Optional[float] = None
    current_right_wrist_roll: Optional[float] = None
    current_right_wrist_yaw: Optional[float] = None
    current_right_wrist_pitch: Optional[float] = None
    current_right_elbow_pitch: Optional[float] = None
    current_right_shoulder_pitch: Optional[float] = None
    current_right_shoulder_roll: Optional[float] = None
    current_left_shoulder_pitch: Optional[float] = None
    current_left_elbow_pitch: Optional[float] = None
    current_left_shoulder_roll: Optional[float] = None
    current_left_wrist_yaw: Optional[float] = None
    current_left_wrist_pitch: Optional[float] = None
    current_left_wrist_roll: Optional[float] = None
    current_left_gripper_position: Optional[float] = None
    current_right_gripper_position: Optional[float] = None
    current_right_dx: float = 0.0
    current_right_dy: float = 0.0
    current_right_dz: float = 0.0
    current_left_dx: float = 0.0
    current_left_dy: float = 0.0
    current_left_dz: float = 0.0
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
        "--left-linear-step",
        type=float,
        default=None,
        help="左臂 X/Z relative IK 步长；默认沿用 --linear-step。",
    )
    parser.add_argument(
        "--left-linear-step-small",
        type=float,
        default=None,
        help="左臂 Y relative IK 小步长；默认沿用 --linear-step-small。",
    )
    parser.add_argument(
        "--right-linear-step",
        type=float,
        default=DEFAULT_RIGHT_LINEAR_STEP,
        help=f"右臂 X/Z relative IK 步长，默认 {DEFAULT_RIGHT_LINEAR_STEP}",
    )
    parser.add_argument(
        "--right-linear-step-small",
        type=float,
        default=DEFAULT_RIGHT_LINEAR_STEP_SMALL,
        help=f"右臂 Y relative IK 小步长，默认 {DEFAULT_RIGHT_LINEAR_STEP_SMALL}",
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
        help=(
            "右夹爪夹杯目标位置，默认 "
            f"{DEFAULT_RIGHT_GRIPPER_CUP_POSITION}；这是 pour_align_calib.py "
            "的现场标定覆盖值。若要严格复现 coffee_replay_safe.py 原始 "
            "robot.right_gripper.set_position(0.6)，请显式传 "
            "--right-gripper-cup-position 0.6"
        ),
    )
    parser.add_argument(
        "--right-gripper-step",
        type=float,
        default=DEFAULT_RIGHT_GRIPPER_STEP,
        help=f"右夹爪微调步长，默认 {DEFAULT_RIGHT_GRIPPER_STEP}",
    )
    parser.add_argument(
        "--right-wrist-step",
        type=float,
        default=DEFAULT_RIGHT_WRIST_STEP,
        help=f"右手腕微调步长，默认 {DEFAULT_RIGHT_WRIST_STEP}",
    )
    parser.add_argument(
        "--left-wrist-step",
        type=float,
        default=DEFAULT_LEFT_WRIST_STEP,
        help=f"左手腕 roll/pitch/yaw 微调步长，默认 {DEFAULT_LEFT_WRIST_STEP}",
    )
    parser.add_argument(
        "--right-arm-clearance-step",
        type=float,
        default=DEFAULT_RIGHT_ARM_CLEARANCE_STEP,
        help=f"右臂接奶间隙 elbow/shoulder_pitch 微调步长，默认 {DEFAULT_RIGHT_ARM_CLEARANCE_STEP}",
    )
    parser.add_argument(
        "--right-pour-ready-wrist-yaw",
        type=float,
        default=DEFAULT_RIGHT_POUR_READY_WRIST_YAW,
        help=f"右手接奶位 wrist_yaw，默认 {DEFAULT_RIGHT_POUR_READY_WRIST_YAW}",
    )
    parser.add_argument(
        "--right-pour-ready-wrist-pitch",
        type=float,
        default=DEFAULT_RIGHT_POUR_READY_WRIST_PITCH,
        help=f"右手接奶位 wrist_pitch，默认 {DEFAULT_RIGHT_POUR_READY_WRIST_PITCH}",
    )
    parser.add_argument(
        "--right-pour-ready-wrist-roll",
        type=float,
        default=DEFAULT_RIGHT_POUR_READY_WRIST_ROLL,
        help=f"右手接奶位 wrist_roll，默认 {DEFAULT_RIGHT_POUR_READY_WRIST_ROLL}",
    )
    parser.add_argument(
        "--right-pour-ready-shoulder-roll",
        type=float,
        default=DEFAULT_RIGHT_POUR_READY_SHOULDER_ROLL,
        help=f"右手接奶位 shoulder_roll，默认 {DEFAULT_RIGHT_POUR_READY_SHOULDER_ROLL}",
    )
    parser.add_argument(
        "--right-pour-ready-elbow-pitch",
        type=float,
        default=DEFAULT_RIGHT_POUR_READY_ELBOW_PITCH,
        help=f"右手接奶位 elbow_pitch，默认 {DEFAULT_RIGHT_POUR_READY_ELBOW_PITCH}",
    )
    parser.add_argument(
        "--right-pour-ready-shoulder-pitch",
        type=float,
        default=None,
        help="右手接奶位可选 shoulder_pitch 覆盖；默认不主动设置该关节。",
    )
    parser.add_argument(
        "--left-pour-ready-shoulder-pitch",
        type=float,
        default=DEFAULT_LEFT_POUR_READY_SHOULDER_PITCH,
        help=f"左手倒奶预备 shoulder_pitch，默认 {DEFAULT_LEFT_POUR_READY_SHOULDER_PITCH}",
    )
    parser.add_argument(
        "--left-pour-ready-elbow-pitch",
        type=float,
        default=DEFAULT_LEFT_POUR_READY_ELBOW_PITCH,
        help=f"左手倒奶预备 elbow_pitch，默认 {DEFAULT_LEFT_POUR_READY_ELBOW_PITCH}",
    )
    parser.add_argument(
        "--left-pour-ready-shoulder-roll",
        type=float,
        default=DEFAULT_LEFT_POUR_READY_SHOULDER_ROLL,
        help=f"左手倒奶预备 shoulder_roll，默认 {DEFAULT_LEFT_POUR_READY_SHOULDER_ROLL}",
    )
    parser.add_argument(
        "--left-pour-ready-wrist-pitch",
        type=float,
        default=DEFAULT_LEFT_POUR_READY_WRIST_PITCH,
        help=f"左手 move_to_pour_pose wrist_pitch，默认 {DEFAULT_LEFT_POUR_READY_WRIST_PITCH}",
    )
    parser.add_argument(
        "--left-pour-wrist-roll-prep",
        type=float,
        default=DEFAULT_LEFT_POUR_WRIST_ROLL_PREP,
        help=(
            "左手倒奶 wrist_roll 预备参考值，默认 "
            f"{DEFAULT_LEFT_POUR_WRIST_ROLL_PREP}；replay_left_pour_prep_frame 不会自动执行"
        ),
    )
    parser.add_argument(
        "--left-arm-pour-adjust-step",
        type=float,
        default=DEFAULT_LEFT_ARM_POUR_ADJUST_STEP,
        help=f"左臂倒奶预备关节微调步长，默认 {DEFAULT_LEFT_ARM_POUR_ADJUST_STEP}",
    )
    parser.add_argument(
        "--max-wrist-roll",
        type=float,
        default=DEFAULT_MAX_WRIST_ROLL,
        help=f"允许的最大 wrist_roll 目标，默认 {DEFAULT_MAX_WRIST_ROLL}",
    )
    parser.add_argument(
        "--max-repeat-count",
        type=int,
        default=DEFAULT_MAX_REPEAT_COUNT,
        help=f"批量小步命令最大重复次数，默认 {DEFAULT_MAX_REPEAT_COUNT}",
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
    if args.left_linear_step is None:
        args.left_linear_step = args.linear_step
    if args.left_linear_step_small is None:
        args.left_linear_step_small = args.linear_step_small
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
    if args.left_linear_step <= 0.0:
        raise SystemExit("--left-linear-step 必须大于 0")
    if args.left_linear_step_small <= 0.0:
        raise SystemExit("--left-linear-step-small 必须大于 0")
    if args.right_linear_step <= 0.0:
        raise SystemExit("--right-linear-step 必须大于 0")
    if args.right_linear_step_small <= 0.0:
        raise SystemExit("--right-linear-step-small 必须大于 0")
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
    if args.right_wrist_step <= 0.0:
        raise SystemExit("--right-wrist-step 必须大于 0")
    if args.left_wrist_step <= 0.0:
        raise SystemExit("--left-wrist-step 必须大于 0")
    if args.right_arm_clearance_step <= 0.0:
        raise SystemExit("--right-arm-clearance-step 必须大于 0")
    if args.left_arm_pour_adjust_step <= 0.0:
        raise SystemExit("--left-arm-pour-adjust-step 必须大于 0")
    if args.max_wrist_roll < 0.0:
        raise SystemExit("--max-wrist-roll 必须大于等于 0")
    if args.max_repeat_count <= 0:
        raise SystemExit("--max-repeat-count 必须是正整数")


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
    repeat_command: str = "",
    repeat_index: Optional[int] = None,
    repeat_total: Optional[int] = None,
    repeat_status: str = "",
    completed_steps: Optional[int] = None,
    candidate_name: str = "",
    candidate_file: str = "",
) -> None:
    row = {
        "timestamp": now_iso(),
        "command": action.command,
        "command_type": action.command_type,
        "arm": action.arm,
        "dx": fmt_float(action.dx),
        "dy": fmt_float(action.dy),
        "dz": fmt_float(action.dz),
        "cumulative_right_dx": fmt_float(state.current_right_dx),
        "cumulative_right_dy": fmt_float(state.current_right_dy),
        "cumulative_right_dz": fmt_float(state.current_right_dz),
        "cumulative_left_dx": fmt_float(state.current_left_dx),
        "cumulative_left_dy": fmt_float(state.current_left_dy),
        "cumulative_left_dz": fmt_float(state.current_left_dz),
        "repeat_command": repeat_command,
        "repeat_index": "" if repeat_index is None else str(repeat_index),
        "repeat_total": "" if repeat_total is None else str(repeat_total),
        "repeat_status": repeat_status,
        "completed_steps": "" if completed_steps is None else str(completed_steps),
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
        "candidate_name": candidate_name,
        "candidate_file": candidate_file,
    }
    with state.log_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDNAMES)
        writer.writerow(row)


def pretty_path(path: Path) -> str:
    try:
        return str(path.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def git_value(args: list[str]) -> str:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=PROJECT_ROOT,
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except Exception:
        return ""
    if result.returncode != 0:
        return ""
    return result.stdout.strip()


def step_count_for_offset(offset: float, step: float, positive: str, negative: str) -> str:
    if abs(offset) < 1e-9:
        return ""
    count = offset / step
    rounded = round(count)
    command = positive if offset > 0 else negative
    if abs(count - rounded) < 1e-6:
        return f"{command} {abs(rounded)}"
    return f"# approximate {command}: offset={offset:+.6f}, step={step:.6f}"


def suggested_replay_commands(state: SessionState, args: Optional[argparse.Namespace] = None) -> list[str]:
    commands = ["replay_right_pour_ready"]
    right_linear_step = args.right_linear_step if args is not None else DEFAULT_RIGHT_LINEAR_STEP
    right_linear_step_small = (
        args.right_linear_step_small if args is not None else DEFAULT_RIGHT_LINEAR_STEP_SMALL
    )
    left_linear_step = args.left_linear_step if args is not None else DEFAULT_LINEAR_STEP
    left_linear_step_small = (
        args.left_linear_step_small if args is not None else DEFAULT_LINEAR_STEP_SMALL
    )
    offset_commands = [
        step_count_for_offset(state.current_right_dx, right_linear_step, "right_x+", "right_x-"),
        step_count_for_offset(state.current_right_dy, right_linear_step_small, "right_y+", "right_y-"),
        step_count_for_offset(state.current_right_dz, right_linear_step, "right_z+", "right_z-"),
        step_count_for_offset(state.current_left_dx, left_linear_step, "left_x+", "left_x-"),
        step_count_for_offset(state.current_left_dy, left_linear_step_small, "left_y+", "left_y-"),
        step_count_for_offset(state.current_left_dz, left_linear_step, "left_z+", "left_z-"),
    ]
    commands.extend(command for command in offset_commands if command)
    return commands


def state_snapshot(
    state: SessionState,
    args: Optional[argparse.Namespace] = None,
    *,
    candidate_name: str = "",
    user_note: str = "",
) -> dict:
    return {
        "timestamp": now_iso(),
        "candidate_name": candidate_name,
        "robot_sn": getattr(args, "sn", "") if args is not None else "",
        "robot_ip": getattr(args, "real_ip", "") if args is not None else "",
        "git_branch": git_value(["rev-parse", "--abbrev-ref", "HEAD"]),
        "git_commit": git_value(["rev-parse", "HEAD"]),
        "dry_run": state.dry_run,
        "execute": state.execute,
        "right_offset_dx": state.current_right_dx,
        "right_offset_dy": state.current_right_dy,
        "right_offset_dz": state.current_right_dz,
        "left_offset_dx": state.current_left_dx,
        "left_offset_dy": state.current_left_dy,
        "left_offset_dz": state.current_left_dz,
        "right_wrist_yaw": state.current_right_wrist_yaw,
        "right_wrist_pitch": state.current_right_wrist_pitch,
        "right_wrist_roll": state.current_right_wrist_roll,
        "right_elbow_pitch": state.current_right_elbow_pitch,
        "right_shoulder_pitch": state.current_right_shoulder_pitch,
        "right_shoulder_roll": state.current_right_shoulder_roll,
        "left_wrist_yaw": state.current_left_wrist_yaw,
        "left_wrist_pitch": state.current_left_wrist_pitch,
        "left_wrist_roll": state.current_left_wrist_roll,
        "left_elbow_pitch": state.current_left_elbow_pitch,
        "left_shoulder_pitch": state.current_left_shoulder_pitch,
        "left_shoulder_roll": state.current_left_shoulder_roll,
        "right_gripper_position": state.current_right_gripper_position,
        "left_gripper_position": state.current_left_gripper_position,
        "suggested_replay_commands": suggested_replay_commands(state, args),
        "user_note": user_note,
    }


HELP_SHORT = """
Help:
  默认界面是二级菜单；输入 1-7 进入分组菜单，输入 q 退出。
  也可以直接输入专家命令，例如 replay_right_pour_ready / x+ / obs edge。
  批量小步命令示例：right_x+ 14 / left_z- 3 / right_roll+ 2。

Expert help:
  help all     show full expert command list
  help right   show right-arm replay/gripper/wrist/clearance commands
  help left    show left gripper/alignment/wrist commands
  help replay  show replay_right_* commands
  show_state   print cumulative offsets and tracked calibration state
  save_pose <candidate_name>  save reproducible candidate pose snapshot
""".strip()


HELP_REPLAY = """
Right replay stages:
  replay_right_grasp_cup
                       replay coffee_replay_safe right_hand_grasp_cup stage for right cup grasp
                       right arm order/block/sleep/wait are aligned; cup gripper value
                       uses --right-gripper-cup-position, default 0.80
  replay_right_move_to_coffee_machine
                       replay coffee_replay_safe right_hand_move_to_coffee_machine stage
  replay_right_retreat_after_coffee
                       replay coffee_replay_safe right_hand_retreat_after_coffee stage, contains right_arm.home()
  replay_right_pour_ready
                       replay only right-hand actions inside left_hand_move_to_pour_pose
  right_pour_ready / right_cup_pose
                       alias of replay_right_pour_ready
  right_table_pregrasp / right_table_grasp_pose / right_lift_cup / right_transfer_cup
                       deprecated; no action, use replay_right_* commands
""".strip()


HELP_RIGHT = """
Right commands:
  right_open           set right_gripper to configured open position
  right_grip           set right_gripper to configured cup position, default 0.80
                       calibration override; pass --right-gripper-cup-position 0.6
                       to match coffee_replay_safe.py's original cup grasp
  right_loose          loosen right_gripper by configured step
  right_tight          tighten right_gripper by configured step
  right_roll+ / right_roll-      adjust right wrist_roll by step
  right_pitch+ / right_pitch-    adjust right wrist_pitch by step
  right_yaw+ / right_yaw-        adjust right wrist_yaw by step
  right_set_roll <value>         set right wrist_roll target
  right_set_pitch <value>        set right wrist_pitch target
  right_set_yaw <value>          set right wrist_yaw target
  right_x+ / right_x-      right arm relative IK X +/- step
  right_y+ / right_y-      right arm relative IK Y +/- small step
  right_z+ / right_z-      right arm relative IK Z +/- step
  right_elbow+ / right_elbow-                  adjust right elbow_pitch by clearance step
  right_shoulder_pitch+ / right_shoulder_pitch- adjust right shoulder_pitch by clearance step
  right_set_elbow <value>                      set right elbow_pitch target
  right_set_shoulder_pitch <value>             set right shoulder_pitch target

Clearance tuning:
  If left/right vertical distance is too close, first test right_elbow+ / right_elbow-.
  If elbow is not enough, then test right_shoulder_pitch+ / right_shoulder_pitch-.
  Change only one joint each time, default 0.05 rad or smaller.
  Do not use shoulder_roll first for vertical clearance.
""".strip()


HELP_LEFT = """
Left commands:
  left_open            set left_gripper to configured open position
  left_grip            set left_gripper to configured pitcher position, default 0.70
  left_loose           loosen left_gripper by configured step
  left_tight           tighten left_gripper by configured step
  grip                 alias of left_grip
  replay_left_move_to_pour_pose_left_only
                       replay only left-arm actions from left_hand_move_to_pour_pose
  replay_left_pour_prep_frame
                       replay left pour-prep frame; does not set wrist_roll=1.05/1.25
  left_set_shoulder_pitch <value>  set left shoulder_pitch
  left_set_elbow <value>           set left elbow_pitch
  left_set_shoulder_roll <value>   set left shoulder_roll
  left_set_wrist_pitch <value>     set left wrist_pitch
  left_set_wrist_roll <value>      set left wrist_roll
  left_shoulder_pitch+ / left_shoulder_pitch- adjust left shoulder_pitch by step
  left_elbow+ / left_elbow-                 adjust left elbow_pitch by step
  left_shoulder_roll+ / left_shoulder_roll- adjust left shoulder_roll by step
  left_wrist_pitch+ / left_wrist_pitch-     adjust left wrist_pitch by step
  left_wrist_roll+ / left_wrist_roll-       adjust left wrist_roll by step
  left_x+ / left_x-        left arm relative IK X +/- step
  left_y+ / left_y-        left arm relative IK Y +/- small step
  left_z+ / left_z-        left arm relative IK Z +/- step
  x+ / x-                  alias of left_x+ / left_x-
  y+ / y-                  alias of left_y+ / left_y-
  z+ / z-                  alias of left_z+ / left_z-
  roll0                set left wrist_roll target to 0
  roll03               set left wrist_roll target to 0.3
  roll05               set left wrist_roll target to 0.5
  roll07               set left wrist_roll target to 0.7
  left_roll+ / left_roll-  adjust left wrist_roll by step
  left_pitch+ / left_pitch- adjust left wrist_pitch by step
  left_yaw+ / left_yaw-    adjust left wrist_yaw by step
  left_set_roll <value>
  left_set_pitch <value>
  left_set_yaw <value>
  yaw+ / yaw-          small left wrist_yaw target step when SDK supports it
  pitch+ / pitch-      small left wrist_pitch target step when SDK supports it
""".strip()


HELP_ALL = """
Commands:
  help                 show commands
  help all             show full expert command list
  help right           show right-side commands
  help left            show left-side commands
  help replay          show right replay commands
  show_state           print cumulative offsets and tracked calibration state
  save_pose <candidate_name> save reproducible candidate pose snapshot
  left_open            set left_gripper to configured open position
  left_grip            set left_gripper to configured pitcher position, default 0.70
  left_loose           loosen left_gripper by configured step
  left_tight           tighten left_gripper by configured step
  grip                 alias of left_grip
  replay_left_move_to_pour_pose_left_only
                       replay only left-arm actions from left_hand_move_to_pour_pose
  replay_left_pour_prep_frame
                       replay left pour-prep frame; does not set wrist_roll=1.05/1.25
  left_set_shoulder_pitch <value>  set left shoulder_pitch
  left_set_elbow <value>           set left elbow_pitch
  left_set_shoulder_roll <value>   set left shoulder_roll
  left_set_wrist_pitch <value>     set left wrist_pitch
  left_set_wrist_roll <value>      set left wrist_roll
  left_shoulder_pitch+ / left_shoulder_pitch- adjust left shoulder_pitch by step
  left_elbow+ / left_elbow-                 adjust left elbow_pitch by step
  left_shoulder_roll+ / left_shoulder_roll- adjust left shoulder_roll by step
  left_wrist_pitch+ / left_wrist_pitch-     adjust left wrist_pitch by step
  left_wrist_roll+ / left_wrist_roll-       adjust left wrist_roll by step
  right_open           set right_gripper to configured open position
  right_grip           set right_gripper to configured cup position, default 0.80
                       calibration override; pass --right-gripper-cup-position 0.6
                       to match coffee_replay_safe.py's original cup grasp
  right_loose          loosen right_gripper by configured step
  right_tight          tighten right_gripper by configured step
  replay_right_grasp_cup
                       replay coffee_replay_safe right_hand_grasp_cup stage for right cup grasp
                       right arm order/block/sleep/wait are aligned; cup gripper value
                       uses --right-gripper-cup-position, default 0.80
  replay_right_move_to_coffee_machine
                       replay coffee_replay_safe right_hand_move_to_coffee_machine stage
  replay_right_retreat_after_coffee
                       replay coffee_replay_safe right_hand_retreat_after_coffee stage, contains right_arm.home()
  replay_right_pour_ready
                       replay only right-hand actions inside left_hand_move_to_pour_pose
  right_pour_ready / right_cup_pose
                       alias of replay_right_pour_ready
  right_roll+ / right_roll-      adjust right wrist_roll by step
  right_pitch+ / right_pitch-    adjust right wrist_pitch by step
  right_yaw+ / right_yaw-        adjust right wrist_yaw by step
  right_set_roll <value>         set right wrist_roll target
  right_set_pitch <value>        set right wrist_pitch target
  right_set_yaw <value>          set right wrist_yaw target
  right_x+ / right_x-      right arm relative IK X +/- step
  right_y+ / right_y-      right arm relative IK Y +/- small step
  right_z+ / right_z-      right arm relative IK Z +/- step
  right_elbow+ / right_elbow-                  adjust right elbow_pitch by clearance step
  right_shoulder_pitch+ / right_shoulder_pitch- adjust right shoulder_pitch by clearance step
  right_set_elbow <value>                      set right elbow_pitch target
  right_set_shoulder_pitch <value>             set right shoulder_pitch target
  right_table_pregrasp / right_table_grasp_pose / right_lift_cup / right_transfer_cup
                       deprecated; no action, use replay_right_* commands
  left_x+ / left_x-        left arm relative IK X +/- step
  left_y+ / left_y-        left arm relative IK Y +/- small step
  left_z+ / left_z-        left arm relative IK Z +/- step
  x+ / x-                  alias of left_x+ / left_x-
  y+ / y-                  alias of left_y+ / left_y-
  z+ / z-                  alias of left_z+ / left_z-
  roll0                set left wrist_roll target to 0
  roll03               set left wrist_roll target to 0.3
  roll05               set left wrist_roll target to 0.5
  roll07               set left wrist_roll target to 0.7
  left_roll+ / left_roll-  adjust left wrist_roll by step
  left_pitch+ / left_pitch- adjust left wrist_pitch by step
  left_yaw+ / left_yaw-    adjust left wrist_yaw by step
  left_set_roll <value>
  left_set_pitch <value>
  left_set_yaw <value>
  yaw+ / yaw-          small left wrist_yaw target step when SDK supports it
  pitch+ / pitch-      small left wrist_pitch target step when SDK supports it
  obs [value]          record observation: spout_in_cup / edge / outside / near_collision / unsafe / uncertain
  save [note]          save current calibration note
  save_pose <candidate_name> save full candidate state to candidates jsonl
  list_candidates      show last 10 saved candidates
  quit                 exit

Clearance tuning:
  If left/right vertical distance is too close, first test right_elbow+ / right_elbow-.
  If elbow is not enough, then test right_shoulder_pitch+ / right_shoulder_pitch-.
  Change only one joint each time, default 0.05 rad or smaller.
  Do not use shoulder_roll first for vertical clearance.
""".strip()


def print_help(topic: str = "short") -> None:
    if topic == "all":
        print(HELP_ALL)
    elif topic == "right":
        print(HELP_REPLAY)
        print()
        print(HELP_RIGHT)
    elif topic == "left":
        print(HELP_LEFT)
    elif topic == "replay":
        print(HELP_REPLAY)
    else:
        print(HELP_SHORT)


def print_startup_banner(args: argparse.Namespace, state: SessionState) -> None:
    mode = "EXECUTE" if args.execute else "DRY-RUN"
    print("=" * 72)
    print(f"[Pour Align Calib] {mode}")
    print(f"log: {pretty_path(state.log_path)}")
    print(f"candidate_log: {pretty_path(state.candidate_path)}")
    print("空壶标定：先分阶段确认右手取杯到倒奶前姿态，再用左手 relative IK 小步对位。")
    if args.execute:
        print("实机模式：每个真实动作都需要输入 y 二次确认。")
    else:
        print("dry-run：不会连接机器人，只打印动作计划并写日志。")
    print("输入 1-7 进入菜单，help 查看简短说明，help all 查看完整专家命令，q 退出。")


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
    stage_name = replay_stage_name(command)
    if stage_name in RIGHT_REPLAY_STAGES:
        return RIGHT_REPLAY_STAGES[stage_name]
    return LEFT_REPLAY_STAGES[stage_name]


def step_enabled(step: dict, args: argparse.Namespace) -> bool:
    enabled_if_not_none = step.get("enabled_if_not_none")
    if enabled_if_not_none is not None:
        return getattr(args, enabled_if_not_none) is not None
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
    if targets == {"left_arm"}:
        return "left"
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
        enabled_if_not_none = step.get("enabled_if_not_none")
        if enabled_if_not_none is not None:
            return prefix + f"[skipped unless --{enabled_if_not_none.replace('_', '-')} is set]"
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
        if block is not None:
            return prefix + f"robot.{target}.home(block={block})"
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
    if action.command_type == "arm_relative_ik":
        arm_name = "right_arm" if action.arm == "right" else "left_arm"
        return (
            f"robot.{arm_name}.ik("
            f"{action.dx or 0.0:.6f}, {action.dy or 0.0:.6f}, {action.dz or 0.0:.6f}, "
            "0, 0, 0, block=True, abs=False)"
        )
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
    if action.command_type == "left_wrist_adjust":
        if action.wrist_roll_target is not None:
            return f"robot.left_arm.set_wrist_roll({action.wrist_roll_target:.6f}, block=True)"
        if action.wrist_pitch_delta_or_target is not None:
            return (
                "robot.left_arm.set_wrist_pitch("
                f"{action.wrist_pitch_delta_or_target:.6f}, block=True)"
            )
        if action.wrist_yaw_delta_or_target is not None:
            return (
                "robot.left_arm.set_wrist_yaw("
                f"{action.wrist_yaw_delta_or_target:.6f}, block=True)"
            )
    if action.command_type == "right_wrist_adjust":
        if action.wrist_roll_target is not None:
            return f"robot.right_arm.set_wrist_roll({action.wrist_roll_target:.6f}, block=True)"
        if action.wrist_pitch_delta_or_target is not None:
            return (
                "robot.right_arm.set_wrist_pitch("
                f"{action.wrist_pitch_delta_or_target:.6f}, block=True)"
            )
        if action.wrist_yaw_delta_or_target is not None:
            return (
                "robot.right_arm.set_wrist_yaw("
                f"{action.wrist_yaw_delta_or_target:.6f}, block=True)"
            )
    if action.command_type == "right_arm_clearance_adjust":
        try:
            payload = json.loads(action.joint_targets)
            joint = payload["joint"]
            target = float(payload["target"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return action.command
        method_name = "set_elbow_pitch" if joint == "elbow_pitch" else "set_shoulder_pitch"
        return f"robot.right_arm.{method_name}({target:.6f}, block=True)"
    if action.command_type == "left_arm_pour_adjust":
        try:
            payload = json.loads(action.joint_targets)
            joint = payload["joint"]
            target = float(payload["target"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return action.command
        method_by_joint = {
            "left_shoulder_pitch": "set_shoulder_pitch",
            "left_elbow_pitch": "set_elbow_pitch",
            "left_shoulder_roll": "set_shoulder_roll",
            "left_wrist_pitch": "set_wrist_pitch",
            "left_wrist_roll": "set_wrist_roll",
        }
        method_name = method_by_joint.get(joint)
        if method_name is None:
            return action.command
        return f"robot.left_arm.{method_name}({target:.6f}, block=True)"
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
    if action.command == "replay_left_move_to_pour_pose_left_only":
        print("[warning] replay_left_move_to_pour_pose_left_only 只控制左臂，但包含 left_arm.home(block=False)。")
    if action.command == "replay_left_pour_prep_frame":
        print("[info] replay_left_pour_prep_frame 不会自动执行 wrist_roll=1.05、1.25 或左右摆动。")
    print(f"[confirm] {describe_action(action, args)}")
    user_input = input("输入 y 执行该 replay stage，其他任意输入跳过：").strip().lower()
    return user_input == "y"


def execute_action(robot, action: Action, args: argparse.Namespace, state: SessionState) -> None:
    execute_action_once(robot, action, args, state)


def execute_action_once(
    robot,
    action: Action,
    args: argparse.Namespace,
    state: SessionState,
    *,
    skip_confirm: bool = False,
    repeat_command: str = "",
    repeat_index: Optional[int] = None,
    repeat_total: Optional[int] = None,
) -> str:
    print(f"[plan] {describe_action(action)}")

    if state.dry_run:
        append_log(
            state,
            action,
            user_confirmed=None,
            status="dry_run",
            repeat_command=repeat_command,
            repeat_index=repeat_index,
            repeat_total=repeat_total,
            repeat_status="dry_run" if repeat_command else "",
            completed_steps=repeat_index if repeat_command else None,
        )
        print("[dry-run] 已记录，不连接机器人。")
        if action.command_type == "arm_relative_ik":
            print("[dry-run] relative IK 累计偏移不更新；实机成功执行后才累计。")
        else:
            update_tracked_state(action, state)
        return "dry_run"

    confirmed = True if skip_confirm else confirm_real_action(action)
    if not confirmed:
        append_log(
            state,
            action,
            user_confirmed=False,
            status="skipped_by_user",
            repeat_command=repeat_command,
            repeat_index=repeat_index,
            repeat_total=repeat_total,
            repeat_status="skipped_by_user" if repeat_command else "",
            completed_steps=(repeat_index - 1) if repeat_index else None,
        )
        print("[skip] 用户未确认，动作已跳过。")
        return "skipped_by_user"

    status = "ok"
    try:
        start_time = time.perf_counter()
        if action.command_type in {"relative_ik", "arm_relative_ik"}:
            arm = robot.right_arm if action.arm == "right" else robot.left_arm
            arm.ik(
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
        elif action.command_type == "left_wrist_adjust":
            if action.wrist_roll_target is not None:
                robot.left_arm.set_wrist_roll(action.wrist_roll_target, block=True)
            elif action.wrist_pitch_delta_or_target is not None:
                robot.left_arm.set_wrist_pitch(
                    action.wrist_pitch_delta_or_target,
                    block=True,
                )
            elif action.wrist_yaw_delta_or_target is not None:
                robot.left_arm.set_wrist_yaw(
                    action.wrist_yaw_delta_or_target,
                    block=True,
                )
            else:
                status = "unsupported"
        elif action.command_type == "right_wrist_adjust":
            if action.wrist_roll_target is not None:
                robot.right_arm.set_wrist_roll(action.wrist_roll_target, block=True)
            elif action.wrist_pitch_delta_or_target is not None:
                robot.right_arm.set_wrist_pitch(
                    action.wrist_pitch_delta_or_target,
                    block=True,
                )
            elif action.wrist_yaw_delta_or_target is not None:
                robot.right_arm.set_wrist_yaw(
                    action.wrist_yaw_delta_or_target,
                    block=True,
                )
            else:
                status = "unsupported"
        elif action.command_type == "right_arm_clearance_adjust":
            try:
                payload = json.loads(action.joint_targets)
                joint = payload["joint"]
                target = float(payload["target"])
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                status = "unsupported"
            else:
                if joint == "elbow_pitch":
                    robot.right_arm.set_elbow_pitch(target, block=True)
                elif joint == "shoulder_pitch":
                    robot.right_arm.set_shoulder_pitch(target, block=True)
                else:
                    status = "unsupported"
        elif action.command_type == "left_arm_pour_adjust":
            try:
                payload = json.loads(action.joint_targets)
                joint = payload["joint"]
                target = float(payload["target"])
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                status = "unsupported"
            else:
                if joint == "left_shoulder_pitch":
                    robot.left_arm.set_shoulder_pitch(target, block=True)
                elif joint == "left_elbow_pitch":
                    robot.left_arm.set_elbow_pitch(target, block=True)
                elif joint == "left_shoulder_roll":
                    robot.left_arm.set_shoulder_roll(target, block=True)
                elif joint == "left_wrist_pitch":
                    robot.left_arm.set_wrist_pitch(target, block=True)
                elif joint == "left_wrist_roll":
                    robot.left_arm.set_wrist_roll(target, block=True)
                else:
                    status = "unsupported"
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
            status = "success"
    except Exception as exc:  # pragma: no cover - real SDK/runtime only
        status = f"error: {exc}"
        print(f"[error] {exc}")

    append_log(
        state,
        action,
        user_confirmed=True,
        status=status,
        repeat_command=repeat_command,
        repeat_index=repeat_index,
        repeat_total=repeat_total,
        repeat_status=("ok" if status == "success" else "partial") if repeat_command else "",
        completed_steps=repeat_index if status == "success" and repeat_index is not None else None,
    )
    return status


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
        update_tracked_state_from_replay_stage(command, args, state)
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
            update_tracked_state_from_replay_stage(command, args, state)
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
        if block is None:
            method()
        else:
            method(block=block)
    elif block is None:
        method(value)
    else:
        method(value, block=block)


def update_tracked_state_from_replay_stage(
    command: str,
    args: argparse.Namespace,
    state: SessionState,
) -> None:
    stage_name = replay_stage_name(command)
    if stage_name == "replay_left_move_to_pour_pose_left_only":
        state.current_left_elbow_pitch = 0.4
        state.current_left_shoulder_pitch = 0.0
        state.current_left_shoulder_roll = 0.0
        state.current_left_wrist_pitch = args.left_pour_ready_wrist_pitch
        state.current_left_wrist_roll = 0.0
        state.current_left_wrist_yaw = 0.0
        state.current_wrist_pitch = args.left_pour_ready_wrist_pitch
        state.current_wrist_roll = 0.0
        state.current_wrist_yaw = 0.0
        return
    if stage_name == "replay_left_pour_prep_frame":
        state.current_left_shoulder_pitch = args.left_pour_ready_shoulder_pitch
        state.current_left_elbow_pitch = args.left_pour_ready_elbow_pitch
        state.current_left_shoulder_roll = args.left_pour_ready_shoulder_roll
        return
    if stage_name != "replay_right_pour_ready":
        if stage_name == "replay_right_retreat_after_coffee":
            state.current_right_elbow_pitch = 0.0
            state.current_right_shoulder_pitch = 0.0
        return
    state.current_right_wrist_yaw = args.right_pour_ready_wrist_yaw
    state.current_right_wrist_pitch = args.right_pour_ready_wrist_pitch
    state.current_right_wrist_roll = args.right_pour_ready_wrist_roll
    state.current_right_shoulder_roll = args.right_pour_ready_shoulder_roll
    if args.right_pour_ready_elbow_pitch is not None:
        state.current_right_elbow_pitch = args.right_pour_ready_elbow_pitch
    if args.right_pour_ready_shoulder_pitch is not None:
        state.current_right_shoulder_pitch = args.right_pour_ready_shoulder_pitch


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
    if action.command_type == "arm_relative_ik":
        if action.arm == "right":
            state.current_right_dx += action.dx or 0.0
            state.current_right_dy += action.dy or 0.0
            state.current_right_dz += action.dz or 0.0
        else:
            state.current_left_dx += action.dx or 0.0
            state.current_left_dy += action.dy or 0.0
            state.current_left_dz += action.dz or 0.0
    elif action.command_type == "wrist_roll":
        state.current_wrist_roll = action.wrist_roll_target
        state.current_left_wrist_roll = action.wrist_roll_target
    elif action.command_type == "wrist_yaw":
        state.current_wrist_yaw = action.wrist_yaw_delta_or_target
        state.current_left_wrist_yaw = action.wrist_yaw_delta_or_target
    elif action.command_type == "wrist_pitch":
        state.current_wrist_pitch = action.wrist_pitch_delta_or_target
        state.current_left_wrist_pitch = action.wrist_pitch_delta_or_target
    elif action.command_type == "left_wrist_adjust":
        if action.wrist_roll_target is not None:
            state.current_left_wrist_roll = action.wrist_roll_target
            state.current_wrist_roll = action.wrist_roll_target
        elif action.wrist_yaw_delta_or_target is not None:
            state.current_left_wrist_yaw = action.wrist_yaw_delta_or_target
            state.current_wrist_yaw = action.wrist_yaw_delta_or_target
        elif action.wrist_pitch_delta_or_target is not None:
            state.current_left_wrist_pitch = action.wrist_pitch_delta_or_target
            state.current_wrist_pitch = action.wrist_pitch_delta_or_target
    elif action.command_type == "right_wrist_adjust":
        if action.wrist_roll_target is not None:
            state.current_right_wrist_roll = action.wrist_roll_target
        elif action.wrist_yaw_delta_or_target is not None:
            state.current_right_wrist_yaw = action.wrist_yaw_delta_or_target
        elif action.wrist_pitch_delta_or_target is not None:
            state.current_right_wrist_pitch = action.wrist_pitch_delta_or_target
    elif action.command_type == "right_arm_clearance_adjust":
        try:
            payload = json.loads(action.joint_targets)
            joint = payload["joint"]
            target = float(payload["target"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        if joint == "elbow_pitch":
            state.current_right_elbow_pitch = target
        elif joint == "shoulder_pitch":
            state.current_right_shoulder_pitch = target
    elif action.command_type == "left_arm_pour_adjust":
        try:
            payload = json.loads(action.joint_targets)
            joint = payload["joint"]
            target = float(payload["target"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        if joint == "left_shoulder_pitch":
            state.current_left_shoulder_pitch = target
        elif joint == "left_elbow_pitch":
            state.current_left_elbow_pitch = target
        elif joint == "left_shoulder_roll":
            state.current_left_shoulder_roll = target
        elif joint == "left_wrist_pitch":
            state.current_left_wrist_pitch = target
            state.current_wrist_pitch = target
        elif joint == "left_wrist_roll":
            state.current_left_wrist_roll = target
            state.current_wrist_roll = target
    elif action.command_type == "gripper":
        if action.arm == "left":
            state.current_left_gripper_position = action.gripper_position
        elif action.arm == "right":
            state.current_right_gripper_position = action.gripper_position


def build_linear_action(command: str, args: argparse.Namespace) -> Action:
    alias_by_command = {
        "x+": "left_x+",
        "x-": "left_x-",
        "y+": "left_y+",
        "y-": "left_y-",
        "z+": "left_z+",
        "z-": "left_z-",
    }
    canonical_command = alias_by_command.get(command, command)
    arm = "right" if canonical_command.startswith("right_") else "left"
    axis_command = canonical_command.split("_", 1)[1]
    linear_step = args.right_linear_step if arm == "right" else args.left_linear_step
    linear_step_small = (
        args.right_linear_step_small if arm == "right" else args.left_linear_step_small
    )
    dx = dy = dz = 0.0
    if axis_command == "x+":
        dx = linear_step
    elif axis_command == "x-":
        dx = -linear_step
    elif axis_command == "y+":
        dy = linear_step_small
    elif axis_command == "y-":
        dy = -linear_step_small
    elif axis_command == "z+":
        dz = linear_step
    elif axis_command == "z-":
        dz = -linear_step
    else:
        raise ValueError(f"unsupported linear command: {command}")
    return Action(
        command=command,
        command_type="arm_relative_ik",
        arm=arm,
        dx=dx,
        dy=dy,
        dz=dz,
    )


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
    joint_targets = json.dumps(
        {"joint": "left_wrist_roll", "target": target},
        ensure_ascii=False,
        separators=(",", ":"),
    )
    return Action(
        command=command,
        command_type="left_wrist_adjust",
        arm="left",
        joint_targets=joint_targets,
        wrist_roll_target=target,
    )


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


def build_right_wrist_adjust_action(
    parts: list[str],
    args: argparse.Namespace,
    state: SessionState,
) -> Action:
    command = parts[0].lower()
    if command in {"right_set_roll", "right_set_pitch", "right_set_yaw"}:
        if len(parts) != 2:
            raise ValueError(f"{command} requires exactly one numeric value")
        try:
            target = float(parts[1])
        except ValueError as exc:
            raise ValueError(f"{command} value must be numeric: {parts[1]}") from exc
    elif command in {"right_roll+", "right_roll-"}:
        current = state.current_right_wrist_roll
        if current is None:
            current = args.right_pour_ready_wrist_roll
            print(
                "[info] current_right_wrist_roll 未知，"
                f"以 right_pour_ready_wrist_roll={current:.3f} 作为微调基准。"
            )
        sign = 1.0 if command.endswith("+") else -1.0
        target = current + sign * args.right_wrist_step
    elif command in {"right_pitch+", "right_pitch-"}:
        current = state.current_right_wrist_pitch
        if current is None:
            current = args.right_pour_ready_wrist_pitch
            print(
                "[info] current_right_wrist_pitch 未知，"
                f"以 right_pour_ready_wrist_pitch={current:.3f} 作为微调基准。"
            )
        sign = 1.0 if command.endswith("+") else -1.0
        target = current + sign * args.right_wrist_step
    elif command in {"right_yaw+", "right_yaw-"}:
        current = state.current_right_wrist_yaw
        if current is None:
            current = args.right_pour_ready_wrist_yaw
            print(
                "[info] current_right_wrist_yaw 未知，"
                f"以 right_pour_ready_wrist_yaw={current:.3f} 作为微调基准。"
            )
        sign = 1.0 if command.endswith("+") else -1.0
        target = current + sign * args.right_wrist_step
    else:
        raise ValueError(f"unsupported right wrist command: {command}")

    kwargs = {}
    if command in {"right_roll+", "right_roll-", "right_set_roll"}:
        kwargs["wrist_roll_target"] = target
    elif command in {"right_pitch+", "right_pitch-", "right_set_pitch"}:
        kwargs["wrist_pitch_delta_or_target"] = target
    else:
        kwargs["wrist_yaw_delta_or_target"] = target

    return Action(
        command=" ".join(parts),
        command_type="right_wrist_adjust",
        arm="right",
        **kwargs,
    )


def build_right_arm_clearance_action(
    parts: list[str],
    args: argparse.Namespace,
    state: SessionState,
) -> Action:
    command = parts[0].lower()
    if command in {"right_set_elbow", "right_set_shoulder_pitch"}:
        if len(parts) != 2:
            raise ValueError(f"{command} requires exactly one numeric value")
        try:
            target = float(parts[1])
        except ValueError as exc:
            raise ValueError(f"{command} value must be numeric: {parts[1]}") from exc
    elif command in {"right_elbow+", "right_elbow-"}:
        current = state.current_right_elbow_pitch
        if current is None:
            current = 0.0
            print(
                "[warn] current_right_elbow_pitch unknown, "
                "using 0.0 as estimate after right_arm.home context"
            )
        sign = 1.0 if command.endswith("+") else -1.0
        target = current + sign * args.right_arm_clearance_step
    elif command in {"right_shoulder_pitch+", "right_shoulder_pitch-"}:
        current = state.current_right_shoulder_pitch
        if current is None:
            current = 0.0
            print(
                "[warn] current_right_shoulder_pitch unknown, "
                "using 0.0 as estimate after right_arm.home context"
            )
        sign = 1.0 if command.endswith("+") else -1.0
        target = current + sign * args.right_arm_clearance_step
    else:
        raise ValueError(f"unsupported right arm clearance command: {command}")

    joint = (
        "elbow_pitch"
        if command in {"right_elbow+", "right_elbow-", "right_set_elbow"}
        else "shoulder_pitch"
    )
    joint_targets = json.dumps(
        {"joint": joint, "target": target},
        ensure_ascii=False,
        separators=(",", ":"),
    )
    return Action(
        command=" ".join(parts),
        command_type="right_arm_clearance_adjust",
        arm="right",
        joint_targets=joint_targets,
    )


def build_left_arm_pour_adjust_action(
    parts: list[str],
    args: argparse.Namespace,
    state: SessionState,
) -> Action:
    command = parts[0].lower()
    direct_joint_by_command = {
        "left_set_shoulder_pitch": "left_shoulder_pitch",
        "left_set_elbow": "left_elbow_pitch",
        "left_set_shoulder_roll": "left_shoulder_roll",
        "left_set_wrist_pitch": "left_wrist_pitch",
        "left_set_wrist_roll": "left_wrist_roll",
    }
    step_joint_by_command = {
        "left_shoulder_pitch+": "left_shoulder_pitch",
        "left_shoulder_pitch-": "left_shoulder_pitch",
        "left_elbow+": "left_elbow_pitch",
        "left_elbow-": "left_elbow_pitch",
        "left_shoulder_roll+": "left_shoulder_roll",
        "left_shoulder_roll-": "left_shoulder_roll",
        "left_wrist_pitch+": "left_wrist_pitch",
        "left_wrist_pitch-": "left_wrist_pitch",
        "left_wrist_roll+": "left_wrist_roll",
        "left_wrist_roll-": "left_wrist_roll",
    }
    defaults_by_joint = {
        "left_shoulder_pitch": args.left_pour_ready_shoulder_pitch,
        "left_elbow_pitch": args.left_pour_ready_elbow_pitch,
        "left_shoulder_roll": args.left_pour_ready_shoulder_roll,
        "left_wrist_pitch": args.left_pour_ready_wrist_pitch,
        "left_wrist_roll": min(args.left_pour_wrist_roll_prep, args.max_wrist_roll),
    }
    current_by_joint = {
        "left_shoulder_pitch": state.current_left_shoulder_pitch,
        "left_elbow_pitch": state.current_left_elbow_pitch,
        "left_shoulder_roll": state.current_left_shoulder_roll,
        "left_wrist_pitch": state.current_left_wrist_pitch,
        "left_wrist_roll": state.current_left_wrist_roll,
    }

    if command in direct_joint_by_command:
        if len(parts) != 2:
            raise ValueError(f"{command} requires exactly one numeric value")
        joint = direct_joint_by_command[command]
        try:
            target = float(parts[1])
        except ValueError as exc:
            raise ValueError(f"{command} value must be numeric: {parts[1]}") from exc
    elif command in step_joint_by_command:
        joint = step_joint_by_command[command]
        current = current_by_joint[joint]
        if current is None:
            current = defaults_by_joint[joint]
            print(f"[info] {joint} 未知，以 {current:.3f} 作为微调基准。")
        sign = 1.0 if command.endswith("+") else -1.0
        target = current + sign * args.left_arm_pour_adjust_step
    else:
        raise ValueError(f"unsupported left arm pour adjust command: {command}")

    joint_targets = json.dumps(
        {"joint": joint, "target": target},
        ensure_ascii=False,
        separators=(",", ":"),
    )
    kwargs = {}
    if joint == "left_wrist_roll":
        kwargs["wrist_roll_target"] = target
    elif joint == "left_wrist_pitch":
        kwargs["wrist_pitch_delta_or_target"] = target
    return Action(
        command=" ".join(parts),
        command_type="left_arm_pour_adjust",
        arm="left",
        joint_targets=joint_targets,
        **kwargs,
    )


def build_left_wrist_adjust_action(
    parts: list[str],
    args: argparse.Namespace,
    state: SessionState,
) -> Action:
    command = parts[0].lower()
    if command in {"left_set_roll", "left_set_pitch", "left_set_yaw"}:
        if len(parts) != 2:
            raise ValueError(f"{command} requires exactly one numeric value")
        try:
            target = float(parts[1])
        except ValueError as exc:
            raise ValueError(f"{command} value must be numeric: {parts[1]}") from exc
    elif command in {"left_roll+", "left_roll-"}:
        current = state.current_left_wrist_roll
        if current is None:
            current = 0.0
            print("[info] current_left_wrist_roll 未知，以 0.000 作为微调基准。")
        sign = 1.0 if command.endswith("+") else -1.0
        target = current + sign * args.left_wrist_step
    elif command in {"left_pitch+", "left_pitch-"}:
        current = state.current_left_wrist_pitch
        if current is None:
            current = 0.0
            print("[info] current_left_wrist_pitch 未知，以 0.000 作为微调基准。")
        sign = 1.0 if command.endswith("+") else -1.0
        target = current + sign * args.left_wrist_step
    elif command in {"left_yaw+", "left_yaw-"}:
        current = state.current_left_wrist_yaw
        if current is None:
            current = 0.0
            print("[info] current_left_wrist_yaw 未知，以 0.000 作为微调基准。")
        sign = 1.0 if command.endswith("+") else -1.0
        target = current + sign * args.left_wrist_step
    else:
        raise ValueError(f"unsupported left wrist command: {command}")

    if command in {"left_roll+", "left_roll-", "left_set_roll"}:
        joint = "left_wrist_roll"
        kwargs = {"wrist_roll_target": target}
    elif command in {"left_pitch+", "left_pitch-", "left_set_pitch"}:
        joint = "left_wrist_pitch"
        kwargs = {"wrist_pitch_delta_or_target": target}
    else:
        joint = "left_wrist_yaw"
        kwargs = {"wrist_yaw_delta_or_target": target}

    joint_targets = json.dumps(
        {"joint": joint, "target": target},
        ensure_ascii=False,
        separators=(",", ":"),
    )
    return Action(
        command=" ".join(parts),
        command_type="left_wrist_adjust",
        arm="left",
        joint_targets=joint_targets,
        **kwargs,
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


def build_small_step_action(parts: list[str], robot, args: argparse.Namespace, state: SessionState) -> Action:
    command = parts[0].lower()
    if command in LINEAR_STEP_COMMANDS:
        return build_linear_action(command, args)
    if command in {
        "right_roll+",
        "right_roll-",
        "right_pitch+",
        "right_pitch-",
        "right_yaw+",
        "right_yaw-",
    }:
        return build_right_wrist_adjust_action(parts, args, state)
    if command in {
        "right_elbow+",
        "right_elbow-",
        "right_shoulder_pitch+",
        "right_shoulder_pitch-",
    }:
        return build_right_arm_clearance_action(parts, args, state)
    if command in {
        "left_roll+",
        "left_roll-",
        "left_pitch+",
        "left_pitch-",
        "left_yaw+",
        "left_yaw-",
    }:
        return build_left_wrist_adjust_action(parts, args, state)
    if command in {"yaw+", "yaw-", "pitch+", "pitch-"}:
        return build_wrist_step_action(command, robot, args, state)
    raise ValueError(f"repeat is not supported for command: {command}")


def repeat_delta_summary(action: Action, repeat_count: int) -> list[str]:
    lines = []
    if action.command_type == "arm_relative_ik":
        if action.dx:
            lines.append(f"per-step dx={action.dx:.3f}, total dx={action.dx * repeat_count:.3f}")
        if action.dy:
            lines.append(f"per-step dy={action.dy:.3f}, total dy={action.dy * repeat_count:.3f}")
        if action.dz:
            lines.append(f"per-step dz={action.dz:.3f}, total dz={action.dz * repeat_count:.3f}")
    elif action.wrist_roll_target is not None:
        lines.append(f"target will advance step-by-step to {action.wrist_roll_target:.6f} ...")
    elif action.wrist_pitch_delta_or_target is not None:
        lines.append(f"target will advance step-by-step to {action.wrist_pitch_delta_or_target:.6f} ...")
    elif action.wrist_yaw_delta_or_target is not None:
        lines.append(f"target will advance step-by-step to {action.wrist_yaw_delta_or_target:.6f} ...")
    return lines


def repeat_confirmation_text(action: Action, repeat_count: int) -> str:
    if action.command_type == "arm_relative_ik":
        parts = []
        if action.dx:
            parts.append(f"每步 {action.dx:.3f} m，累计 {action.dx * repeat_count:.3f} m")
        if action.dy:
            parts.append(f"每步 {action.dy:.3f} m，累计 {action.dy * repeat_count:.3f} m")
        if action.dz:
            parts.append(f"每步 {action.dz:.3f} m，累计 {action.dz * repeat_count:.3f} m")
        return "，".join(parts)
    return "每步使用当前小步逻辑逐次计算目标"


def parse_repeat_count(command: str, value: str, args: argparse.Namespace) -> Optional[int]:
    if not value.isdigit():
        print(f"[blocked] repeat_count 必须是正整数: {value}")
        return None
    count = int(value)
    if count <= 0:
        print(f"[blocked] repeat_count 必须是正整数: {value}")
        return None
    if count > args.max_repeat_count:
        print(
            f"[blocked] repeat_count={count} 超过 --max-repeat-count={args.max_repeat_count}，"
            "未执行。"
        )
        return None
    return count


def handle_repeat_command(
    parts: list[str],
    robot,
    args: argparse.Namespace,
    state: SessionState,
) -> bool:
    command = parts[0].lower()
    if len(parts) != 2:
        return False
    if command in RIGHT_REPLAY_STAGES or command in RIGHT_REPLAY_STAGE_ALIASES or command in LEFT_REPLAY_STAGES:
        print(f"[blocked] replay stage 不支持 repeat: {command}")
        append_log(
            state,
            Action(command=" ".join(parts), command_type="repeat_blocked"),
            user_confirmed=None,
            status="repeat_not_supported_for_replay_stage",
        )
        return True
    if command in {"save", "quit", "exit", "q", "obs", "show_state", "save_pose"}:
        print(f"[blocked] {command} 不支持 repeat。")
        append_log(
            state,
            Action(command=" ".join(parts), command_type="repeat_blocked"),
            user_confirmed=None,
            status="repeat_not_supported",
        )
        return True
    if command not in REPEATABLE_SMALL_STEP_COMMANDS:
        return False

    repeat_count = parse_repeat_count(command, parts[1], args)
    if repeat_count is None:
        append_log(
            state,
            Action(command=" ".join(parts), command_type="repeat_blocked"),
            user_confirmed=None,
            status="invalid_repeat_count",
            repeat_command=command,
            repeat_total=None,
            repeat_status="blocked",
            completed_steps=0,
        )
        return True

    try:
        first_action = build_small_step_action([command], robot, args, state)
    except ValueError as exc:
        print(f"[blocked] {exc}")
        append_log(
            state,
            Action(command=" ".join(parts), command_type="repeat_blocked"),
            user_confirmed=None,
            status=f"blocked: {exc}",
        )
        return True

    print(f"[plan repeat] {command} × {repeat_count}")
    for line in repeat_delta_summary(first_action, repeat_count):
        print(line)

    if state.dry_run:
        dry_run_completed = 0
        for index in range(1, repeat_count + 1):
            action = build_small_step_action([command], robot, args, state)
            execute_action_once(
                robot,
                action,
                args,
                state,
                repeat_command=command,
                repeat_index=index,
                repeat_total=repeat_count,
            )
            dry_run_completed = index
        append_log(
            state,
            Action(command=" ".join(parts), command_type="repeat"),
            user_confirmed=None,
            status="dry_run",
            repeat_command=command,
            repeat_total=repeat_count,
            repeat_status="dry_run",
            completed_steps=dry_run_completed,
        )
        return True

    detail = repeat_confirmation_text(first_action, repeat_count)
    print(f"即将执行 {command} × {repeat_count}，{detail}。输入 y 执行。")
    if input("> ").strip().lower() != "y":
        append_log(
            state,
            Action(command=" ".join(parts), command_type="repeat"),
            user_confirmed=False,
            status="skipped_by_user",
            repeat_command=command,
            repeat_total=repeat_count,
            repeat_status="skipped_by_user",
            completed_steps=0,
        )
        print("[skip] 用户未确认，repeat 已跳过。")
        return True

    completed_steps = 0
    try:
        for index in range(1, repeat_count + 1):
            action = build_small_step_action([command], robot, args, state)
            status = execute_action_once(
                robot,
                action,
                args,
                state,
                skip_confirm=True,
                repeat_command=command,
                repeat_index=index,
                repeat_total=repeat_count,
            )
            if status != "success":
                append_log(
                    state,
                    Action(command=" ".join(parts), command_type="repeat"),
                    user_confirmed=True,
                    status=status,
                    repeat_command=command,
                    repeat_total=repeat_count,
                    repeat_status="partial",
                    completed_steps=completed_steps,
                )
                print(
                    f"[repeat partial] {command}: completed_steps={completed_steps}, "
                    f"requested_steps={repeat_count}"
                )
                return True
            completed_steps = index
    except KeyboardInterrupt:
        append_log(
            state,
            Action(command="keyboard_interrupt", command_type="interrupt"),
            user_confirmed=None,
            status="keyboard_interrupt",
            user_observation="interrupted_during_repeat=true",
            repeat_command=command,
            repeat_total=repeat_count,
            repeat_status="interrupted",
            completed_steps=completed_steps,
        )
        print(
            f"\n[repeat interrupted] repeat_command={command}, "
            f"completed_steps={completed_steps}, requested_steps={repeat_count}"
        )
        raise

    append_log(
        state,
        Action(command=" ".join(parts), command_type="repeat"),
        user_confirmed=True,
        status="success",
        repeat_command=command,
        repeat_total=repeat_count,
        repeat_status="ok",
        completed_steps=completed_steps,
    )
    print(f"[repeat ok] {command}: completed_steps={completed_steps}")
    return True


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


def offset_summary(state: SessionState) -> str:
    return (
        "right_offset=("
        f"dx={state.current_right_dx:+.6f}, "
        f"dy={state.current_right_dy:+.6f}, "
        f"dz={state.current_right_dz:+.6f}) "
        "left_offset=("
        f"dx={state.current_left_dx:+.6f}, "
        f"dy={state.current_left_dy:+.6f}, "
        f"dz={state.current_left_dz:+.6f})"
    )


def print_current_offset(state: SessionState) -> None:
    print("[current offset]")
    print(
        "right: "
        f"dx={state.current_right_dx:+.6f}, "
        f"dy={state.current_right_dy:+.6f}, "
        f"dz={state.current_right_dz:+.6f}"
    )
    print(
        "left:  "
        f"dx={state.current_left_dx:+.6f}, "
        f"dy={state.current_left_dy:+.6f}, "
        f"dz={state.current_left_dz:+.6f}"
    )


def fmt_state_value(value: Optional[float]) -> str:
    if value is None:
        return "?"
    return f"{value:+.6f}"


def handle_show_state(state: SessionState, args: argparse.Namespace) -> None:
    print_current_offset(state)
    print("[tracked joints]")
    print(
        "right_wrist: "
        f"yaw={fmt_state_value(state.current_right_wrist_yaw)}, "
        f"pitch={fmt_state_value(state.current_right_wrist_pitch)}, "
        f"roll={fmt_state_value(state.current_right_wrist_roll)}"
    )
    print(
        "left_wrist:  "
        f"yaw={fmt_state_value(state.current_left_wrist_yaw)}, "
        f"pitch={fmt_state_value(state.current_left_wrist_pitch)}, "
        f"roll={fmt_state_value(state.current_left_wrist_roll)}"
    )
    print(f"right_elbow_pitch={fmt_state_value(state.current_right_elbow_pitch)}")
    print(f"right_shoulder_pitch={fmt_state_value(state.current_right_shoulder_pitch)}")
    print(f"right_shoulder_roll={fmt_state_value(state.current_right_shoulder_roll)}")
    print(f"left_elbow_pitch={fmt_state_value(state.current_left_elbow_pitch)}")
    print(f"left_shoulder_pitch={fmt_state_value(state.current_left_shoulder_pitch)}")
    print(f"left_shoulder_roll={fmt_state_value(state.current_left_shoulder_roll)}")
    print(
        "gripper: "
        f"right={fmt_state_value(state.current_right_gripper_position)}, "
        f"left={fmt_state_value(state.current_left_gripper_position)}"
    )
    print(f"log_path: {pretty_path(state.log_path)}")
    print("[suggested replay]")
    for command in suggested_replay_commands(state, args):
        print(f"  {command}")
    append_log(
        state,
        Action(command="show_state", command_type="show_state"),
        user_confirmed=None,
        status="shown",
        observed_alignment=state.last_observed_alignment,
        user_observation=offset_summary(state),
    )


def handle_save(parts: list[str], state: SessionState) -> None:
    if len(parts) > 1:
        note = " ".join(parts[1:]).strip()
    else:
        note = input("save note> ").strip()
    summary = offset_summary(state)
    user_observation = f"{note} | {summary}" if note else summary
    append_log(
        state,
        Action(command="save", command_type="save"),
        user_confirmed=None,
        status="saved",
        observed_alignment=state.last_observed_alignment,
        user_observation=user_observation,
    )
    print_current_offset(state)
    print("[tracked joints]")
    print(
        "right_wrist: "
        f"yaw={fmt_state_value(state.current_right_wrist_yaw)}, "
        f"pitch={fmt_state_value(state.current_right_wrist_pitch)}, "
        f"roll={fmt_state_value(state.current_right_wrist_roll)}"
    )
    print(
        "left_wrist:  "
        f"yaw={fmt_state_value(state.current_left_wrist_yaw)}, "
        f"pitch={fmt_state_value(state.current_left_wrist_pitch)}, "
        f"roll={fmt_state_value(state.current_left_wrist_roll)}"
    )
    print(f"right_elbow_pitch={fmt_state_value(state.current_right_elbow_pitch)}")
    print(f"right_shoulder_pitch={fmt_state_value(state.current_right_shoulder_pitch)}")
    print(f"left_elbow_pitch={fmt_state_value(state.current_left_elbow_pitch)}")
    print(f"left_shoulder_pitch={fmt_state_value(state.current_left_shoulder_pitch)}")
    print("[save] 当前标定备注已写入 CSV。")


def handle_save_pose(parts: list[str], args: argparse.Namespace, state: SessionState) -> None:
    if len(parts) != 2:
        print("用法: save_pose <candidate_name>")
        return
    candidate_name = parts[1].strip()
    if not CANDIDATE_NAME_RE.fullmatch(candidate_name):
        print("[blocked] candidate_name 只允许字母、数字、下划线和短横线。")
        return

    note = state.last_user_observation
    snapshot = state_snapshot(state, args, candidate_name=candidate_name, user_note=note)
    state.candidate_path.parent.mkdir(parents=True, exist_ok=True)
    with state.candidate_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(snapshot, ensure_ascii=False, separators=(",", ":")) + "\n")

    append_log(
        state,
        Action(command="save_pose", command_type="save_pose"),
        user_confirmed=None,
        status="saved",
        observed_alignment=state.last_observed_alignment,
        user_observation=offset_summary(state),
        candidate_name=candidate_name,
        candidate_file=pretty_path(state.candidate_path),
    )
    print(f"[saved candidate] {candidate_name}")
    print_current_offset(state)
    print("suggested replay:")
    for command in snapshot["suggested_replay_commands"]:
        print(f"  {command}")


def handle_list_candidates(state: SessionState) -> None:
    if not state.candidate_path.exists():
        print(f"[list_candidates] no candidate file: {pretty_path(state.candidate_path)}")
        return
    lines = state.candidate_path.read_text(encoding="utf-8").splitlines()[-10:]
    print(f"[list_candidates] last {len(lines)} from {pretty_path(state.candidate_path)}")
    for line in lines:
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        right_offset = (
            f"({item.get('right_offset_dx', '?'):+.6f}, "
            f"{item.get('right_offset_dy', '?'):+.6f}, "
            f"{item.get('right_offset_dz', '?'):+.6f})"
            if isinstance(item.get("right_offset_dx"), (int, float))
            else "(?, ?, ?)"
        )
        left_offset = (
            f"({item.get('left_offset_dx', '?'):+.6f}, "
            f"{item.get('left_offset_dy', '?'):+.6f}, "
            f"{item.get('left_offset_dz', '?'):+.6f})"
            if isinstance(item.get("left_offset_dx"), (int, float))
            else "(?, ?, ?)"
        )
        commands = "; ".join(item.get("suggested_replay_commands", []))
        print(
            f"- {item.get('candidate_name', '?')} {item.get('timestamp', '?')} "
            f"right={right_offset} left={left_offset} replay={commands}"
        )


def known_commands() -> set[str]:
    return (
        {
            "help",
            "quit",
            "exit",
            "q",
            "show_state",
            "obs",
            "save",
            "save_pose",
            "list_candidates",
            "grip",
            "left_open",
            "left_grip",
            "left_loose",
            "left_tight",
            "right_open",
            "right_grip",
            "right_loose",
            "right_tight",
            "right_set_roll",
            "right_set_pitch",
            "right_set_yaw",
            "right_set_elbow",
            "right_set_shoulder_pitch",
            "left_set_shoulder_pitch",
            "left_set_elbow",
            "left_set_shoulder_roll",
            "left_set_wrist_pitch",
            "left_set_wrist_roll",
            "left_set_roll",
            "left_set_pitch",
            "left_set_yaw",
            "roll0",
            "roll03",
            "roll05",
            "roll07",
            "yaw+",
            "yaw-",
            "pitch+",
            "pitch-",
        }
        | REPEATABLE_SMALL_STEP_COMMANDS
        | RIGHT_REPLAY_STAGES.keys()
        | RIGHT_REPLAY_STAGE_ALIASES.keys()
        | LEFT_REPLAY_STAGES.keys()
        | DEPRECATED_RIGHT_STAGE_COMMANDS
        | {
            "left_shoulder_pitch+",
            "left_shoulder_pitch-",
            "left_elbow+",
            "left_elbow-",
            "left_shoulder_roll+",
            "left_shoulder_roll-",
            "left_wrist_pitch+",
            "left_wrist_pitch-",
            "left_wrist_roll+",
            "left_wrist_roll-",
        }
    )


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
        topic = parts[1].lower() if len(parts) > 1 else "short"
        if topic not in {"short", "all", "right", "left", "replay"}:
            print(f"[help] unknown topic: {topic}; use help all/right/left/replay")
            topic = "short"
        print_help(topic)
        return True
    if (
        len(parts) == 2
        and command in {"save", "quit", "exit", "q", "obs", "show_state", "save_pose"}
        and (parts[1].isdigit() or parts[1].startswith("-"))
        and handle_repeat_command(parts, robot, args, state)
    ):
        return True
    if command in {"quit", "exit", "q"}:
        return False
    if command == "show_state":
        handle_show_state(state, args)
        return True
    if command == "obs":
        handle_observation(parts, state)
        return True
    if command == "save":
        handle_save(parts, state)
        return True
    if command == "save_pose":
        handle_save_pose(parts, args, state)
        return True
    if command == "list_candidates":
        handle_list_candidates(state)
        return True
    if len(parts) == 2 and handle_repeat_command(parts, robot, args, state):
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
    if command in {
        "right_roll+",
        "right_roll-",
        "right_pitch+",
        "right_pitch-",
        "right_yaw+",
        "right_yaw-",
        "right_set_roll",
        "right_set_pitch",
        "right_set_yaw",
    }:
        try:
            action = build_right_wrist_adjust_action(parts, args, state)
        except ValueError as exc:
            print(f"[blocked] {exc}")
            append_log(
                state,
                Action(command=" ".join(parts), command_type="right_wrist_adjust", arm="right"),
                user_confirmed=None,
                status=f"blocked: {exc}",
            )
            return True
        execute_action(robot, action, args, state)
        return True
    if command in {
        "right_elbow+",
        "right_elbow-",
        "right_shoulder_pitch+",
        "right_shoulder_pitch-",
        "right_set_elbow",
        "right_set_shoulder_pitch",
    }:
        try:
            action = build_right_arm_clearance_action(parts, args, state)
        except ValueError as exc:
            print(f"[blocked] {exc}")
            append_log(
                state,
                Action(
                    command=" ".join(parts),
                    command_type="right_arm_clearance_adjust",
                    arm="right",
                ),
                user_confirmed=None,
                status=f"blocked: {exc}",
            )
            return True
        execute_action(robot, action, args, state)
        return True
    if command in {
        "left_set_shoulder_pitch",
        "left_set_elbow",
        "left_set_shoulder_roll",
        "left_set_wrist_pitch",
        "left_set_wrist_roll",
        "left_shoulder_pitch+",
        "left_shoulder_pitch-",
        "left_elbow+",
        "left_elbow-",
        "left_shoulder_roll+",
        "left_shoulder_roll-",
        "left_wrist_pitch+",
        "left_wrist_pitch-",
        "left_wrist_roll+",
        "left_wrist_roll-",
    }:
        try:
            action = build_left_arm_pour_adjust_action(parts, args, state)
        except ValueError as exc:
            print(f"[blocked] {exc}")
            append_log(
                state,
                Action(
                    command=" ".join(parts),
                    command_type="left_arm_pour_adjust",
                    arm="left",
                ),
                user_confirmed=None,
                status=f"blocked: {exc}",
            )
            return True
        execute_action(robot, action, args, state)
        return True
    if command in {
        "left_roll+",
        "left_roll-",
        "left_pitch+",
        "left_pitch-",
        "left_yaw+",
        "left_yaw-",
        "left_set_roll",
        "left_set_pitch",
        "left_set_yaw",
    }:
        try:
            action = build_left_wrist_adjust_action(parts, args, state)
        except ValueError as exc:
            print(f"[blocked] {exc}")
            append_log(
                state,
                Action(
                    command=" ".join(parts),
                    command_type="left_wrist_adjust",
                    arm="left",
                ),
                user_confirmed=None,
                status=f"blocked: {exc}",
            )
            return True
        execute_action(robot, action, args, state)
        return True
    if command in RIGHT_REPLAY_STAGES or command in RIGHT_REPLAY_STAGE_ALIASES or command in LEFT_REPLAY_STAGES:
        execute_replay_stage(command, robot, args, state)
        return True
    if command in DEPRECATED_RIGHT_STAGE_COMMANDS:
        handle_deprecated_right_stage(command, state)
        return True
    if command in {"x+", "x-", "y+", "y-", "z+", "z-"}:
        print(f"[alias] {command} 等价于 left_{command}")
        execute_action(robot, build_linear_action(command, args), args, state)
        return True
    if command in {
        "left_x+",
        "left_x-",
        "left_y+",
        "left_y-",
        "left_z+",
        "left_z-",
        "right_x+",
        "right_x-",
        "right_y+",
        "right_y-",
        "right_z+",
        "right_z-",
    }:
        execute_action(robot, build_linear_action(command, args), args, state)
        return True
    if command in {"roll0", "roll03", "roll05", "roll07"}:
        try:
            action = build_roll_action(command, args)
        except ValueError as exc:
            print(f"[blocked] {exc}")
            append_log(
                state,
                Action(command=command, command_type="left_wrist_adjust", arm="left"),
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

    suggestion = difflib.get_close_matches(command, sorted(known_commands()), n=1, cutoff=0.78)
    if suggestion:
        print(f"未知命令: {command}；是否想输入 {suggestion[0]}？输入 help 查看命令。")
    else:
        print(f"未知命令: {command}；输入 help 查看命令。")
    append_log(
        state,
        Action(command=command, command_type="unknown"),
        user_confirmed=None,
        status="unknown_command",
    )
    return True


MAIN_MENU = """
================ Pour Align Calib ================
[1] 右手流程复现 / Right replay stages
[2] 右手夹爪 / Right gripper
[3] 右手接奶位微调 / Right pour-ready adjust
[4] 左手夹爪 / Left gripper
[5] 左手空壶对杯口 / Left alignment
[6] 记录与日志 / Observation & save
[7] 专家命令帮助 / Expert command help
[q] 退出
也可以直接输入专家命令，例如 replay_right_pour_ready / x+ / obs ...
""".strip()


SUBMENUS = {
    "1": {
        "title": "右手流程复现 / Right replay stages",
        "items": [
            ("1", "replay_right_grasp_cup", "replay_right_grasp_cup"),
            ("2", "replay_right_move_to_coffee_machine", "replay_right_move_to_coffee_machine"),
            (
                "3",
                "replay_right_retreat_after_coffee  [contains right_arm.home()]",
                "replay_right_retreat_after_coffee",
            ),
            ("4", "replay_right_pour_ready", "replay_right_pour_ready"),
        ],
    },
    "2": {
        "title": "右手夹爪 / Right gripper",
        "items": [
            ("1", "right_open", "right_open"),
            ("2", "right_grip", "right_grip"),
            ("3", "right_loose", "right_loose"),
            ("4", "right_tight", "right_tight"),
        ],
    },
    "3": {
        "title": "右手接奶位微调 / Right pour-ready adjust",
        "items": [
            ("1", "right_roll+", "right_roll+"),
            ("2", "right_roll-", "right_roll-"),
            ("3", "right_pitch+", "right_pitch+"),
            ("4", "right_pitch-", "right_pitch-"),
            ("5", "right_yaw+", "right_yaw+"),
            ("6", "right_yaw-", "right_yaw-"),
            ("7", "right_x+", "right_x+"),
            ("8", "right_x-", "right_x-"),
            ("9", "right_y+", "right_y+"),
            ("10", "right_y-", "right_y-"),
            ("11", "right_z+", "right_z+"),
            ("12", "right_z-", "right_z-"),
            ("13", "right_elbow+", "right_elbow+"),
            ("14", "right_elbow-", "right_elbow-"),
            ("15", "right_shoulder_pitch+", "right_shoulder_pitch+"),
            ("16", "right_shoulder_pitch-", "right_shoulder_pitch-"),
            ("17", "输入 right_set_roll <value>", "right_set_roll"),
            ("18", "输入 right_set_pitch <value>", "right_set_pitch"),
            ("19", "输入 right_set_yaw <value>", "right_set_yaw"),
            ("20", "输入 right_set_elbow <value>", "right_set_elbow"),
            ("21", "输入 right_set_shoulder_pitch <value>", "right_set_shoulder_pitch"),
        ],
    },
    "4": {
        "title": "左手夹爪 / Left gripper",
        "items": [
            ("1", "left_open", "left_open"),
            ("2", "left_grip", "left_grip"),
            ("3", "left_loose", "left_loose"),
            ("4", "left_tight", "left_tight"),
        ],
    },
    "5": {
        "title": "左手空壶对杯口 / Left alignment",
        "items": [
            ("1", "replay_left_move_to_pour_pose_left_only", "replay_left_move_to_pour_pose_left_only"),
            ("2", "replay_left_pour_prep_frame", "replay_left_pour_prep_frame"),
            ("3", "输入 left_set_shoulder_pitch <value>", "left_set_shoulder_pitch"),
            ("4", "输入 left_set_elbow <value>", "left_set_elbow"),
            ("5", "输入 left_set_shoulder_roll <value>", "left_set_shoulder_roll"),
            ("6", "输入 left_set_wrist_pitch <value>", "left_set_wrist_pitch"),
            ("7", "输入 left_set_wrist_roll <value>", "left_set_wrist_roll"),
            ("8", "left_shoulder_pitch+", "left_shoulder_pitch+"),
            ("9", "left_shoulder_pitch-", "left_shoulder_pitch-"),
            ("10", "left_elbow+", "left_elbow+"),
            ("11", "left_elbow-", "left_elbow-"),
            ("12", "left_shoulder_roll+", "left_shoulder_roll+"),
            ("13", "left_shoulder_roll-", "left_shoulder_roll-"),
            ("14", "left_x+", "left_x+"),
            ("15", "left_x-", "left_x-"),
            ("16", "left_y+", "left_y+"),
            ("17", "left_y-", "left_y-"),
            ("18", "left_z+", "left_z+"),
            ("19", "left_z-", "left_z-"),
            ("20", "left_roll+", "left_roll+"),
            ("21", "left_roll-", "left_roll-"),
            ("22", "left_pitch+", "left_pitch+"),
            ("23", "left_pitch-", "left_pitch-"),
            ("24", "left_yaw+", "left_yaw+"),
            ("25", "left_yaw-", "left_yaw-"),
            ("26", "输入 left_set_roll <value>", "left_set_roll"),
            ("27", "输入 left_set_pitch <value>", "left_set_pitch"),
            ("28", "输入 left_set_yaw <value>", "left_set_yaw"),
            ("29", "roll0", "roll0"),
            ("30", "roll03", "roll03"),
            ("31", "roll05", "roll05"),
            ("32", "roll07", "roll07"),
            ("33", "yaw+  [compat]", "yaw+"),
            ("34", "yaw-  [compat]", "yaw-"),
            ("35", "pitch+  [compat]", "pitch+"),
            ("36", "pitch-  [compat]", "pitch-"),
        ],
    },
    "6": {
        "title": "记录与日志 / Observation & save",
        "items": [
            ("1", "obs", "obs"),
            ("2", "show_state", "show_state"),
            ("3", "save [note]", "save"),
            ("4", "输入 save_pose <candidate_name>", "save_pose"),
            ("5", "list_candidates", "list_candidates"),
            ("6", "显示当前 log_path", "__show_log_path__"),
        ],
    },
    "7": {
        "title": "专家命令帮助 / Expert command help",
        "items": [
            ("1", "help short", "help"),
            ("2", "help all", "help all"),
            ("3", "help right", "help right"),
            ("4", "help left", "help left"),
            ("5", "help replay", "help replay"),
        ],
    },
}


VALUE_PROMPTS = {
    "right_set_roll": "请输入 right wrist_roll target，例如 0.10：",
    "right_set_pitch": "请输入 right wrist_pitch target，例如 0.10：",
    "right_set_yaw": "请输入 right wrist_yaw target，例如 -0.70：",
    "right_set_elbow": "请输入 right elbow_pitch target，例如 0.05：",
    "right_set_shoulder_pitch": "请输入 right shoulder_pitch target，例如 -0.05：",
    "left_set_shoulder_pitch": "请输入 left shoulder_pitch target，例如 -0.35：",
    "left_set_elbow": "请输入 left elbow_pitch target，例如 -0.42：",
    "left_set_shoulder_roll": "请输入 left shoulder_roll target，例如 0.50：",
    "left_set_wrist_pitch": "请输入 left wrist_pitch target，例如 -0.45：",
    "left_set_wrist_roll": "请输入 left wrist_roll target，例如 0.70：",
    "left_set_roll": "请输入 left wrist_roll target，例如 0.30：",
    "left_set_pitch": "请输入 left wrist_pitch target，例如 -0.20：",
    "left_set_yaw": "请输入 left wrist_yaw target，例如 0.10：",
    "save_pose": "请输入 candidate_name，只能包含字母、数字、下划线、短横线：",
}


def print_main_menu() -> None:
    print(MAIN_MENU)


def print_submenu(menu_key: str) -> None:
    menu = SUBMENUS[menu_key]
    print(f"================ {menu['title']} ================")
    for key, label, _command in menu["items"]:
        print(f"{key} {label}")
    print("b 返回主菜单")
    print("q 退出")


def submenu_command_from_choice(menu_key: str, choice: str, state: SessionState) -> Optional[str]:
    menu = SUBMENUS[menu_key]
    command_by_key = {key: command for key, _label, command in menu["items"]}
    command = command_by_key.get(choice)
    if command is None:
        return None
    if command == "__show_log_path__":
        print(f"log_path: {pretty_path(state.log_path)}")
        return ""
    prompt = VALUE_PROMPTS.get(command)
    if prompt is None:
        return command
    value = input(prompt).strip()
    if not value:
        print("[skip] 未输入数值，已取消。")
        return ""
    return f"{command} {value}"


def repl_loop(robot, args: argparse.Namespace, state: SessionState) -> None:
    active_menu: Optional[str] = None
    print_main_menu()
    while True:
        try:
            prompt = "pour-align> " if active_menu is None else f"pour-align:{active_menu}> "
            raw_line = input(prompt)
        except EOFError:
            print()
            break
        line = raw_line.strip()
        if not line:
            continue
        lowered = line.lower()
        if active_menu is not None:
            if lowered in {"b", "back"}:
                active_menu = None
                print_main_menu()
                continue
            if lowered in {"q", "quit", "exit"}:
                break
            mapped_command = submenu_command_from_choice(active_menu, lowered, state)
            if mapped_command is None:
                if not process_command(line, robot, args, state):
                    break
                continue
            if not mapped_command:
                continue
            if not process_command(mapped_command, robot, args, state):
                break
            continue

        if lowered in SUBMENUS:
            active_menu = lowered
            print_submenu(active_menu)
            continue
        if not process_command(line, robot, args, state):
            break


def main() -> int:
    args = parse_args()
    validate_args(args)

    log_path = resolve_log_path(args.log_file)
    state = SessionState(
        log_path=log_path,
        candidate_path=CANDIDATE_FILE,
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
