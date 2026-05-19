"""
Mantis IK tuning console.

警告：
- 当前脚本可能控制真实机器人。
- 第一次必须 dry-run。
- 第一次实机必须空载。
- 不要放杯子、奶壶、液体、咖啡机障碍物。
- 必须有人看护急停。
- README 示例点只是示例，不等于安全点。
"""

from __future__ import annotations

import argparse
import csv
import json
import shlex
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

from connection_selector import add_connection_args, connect_robot_with_selector


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_ROBOT_VERSION = "3.0"
DEFAULT_MAX_DELTA = 0.005
DEFAULT_MAX_ROTATION_DELTA = 0.03
DEFAULT_AXIS_DELTA = 0.003
CONTROL_MODE_CHOICES = ("relative", "absolute", "mixed")
README_EXAMPLE_POSE = (0.5, 0.2, 0.3, 0.0, 0.0, 0.0)
SAFE_CANDIDATES_PATH = PROJECT_ROOT / "logs" / "ik_safe_candidates.json"
RISK_OBSERVATIONS = {"bad", "near_collision", "jitter", "abort", "危险/中止"}
UNSAFE_OBSERVATION_KEYWORDS = (
    "near_collision",
    "collision",
    "撞",
    "碰",
    "卡",
    "顶住",
    "抖动",
    "异响",
    "jitter",
    "bad",
    "abort",
    "dangerous",
    "risk",
    "不安全",
    "危险",
)
SAFE_OBSERVATIONS = {"ok", "safe", "1", "平稳", "正常", "安全、平稳"}
COMMAND_KEYWORDS = {
    "help",
    "guide",
    "demo",
    "mode",
    "readme_abs",
    "abs",
    "rel",
    "axis",
    "summary",
    "save",
    "status",
    "last",
    "note",
    "quit",
}
OBSERVATION_CHOICES = (
    "ok",
    "bad",
    "near_collision",
    "no_motion",
    "wrong_direction",
    "jitter",
    "no_solution",
    "abort",
)
ABS_OBSERVATION_MENU = {
    "1": "ok",
    "2": "bad",
    "3": "near_collision",
    "4": "no_motion",
    "5": "jitter",
    "6": "no_solution",
    "7": "abort",
}
AXIS_DIRECTION_MENU = {
    "1": "向身体内侧",
    "2": "向身体外侧",
    "3": "向左",
    "4": "向右",
    "5": "向上",
    "6": "向下",
    "7": "几乎不动",
    "8": "方向不确定",
    "9": "危险/中止",
}
AXIS_SAFETY_MENU = {
    "1": "safe",
    "2": "unsafe",
    "3": "jitter",
    "4": "blocked",
    "5": "uncertain",
}
AXIS_SAFETY_DESCRIPTIONS = {
    "safe": "安全、平稳",
    "unsafe": "不安全、接近碰撞",
    "jitter": "抖动/异响",
    "blocked": "卡住/顶住",
    "uncertain": "不确定",
}
EXPECTED_AXIS_DIRECTIONS = {
    "+X": "向身体内侧",
    "-X": "向身体外侧",
    "+Y": "向左",
    "-Y": "向右",
    "+Z": "向上",
    "-Z": "向下",
}
CSV_FIELDNAMES = [
    "timestamp",
    "trial_id",
    "arm",
    "command_type",
    "abs_mode",
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
    "block",
    "status",
    "error",
    "duration_s",
    "observed_direction",
    "safety_status",
    "user_observation",
    "note",
]


class DryRunArm:
    def __init__(self, side: str) -> None:
        self.side = side

    def ik(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        *,
        block: bool = True,
        abs: bool = True,
    ) -> None:
        _ = (x, y, z, roll, pitch, yaw, block, abs)
        return None


class DryRunRobot:
    def __init__(self) -> None:
        self.left_arm = DryRunArm("left")
        self.right_arm = DryRunArm("right")

    def stop(self) -> None:
        return None

    def disconnect(self) -> None:
        return None


@dataclass(frozen=True)
class IkCommand:
    arm: str
    command_type: str
    abs_mode: bool
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    block: bool = True
    note: str = ""
    label: str = ""

    def call_repr(self) -> str:
        return (
            f"robot.{self.arm}_arm.ik("
            f"{self.x:.6f}, {self.y:.6f}, {self.z:.6f}, "
            f"{self.roll:.6f}, {self.pitch:.6f}, {self.yaw:.6f}, "
            f"block={self.block}, abs={self.abs_mode})"
        )


@dataclass
class TrialResult:
    timestamp: str
    trial_id: str
    arm: str
    command_type: str
    abs_mode: Optional[bool]
    x: Optional[float]
    y: Optional[float]
    z: Optional[float]
    roll: Optional[float]
    pitch: Optional[float]
    yaw: Optional[float]
    block: Optional[bool]
    status: str
    error: str
    duration_s: float
    user_observation: str
    note: str
    observed_direction: str = ""
    safety_status: str = ""


@dataclass(frozen=True)
class ObservationRecord:
    user_observation: str = ""
    observed_direction: str = ""
    safety_status: str = ""


@dataclass
class SessionState:
    log_path: Path
    dry_run: bool
    arm_name: str
    ik_control_mode: str
    current_candidate_abs: Optional[IkCommand] = None
    last_ok_abs_command: Optional[IkCommand] = None
    last_abs_ok_pose: Optional[IkCommand] = None
    last_result: Optional[TrialResult] = None
    pending_line: Optional[str] = None
    saved_candidate_names: list[str] = field(default_factory=list)
    trial_counter: int = 0
    stop_requested: bool = False
    demo_attempted: bool = False
    demo_passed: bool = False
    abs_attempted: bool = False
    abs_ok: bool = False
    abs_failed: bool = False
    abs_last_observation: str = ""
    abs_safety_status: str = "unknown"
    axis_attempted: bool = False
    axis_completed: bool = False
    axis_mode: str = ""
    axis_observations: dict[str, str] = field(default_factory=dict)
    axis_observed_directions: dict[str, str] = field(default_factory=dict)
    axis_safety_statuses: dict[str, str] = field(default_factory=dict)
    axis_results: dict[str, str] = field(default_factory=dict)
    risk_detected: bool = False
    saved_candidate_path: Optional[Path] = None
    last_error: str = ""
    last_recommendation: str = ""
    anchor_ready: bool = False
    anchor_source: str = ""
    relative_steps_ok: bool = False

    def next_trial_id(self) -> str:
        self.trial_counter += 1
        return f"T{self.trial_counter:04d}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mantis 3.0 左右臂 IK 调试控制台")
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印动作计划并记录日志，不连接机器人",
    )
    mode_group.add_argument(
        "--execute",
        action="store_true",
        help="允许连接机器人并执行 IK；必须配合 --i-understand-real-robot-risk",
    )
    parser.add_argument(
        "--i-understand-real-robot-risk",
        action="store_true",
        help="二次确认已知晓实机风险；仅与 --execute 同时传入时有效",
    )
    parser.add_argument(
        "--arm",
        choices=("left", "right"),
        required=True,
        help="选择要调试的手臂",
    )
    parser.add_argument(
        "--use-readme-example",
        action="store_true",
        help="启动时把 README 示例点载入为当前候选绝对点，但不会自动执行",
    )
    parser.add_argument(
        "--max-delta",
        type=float,
        default=DEFAULT_MAX_DELTA,
        help=f"相对平移增量单轴绝对值上限，默认 {DEFAULT_MAX_DELTA}",
    )
    parser.add_argument(
        "--allow-large-delta",
        action="store_true",
        help="允许 rel / axis 使用超过 --max-delta 的平移增量",
    )
    parser.add_argument(
        "--allow-large-rotation",
        action="store_true",
        help=f"允许相对姿态增量单轴绝对值超过 {DEFAULT_MAX_ROTATION_DELTA} rad",
    )
    parser.add_argument(
        "--ask-observation-dry-run",
        action="store_true",
        help="dry-run 下也询问 observation；默认 dry-run 自动记录 user_observation=dry_run",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="打印详细 target/call/trial 调试信息",
    )
    parser.add_argument(
        "--expert",
        action="store_true",
        help="启用英文命令模式；默认进入数字菜单向导模式",
    )
    parser.add_argument(
        "--ik-control-mode",
        choices=CONTROL_MODE_CHOICES,
        default="relative",
        help="IK 控制方式：relative / absolute / mixed，默认 relative",
    )
    parser.add_argument(
        "--log-file",
        default=None,
        help="CSV 日志路径；不指定时自动写入 logs/ik_tune_trials_YYYYmmdd_HHMMSS.csv",
    )
    parser.add_argument(
        "--self-check-observation-safety",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    add_connection_args(parser, default_profile="interactive")
    parser.set_defaults(robot_version=DEFAULT_ROBOT_VERSION)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.max_delta <= 0.0:
        raise ValueError("--max-delta 必须大于 0")

    if not args.execute:
        args.dry_run = True

    if args.execute and not args.i_understand_real_robot_risk:
        raise SystemExit(
            "拒绝执行：实机模式必须同时传入 --execute 和 "
            "--i-understand-real-robot-risk。"
        )


def resolve_log_path(log_file: Optional[str]) -> Path:
    if log_file:
        path = Path(log_file).expanduser()
        if not path.is_absolute():
            path = PROJECT_ROOT / path
        return path
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return PROJECT_ROOT / "logs" / f"ik_tune_trials_{timestamp}.csv"


def pretty_path(path: Path) -> str:
    try:
        return str(path.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def mode_label(dry_run: bool) -> str:
    return "DRY-RUN" if dry_run else "EXECUTE"


def parse_observation_safety(user_observation: str) -> str:
    observation = user_observation.strip()
    if not observation:
        return "unknown"
    normalized = observation.lower()
    if "safety=" in normalized:
        safety_value = normalized.rsplit("safety=", 1)[1].split(";", 1)[0].strip()
        if safety_value in {"safe", "unsafe"}:
            return safety_value
        if safety_value in {"jitter", "blocked", "uncertain"}:
            return "unsafe" if safety_value in {"jitter", "blocked"} else "unknown"
    if normalized in SAFE_OBSERVATIONS or observation in SAFE_OBSERVATIONS:
        return "safe"
    if any(keyword in normalized for keyword in UNSAFE_OBSERVATION_KEYWORDS):
        return "unsafe"
    return "unknown"


def sdk_status_ok(status: str) -> bool:
    return status in {
        "ok",
        "dry_run",
        "sdk_ok_safety_safe",
        "sdk_ok_safety_unsafe",
        "sdk_ok_safety_unknown",
    }


def sdk_status_error(status: str) -> bool:
    return status in {"error", "sdk_error"}


def compose_result_status(sdk_status: str, safety_status: str) -> str:
    if sdk_status == "dry_run":
        return "dry_run"
    if sdk_status == "ok":
        if safety_status == "safe":
            return "sdk_ok_safety_safe"
        if safety_status == "unsafe":
            return "sdk_ok_safety_unsafe"
        return "sdk_ok_safety_unknown"
    if sdk_status == "error":
        return "sdk_error"
    return sdk_status


def normalize_safety_for_status(safety_status: str) -> str:
    if safety_status == "safe":
        return "safe"
    if safety_status in {"unsafe", "jitter", "blocked"}:
        return "unsafe"
    return "unknown"


def print_unsafe_warning() -> None:
    print("[UNSAFE]")
    print("本次 IK 动作不安全，已阻止后续 axis/rel/save。")
    print("请退出并检查机器人姿态/日志。")


def init_csv_log(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    if log_path.exists():
        return
    with log_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()


def append_csv_log(log_path: Path, result: TrialResult) -> None:
    row = {
        "timestamp": result.timestamp,
        "trial_id": result.trial_id,
        "arm": result.arm,
        "command_type": result.command_type,
        "abs_mode": _format_optional_bool(result.abs_mode),
        "x": _format_optional_float(result.x),
        "y": _format_optional_float(result.y),
        "z": _format_optional_float(result.z),
        "roll": _format_optional_float(result.roll),
        "pitch": _format_optional_float(result.pitch),
        "yaw": _format_optional_float(result.yaw),
        "block": _format_optional_bool(result.block),
        "status": result.status,
        "error": result.error,
        "duration_s": f"{result.duration_s:.6f}",
        "observed_direction": result.observed_direction,
        "safety_status": result.safety_status,
        "user_observation": result.user_observation,
        "note": result.note,
    }
    with log_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDNAMES)
        writer.writerow(row)


def connect_robot(args: argparse.Namespace):
    if getattr(args, "print_connection_config", False):
        connect_robot_with_selector(args, script_name=__file__)
        return None
    if args.execute:
        return connect_robot_with_selector(args, script_name=__file__)
    return DryRunRobot()


def select_arm(robot, arm_name: str):
    if arm_name == "left":
        return robot.left_arm
    if arm_name == "right":
        return robot.right_arm
    raise ValueError(f"Unsupported arm: {arm_name}")


def vprint(args: argparse.Namespace, *lines: str) -> None:
    if not args.verbose:
        return
    for line in lines:
        print(line)


def build_readme_abs_command(arm_name: str) -> IkCommand:
    x, y, z, roll, pitch, yaw = README_EXAMPLE_POSE
    return IkCommand(
        arm=arm_name,
        command_type="readme_abs",
        abs_mode=True,
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        block=True,
        note="README example candidate only",
        label="README_EXAMPLE",
    )


def build_abs_command(arm_name: str, values: list[float]) -> IkCommand:
    x, y, z, roll, pitch, yaw = values
    return IkCommand(
        arm=arm_name,
        command_type="abs",
        abs_mode=True,
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        block=True,
        note="manual abs target",
        label="MANUAL_ABS",
    )


def build_rel_command(arm_name: str, values: list[float], note: str = "") -> IkCommand:
    x, y, z, roll, pitch, yaw = values
    return IkCommand(
        arm=arm_name,
        command_type="rel",
        abs_mode=False,
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        block=True,
        note=note or "manual relative delta",
        label="MANUAL_REL",
    )


def build_axis_commands(arm_name: str, delta: float) -> list[IkCommand]:
    if delta <= 0.0:
        raise ValueError("axis delta 必须大于 0")

    axis_specs = [
        ("+X", (delta, 0.0, 0.0, 0.0, 0.0, 0.0)),
        ("-X", (-delta, 0.0, 0.0, 0.0, 0.0, 0.0)),
        ("+Y", (0.0, delta, 0.0, 0.0, 0.0, 0.0)),
        ("-Y", (0.0, -delta, 0.0, 0.0, 0.0, 0.0)),
        ("+Z", (0.0, 0.0, delta, 0.0, 0.0, 0.0)),
        ("-Z", (0.0, 0.0, -delta, 0.0, 0.0, 0.0)),
    ]
    commands: list[IkCommand] = []
    for label, values in axis_specs:
        commands.append(
            IkCommand(
                arm=arm_name,
                command_type="axis",
                abs_mode=False,
                x=values[0],
                y=values[1],
                z=values[2],
                roll=values[3],
                pitch=values[4],
                yaw=values[5],
                block=True,
                note=f"axis probe {label}",
                label=label,
            )
        )
    return commands


def current_candidate_name(state: SessionState) -> str:
    if state.current_candidate_abs is None:
        return "NONE"
    return state.current_candidate_abs.label or "ABS_CANDIDATE"


def default_save_name(state: SessionState) -> str:
    if state.ik_control_mode == "relative":
        return f"{state.arm_name}_relative_axis_result_01"
    return f"{state.arm_name}_readme_safe_01"


def last_abs_pose_text(state: SessionState) -> str:
    if state.last_abs_ok_pose is None:
        return "-"
    return f"{state.last_abs_ok_pose.label} {format_pose_short(state.last_abs_ok_pose)}"


def format_pose_short(command: IkCommand) -> str:
    return (
        f"x={command.x:.3f} y={command.y:.3f} z={command.z:.3f} "
        f"r={command.roll:.3f} p={command.pitch:.3f} y={command.yaw:.3f}"
    )


def format_pose_full(command: IkCommand) -> str:
    return (
        f"x={command.x:.6f}, y={command.y:.6f}, z={command.z:.6f}, "
        f"roll={command.roll:.6f}, pitch={command.pitch:.6f}, yaw={command.yaw:.6f}"
    )


def format_delta_short(command: IkCommand) -> str:
    return (
        f"dx={command.x:+.3f} dy={command.y:+.3f} dz={command.z:+.3f} "
        f"dr={command.roll:+.3f} dp={command.pitch:+.3f} dy={command.yaw:+.3f}"
    )


def format_delta_compact(label: str, command: IkCommand) -> str:
    if label in {"+X", "-X"}:
        return f"{label} dx={command.x:+.3f}m"
    if label in {"+Y", "-Y"}:
        return f"{label} dy={command.y:+.3f}m"
    return f"{label} dz={command.z:+.3f}m"


def print_compact_safety_notice() -> None:
    print("实机会控制真实机器人，第一次必须空载、有人看护急停。")
    print("README 示例点不是安全点，实机执行 ABS 前仍需 RUN_ABS。")


def current_stage_text(state: SessionState) -> str:
    if state.ik_control_mode == "relative":
        if state.risk_detected:
            return "relative 风险复核"
        if not state.relative_steps_ok:
            return "relative 小步验证"
        return "relative 验证完成"
    if state.ik_control_mode == "absolute":
        if state.risk_detected:
            return "absolute 风险复核"
        if not state.abs_ok:
            return "absolute 绝对点验证"
        return "absolute 绝对点验证完成"
    if state.dry_run and not state.demo_passed:
        return "mixed dry-run 检查"
    if state.risk_detected:
        return "mixed 风险复核"
    if not state.abs_ok:
        return "mixed 先 absolute 建基准"
    if not state.relative_steps_ok:
        return "mixed relative 探测方向"
    return "mixed 验证完成"


def print_startup_banner(state: SessionState) -> None:
    print_compact_safety_notice()
    print(
        f"[IK Tune] {mode_label(state.dry_run)} | arm={state.arm_name} | "
        f"candidate={current_candidate_name(state)}"
    )
    print(f"log: {pretty_path(state.log_path)}")
    print(f"当前阶段：{current_stage_text(state)}")
    print("推荐输入：")
    print("  demo")
    print("常用：")
    print("  help    查看命令")
    print("  quit    退出")


def is_command_like_line(line: str) -> bool:
    stripped = line.strip()
    if not stripped:
        return False
    return stripped.split(maxsplit=1)[0].lower() in COMMAND_KEYWORDS


def maybe_redirect_commandlike_input(user_input: str, state: SessionState) -> Optional[str]:
    if not is_command_like_line(user_input):
        return None
    redirect = input(
        "你输入的内容看起来像下一条命令，而不是观察结果。"
        "是否把它作为命令执行？[y/N] "
    ).strip().lower()
    if redirect == "y":
        state.pending_line = user_input
        print(f"已将该输入转为下一条待执行命令: {user_input}")
        return "observation_skipped_command_redirect"
    return None


def prompt_generic_observation(state: SessionState) -> str:
    print("请输入本次观察结果，直接回车表示跳过；这里不是命令输入区。")
    print("观察结果建议：" + ", ".join(OBSERVATION_CHOICES) + "，或自定义备注")
    user_input = input("observation> ").strip()
    if not user_input:
        return ""
    redirected = maybe_redirect_commandlike_input(user_input, state)
    if redirected is not None:
        return redirected
    return user_input


def prompt_abs_observation(state: SessionState) -> str:
    print("观察结果：")
    print("  1 = ok，平稳到达")
    print("  2 = bad，姿态不合适")
    print("  3 = near_collision，接近碰撞")
    print("  4 = no_motion，没有明显运动")
    print("  5 = jitter，抖动/异响")
    print("  6 = no_solution，IK失败/无解")
    print("  7 = abort，中止后续")
    user_input = input("请输入编号或备注：").strip()
    if not user_input:
        return ""
    redirected = maybe_redirect_commandlike_input(user_input, state)
    if redirected is not None:
        return redirected
    return ABS_OBSERVATION_MENU.get(user_input, user_input)


def prompt_axis_observation(state: SessionState) -> ObservationRecord:
    print("实际运动方向：")
    print("  1 = 向身体内侧")
    print("  2 = 向身体外侧")
    print("  3 = 向左")
    print("  4 = 向右")
    print("  5 = 向上")
    print("  6 = 向下")
    print("  7 = 几乎不动")
    print("  8 = 方向不确定")
    print("  9 = 危险/中止")
    user_input = input("请输入编号或备注：").strip()
    if not user_input:
        return ObservationRecord()
    redirected = maybe_redirect_commandlike_input(user_input, state)
    if redirected is not None:
        return ObservationRecord(user_observation=redirected)
    observed_direction = AXIS_DIRECTION_MENU.get(user_input, user_input)

    print("安全状态：")
    print("  1 = 安全、平稳")
    print("  2 = 不安全、接近碰撞")
    print("  3 = 抖动/异响")
    print("  4 = 卡住/顶住")
    print("  5 = 不确定")
    safety_input = input("请输入编号或备注：").strip()
    if not safety_input:
        safety_status = "unknown"
        safety_note = ""
    else:
        redirected = maybe_redirect_commandlike_input(safety_input, state)
        if redirected is not None:
            return ObservationRecord(
                user_observation=redirected,
                observed_direction=observed_direction,
            )
        safety_status = AXIS_SAFETY_MENU.get(safety_input, "uncertain")
        safety_note = AXIS_SAFETY_DESCRIPTIONS.get(safety_status, safety_input)
    if safety_input and safety_input not in AXIS_SAFETY_MENU:
        safety_note = safety_input
    user_observation = (
        f"direction={observed_direction}; "
        f"safety={safety_status}; "
        f"note={safety_note or '-'}"
    )
    return ObservationRecord(
        user_observation=user_observation,
        observed_direction=observed_direction,
        safety_status=safety_status,
    )


def should_prompt_observation(args: argparse.Namespace, state: SessionState) -> bool:
    if not state.dry_run:
        return True
    return bool(args.ask_observation_dry_run)


def prompt_float_input(prompt: str, *, default: Optional[float] = None) -> float:
    while True:
        raw = input(prompt).strip()
        if not raw:
            if default is not None:
                return default
            print("该项不能为空，请输入数字。")
            continue
        try:
            return float(raw)
        except ValueError:
            print("请输入数字。")


def prompt_interactive_abs_command(state: SessionState) -> IkCommand:
    print("请输入绝对 IK 点：")
    x = prompt_float_input("请输入 x(m)：")
    y = prompt_float_input("请输入 y(m)：")
    z = prompt_float_input("请输入 z(m)：")
    roll = prompt_float_input("请输入 roll(rad)，默认 0：", default=0.0)
    pitch = prompt_float_input("请输入 pitch(rad)，默认 0：", default=0.0)
    yaw = prompt_float_input("请输入 yaw(rad)，默认 0：", default=0.0)
    return build_abs_command(state.arm_name, [x, y, z, roll, pitch, yaw])


def prompt_interactive_rel_command(state: SessionState) -> IkCommand:
    print("请输入相对 IK 增量：")
    dx = prompt_float_input("请输入 dx(m)，直接回车为 0：", default=0.0)
    dy = prompt_float_input("请输入 dy(m)，直接回车为 0：", default=0.0)
    dz = prompt_float_input("请输入 dz(m)，直接回车为 0：", default=0.0)
    droll = prompt_float_input("请输入 droll(rad)，直接回车为 0：", default=0.0)
    dpitch = prompt_float_input("请输入 dpitch(rad)，直接回车为 0：", default=0.0)
    dyaw = prompt_float_input("请输入 dyaw(rad)，直接回车为 0：", default=0.0)
    return build_rel_command(state.arm_name, [dx, dy, dz, droll, dpitch, dyaw])


def validate_relative_command(command: IkCommand, args: argparse.Namespace) -> None:
    if command.abs_mode:
        return

    if not args.allow_large_delta:
        for axis_name, value in (("dx", command.x), ("dy", command.y), ("dz", command.z)):
            if abs(value) > args.max_delta:
                raise ValueError(
                    f"{axis_name}={value:.6f} 超过 --max-delta={args.max_delta:.6f}；"
                    "如确需执行，请显式传入 --allow-large-delta。"
                )

    if not args.allow_large_rotation:
        for axis_name, value in (
            ("droll", command.roll),
            ("dpitch", command.pitch),
            ("dyaw", command.yaw),
        ):
            if abs(value) > DEFAULT_MAX_ROTATION_DELTA:
                raise ValueError(
                    f"{axis_name}={value:.6f} 超过默认姿态增量限制 "
                    f"{DEFAULT_MAX_ROTATION_DELTA:.6f} rad；"
                    "如确需执行，请显式传入 --allow-large-rotation。"
                )


def record_result(
    state: SessionState,
    *,
    arm: str,
    command_type: str,
    abs_mode: Optional[bool],
    x: Optional[float],
    y: Optional[float],
    z: Optional[float],
    roll: Optional[float],
    pitch: Optional[float],
    yaw: Optional[float],
    block: Optional[bool],
    status: str,
    error: str = "",
    duration_s: float = 0.0,
    user_observation: str = "",
    observed_direction: str = "",
    safety_status: str = "",
    note: str = "",
) -> TrialResult:
    result = TrialResult(
        timestamp=now_iso(),
        trial_id=state.next_trial_id(),
        arm=arm,
        command_type=command_type,
        abs_mode=abs_mode,
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        block=block,
        status=status,
        error=error,
        duration_s=duration_s,
        user_observation=user_observation,
        note=note,
        observed_direction=observed_direction,
        safety_status=safety_status,
    )
    append_csv_log(state.log_path, result)
    state.last_result = result
    return result


def execute_ik(arm, command: IkCommand, args: argparse.Namespace, state: SessionState) -> tuple[str, str, float]:
    start_time = time.perf_counter()
    status = "dry_run" if state.dry_run else "ok"
    error = ""
    vprint(args, f"[verbose] call: {command.call_repr()}")
    try:
        arm.ik(
            command.x,
            command.y,
            command.z,
            command.roll,
            command.pitch,
            command.yaw,
            block=command.block,
            abs=command.abs_mode,
        )
    except Exception as exc:  # pragma: no cover - only hit with real SDK/runtime errors
        status = "error"
        error = str(exc)
    duration_s = time.perf_counter() - start_time
    return status, error, duration_s


def resolve_observation(
    command: IkCommand,
    args: argparse.Namespace,
    state: SessionState,
    status: str,
    preset_observation: Optional[str],
) -> ObservationRecord:
    if not sdk_status_ok(status):
        return ObservationRecord()
    if preset_observation is not None:
        return observation_record_from_preset(command, preset_observation)
    if not should_prompt_observation(args, state):
        if state.dry_run:
            return ObservationRecord(
                user_observation="dry_run",
                observed_direction="dry_run" if command.command_type == "axis" else "",
                safety_status="safe" if command.command_type == "axis" else "",
            )
        return ObservationRecord()
    if state.dry_run:
        user_observation = prompt_generic_observation(state)
        return ObservationRecord(
            user_observation=user_observation,
            safety_status=parse_observation_safety(user_observation),
        )
    if command.command_type == "axis":
        return prompt_axis_observation(state)
    if command.abs_mode:
        user_observation = prompt_abs_observation(state)
    else:
        user_observation = prompt_generic_observation(state)
    return ObservationRecord(
        user_observation=user_observation,
        safety_status=parse_observation_safety(user_observation),
    )


def observation_record_from_preset(command: IkCommand, preset_observation: str) -> ObservationRecord:
    if command.command_type == "axis" and preset_observation.startswith("dry_run_axis_"):
        return ObservationRecord(
            user_observation=preset_observation,
            observed_direction="dry_run",
            safety_status="safe",
        )
    return ObservationRecord(
        user_observation=preset_observation,
        safety_status=parse_observation_safety(preset_observation),
    )


def classify_axis_result(
    label: str,
    observed_direction: str,
    safety_status: str,
    user_observation: str,
    dry_run: bool,
) -> str:
    if not observed_direction and not user_observation:
        return "missing"
    if user_observation == "observation_skipped_command_redirect":
        return "redirected"
    if dry_run:
        if user_observation.startswith("dry_run_axis_") or user_observation == "dry_run":
            return "moved"
        if normalize_safety_for_status(safety_status) == "unsafe":
            return "danger"
        return "noted"
    if observed_direction == "危险/中止":
        return "danger"
    if normalize_safety_for_status(safety_status) != "safe":
        return "danger"
    if observed_direction == "几乎不动":
        return "no_effect"
    if observed_direction == "方向不确定":
        return "direction_uncertain"
    if observed_direction:
        return "moved"
    return "missing"


def axis_all_ok(state: SessionState) -> bool:
    expected_labels = set(EXPECTED_AXIS_DIRECTIONS)
    if state.dry_run:
        return expected_labels.issubset(state.axis_results) and all(
            state.axis_results[label] == "moved" for label in expected_labels
        )
    return expected_labels.issubset(state.axis_results) and all(
        state.axis_results[label] in {"moved", "no_effect"}
        for label in expected_labels
    )


def update_state_after_command(
    command: IkCommand,
    result: TrialResult,
    state: SessionState,
) -> None:
    observation = result.user_observation
    safety_status = result.safety_status or parse_observation_safety(observation)
    normalized_safety = normalize_safety_for_status(safety_status)
    if normalized_safety == "unsafe" or observation in RISK_OBSERVATIONS:
        state.risk_detected = True
        print_unsafe_warning()
    if command.command_type in {"readme_abs", "abs"}:
        state.abs_attempted = True
        state.abs_last_observation = observation
        state.abs_safety_status = normalized_safety
        if not state.dry_run and sdk_status_ok(result.status) and normalized_safety == "safe":
            state.abs_ok = True
            state.abs_failed = False
            state.last_ok_abs_command = command
            state.last_abs_ok_pose = command
            state.anchor_ready = True
            state.anchor_source = f"{command.label or command.command_type} abs ok"
            state.last_error = ""
        elif not state.dry_run and sdk_status_error(result.status):
            state.abs_ok = False
            state.abs_failed = True
            state.anchor_ready = False
            state.last_error = result.error or "SDK IK execution failed"
        elif not state.dry_run and sdk_status_ok(result.status) and normalized_safety == "unsafe":
            state.abs_ok = False
            state.abs_failed = True
            state.anchor_ready = False
            state.risk_detected = True
            state.last_error = (
                "SDK IK executed but user reported unsafe motion: "
                f"{observation}"
            )
            state.last_recommendation = "停止，不要继续 axis/rel；该 ABS 点不可作为 anchor"
        elif not state.dry_run and sdk_status_ok(result.status) and normalized_safety == "unknown":
            state.abs_ok = False
            state.abs_failed = True
            state.anchor_ready = False
            state.last_recommendation = "观察结果不明确，不能建立 anchor；请人工复核"
    if command.command_type == "axis":
        state.axis_attempted = True
        state.axis_mode = mode_label(state.dry_run).lower()
        observed_direction = result.observed_direction
        if state.dry_run and not observed_direction:
            observed_direction = "dry_run"
        state.axis_observations[command.label] = observation or "-"
        state.axis_observed_directions[command.label] = observed_direction or "-"
        state.axis_safety_statuses[command.label] = safety_status or "unknown"
        state.axis_results[command.label] = classify_axis_result(
            command.label,
            observed_direction,
            safety_status,
            observation,
            state.dry_run,
        )
        if not state.dry_run and (
            observed_direction == "危险/中止" or normalized_safety != "safe"
        ):
            if observed_direction == "危险/中止":
                state.axis_results[command.label] = "danger"
            already_risky = state.risk_detected
            state.risk_detected = True
            state.stop_requested = True
            if not already_risky:
                print_unsafe_warning()
    if command.command_type == "rel" and sdk_status_ok(result.status) and observation in {"ok", "dry_run"}:
        state.relative_steps_ok = True


def prompt_switch_control_mode(state: SessionState) -> bool:
    print("请选择 IK 控制方式：")
    print("  1. relative 相对控制优先，推荐拉花小步调试")
    print("  2. absolute 绝对控制，用于测试目标点")
    print("  3. mixed 先绝对建立基准，再相对探测")
    print("  0. 取消")
    user_input = input("请选择 > ").strip()
    mode_map = {
        "1": "relative",
        "2": "absolute",
        "3": "mixed",
    }
    if user_input == "0":
        return False
    selected = mode_map.get(user_input)
    if selected is None:
        print("输入无效，保持当前控制方式。")
        return False
    state.ik_control_mode = selected
    print(f"已切换到 IK 控制方式：{selected}")
    update_last_recommendation(state)
    print_next_action_hint(state)
    return True


def run_logged_command(
    arm,
    command: IkCommand,
    args: argparse.Namespace,
    state: SessionState,
    *,
    preset_observation: Optional[str] = None,
) -> TrialResult:
    try:
        validate_relative_command(command, args)
    except ValueError as exc:
        result = record_result(
            state,
            arm=command.arm,
            command_type=command.command_type,
            abs_mode=command.abs_mode,
            x=command.x,
            y=command.y,
            z=command.z,
            roll=command.roll,
            pitch=command.pitch,
            yaw=command.yaw,
            block=command.block,
            status="rejected",
            error=str(exc),
            note=command.note,
        )
        vprint(args, f"[verbose] rejected: {exc}")
        return result

    sdk_status, error, duration_s = execute_ik(arm, command, args, state)
    observation = resolve_observation(command, args, state, sdk_status, preset_observation)
    safety_status = observation.safety_status or parse_observation_safety(observation.user_observation)
    status_safety = normalize_safety_for_status(safety_status)
    if observation.observed_direction == "危险/中止":
        status_safety = "unsafe"
    status = compose_result_status(sdk_status, status_safety)
    result = record_result(
        state,
        arm=command.arm,
        command_type=command.command_type,
        abs_mode=command.abs_mode,
        x=command.x,
        y=command.y,
        z=command.z,
        roll=command.roll,
        pitch=command.pitch,
        yaw=command.yaw,
        block=command.block,
        status=status,
        error=error,
        duration_s=duration_s,
        user_observation=observation.user_observation,
        observed_direction=observation.observed_direction,
        safety_status=safety_status,
        note=command.note,
    )
    update_state_after_command(command, result, state)
    if observation.user_observation == "abort":
        state.stop_requested = True
        state.risk_detected = True
    vprint(
        args,
        f"[verbose] target: {format_pose_full(command) if command.abs_mode else format_delta_short(command)}",
        f"[verbose] trial={result.trial_id} status={result.status} "
        f"duration={result.duration_s:.3f}s observation={result.user_observation or '-'}",
    )
    return result


def confirm_abs_execute(command: IkCommand, args: argparse.Namespace) -> bool:
    if args.verbose:
        print(f"[verbose] full target: {format_pose_full(command)}")
        print(f"[verbose] call: {command.call_repr()}")
    print(f"[ABS] {command.label}")
    print(f"target: {format_pose_short(command)}")
    if command.label == "README_EXAMPLE":
        print("")
        print("检查：")
        print("1. 左臂空载")
        print("2. 周围无遮挡")
        print("3. 急停可触达")
        print("4. 不放杯子/奶壶/液体")
    user_input = input("\n确认执行请输入 RUN_ABS，输入 q 取消：").strip()
    if user_input == "RUN_ABS":
        return True
    print("已取消本次 ABS。")
    return False


def confirm_rel_execute(command: IkCommand, args: argparse.Namespace, prompt: str) -> bool:
    if args.verbose:
        print(f"[verbose] delta: {format_delta_short(command)}")
        print(f"[verbose] call: {command.call_repr()}")
    user_input = input(prompt).strip().lower()
    if user_input == "q":
        return False
    return True


def print_relative_mode_notice() -> None:
    print("当前将直接使用 SDK 的相对 IK abs=False，小步长测试。")
    print("请确认左臂当前姿态安全、周围无遮挡。")


def ensure_relative_motion_allowed(arm, args: argparse.Namespace, state: SessionState) -> bool:
    if state.risk_detected:
        print("检测到不安全动作，停止 IK 调试。")
        print("不要继续 axis/rel；请退出并检查机器人姿态/日志。")
        return False
    if state.ik_control_mode == "relative":
        print_relative_mode_notice()
        return True
    return ensure_relative_anchor_ready(arm, args, state)


def ensure_relative_anchor_ready(arm, args: argparse.Namespace, state: SessionState) -> bool:
    if state.risk_detected:
        print("检测到不安全动作，停止 IK 调试。")
        print("不要继续 axis/rel；请退出并检查机器人姿态/日志。")
        return False
    if state.anchor_ready:
        return True

    print("[RELATIVE] 还没有确认相对控制基准点。")
    print("相对 IK abs=False 会基于 SDK 内部维护的目标位姿累积。")
    print("建议先选择：")
    print("  1. 用 README ABS 建立基准 readme_abs")
    print("  2. 手动输入 ABS 点建立基准 abs ...")
    print("  3. 我确认当前 SDK 内部目标已经安全，可继续相对控制")
    print("  0. 取消")

    while True:
        user_input = input("请选择 > ").strip()
        if user_input == "0":
            return False
        if user_input == "1":
            run_abs_flow(arm, args, state, build_readme_abs_command(state.arm_name))
            return state.anchor_ready
        if user_input == "2":
            run_abs_flow(arm, args, state, prompt_interactive_abs_command(state))
            return state.anchor_ready
        if user_input == "3":
            confirm = input(
                "如确认当前 SDK 内部目标已可作为安全基准，请输入 USE_CURRENT_ANCHOR："
            ).strip()
            if confirm == "USE_CURRENT_ANCHOR":
                state.anchor_ready = True
                state.anchor_source = "USER_CONFIRMED_CURRENT_TARGET"
                print("已确认当前 SDK 内部目标可作为相对控制基准。")
                return True
            print("未确认基准，保持未就绪。")
            return False
        print("输入无效，请重新选择。")


def describe_abs_outcome(result: TrialResult, state: SessionState) -> tuple[str, str]:
    if sdk_status_ok(result.status):
        if state.dry_run:
            return "PASS", "yes"
        safety_status = normalize_safety_for_status(
            result.safety_status or parse_observation_safety(result.user_observation)
        )
        if safety_status == "safe":
            return "PASS", "yes"
        if safety_status == "unsafe":
            return "STOP", "no"
        return "REVIEW", "no"
    if result.status == "skipped":
        return "SKIP", "no"
    return "FAIL", "no"


def print_readme_abs_result(result: TrialResult, command: IkCommand, state: SessionState) -> None:
    safety_status = normalize_safety_for_status(
        result.safety_status or parse_observation_safety(result.user_observation)
    )
    sdk_label = "OK" if sdk_status_ok(result.status) else "ERROR"
    if result.status == "skipped":
        sdk_label = "SKIPPED"
    safety_label = safety_status.upper()
    can_continue = "yes" if sdk_status_ok(result.status) and (state.dry_run or safety_status == "safe") else "no"
    print(f"[ABS] {command.label}")
    print(f"target: {format_pose_short(command)}")
    print(f"mode: {mode_label(state.dry_run).lower()}")
    print(f"sdk: {sdk_label}")
    print(f"safety: {safety_label}")
    print(f"continue: {can_continue}")
    if state.dry_run:
        print("next: execute 模式下空载测试时，需要输入 RUN_ABS")
    else:
        if safety_status == "safe":
            print("next: axis 0.003")
        elif safety_status == "unsafe":
            print("next: 停止，不要执行 axis/rel；更换 ABS 点或改用相对小步前先建立安全 anchor")
        else:
            print("next: 观察结果不明确，不能建立 anchor；请人工复核")


def run_abs_flow(arm, args: argparse.Namespace, state: SessionState, command: IkCommand) -> None:
    state.current_candidate_abs = command
    if state.dry_run:
        if args.verbose:
            print(f"[verbose] full target: {format_pose_full(command)}")
            print(f"[verbose] call: {command.call_repr()}")
        result = run_logged_command(arm, command, args, state)
        print_readme_abs_result(result, command, state)
        update_last_recommendation(state)
        return

    if not confirm_abs_execute(command, args):
        result = record_result(
            state,
            arm=command.arm,
            command_type=command.command_type,
            abs_mode=command.abs_mode,
            x=command.x,
            y=command.y,
            z=command.z,
            roll=command.roll,
            pitch=command.pitch,
            yaw=command.yaw,
            block=command.block,
            status="skipped",
            error="user_declined_confirmation",
            note=command.note,
        )
        state.last_result = result
        print("result: SKIP")
        print("continue: no")
        update_last_recommendation(state)
        return

    result = run_logged_command(arm, command, args, state)
    print_readme_abs_result(result, command, state)
    update_last_recommendation(state)


def demo_step_passed(result: TrialResult) -> bool:
    return sdk_status_ok(result.status)


def run_demo(arm, args: argparse.Namespace, state: SessionState) -> None:
    if not state.dry_run:
        print("demo 只允许在 dry-run 下自动运行。")
        print("实机模式请手动执行 readme_abs / axis 0.003，并保持每一步人工确认。")
        print_next_action_hint(state)
        return

    state.demo_attempted = True
    state.axis_attempted = False
    state.axis_completed = False
    state.axis_observations.clear()
    state.axis_observed_directions.clear()
    state.axis_safety_statuses.clear()
    state.axis_results.clear()
    state.relative_steps_ok = False

    step_results: list[tuple[str, bool]] = []
    print("[DEMO] dry-run 检查开始")

    if state.ik_control_mode == "relative":
        print("relative 模式 dry-run：仅测试 abs=False 的 XYZ 小步命令。")
        axis_commands = build_axis_commands(state.arm_name, DEFAULT_AXIS_DELTA)
        for axis_command in axis_commands:
            result = run_logged_command(
                arm,
                axis_command,
                args,
                state,
                preset_observation=f"dry_run_axis_{axis_command.label}_ok",
            )
            step_results.append((f"{axis_command.label} 0.003m", demo_step_passed(result)))
    elif state.ik_control_mode == "absolute":
        readme_command = build_readme_abs_command(state.arm_name)
        state.current_candidate_abs = readme_command
        readme_result = run_logged_command(
            arm,
            readme_command,
            args,
            state,
            preset_observation="dry_run_readme_abs_ok",
        )
        step_results.append(("README ABS", demo_step_passed(readme_result)))
    else:
        readme_command = build_readme_abs_command(state.arm_name)
        state.current_candidate_abs = readme_command
        readme_result = run_logged_command(
            arm,
            readme_command,
            args,
            state,
            preset_observation="dry_run_readme_abs_ok",
        )
        step_results.append(("README ABS", demo_step_passed(readme_result)))
        axis_commands = build_axis_commands(state.arm_name, DEFAULT_AXIS_DELTA)
        for axis_command in axis_commands:
            result = run_logged_command(
                arm,
                axis_command,
                args,
                state,
                preset_observation=f"dry_run_axis_{axis_command.label}_ok",
            )
            step_results.append((f"{axis_command.label} 0.003m", demo_step_passed(result)))

    state.demo_passed = all(passed for _, passed in step_results)
    if state.ik_control_mode != "absolute":
        state.axis_completed = state.demo_passed
        state.relative_steps_ok = state.demo_passed
    for index, (label, passed) in enumerate(step_results, start=1):
        dots = "." * max(1, 20 - len(label))
        print(f"  {index}/{len(step_results)} {label} {dots} {'PASS' if passed else 'FAIL'}")
    print("")
    if state.demo_passed:
        print("结论：dry-run 命令链正常")
        print("continue: yes")
    else:
        print("结论：dry-run 命令链异常")
        print("continue: no")
    print(f"日志：{pretty_path(state.log_path)}")
    if state.ik_control_mode == "relative":
        print("下一步：execute 下直接用 axis 0.003 记录真实方向。")
    elif state.ik_control_mode == "absolute":
        print("下一步：absolute 模式下可空载测试 readme_abs。")
    else:
        print("下一步：先验证安全门槛，再做实机空载 readme_abs。")
    update_last_recommendation(state)


def format_axis_result_label(result: str) -> str:
    mapping = {
        "moved": "moved",
        "danger": "danger",
        "no_effect": "no_effect",
        "direction_uncertain": "direction_uncertain",
        "redirected": "review",
        "missing": "review",
        "noted": "noted",
    }
    return mapping.get(result, result)


def format_axis_safety_label(safety_status: str) -> str:
    mapping = {
        "safe": "safe",
        "unsafe": "unsafe",
        "jitter": "jitter",
        "blocked": "blocked",
        "uncertain": "uncertain",
        "unknown": "unknown",
    }
    return mapping.get(safety_status or "unknown", safety_status or "unknown")


def summarize_axis_pair(state: SessionState, axis_name: str) -> str:
    plus = state.axis_results.get(f"+{axis_name}", "missing")
    minus = state.axis_results.get(f"-{axis_name}", "missing")

    def is_usable(result: str) -> bool:
        return result == "moved"

    def is_no_effect(result: str) -> bool:
        return result == "no_effect"

    if is_usable(plus) and is_usable(minus):
        return "usable"
    if is_no_effect(plus) and is_no_effect(minus):
        return "no_effect"
    parts: list[str] = []
    for prefix, result in (("+", plus), ("-", minus)):
        if is_usable(result):
            parts.append(f"{prefix}{axis_name} usable")
        elif is_no_effect(result):
            parts.append(f"{prefix}{axis_name} no_effect")
        else:
            parts.append(f"{prefix}{axis_name} {format_axis_result_label(result)}")
    return "partial, " + ", ".join(parts)


def print_axis_summary(state: SessionState, delta: float) -> None:
    print("[AXIS SUMMARY]")
    print("cmd   observed      safety   effect")
    for label in ("+X", "-X", "+Y", "-Y", "+Z", "-Z"):
        observed = state.axis_observed_directions.get(label, "-")
        safety = format_axis_safety_label(state.axis_safety_statuses.get(label, "unknown"))
        effect = format_axis_result_label(state.axis_results.get(label, "missing"))
        print(f"{label:<4} {observed:<12} {safety:<8} {effect}")
    print("")
    print("usable_relative_axes:")
    for axis_name in ("X", "Y", "Z"):
        print(f"  {axis_name}: {summarize_axis_pair(state, axis_name)}")
    print("")
    if state.risk_detected or any(
        state.axis_results.get(label) == "danger" for label in EXPECTED_AXIS_DIRECTIONS
    ):
        print("结论：")
        print("- 不建议继续。请停止 relative IK 调试。")
        print("推荐下一步：")
        print("  stop")
        return
    if axis_all_ok(state):
        print("结论：")
        print("- 坐标方向已记录")
        print("- 若无危险项，可 save 当前 relative_axis_result")
        print("推荐下一步：")
        print(f"  save {default_save_name(state)}")
        return
    if state.axis_completed and not state.risk_detected:
        print("结论：")
        print("- 测试结果已记录，部分方向需复核")
        print("- 若无危险项，可 save 当前 relative_axis_result")
        print("推荐下一步：")
        print(f"  save {default_save_name(state)}")
        return
    print("结论：")
    print("- 方向记录不完整或与预期不一致")
    print("- 建议先复核结果，再决定是否重试")
    print("推荐下一步：")
    print("  summary")


def execute_axis_sequence(arm, args: argparse.Namespace, state: SessionState, delta: float) -> None:
    commands = build_axis_commands(state.arm_name, delta)
    state.axis_attempted = True
    state.axis_completed = False
    state.relative_steps_ok = False
    state.axis_mode = mode_label(state.dry_run).lower()
    state.axis_observations.clear()
    state.axis_observed_directions.clear()
    state.axis_safety_statuses.clear()
    state.axis_results.clear()

    if state.dry_run:
        print(f"[AXIS] delta={delta:.3f}m | mode=dry-run")
        all_passed = True
        for axis_command in commands:
            result = run_logged_command(arm, axis_command, args, state)
            passed = sdk_status_ok(result.status)
            all_passed = all_passed and passed
            print(f"  {axis_command.label} .... {'PASS' if passed else 'FAIL'}")
        state.axis_completed = all_passed
        state.relative_steps_ok = all_passed
        print("")
        print("结论：axis dry-run 正常" if all_passed else "结论：axis dry-run 异常")
        print(f"continue: {'yes' if all_passed else 'no'}")
        print("下一步：execute 下用 axis 0.003 记录真实方向")
        update_last_recommendation(state)
        return

    for index, axis_command in enumerate(commands, start=1):
        print(f"[AXIS {index}/6] {format_delta_compact(axis_command.label, axis_command)}")
        print("本步将调用：")
        print(axis_command.call_repr())
        print("")
        print("检查：")
        print("1. 左臂当前姿态安全")
        print("2. 周围无遮挡")
        print("3. 急停可触达")
        print(f"4. 本步只有 {abs(axis_command.x or axis_command.y or axis_command.z) * 1000:.0f}mm 小位移")
        print("")
        if not confirm_rel_execute(
            axis_command,
            args,
            "确认执行？Enter=执行，q=停止 axis\n",
        ):
            result = record_result(
                state,
                arm=axis_command.arm,
                command_type=axis_command.command_type,
                abs_mode=axis_command.abs_mode,
                x=axis_command.x,
                y=axis_command.y,
                z=axis_command.z,
                roll=axis_command.roll,
                pitch=axis_command.pitch,
                yaw=axis_command.yaw,
                block=axis_command.block,
                status="skipped",
                error="user_declined_confirmation",
                note=axis_command.note,
            )
            state.last_result = result
            state.axis_observations[axis_command.label] = "skip"
            state.axis_observed_directions[axis_command.label] = "-"
            state.axis_safety_statuses[axis_command.label] = "unknown"
            state.axis_results[axis_command.label] = "review"
            break
        result = run_logged_command(arm, axis_command, args, state)
        if state.risk_detected:
            break
        if result.user_observation == "observation_skipped_command_redirect":
            break
        if state.pending_line is not None:
            break

    state.axis_completed = (
        set(EXPECTED_AXIS_DIRECTIONS).issubset(state.axis_results)
        and not state.risk_detected
    )
    state.relative_steps_ok = axis_all_ok(state) and not state.risk_detected
    print_axis_summary(state, delta)
    update_last_recommendation(state)


def save_candidate(name: str, state: SessionState) -> TrialResult:
    if not can_save_safe_point(state):
        print("当前没有可保存的安全结果，或已经检测到风险，不能保存。")
        return record_result(
            state,
            arm=state.arm_name,
            command_type="save",
            abs_mode=None,
            x=None,
            y=None,
            z=None,
            roll=None,
            pitch=None,
            yaw=None,
            block=None,
            status="error",
            error="no_safe_result_available",
            note=f"save {name}",
        )

    if state.ik_control_mode == "relative":
        SAFE_CANDIDATES_PATH.parent.mkdir(parents=True, exist_ok=True)
        candidates = load_safe_candidates()
        candidates[name] = {
            "timestamp": now_iso(),
            "arm": state.arm_name,
            "ik_control_mode": "relative",
            "result_type": "relative_axis_result",
            "usable_relative_axes": {
                axis_name: summarize_axis_pair(state, axis_name)
                for axis_name in ("X", "Y", "Z")
            },
            "axis_observations": dict(state.axis_observations),
            "axis_observed_directions": dict(state.axis_observed_directions),
            "axis_safety_statuses": dict(state.axis_safety_statuses),
            "axis_results": dict(state.axis_results),
            "note": "relative axis direction and safety result",
        }
        SAFE_CANDIDATES_PATH.write_text(
            json.dumps(candidates, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        if name not in state.saved_candidate_names:
            state.saved_candidate_names.append(name)
        state.saved_candidate_path = SAFE_CANDIDATES_PATH
        print(f"saved: {pretty_path(SAFE_CANDIDATES_PATH)}")
        result = record_result(
            state,
            arm=state.arm_name,
            command_type="save",
            abs_mode=False,
            x=None,
            y=None,
            z=None,
            roll=None,
            pitch=None,
            yaw=None,
            block=None,
            status="saved",
            note=f"save {name}",
        )
        print_next_action_hint(state)
        return result

    source_command = state.last_ok_abs_command or state.current_candidate_abs
    if source_command is None:
        print("当前没有可保存的绝对点候选。")
        return record_result(
            state,
            arm=state.arm_name,
            command_type="save",
            abs_mode=None,
            x=None,
            y=None,
            z=None,
            roll=None,
            pitch=None,
            yaw=None,
            block=None,
            status="error",
            error="no_absolute_candidate_available",
            note=f"save {name}",
        )

    SAFE_CANDIDATES_PATH.parent.mkdir(parents=True, exist_ok=True)
    candidates = load_safe_candidates()
    candidates[name] = {
        "timestamp": now_iso(),
        "arm": source_command.arm,
        "x": source_command.x,
        "y": source_command.y,
        "z": source_command.z,
        "roll": source_command.roll,
        "pitch": source_command.pitch,
        "yaw": source_command.yaw,
        "block": source_command.block,
        "abs_mode": source_command.abs_mode,
        "source_command_type": source_command.command_type,
        "note": source_command.note,
    }
    SAFE_CANDIDATES_PATH.write_text(
        json.dumps(candidates, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if name not in state.saved_candidate_names:
        state.saved_candidate_names.append(name)
    state.saved_candidate_path = SAFE_CANDIDATES_PATH
    print(f"saved: {pretty_path(SAFE_CANDIDATES_PATH)}")
    result = record_result(
        state,
        arm=source_command.arm,
        command_type="save",
        abs_mode=source_command.abs_mode,
        x=source_command.x,
        y=source_command.y,
        z=source_command.z,
        roll=source_command.roll,
        pitch=source_command.pitch,
        yaw=source_command.yaw,
        block=source_command.block,
        status="saved",
        note=f"save {name}",
    )
    print_next_action_hint(state)
    return result


def load_safe_candidates() -> dict[str, object]:
    if not SAFE_CANDIDATES_PATH.exists():
        return {}
    try:
        data = json.loads(SAFE_CANDIDATES_PATH.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    return data if isinstance(data, dict) else {}


def log_note(note_text: str, state: SessionState) -> TrialResult:
    print(f"noted: {note_text}")
    return record_result(
        state,
        arm=state.arm_name,
        command_type="note",
        abs_mode=None,
        x=None,
        y=None,
        z=None,
        roll=None,
        pitch=None,
        yaw=None,
        block=None,
        status="noted",
        note=note_text,
    )


def get_guide_lines(state: SessionState) -> list[str]:
    if state.risk_detected:
        return [
            "1. 检测到不安全动作，停止 IK 调试",
            "2. 不要继续 axis / rel / save",
            "3. 请退出并检查机器人姿态/日志",
        ]
    if state.ik_control_mode == "relative":
        if not state.relative_steps_ok:
            return [
                "1. 直接输入 axis 0.003",
                "2. 每步确认方向和安全状态",
                "3. 或手动输入 rel 小步调试",
            ]
        return [
            "1. relative 小步验证已完成",
            f"2. 可输入 save {default_save_name(state)}",
            "3. 然后退出或切到 mixed / absolute 继续",
        ]
    if state.ik_control_mode == "absolute":
        if not state.abs_ok:
            return [
                "1. 输入 readme_abs",
                "2. 或手动输入 abs 点",
                "3. 实机前仍需 RUN_ABS",
            ]
        return [
            "1. absolute 点验证完成",
            f"2. 可输入 save {default_save_name(state)}",
            "3. 或切到 relative / mixed 继续",
        ]
    if state.dry_run and not state.demo_passed:
        return [
            "1. 输入 demo，检查命令链和日志",
            "2. 检查 summary 是否 PASS",
            "3. 再切到 execute 做空载实机",
        ]
    if not state.abs_ok:
        return [
            "1. 输入 readme_abs",
            "2. 观察左臂是否平稳到达",
            "3. 根据提示输入 ok/bad/abort",
        ]
    return [
        "1. 输入 axis 0.003",
        "2. 记录实际方向和安全状态",
        f"3. 若均安全，输入 save {default_save_name(state)}",
    ]


def can_save_safe_point(state: SessionState) -> bool:
    if state.ik_control_mode == "relative":
        return bool(state.axis_completed and not state.risk_detected)
    return bool(
        state.anchor_ready
        and not state.risk_detected
        and state.abs_ok
        and state.last_ok_abs_command is not None
    )


def get_menu_recommendation(state: SessionState) -> tuple[str, str]:
    if state.risk_detected:
        return "0", "检测到不安全动作，停止 IK 调试"
    if state.ik_control_mode == "relative":
        if state.dry_run:
            return "2", "测试 XYZ 小步方向 axis 0.003"
        if can_save_safe_point(state) and not state.saved_candidate_names:
            return "5", f"保存当前相对调试结果 {default_save_name(state)}"
        if state.axis_completed and state.saved_candidate_names:
            return "0", "已保存 relative_axis_result，可以退出"
        if not state.axis_completed:
            return "2", "先测试 XYZ 小步方向 axis 0.003"
        return "4", "查看当前摘要 summary"
    if state.ik_control_mode == "absolute":
        if not state.abs_ok:
            return "1", "谨慎测试绝对 IK 点"
        if can_save_safe_point(state) and not state.saved_candidate_names:
            return "4", f"保存当前安全点 {default_save_name(state)}"
        if state.saved_candidate_names:
            return "0", "已保存安全点，可以退出"
        return "3", "查看当前摘要 summary"
    if state.dry_run and not state.demo_passed:
        return "1", "先做 dry-run demo"
    if state.dry_run and state.demo_passed:
        return "0", "dry-run 已完成，退出后切到 execute"
    if not state.abs_ok:
        return "2", "先做 readme_abs 空载验证"
    if can_save_safe_point(state) and not state.saved_candidate_names:
        return "5", f"保存当前相对调试结果 {default_save_name(state)}"
    if state.axis_completed and state.saved_candidate_names:
        return "0", "已保存结果，可以退出"
    if not state.axis_completed:
        return "3", "测试 XYZ 小步方向 axis 0.003"
    return "4", "查看当前摘要 summary"


def format_menu_next_line(state: SessionState) -> str:
    choice, label = get_menu_recommendation(state)
    return f"选择 {choice}：{label}"


def print_guide(state: SessionState) -> None:
    print("[GUIDE]")
    for line in get_guide_lines(state):
        print(line)


def demo_status_text(state: SessionState) -> str:
    if state.demo_passed:
        return "PASS"
    if state.demo_attempted:
        return "FAIL"
    return "NOT RUN"


def abs_status_text(state: SessionState) -> str:
    if state.abs_ok:
        return "PASS"
    if state.dry_run:
        return "NOT RUN"
    if state.abs_attempted:
        return "FAIL"
    return "NOT RUN"


def axis_status_text(state: SessionState) -> str:
    if state.axis_completed and axis_all_ok(state):
        return "PASS"
    if state.axis_attempted and state.axis_completed:
        return "REVIEW"
    if state.axis_attempted:
        return "PARTIAL"
    return "NOT RUN"


def saved_result_label(state: SessionState) -> str:
    return "saved result" if state.ik_control_mode == "relative" else "saved point"


def compute_next_recommendation(state: SessionState) -> str:
    return format_menu_next_line(state)


def print_next_action_hint(state: SessionState) -> None:
    state.last_recommendation = compute_next_recommendation(state)
    print(f"next: {state.last_recommendation}")


def update_last_recommendation(state: SessionState) -> None:
    if state.risk_detected and state.last_recommendation:
        return
    state.last_recommendation = compute_next_recommendation(state)


def print_summary(state: SessionState) -> None:
    candidate = "-"
    if state.current_candidate_abs is not None:
        candidate = f"{state.current_candidate_abs.label} {format_pose_short(state.current_candidate_abs)}"
    can_continue = "NO" if state.risk_detected else "YES"
    print("[SUMMARY]")
    print(f"mode: {mode_label(state.dry_run)}")
    print(f"arm: {state.arm_name}")
    print(f"ik_control: {state.ik_control_mode}")
    if state.risk_detected:
        print("risk: DETECTED")
        if state.ik_control_mode == "relative":
            print("relative_mode: ENABLED")
            print("axis probe: BLOCKED")
        else:
            print("anchor_ready: NO")
            print("axis probe: BLOCKED")
        print(f"{saved_result_label(state)}: NO")
        print("")
        print("next:")
        if state.ik_control_mode == "relative":
            print("  停止。不要继续 rel/axis。")
        else:
            print("  停止。不要继续 axis/rel/save。")
            print("  当前 ABS 点不安全，请换安全起点或回到关节空间控制。")
        return
    if state.ik_control_mode == "relative":
        print("relative_mode: ENABLED")
        print("")
        print("checks:")
        print(f"  last_relative_step: {axis_status_text(state)}")
        print(f"  relative axis: {axis_status_text(state)}")
        print(f"  risk: {'DETECTED' if state.risk_detected else 'NONE'}")
        print(f"  saved result: {'YES' if state.saved_candidate_names else 'NO'}")
        print("")
        print("next:")
        print(f"  {format_menu_next_line(state)}")
        return
    print(f"candidate: {candidate}")
    print(f"anchor_ready: {'YES' if state.anchor_ready else 'NO'}")
    print(f"anchor_source: {state.anchor_source or '-'}")
    print(f"last_abs_ok_pose: {last_abs_pose_text(state)}")
    print(f"relative_steps_ok: {'YES' if state.relative_steps_ok else 'NO'}")
    print("")
    print("checks:")
    print(f"  dry-run demo: {demo_status_text(state)}")
    print(f"  abs anchor: {'PASS' if state.anchor_ready else abs_status_text(state)}")
    print(f"  relative axis: {'PASS' if state.relative_steps_ok else axis_status_text(state)}")
    print(f"  saved point: {'YES' if state.saved_candidate_names else 'NO'}")
    print(f"  risk: {'DETECTED' if state.risk_detected else 'NONE'}")
    print(f"  continue: {can_continue}")
    print("")
    print("next:")
    print(f"  {format_menu_next_line(state)}")


def print_help() -> None:
    print("[HELP]")
    print("[IK 控制方式]")
    print("relative 模式：")
    print("  用 abs=False 做小步相对 IK。")
    print("  不再先执行 readme_abs。")
    print("  推荐流程：")
    print("    1. 先 dry-run demo")
    print("    2. 实机 axis 0.003")
    print("    3. 逐步记录实际方向和安全状态")
    print("    4. 再手动 rel 小步调试")
    print("")
    print("absolute 模式：")
    print("  用 abs=True 测试绝对位姿，风险更高。")
    print("  README 示例点不保证安全。")
    print("  README absolute 示例点曾导致左臂危险构型，不建议实机继续使用，除非有人确认安全。")
    print("")
    print("mixed 模式：")
    print("  先用 absolute 建立基准，再用 relative 探测方向")
    print("")
    print("第一次 dry-run：")
    print("  1 -> 4 -> 0")
    print("")
    print("第一次实机：")
    print("  2 -> 选择观察结果")
    print("  3 -> 逐步记录方向")
    print("  4 -> 检查 summary")
    print("  5 -> 保存 relative_axis_result")
    print("  0 -> 退出")
    print("")
    print("危险情况：")
    print("  任何接近碰撞、抖动、异响，都选择 abort 或退出，不继续 axis。")
    print("")
    print("高级用户：")
    print("  可用 --expert 启动英文命令模式。")
    print("")
    print("高级命令：")
    print("  abs x y z r p y")
    print("  rel dx dy dz dr dp dy")
    print("  status")
    print("  last")
    print("  note text")


def print_menu(state: SessionState) -> None:
    recommended_choice, recommendation_text = get_menu_recommendation(state)
    print("")
    print(f"当前建议：{recommendation_text}")
    print("")
    print("请选择操作：")
    if state.ik_control_mode == "relative":
        print("  1. 运行 relative dry-run demo")
        print(f"  2. 测试 XYZ 小步方向 axis {DEFAULT_AXIS_DELTA:.3f}")
        print("  3. 手动相对移动 rel")
        print("  4. 查看当前摘要 summary")
        print("  5. 保存当前相对调试结果 save")
        print("  6. 切换控制方式")
        print("  0. 退出")
    elif state.ik_control_mode == "absolute":
        print("  1. 尝试 README 绝对 IK 点 readme_abs")
        print("  2. 手动输入绝对 IK 点 abs")
        print("  3. 查看当前摘要 summary")
        print("  4. 保存当前安全点 save")
        print("  5. 切换控制方式")
        print("  0. 退出")
    else:
        print("  1. 运行 demo")
        print("  2. 尝试 README 绝对 IK 点 readme_abs")
        print(f"  3. 测试 XYZ 小步方向 axis {DEFAULT_AXIS_DELTA:.3f}")
        print("  4. 查看当前摘要 summary")
        print("  5. 保存当前相对调试结果 save")
        print("  6. 切换控制方式")
        print("  0. 退出")
    print("")
    print(f"推荐：{recommended_choice}")


def prompt_save_name(state: SessionState) -> str:
    default_name = default_save_name(state)
    user_input = input(
        f"保存名称，直接回车使用默认 {default_name}："
    ).strip()
    return user_input or default_name


def command_from_menu_choice(choice: str, state: SessionState) -> Optional[str]:
    if state.ik_control_mode == "relative":
        if choice == "1":
            return "demo"
        if choice == "2":
            return f"axis {DEFAULT_AXIS_DELTA:.3f}"
        if choice == "3":
            return "__interactive_rel__"
        if choice == "4":
            return "summary"
        if choice == "5":
            if not can_save_safe_point(state):
                print("当前没有可保存的安全结果，或已经检测到风险，不能保存。")
                update_last_recommendation(state)
                return None
            return f"save {prompt_save_name(state)}"
        if choice == "6":
            return "__switch_mode__"
        if choice == "0":
            return "quit"
        return None
    if state.ik_control_mode == "absolute":
        if choice == "1":
            return "readme_abs"
        if choice == "2":
            return "__interactive_abs__"
        if choice == "3":
            return "summary"
        if choice == "4":
            if not can_save_safe_point(state):
                print("当前没有安全 anchor，或已经检测到风险，不能保存为安全点。")
                update_last_recommendation(state)
                return None
            return f"save {prompt_save_name(state)}"
        if choice == "5":
            return "__switch_mode__"
        if choice == "0":
            return "quit"
        return None
    if choice == "1":
        return "demo"
    if choice == "2":
        return "readme_abs"
    if choice == "3":
        return f"axis {DEFAULT_AXIS_DELTA:.3f}"
    if choice == "4":
        return "summary"
    if choice == "5":
        if not can_save_safe_point(state):
            print("当前没有可保存的安全结果，或已经检测到风险，不能保存。")
            update_last_recommendation(state)
            return None
        return f"save {prompt_save_name(state)}"
    if choice == "6":
        return "__switch_mode__"
    if choice == "0":
        return "quit"
    return None


def print_status(args: argparse.Namespace, state: SessionState) -> None:
    print("[STATUS]")
    print(f"mode: {mode_label(state.dry_run)}")
    print(f"arm: {state.arm_name}")
    print(f"ik_control_mode: {state.ik_control_mode}")
    print(f"log: {state.log_path}")
    print(
        "candidate_abs: "
        + (format_pose_full(state.current_candidate_abs) if state.current_candidate_abs else "-")
    )
    print(
        "last_ok_abs: "
        + (format_pose_full(state.last_ok_abs_command) if state.last_ok_abs_command else "-")
    )
    print(f"last_abs_ok_pose: {last_abs_pose_text(state)}")
    print(f"anchor_ready: {state.anchor_ready}")
    print(f"anchor_source: {state.anchor_source or '-'}")
    print(f"abs_safety_status: {state.abs_safety_status}")
    print(f"risk_detected: {state.risk_detected}")
    print(f"last_error: {state.last_error or '-'}")
    print(f"relative_steps_ok: {state.relative_steps_ok}")
    if state.axis_attempted:
        print("usable_relative_axes:")
        for axis_name in ("X", "Y", "Z"):
            print(f"  {axis_name}: {summarize_axis_pair(state, axis_name)}")
    print(f"max_delta: {args.max_delta}")
    print(f"allow_large_delta: {args.allow_large_delta}")
    print(f"allow_large_rotation: {args.allow_large_rotation}")
    print(f"ask_observation_dry_run: {args.ask_observation_dry_run}")
    print(f"verbose: {args.verbose}")
    print(f"last_recommendation: {state.last_recommendation or '-'}")


def format_trial_target(result: TrialResult) -> str:
    if result.abs_mode is None or result.x is None:
        return "-"
    names = ("x", "y", "z", "roll", "pitch", "yaw")
    if result.abs_mode is False:
        names = ("dx", "dy", "dz", "droll", "dpitch", "dyaw")
    values = (result.x, result.y, result.z, result.roll, result.pitch, result.yaw)
    return ", ".join(
        f"{name}={value:.6f}" for name, value in zip(names, values) if value is not None
    )


def print_last_result(state: SessionState) -> None:
    if state.last_result is None:
        print("当前还没有 trial 记录。")
        return
    result = state.last_result
    abs_rel = "-"
    if result.abs_mode is True:
        abs_rel = "abs"
    elif result.abs_mode is False:
        abs_rel = "rel"
    print("[LAST]")
    print(f"trial_id: {result.trial_id}")
    print(f"command_type: {result.command_type}")
    print(f"abs/rel: {abs_rel}")
    print(f"target/delta: {format_trial_target(result)}")
    print(f"status: {result.status}")
    print(f"observed_direction: {result.observed_direction or '-'}")
    print(
        "safety: "
        f"{result.safety_status or parse_observation_safety(result.user_observation)}"
    )
    print(f"observation: {result.user_observation or '-'}")
    print(f"note: {result.note or '-'}")


def parse_six_floats(tokens: list[str], command_name: str) -> Optional[list[float]]:
    if len(tokens) != 7:
        print(f"用法: {command_name} x y z roll pitch yaw")
        return None
    try:
        return [float(value) for value in tokens[1:]]
    except ValueError:
        print("参数必须全部是数字")
        return None


def print_exit_summary(state: SessionState) -> None:
    if state.dry_run:
        result_text = "dry-run only"
    elif state.risk_detected:
        result_text = "risk detected"
    elif state.axis_completed:
        result_text = "axis completed"
    elif state.abs_ok:
        result_text = "abs ok"
    else:
        result_text = "dry-run only"
    saved_text = pretty_path(state.saved_candidate_path) if state.saved_candidate_path else "none"
    print("[EXIT]")
    print(f"log: {pretty_path(state.log_path)}")
    print(f"saved: {saved_text}")
    print(f"result: {result_text}")
    if state.axis_attempted:
        print("usable_relative_axes:")
        for axis_name in ("X", "Y", "Z"):
            print(f"  {axis_name}: {summarize_axis_pair(state, axis_name)}")
    if state.risk_detected:
        print("risk: DETECTED")
        if state.ik_control_mode == "relative":
            print("relative_mode: ENABLED")
            print("axis probe: BLOCKED")
            print("next: 停止。不要继续 rel/axis。")
        else:
            print("anchor_ready: NO")
            print("axis probe: BLOCKED")
            print("next: 停止。不要继续 axis/rel/save。当前 ABS 点不安全，请换安全起点或回到关节空间控制。")
        return
    print(f"next: {compute_next_recommendation(state)}")


def process_repl_command(
    line: str,
    arm,
    args: argparse.Namespace,
    state: SessionState,
) -> bool:
    command_name = line.split(maxsplit=1)[0].lower()

    if command_name == "__switch_mode__":
        prompt_switch_control_mode(state)
        return True
    if command_name == "__interactive_abs__":
        run_abs_flow(arm, args, state, prompt_interactive_abs_command(state))
        return True
    if command_name == "__interactive_rel__":
        if not ensure_relative_motion_allowed(arm, args, state):
            update_last_recommendation(state)
            return True
        command = prompt_interactive_rel_command(state)
        print("[REL]")
        print(f"delta: {format_delta_short(command)}")
        if not state.dry_run:
            print("本步将调用：")
            print(command.call_repr())
            print("")
            print("检查：")
            print("1. 左臂当前姿态安全")
            print("2. 周围无遮挡")
            print("3. 急停可触达")
            print("4. 本步是小位移/小姿态增量")
            print("")
        if not state.dry_run and not confirm_rel_execute(command, args, "确认执行？Enter=执行，q=取消\n"):
            print("已取消本次相对移动。")
            update_last_recommendation(state)
            return True
        result = run_logged_command(arm, command, args, state)
        outcome = "PASS" if sdk_status_ok(result.status) else "FAIL"
        print(f"result: {outcome}")
        print(f"continue: {'no' if state.risk_detected else 'yes'}")
        print_next_action_hint(state)
        return True
    if command_name == "help":
        print_help()
        return True
    if command_name == "guide":
        print_guide(state)
        return True
    if command_name == "mode":
        tokens = line.split()
        if len(tokens) != 2 or tokens[1] not in CONTROL_MODE_CHOICES:
            print("用法: mode relative|absolute|mixed")
            return True
        state.ik_control_mode = tokens[1]
        print(f"已切换到 IK 控制方式：{state.ik_control_mode}")
        print_next_action_hint(state)
        return True
    if command_name == "summary":
        print_summary(state)
        return True
    if command_name == "demo":
        run_demo(arm, args, state)
        return True
    if command_name == "status":
        print_status(args, state)
        return True
    if command_name == "last":
        print_last_result(state)
        return True
    if command_name == "quit":
        return False
    if command_name == "readme_abs":
        run_abs_flow(arm, args, state, build_readme_abs_command(state.arm_name))
        return True
    if command_name == "note":
        note_text = line.partition(" ")[2].strip()
        if not note_text:
            print("用法: note 任意文本")
            return True
        log_note(note_text, state)
        print_next_action_hint(state)
        return True
    if command_name == "save":
        try:
            tokens = shlex.split(line)
        except ValueError as exc:
            print(f"解析失败: {exc}")
            return True
        if len(tokens) == 1:
            if not can_save_safe_point(state):
                print("当前没有可保存的安全结果，或已经检测到风险，不能保存。")
                return True
            save_candidate(default_save_name(state), state)
            return True
        if len(tokens) != 2:
            print("用法: save name")
            return True
        save_candidate(tokens[1], state)
        return True

    try:
        tokens = shlex.split(line)
    except ValueError as exc:
        print(f"解析失败: {exc}")
        return True

    if command_name == "abs":
        values = parse_six_floats(tokens, "abs")
        if values is None:
            return True
        run_abs_flow(arm, args, state, build_abs_command(state.arm_name, values))
        return True

    if command_name == "rel":
        if state.risk_detected:
            print("检测到不安全动作，停止 IK 调试。")
            print("不要继续 rel；请退出并检查机器人姿态/日志。")
            return True
        values = parse_six_floats(tokens, "rel")
        if values is None:
            return True
        if not ensure_relative_motion_allowed(arm, args, state):
            update_last_recommendation(state)
            return True
        command = build_rel_command(state.arm_name, values)
        print("[REL]")
        print(f"delta: {format_delta_short(command)}")
        if not state.dry_run:
            print("本步将调用：")
            print(command.call_repr())
            print("")
            print("检查：")
            print("1. 左臂当前姿态安全")
            print("2. 周围无遮挡")
            print("3. 急停可触达")
            print("4. 本步是小位移/小姿态增量")
            print("")
        if not state.dry_run and not confirm_rel_execute(command, args, "确认执行？Enter=执行，q=取消\n"):
            record_result(
                state,
                arm=command.arm,
                command_type=command.command_type,
                abs_mode=command.abs_mode,
                x=command.x,
                y=command.y,
                z=command.z,
                roll=command.roll,
                pitch=command.pitch,
                yaw=command.yaw,
                block=command.block,
                status="skipped",
                error="user_declined_confirmation",
                note=command.note,
            )
            print("result: SKIP")
            print("continue: no")
            print_next_action_hint(state)
            return True
        result = run_logged_command(arm, command, args, state)
        outcome = "PASS" if sdk_status_ok(result.status) else "FAIL"
        print("[REL]")
        print(f"delta: {format_delta_short(command)}")
        print(f"result: {outcome}")
        print(f"continue: {'no' if state.risk_detected else 'yes'}")
        print_next_action_hint(state)
        return True

    if command_name == "axis":
        if state.risk_detected:
            print("检测到不安全动作，停止 IK 调试。")
            print("不要继续 axis；请退出并检查机器人姿态/日志。")
            return True
        if len(tokens) > 2:
            print("用法: axis delta")
            return True
        if len(tokens) == 2:
            try:
                delta = float(tokens[1])
            except ValueError:
                print("axis delta 必须是数字")
                return True
        else:
            delta = min(args.max_delta, DEFAULT_AXIS_DELTA)
        if not ensure_relative_motion_allowed(arm, args, state):
            update_last_recommendation(state)
            return True
        try:
            execute_axis_sequence(arm, args, state, delta)
        except ValueError as exc:
            print(f"参数错误: {exc}")
        return True

    print(f"未知命令: {command_name}；输入 help 查看帮助。")
    return True


def repl_loop(arm, args: argparse.Namespace, state: SessionState) -> None:
    while not state.stop_requested:
        try:
            if state.pending_line is not None:
                raw_line = state.pending_line
                state.pending_line = None
                print(f"[pending command] {raw_line}")
            elif args.expert:
                raw_line = input(f"ik({state.arm_name})> ")
            else:
                print_menu(state)
                recommended_choice, _ = get_menu_recommendation(state)
                raw_line = input("请输入编号，直接回车执行推荐项 > ")
                if not raw_line.strip():
                    raw_line = recommended_choice
        except EOFError:
            print()
            break

        line = raw_line.strip()
        if not line:
            continue
        if not args.expert and line.isdigit():
            mapped_command = command_from_menu_choice(line, state)
            if mapped_command is None:
                continue
            line = mapped_command
        should_continue = process_repl_command(line, arm, args, state)
        if not should_continue:
            break


def safe_shutdown(robot) -> None:
    if robot is None:
        return
    try:
        robot.stop()
    except Exception as exc:  # pragma: no cover - only hit with real SDK/runtime errors
        print(f"stop() 失败: {exc}")
    try:
        robot.disconnect()
    except Exception as exc:  # pragma: no cover - only hit with real SDK/runtime errors
        print(f"disconnect() 失败: {exc}")


def _format_optional_float(value: Optional[float]) -> str:
    if value is None:
        return ""
    return f"{value:.6f}"


def _format_optional_bool(value: Optional[bool]) -> str:
    if value is None:
        return ""
    return "true" if value else "false"


def run_observation_safety_self_check() -> int:
    state = SessionState(
        log_path=Path("/tmp/ik_tune_observation_safety_self_check.csv"),
        dry_run=False,
        arm_name="left",
        ik_control_mode="relative",
    )
    command = build_readme_abs_command("left")
    result = TrialResult(
        timestamp=now_iso(),
        trial_id="SELF",
        arm="left",
        command_type="readme_abs",
        abs_mode=True,
        x=command.x,
        y=command.y,
        z=command.z,
        roll=command.roll,
        pitch=command.pitch,
        yaw=command.yaw,
        block=command.block,
        status=compose_result_status("ok", parse_observation_safety("2关节撞上了")),
        error="",
        duration_s=0.0,
        user_observation="2关节撞上了",
        note="self check",
        safety_status=parse_observation_safety("2关节撞上了"),
    )
    update_state_after_command(command, result, state)
    print("safety_status:", state.abs_safety_status)
    print("anchor_ready:", state.anchor_ready)
    print("risk_detected:", state.risk_detected)
    print("status:", result.status)
    return 0


def main() -> int:
    if "--self-check-observation-safety" in sys.argv:
        return run_observation_safety_self_check()
    args = parse_args()
    print_compact_safety_notice()
    sys.stdout.flush()
    validate_args(args)

    log_path = resolve_log_path(args.log_file)
    init_csv_log(log_path)
    state = SessionState(
        log_path=log_path,
        dry_run=args.dry_run,
        arm_name=args.arm,
        ik_control_mode=args.ik_control_mode,
        current_candidate_abs=build_readme_abs_command(args.arm)
        if args.use_readme_example
        else None,
    )
    print(
        f"[IK Tune] {mode_label(state.dry_run)} | arm={state.arm_name} | "
        f"mode={state.ik_control_mode}"
    )
    print(f"log: {pretty_path(state.log_path)}")
    if args.expert:
        print(f"当前阶段：{current_stage_text(state)}")
        print("expert 模式：可直接输入 demo / readme_abs / axis / rel / summary / mode ... / quit")

    robot = None
    try:
        robot = connect_robot(args)
        if robot is None:
            return 0
        arm = select_arm(robot, args.arm)
        repl_loop(arm, args, state)
        print_exit_summary(state)
    except KeyboardInterrupt:
        print("\n捕获 Ctrl-C，尝试停止机器人并断开连接。")
        record_result(
            state,
            arm=state.arm_name,
            command_type="interrupt",
            abs_mode=None,
            x=None,
            y=None,
            z=None,
            roll=None,
            pitch=None,
            yaw=None,
            block=None,
            status="keyboard_interrupt",
            note="KeyboardInterrupt",
        )
        safe_shutdown(robot)
    finally:
        safe_shutdown(robot)

    return 0


if __name__ == "__main__":
    sys.exit(main())
