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
DEFAULT_MAX_ROTATION_DELTA = 0.05
DEFAULT_AXIS_DELTA = 0.003
README_EXAMPLE_POSE = (0.5, 0.2, 0.3, 0.0, 0.0, 0.0)
SAFE_CANDIDATES_PATH = PROJECT_ROOT / "logs" / "ik_safe_candidates.json"
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
COMMAND_KEYWORDS = {
    "help",
    "demo",
    "readme_abs",
    "abs",
    "rel",
    "axis",
    "note",
    "save",
    "status",
    "last",
    "quit",
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


@dataclass
class SessionState:
    log_path: Path
    dry_run: bool
    arm_name: str
    current_candidate_abs: Optional[IkCommand] = None
    last_ok_abs_command: Optional[IkCommand] = None
    last_result: Optional[TrialResult] = None
    pending_line: Optional[str] = None
    saved_candidate_names: list[str] = field(default_factory=list)
    trial_counter: int = 0
    stop_requested: bool = False

    def next_trial_id(self) -> str:
        self.trial_counter += 1
        return f"T{self.trial_counter:04d}"


def print_safety_banner() -> None:
    print("=" * 72)
    print("IK Tune Console 安全提示")
    print("- 当前脚本可能控制真实机器人")
    print("- 第一次必须 dry-run")
    print("- 第一次实机必须空载")
    print("- 不要放杯子、奶壶、液体、咖啡机障碍物")
    print("- 必须有人看护急停")
    print("- README 示例点只是示例，不等于安全点")
    print("=" * 72)


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
        "--log-file",
        default=None,
        help="CSV 日志路径；不指定时自动写入 logs/ik_tune_trials_YYYYmmdd_HHMMSS.csv",
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


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def mode_label(dry_run: bool) -> str:
    return "DRY-RUN" if dry_run else "EXECUTE"


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


def format_command_target(command: IkCommand) -> str:
    if command.abs_mode:
        return (
            f"x={command.x:.6f}, y={command.y:.6f}, z={command.z:.6f}, "
            f"roll={command.roll:.6f}, pitch={command.pitch:.6f}, yaw={command.yaw:.6f}"
        )
    return (
        f"dx={command.x:.6f}, dy={command.y:.6f}, dz={command.z:.6f}, "
        f"droll={command.roll:.6f}, dpitch={command.pitch:.6f}, dyaw={command.yaw:.6f}"
    )


def format_trial_target(result: TrialResult) -> str:
    if result.abs_mode is None or result.x is None:
        return "-"
    prefix = ("x", "y", "z", "roll", "pitch", "yaw")
    if result.abs_mode is False:
        prefix = ("dx", "dy", "dz", "droll", "dpitch", "dyaw")
    values = (result.x, result.y, result.z, result.roll, result.pitch, result.yaw)
    return ", ".join(
        f"{name}={value:.6f}" for name, value in zip(prefix, values) if value is not None
    )


def print_startup_guide(args: argparse.Namespace, state: SessionState) -> None:
    print("下一步建议：")
    print("1. 第一次 dry-run 推荐输入：demo")
    print("2. 实机第一次推荐输入：readme_abs，然后观察，再 axis 0.003")
    print("3. 输入 help 查看命令")
    print(f"4. 当前模式：{mode_label(state.dry_run)}")
    print(f"5. 当前手臂：{state.arm_name}")
    print(f"6. 当前日志文件路径：{state.log_path}")
    print(f"7. dry-run 是否询问 observation：{'yes' if args.ask_observation_dry_run else 'no'}")
    if state.current_candidate_abs is not None:
        print("已载入 README 示例候选点，但不会自动执行。")


def confirm_abs_target(command: IkCommand, dry_run: bool) -> bool:
    print("-" * 72)
    print("ABS target")
    print(f"target = ({format_command_target(command)})")
    print(f"call   = {command.call_repr()}")
    if dry_run:
        print("DRY-RUN：不会连接机器人，只记录这一步。")
        return True
    user_input = input("输入大写 RUN_ABS 才执行；其他任意输入将跳过: ").strip()
    if user_input == "RUN_ABS":
        return True
    print("已跳过该绝对 IK 动作。")
    return False


def confirm_rel_target(command: IkCommand, dry_run: bool) -> bool:
    print("-" * 72)
    print("REL target")
    print(f"delta  = ({format_command_target(command)})")
    print(f"call   = {command.call_repr()}")
    if dry_run:
        print("DRY-RUN：不会连接机器人，只记录这一步。")
        return True
    user_input = input("按 Enter 执行相对 IK，输入 q 跳过: ").strip().lower()
    if user_input == "q":
        print("已跳过该相对 IK 动作。")
        return False
    return True


def execute_ik(arm, command: IkCommand, dry_run: bool) -> tuple[str, str, float]:
    start_time = time.perf_counter()
    status = "dry_run" if dry_run else "ok"
    error = ""
    print(f"执行计划: {command.call_repr()}")
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


def is_command_like_line(line: str) -> bool:
    stripped = line.strip()
    if not stripped:
        return False
    command_name = stripped.split(maxsplit=1)[0].lower()
    return command_name in COMMAND_KEYWORDS


def should_prompt_observation(args: argparse.Namespace, state: SessionState) -> bool:
    if not state.dry_run:
        return True
    return bool(args.ask_observation_dry_run)


def prompt_observation(state: SessionState) -> str:
    print("请输入本次观察结果，直接回车表示跳过；这里不是命令输入区。")
    print("观察结果建议：" + ", ".join(OBSERVATION_CHOICES) + "，或自定义备注")
    user_input = input("observation> ").strip()
    if not user_input:
        return ""
    if is_command_like_line(user_input):
        redirect = input(
            "你输入的内容看起来像下一条命令，而不是观察结果。"
            "是否把它作为命令执行？[y/N] "
        ).strip().lower()
        if redirect == "y":
            state.pending_line = user_input
            print(f"已将该输入转为下一条待执行命令: {user_input}")
            return "observation_skipped_command_redirect"
    return user_input


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
    )
    append_csv_log(state.log_path, result)
    state.last_result = result
    return result


def resolve_observation(
    args: argparse.Namespace,
    state: SessionState,
    status: str,
    preset_observation: Optional[str],
) -> str:
    if status not in {"ok", "dry_run"}:
        return ""
    if preset_observation is not None:
        return preset_observation
    if should_prompt_observation(args, state):
        return prompt_observation(state)
    if state.dry_run:
        return "dry_run"
    return ""


def handle_ik_command(
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
        print(f"拒绝执行: {exc}")
        return record_result(
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

    confirmed = (
        confirm_abs_target(command, state.dry_run)
        if command.abs_mode
        else confirm_rel_target(command, state.dry_run)
    )
    if not confirmed:
        return record_result(
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

    status, error, duration_s = execute_ik(arm, command, state.dry_run)
    user_observation = resolve_observation(args, state, status, preset_observation)
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
        user_observation=user_observation,
        note=command.note,
    )
    print(
        f"trial={result.trial_id} status={result.status} "
        f"duration={result.duration_s:.3f}s observation={result.user_observation or '-'}"
    )

    if status == "ok" and command.abs_mode:
        state.last_ok_abs_command = command
    if user_observation == "abort":
        state.stop_requested = True
    return result


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
        label="README example",
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
        label="manual ABS",
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
        label="manual REL",
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


def axis_overview_entry(command: IkCommand) -> str:
    if command.label in {"+X", "-X"}:
        return f"{command.label}: dx={command.x:+.3f}"
    if command.label in {"+Y", "-Y"}:
        return f"{command.label}: dy={command.y:+.3f}"
    return f"{command.label}: dz={command.z:+.3f}"


def print_axis_overview(commands: list[IkCommand]) -> None:
    print("即将依次测试 6 个相对 IK 小步：")
    for index, command in enumerate(commands, start=1):
        print(f"{index}. {axis_overview_entry(command)}")


def execute_axis_sequence(
    arm,
    args: argparse.Namespace,
    state: SessionState,
    delta: float,
    *,
    preset_observations: Optional[list[str]] = None,
) -> None:
    commands = build_axis_commands(state.arm_name, delta)
    print_axis_overview(commands)
    for index, axis_command in enumerate(commands):
        preset = None
        if preset_observations is not None and index < len(preset_observations):
            preset = preset_observations[index]
        handle_ik_command(
            arm,
            axis_command,
            args,
            state,
            preset_observation=preset,
        )
        if state.stop_requested:
            print("检测到 abort，停止后续 axis。")
            break


def run_demo(arm, args: argparse.Namespace, state: SessionState) -> None:
    if not state.dry_run:
        print("demo 只允许在 dry-run 下自动运行。")
        print("实机模式请手动执行 readme_abs / axis 0.003，并保持每一步人工确认。")
        return

    print("开始 dry-run demo：将自动演示 readme_abs + axis 0.003。")
    readme_command = build_readme_abs_command(state.arm_name)
    state.current_candidate_abs = readme_command
    handle_ik_command(
        arm,
        readme_command,
        args,
        state,
        preset_observation="dry_run_readme_abs_ok",
    )
    if state.stop_requested:
        return

    execute_axis_sequence(
        arm,
        args,
        state,
        DEFAULT_AXIS_DELTA,
        preset_observations=[
            "dry_run_axis_+X_ok",
            "dry_run_axis_-X_ok",
            "dry_run_axis_+Y_ok",
            "dry_run_axis_-Y_ok",
            "dry_run_axis_+Z_ok",
            "dry_run_axis_-Z_ok",
        ],
    )
    print(f"demo 已完成，日志文件路径：{state.log_path}")
    print("下一步建议：")
    print("- 如需回看本次状态，可输入 status 或 last")
    print("- 第一次实机建议手动执行：readme_abs，然后观察，再 axis 0.003")
    print("- 若当前候选点看起来合适，可稍后使用 save <name> 保存")


def save_candidate(name: str, state: SessionState) -> TrialResult:
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
    print(f"已保存候选点: {name} -> {SAFE_CANDIDATES_PATH}")
    return record_result(
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


def load_safe_candidates() -> dict[str, object]:
    if not SAFE_CANDIDATES_PATH.exists():
        return {}
    try:
        data = json.loads(SAFE_CANDIDATES_PATH.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    return data if isinstance(data, dict) else {}


def log_note(note_text: str, state: SessionState) -> TrialResult:
    print(f"记录备注: {note_text}")
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


def print_help() -> None:
    print("推荐 dry-run：")
    print("  demo")
    print("")
    print("推荐第一次实机：")
    print("  readme_abs")
    print("  axis 0.003")
    print("  save left_readme_safe_01")
    print("  quit")
    print("")
    print("常用命令：")
    print("  readme_abs")
    print("  abs x y z roll pitch yaw")
    print("  rel dx dy dz droll dpitch dyaw")
    print("  axis 0.003")
    print("  save name")
    print("  status")
    print("  last")
    print("  note text")
    print("  quit")
    print("")
    print("观察结果建议：")
    print("  ok")
    print("  bad")
    print("  near_collision")
    print("  no_motion")
    print("  wrong_direction")
    print("  jitter")
    print("  no_solution")
    print("  abort")


def print_status(args: argparse.Namespace, state: SessionState) -> None:
    print("当前会话状态：")
    print(f"- mode: {mode_label(state.dry_run)}")
    print(f"- arm: {state.arm_name}")
    print(f"- log path: {state.log_path}")
    print(
        "- current candidate abs: "
        + (format_command_target(state.current_candidate_abs) if state.current_candidate_abs else "-")
    )
    print(
        "- last ok abs command: "
        + (format_command_target(state.last_ok_abs_command) if state.last_ok_abs_command else "-")
    )
    print(f"- max_delta: {args.max_delta}")
    print(f"- allow_large_delta: {args.allow_large_delta}")
    print(f"- allow_large_rotation: {args.allow_large_rotation}")
    print(f"- dry-run ask observation: {args.ask_observation_dry_run}")


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
    print("最近一次 trial：")
    print(f"- trial_id: {result.trial_id}")
    print(f"- command_type: {result.command_type}")
    print(f"- abs/rel: {abs_rel}")
    print(f"- target/delta: {format_trial_target(result)}")
    print(f"- status: {result.status}")
    print(f"- observation: {result.user_observation or '-'}")
    print(f"- note: {result.note or '-'}")


def print_exit_summary(args: argparse.Namespace, state: SessionState) -> None:
    print("=" * 72)
    print("退出提示")
    print(f"1. 日志文件路径：{state.log_path}")
    if state.saved_candidate_names:
        print(f"2. 候选点 JSON 路径：{SAFE_CANDIDATES_PATH}")
    else:
        print("2. 候选点 JSON 路径：尚未保存候选点")
    print("3. 下一步建议：")
    if state.dry_run:
        print("- 还未进入 execute，建议先继续核对安全门槛，再做单步实机确认。")
    if state.last_ok_abs_command is not None and not state.saved_candidate_names:
        print("- 已经有 status=ok 的 ABS 点，建议执行 save <name>。")
    if state.saved_candidate_names:
        print("- 已保存候选点，后续可将该点用于 coffee_replay_safe.py 的 IK 阶段。")
    elif state.current_candidate_abs is not None:
        print("- 当前仍有一个候选 ABS 点；如需保留，可执行 save <name>。")
    if not args.execute:
        print("- 第一次实机请保持空载、人工看护急停，并继续逐步确认。")


def parse_six_floats(tokens: list[str], command_name: str) -> Optional[list[float]]:
    if len(tokens) != 7:
        print(f"用法: {command_name} x y z roll pitch yaw")
        return None
    try:
        return [float(value) for value in tokens[1:]]
    except ValueError:
        print("参数必须全部是数字")
        return None


def process_repl_command(
    line: str,
    arm,
    args: argparse.Namespace,
    state: SessionState,
) -> bool:
    command_name = line.split(maxsplit=1)[0].lower()

    if command_name == "help":
        print_help()
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
        command = build_readme_abs_command(state.arm_name)
        state.current_candidate_abs = command
        handle_ik_command(arm, command, args, state)
        return True
    if command_name == "note":
        note_text = line.partition(" ")[2].strip()
        if not note_text:
            print("用法: note 任意文本")
            return True
        log_note(note_text, state)
        return True
    if command_name == "save":
        try:
            tokens = shlex.split(line)
        except ValueError as exc:
            print(f"解析失败: {exc}")
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
        command = build_abs_command(state.arm_name, values)
        state.current_candidate_abs = command
        handle_ik_command(arm, command, args, state)
        return True

    if command_name == "rel":
        values = parse_six_floats(tokens, "rel")
        if values is None:
            return True
        command = build_rel_command(state.arm_name, values)
        handle_ik_command(arm, command, args, state)
        return True

    if command_name == "axis":
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
        try:
            execute_axis_sequence(arm, args, state, delta)
        except ValueError as exc:
            print(f"参数错误: {exc}")
        return True

    print(f"未知命令: {command_name}；输入 help 查看帮助。")
    return True


def repl_loop(arm, args: argparse.Namespace, state: SessionState) -> None:
    print_help()
    while not state.stop_requested:
        try:
            if state.pending_line is not None:
                raw_line = state.pending_line
                state.pending_line = None
                print(f"[pending command] {raw_line}")
            else:
                raw_line = input(f"ik({state.arm_name})> ")
        except EOFError:
            print()
            break

        line = raw_line.strip()
        if not line:
            continue
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


def main() -> int:
    print_safety_banner()
    sys.stdout.flush()
    args = parse_args()
    validate_args(args)

    log_path = resolve_log_path(args.log_file)
    init_csv_log(log_path)
    state = SessionState(
        log_path=log_path,
        dry_run=args.dry_run,
        arm_name=args.arm,
        current_candidate_abs=build_readme_abs_command(args.arm)
        if args.use_readme_example
        else None,
    )

    print_startup_guide(args, state)
    print(f"robot_version: {DEFAULT_ROBOT_VERSION}")
    if state.current_candidate_abs is not None:
        print(
            "README 候选点: "
            "(x=0.500000, y=0.200000, z=0.300000, roll=0.000000, pitch=0.000000, yaw=0.000000)"
        )

    robot = None
    try:
        robot = connect_robot(args)
        if robot is None:
            return 0
        arm = select_arm(robot, args.arm)
        repl_loop(arm, args, state)
        print_exit_summary(args, state)
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

    print(f"日志已写入: {log_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
