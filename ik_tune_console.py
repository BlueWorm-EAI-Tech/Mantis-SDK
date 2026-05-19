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
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

from connection_selector import add_connection_args, connect_robot_with_selector


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_ROBOT_VERSION = "3.0"
DEFAULT_MAX_DELTA = 0.005
DEFAULT_MAX_ROTATION_DELTA = 0.05
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


def confirm_abs_target(command: IkCommand, dry_run: bool) -> bool:
    print("-" * 72)
    print("ABS target")
    print(
        "target = "
        f"(x={command.x:.6f}, y={command.y:.6f}, z={command.z:.6f}, "
        f"roll={command.roll:.6f}, pitch={command.pitch:.6f}, yaw={command.yaw:.6f})"
    )
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
    print(
        "delta  = "
        f"(dx={command.x:.6f}, dy={command.y:.6f}, dz={command.z:.6f}, "
        f"droll={command.roll:.6f}, dpitch={command.pitch:.6f}, dyaw={command.yaw:.6f})"
    )
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


def prompt_observation() -> str:
    print(
        "观察结果可输入："
        + ", ".join(OBSERVATION_CHOICES[:-1])
        + ", abort，或任意自定义备注"
    )
    return input("observation> ").strip()


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
    return result


def handle_ik_command(
    arm,
    command: IkCommand,
    args: argparse.Namespace,
    state: SessionState,
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
    user_observation = prompt_observation() if status in {"ok", "dry_run"} else ""
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
            )
        )
    return commands


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
    default_axis_delta = min(DEFAULT_MAX_DELTA, 0.005)
    print("可用命令:")
    print("  help")
    print("  readme_abs")
    print("  abs x y z roll pitch yaw")
    print("  rel dx dy dz droll dpitch dyaw")
    print(f"  axis [delta]    # 默认 delta={default_axis_delta:.3f}，建议不超过 0.005")
    print("  note 任意文本")
    print("  save name")
    print("  quit")


def repl_loop(arm, args: argparse.Namespace, state: SessionState) -> None:
    print_help()
    while not state.stop_requested:
        try:
            raw_line = input(f"ik({state.arm_name})> ")
        except EOFError:
            print()
            break

        line = raw_line.strip()
        if not line:
            continue

        command_name = line.split(maxsplit=1)[0].lower()
        if command_name == "help":
            print_help()
            continue
        if command_name == "quit":
            break
        if command_name == "readme_abs":
            command = build_readme_abs_command(state.arm_name)
            state.current_candidate_abs = command
            handle_ik_command(arm, command, args, state)
            continue
        if command_name == "note":
            note_text = line.partition(" ")[2].strip()
            if not note_text:
                print("用法: note 任意文本")
                continue
            log_note(note_text, state)
            continue
        if command_name == "save":
            try:
                tokens = shlex.split(line)
            except ValueError as exc:
                print(f"解析失败: {exc}")
                continue
            if len(tokens) != 2:
                print("用法: save name")
                continue
            save_candidate(tokens[1], state)
            continue

        try:
            tokens = shlex.split(line)
        except ValueError as exc:
            print(f"解析失败: {exc}")
            continue

        if command_name == "abs":
            values = parse_six_floats(tokens, "abs")
            if values is None:
                continue
            command = build_abs_command(state.arm_name, values)
            state.current_candidate_abs = command
            handle_ik_command(arm, command, args, state)
            continue

        if command_name == "rel":
            values = parse_six_floats(tokens, "rel")
            if values is None:
                continue
            command = build_rel_command(state.arm_name, values)
            handle_ik_command(arm, command, args, state)
            continue

        if command_name == "axis":
            if len(tokens) > 2:
                print("用法: axis delta")
                continue
            if len(tokens) == 2:
                try:
                    delta = float(tokens[1])
                except ValueError:
                    print("axis delta 必须是数字")
                    continue
            else:
                delta = min(args.max_delta, 0.005)
            try:
                axis_commands = build_axis_commands(state.arm_name, delta)
            except ValueError as exc:
                print(f"参数错误: {exc}")
                continue
            for axis_command in axis_commands:
                handle_ik_command(arm, axis_command, args, state)
                if state.stop_requested:
                    print("检测到 abort，准备退出 REPL。")
                    break
            continue

        print(f"未知命令: {command_name}；输入 help 查看帮助。")


def parse_six_floats(tokens: list[str], command_name: str) -> Optional[list[float]]:
    if len(tokens) != 7:
        print(f"用法: {command_name} x y z roll pitch yaw")
        return None
    try:
        return [float(value) for value in tokens[1:]]
    except ValueError:
        print("参数必须全部是数字")
        return None


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

    mode_label = "DRY-RUN" if state.dry_run else "EXECUTE"
    print(f"mode: {mode_label}")
    print(f"arm: {args.arm}")
    print(f"log: {log_path}")
    print(f"robot_version: {DEFAULT_ROBOT_VERSION}")
    if state.current_candidate_abs is not None:
        print(
            "已载入 README 示例候选点: "
            "(x=0.500000, y=0.200000, z=0.300000, roll=0.000000, pitch=0.000000, yaw=0.000000)"
        )

    robot = None
    try:
        robot = connect_robot(args)
        if robot is None:
            return 0
        arm = select_arm(robot, args.arm)
        repl_loop(arm, args, state)
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
