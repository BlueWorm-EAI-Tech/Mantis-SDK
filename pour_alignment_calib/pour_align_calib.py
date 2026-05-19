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
DEFAULT_MAX_WRIST_ROLL = 0.70
LOG_DIR = Path(__file__).resolve().parent / "logs"

# Source: coffee.py left-hand pour preparation stage, mirrored by
# coffee_replay_safe.py::left_hand_move_to_pour_pose as the right-hand
# receive-milk/cup-mouth pose. This command intentionally excludes the
# surrounding left-arm moves, sleeps, gripper calls, and any home().
RIGHT_CUP_POSE_STEPS = (
    ("set_wrist_yaw", -0.7, False),
    ("set_wrist_pitch", -0.5, False),
    ("set_wrist_roll", 0.3, False),
    ("set_shoulder_roll", 0.7, False),
)

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
    current_gripper_position: Optional[float] = None
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
        help=f"左夹爪目标位置，默认 {DEFAULT_GRIPPER_POSITION}",
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
    add_connection_args(parser, default_profile="interactive")
    parser.set_defaults(robot_version=DEFAULT_ROBOT_VERSION)
    return parser.parse_args()


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
  grip                 set left_gripper to configured position
  x+ / x-              left_arm relative IK X +/- step
  y+ / y-              left_arm relative IK Y +/- small step
  z+ / z-              left_arm relative IK Z +/- step
  roll0                set left wrist_roll target to 0
  roll03               set left wrist_roll target to 0.3
  roll05               set left wrist_roll target to 0.5
  roll07               set left wrist_roll target to 0.7
  right_cup_pose       set right arm to coffee.py pour-stage cup pose
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
    print("空壶标定：固定右手杯子位置后，用左手 relative IK 小步对位。")
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


def right_cup_pose_joint_targets() -> str:
    return json.dumps(
        [
            {"joint": name.removeprefix("set_"), "target": target, "block": block}
            for name, target, block in RIGHT_CUP_POSE_STEPS
        ],
        ensure_ascii=False,
        separators=(",", ":"),
    )


def build_right_cup_pose_action() -> Action:
    return Action(
        command="right_cup_pose",
        command_type="right_arm_joint_pose",
        arm="right",
        joint_targets=right_cup_pose_joint_targets(),
    )


def describe_action(action: Action) -> str:
    if action.command_type == "right_arm_joint_pose":
        calls = [
            f"robot.right_arm.{method}({target:.6f}, block={block})"
            for method, target, block in RIGHT_CUP_POSE_STEPS
        ]
        return "\n  ".join(calls)
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
        return f"robot.left_gripper.set_position({action.gripper_position:.6f})"
    return action.command


def confirm_real_action(action: Action) -> bool:
    print(f"[confirm] {describe_action(action)}")
    user_input = input("输入 y 执行该真实动作，其他任意输入跳过：").strip().lower()
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
            robot.left_gripper.set_position(action.gripper_position)
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


def execute_right_cup_pose(robot, args: argparse.Namespace, state: SessionState) -> None:
    action = build_right_cup_pose_action()
    print("[plan] right_cup_pose will run:")
    print(f"  {describe_action(action)}")

    if state.dry_run:
        append_log(state, action, user_confirmed=None, status="dry_run")
        print("[dry-run] 已记录，不连接机器人，不移动右臂。")
        return

    confirmed = confirm_real_action(action)
    if not confirmed:
        append_log(state, action, user_confirmed=False, status="skipped_by_user")
        print("[skip] 用户未确认，right_cup_pose 已跳过。")
        return

    status = "ok"
    try:
        start_time = time.perf_counter()
        for method_name, target, block in RIGHT_CUP_POSE_STEPS:
            getattr(robot.right_arm, method_name)(target, block=block)
        duration_s = time.perf_counter() - start_time
        print(f"[ok] right_cup_pose 指令已发送，用时 {duration_s:.3f}s。")
    except Exception as exc:  # pragma: no cover - real SDK/runtime only
        status = f"error: {exc}"
        print(f"[error] {exc}")

    append_log(state, action, user_confirmed=True, status=status)


def update_tracked_state(action: Action, state: SessionState) -> None:
    if action.command_type == "wrist_roll":
        state.current_wrist_roll = action.wrist_roll_target
    elif action.command_type == "wrist_yaw":
        state.current_wrist_yaw = action.wrist_yaw_delta_or_target
    elif action.command_type == "wrist_pitch":
        state.current_wrist_pitch = action.wrist_pitch_delta_or_target
    elif action.command_type == "gripper":
        state.current_gripper_position = action.gripper_position


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
        action = Action(
            command=command,
            command_type="gripper",
            arm="left",
            gripper_position=args.gripper_position,
        )
        execute_action(robot, action, args, state)
        return True
    if command == "right_cup_pose":
        execute_right_cup_pose(robot, args, state)
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
