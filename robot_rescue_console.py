from __future__ import annotations

import argparse
import csv
import subprocess
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

from connection_selector import (
    add_connection_args,
    connect_robot_with_selector,
    select_connection_profile,
)
from mantis import Mantis


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_LOG_FILE = "docs/robot_rescue_console_log.csv"
DEFAULT_ROBOT_VERSION = "3.0"
NUM_ARM_JOINTS = 7
GRIPPER_MIN = 0.0
GRIPPER_MAX = 1.0
PRE_SHUTDOWN_VERTICAL_V2_CRITICAL_JOINTS = ("shoulder_pitch", "elbow_pitch")
PRE_SHUTDOWN_VERTICAL_V2_DEFAULT_SHOULDER_PITCH = 0.000
PRE_SHUTDOWN_VERTICAL_V2_DEFAULT_ELBOW_PITCH = 1.012
PRE_SHUTDOWN_VERTICAL_V2_SHOULDER_PITCH_RANGE = (-0.30, 0.30)
PRE_SHUTDOWN_VERTICAL_V2_ELBOW_PITCH_RANGE = (0.80, 1.05)

# pre_shutdown_vertical_v2:
# 更垂直的关机前候选姿态。
# 调整原则：
# - shoulder_pitch 减小，使大臂更向下；
# - elbow_pitch 增大，使小臂更向下，并增大大臂/小臂夹角；
# - shoulder_roll、shoulder_yaw、wrist_* 暂时保持原安全值或中性值，避免引入额外碰撞风险。
# 该姿态必须先单臂空载实机验证，再用于双臂关机前收尾。
#
# 实机测试记录：
# 2026-05-18 测试 shoulder_pitch=0.200, elbow_pitch=1.012 时，
# 观察到最终姿态仍不够垂直，且运动过程中有先抬起再放下现象。
# 因此本次优先将 shoulder_pitch 从 0.200 继续减小到 0.000，
# elbow_pitch 暂时保持 1.012，避免继续逼近肘关节上限。
RIGHT_PRE_SHUTDOWN_POSE_VERTICAL_V2 = {
    "shoulder_pitch": PRE_SHUTDOWN_VERTICAL_V2_DEFAULT_SHOULDER_PITCH,
    "shoulder_yaw": 0.00,
    "shoulder_roll": 0.00,
    "elbow_pitch": PRE_SHUTDOWN_VERTICAL_V2_DEFAULT_ELBOW_PITCH,
    "wrist_roll": 0.00,
    "wrist_pitch": 0.00,
    "wrist_yaw": 0.00,
}

LEFT_PRE_SHUTDOWN_POSE_VERTICAL_V2 = {
    "shoulder_pitch": PRE_SHUTDOWN_VERTICAL_V2_DEFAULT_SHOULDER_PITCH,
    "shoulder_yaw": 0.00,
    "shoulder_roll": 0.00,
    "elbow_pitch": PRE_SHUTDOWN_VERTICAL_V2_DEFAULT_ELBOW_PITCH,
    "wrist_roll": 0.00,
    "wrist_pitch": 0.00,
    "wrist_yaw": 0.00,
}

JOINT_INFOS = [
    ("shoulder_pitch", "肩俯仰"),
    ("shoulder_yaw", "肩偏航"),
    ("shoulder_roll", "肩翻滚"),
    ("elbow_pitch", "肘俯仰"),
    ("wrist_roll", "腕翻滚"),
    ("wrist_pitch", "腕俯仰"),
    ("wrist_yaw", "腕偏航"),
]

CSV_FIELDNAMES = [
    "timestamp",
    "session_id",
    "conn_profile",
    "real_ip",
    "sn",
    "robot_version",
    "menu_choice",
    "action",
    "target",
    "value",
    "side",
    "joint_index",
    "joint_name",
    "step_value",
    "command_value",
    "status",
    "error",
    "left_gripper_est",
    "right_gripper_est",
    "left_arm_estimate",
    "right_arm_estimate",
    "notes",
    "git_branch",
    "git_commit",
]

_GIT_METADATA_CACHE: Optional[tuple[str, str]] = None


@dataclass
class ConsoleState:
    session_id: str
    left_gripper_est: float
    right_gripper_est: float
    left_joint_est: list[Optional[float]] = field(
        default_factory=lambda: [None] * NUM_ARM_JOINTS
    )
    right_joint_est: list[Optional[float]] = field(
        default_factory=lambda: [None] * NUM_ARM_JOINTS
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mantis 实机救援/收尾辅助控制台（非物理急停工具）"
    )
    parser.add_argument(
        "--gripper-step",
        type=float,
        default=0.05,
        help="每次夹爪松一点/紧一点的步长，默认 0.05",
    )
    parser.add_argument(
        "--left-gripper-init",
        type=float,
        default=0.80,
        help="控制台内部记录的左夹爪初始估计位置，默认 0.80",
    )
    parser.add_argument(
        "--right-gripper-init",
        type=float,
        default=0.80,
        help="控制台内部记录的右夹爪初始估计位置，默认 0.80",
    )
    parser.add_argument(
        "--joint-step",
        type=float,
        default=0.05,
        help="单关节小步移动默认步长（rad），默认 0.05",
    )
    parser.add_argument(
        "--max-joint-step",
        type=float,
        default=0.10,
        help="单次允许的最大单关节步长（rad），默认 0.10",
    )
    parser.add_argument(
        "--log-file",
        default=DEFAULT_LOG_FILE,
        help=f"CSV 操作日志路径，默认 {DEFAULT_LOG_FILE}",
    )
    parser.add_argument(
        "--preview-pre-shutdown-pose",
        action="store_true",
        help="只预览 pre_shutdown_vertical_v2 姿态参数，不连接机器人，也不执行动作",
    )
    parser.add_argument(
        "--pre-shutdown-shoulder-pitch",
        type=float,
        default=None,
        help=(
            "覆盖 pre_shutdown_vertical_v2 的 shoulder_pitch；"
            f"默认使用 {PRE_SHUTDOWN_VERTICAL_V2_DEFAULT_SHOULDER_PITCH:.3f}"
        ),
    )
    parser.add_argument(
        "--pre-shutdown-elbow-pitch",
        type=float,
        default=None,
        help=(
            "覆盖 pre_shutdown_vertical_v2 的 elbow_pitch；"
            f"默认使用 {PRE_SHUTDOWN_VERTICAL_V2_DEFAULT_ELBOW_PITCH:.3f}"
        ),
    )
    add_connection_args(parser, default_profile="interactive")
    parser.set_defaults(robot_version=DEFAULT_ROBOT_VERSION)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    for name, value in (
        ("--gripper-step", args.gripper_step),
        ("--joint-step", args.joint_step),
        ("--max-joint-step", args.max_joint_step),
    ):
        if value <= 0.0:
            raise ValueError(f"{name} 必须大于 0")

    if args.joint_step > args.max_joint_step:
        raise ValueError("--joint-step 不能大于 --max-joint-step")

    for name, value in (
        ("--left-gripper-init", args.left_gripper_init),
        ("--right-gripper-init", args.right_gripper_init),
    ):
        if not GRIPPER_MIN <= value <= GRIPPER_MAX:
            raise ValueError(f"{name} 必须在 0.0 到 1.0 之间")

    shoulder_pitch = get_pre_shutdown_vertical_v2_shoulder_pitch(args)
    shoulder_lower, shoulder_upper = PRE_SHUTDOWN_VERTICAL_V2_SHOULDER_PITCH_RANGE
    if not shoulder_lower <= shoulder_pitch <= shoulder_upper:
        raise ValueError(
            "--pre-shutdown-shoulder-pitch 超出允许范围 "
            f"[{shoulder_lower:.2f}, {shoulder_upper:.2f}]"
        )

    elbow_pitch = get_pre_shutdown_vertical_v2_elbow_pitch(args)
    elbow_lower, elbow_upper = PRE_SHUTDOWN_VERTICAL_V2_ELBOW_PITCH_RANGE
    if not elbow_lower <= elbow_pitch <= elbow_upper:
        raise ValueError(
            "--pre-shutdown-elbow-pitch 超出允许范围 "
            f"[{elbow_lower:.2f}, {elbow_upper:.2f}]"
        )


def resolve_log_path(log_file: str) -> Path:
    path = Path(log_file).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def format_optional_value(value: Optional[float]) -> str:
    if value is None:
        return "?"
    return f"{value:.3f}"


def format_gripper_value(value: float) -> str:
    return f"{value:.2f}"


def format_joint_estimates(values: list[Optional[float]]) -> str:
    return "[" + ", ".join(format_optional_value(value) for value in values) + "]"


def get_pre_shutdown_vertical_v2_shoulder_pitch(args: argparse.Namespace) -> float:
    if args.pre_shutdown_shoulder_pitch is not None:
        return args.pre_shutdown_shoulder_pitch
    return PRE_SHUTDOWN_VERTICAL_V2_DEFAULT_SHOULDER_PITCH


def get_pre_shutdown_vertical_v2_elbow_pitch(args: argparse.Namespace) -> float:
    if args.pre_shutdown_elbow_pitch is not None:
        return args.pre_shutdown_elbow_pitch
    return PRE_SHUTDOWN_VERTICAL_V2_DEFAULT_ELBOW_PITCH


def get_pre_shutdown_vertical_v2_pitch_sources(
    args: argparse.Namespace,
) -> tuple[str, str]:
    shoulder_source = (
        "CLI 覆盖"
        if args.pre_shutdown_shoulder_pitch is not None
        else "默认值"
    )
    elbow_source = (
        "CLI 覆盖"
        if args.pre_shutdown_elbow_pitch is not None
        else "默认值"
    )
    return shoulder_source, elbow_source


def pose_dict_to_joint_estimates(pose: dict[str, Optional[float]]) -> list[Optional[float]]:
    return [pose.get(joint_name) for joint_name, _ in JOINT_INFOS]


def format_pose_dict(pose: dict[str, Optional[float]]) -> str:
    joint_parts = [
        f"{joint_name}={format_optional_value(pose.get(joint_name))}"
        for joint_name, _ in JOINT_INFOS
    ]
    return "pose=pre_shutdown_vertical_v2, " + ", ".join(joint_parts)


def get_pre_shutdown_vertical_v2_pose(side: str) -> dict[str, Optional[float]]:
    if side == "left":
        return LEFT_PRE_SHUTDOWN_POSE_VERTICAL_V2
    return RIGHT_PRE_SHUTDOWN_POSE_VERTICAL_V2


def resolve_pre_shutdown_vertical_v2_pose(
    args: argparse.Namespace,
    side: str,
) -> dict[str, Optional[float]]:
    pose = dict(get_pre_shutdown_vertical_v2_pose(side))
    pose["shoulder_pitch"] = get_pre_shutdown_vertical_v2_shoulder_pitch(args)
    pose["elbow_pitch"] = get_pre_shutdown_vertical_v2_elbow_pitch(args)
    return pose


def get_git_metadata() -> tuple[str, str]:
    global _GIT_METADATA_CACHE
    if _GIT_METADATA_CACHE is not None:
        return _GIT_METADATA_CACHE

    def run_git_command(command: list[str]) -> str:
        result = subprocess.run(
            command,
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            return "unknown"
        value = result.stdout.strip()
        return value or "unknown"

    _GIT_METADATA_CACHE = (
        run_git_command(["git", "branch", "--show-current"]),
        run_git_command(["git", "rev-parse", "--short", "HEAD"]),
    )
    return _GIT_METADATA_CACHE


def ensure_log_schema(log_path: Path) -> None:
    if not log_path.exists() or log_path.stat().st_size == 0:
        return

    with log_path.open("r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        existing_fieldnames = reader.fieldnames or []
        if existing_fieldnames == CSV_FIELDNAMES:
            return
        existing_rows = list(reader)

    migrated_rows = []
    for row in existing_rows:
        migrated_row = {field: row.get(field, "") for field in CSV_FIELDNAMES}
        if not migrated_row["target"]:
            migrated_row["target"] = row.get("side", "")
        if not migrated_row["value"]:
            migrated_row["value"] = row.get("command_value", "")
        migrated_rows.append(migrated_row)

    with log_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        writer.writerows(migrated_rows)


def append_log_row(
    args: argparse.Namespace,
    state: ConsoleState,
    *,
    menu_choice: str,
    action: str,
    status: str,
    target: str = "",
    value: str = "",
    side: str = "",
    joint_index: str = "",
    joint_name: str = "",
    step_value: str = "",
    command_value: str = "",
    error: str = "",
    notes: str = "",
) -> None:
    if getattr(args, "print_connection_config", False):
        return

    log_path = resolve_log_path(args.log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    ensure_log_schema(log_path)
    need_header = not log_path.exists() or log_path.stat().st_size == 0
    git_branch, git_commit = get_git_metadata()

    row = {
        "timestamp": now_iso(),
        "session_id": state.session_id,
        "conn_profile": getattr(args, "effective_conn_profile", args.conn_profile),
        "real_ip": args.real_ip,
        "sn": args.sn,
        "robot_version": getattr(args, "robot_version", ""),
        "menu_choice": menu_choice,
        "action": action,
        "target": target or side,
        "value": value or command_value,
        "side": side,
        "joint_index": joint_index,
        "joint_name": joint_name,
        "step_value": step_value,
        "command_value": command_value,
        "status": status,
        "error": error,
        "left_gripper_est": f"{state.left_gripper_est:.3f}",
        "right_gripper_est": f"{state.right_gripper_est:.3f}",
        "left_arm_estimate": format_joint_estimates(state.left_joint_est),
        "right_arm_estimate": format_joint_estimates(state.right_joint_est),
        "notes": notes,
        "git_branch": git_branch,
        "git_commit": git_commit,
    }

    with log_path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        if need_header:
            writer.writeheader()
        writer.writerow(row)


def print_safety_notice(args: argparse.Namespace) -> None:
    print("=" * 72)
    print("Mantis 救援/收尾辅助控制台")
    print("=" * 72)
    print("安全提示：")
    print("- 当前脚本会控制真实机器人或仿真 Bridge；")
    print("- 本脚本是救援/收尾辅助工具，不是物理急停；")
    print("- 遇到立即碰撞、夹坏物体、失稳掉落等危险时，请优先使用物理急停；")
    print("- 执行 home、单关节移动、关机前姿态前，必须确认周围安全；")
    print("- 如果夹爪正夹着杯子或拉花壶，不要无条件打开夹爪，必须确认物体由桌面或人工支撑；")
    print("- 关机前垂直安全姿态 pre_shutdown_vertical_v2 需要先单臂空载实机验证。")
    print("=" * 72)
    print("[当前配置]")
    print(f"  robot_version: {getattr(args, 'robot_version', '')}")
    print(f"  conn_profile: {args.conn_profile}")
    print(f"  real_ip: {args.real_ip}")
    print(f"  sn: {args.sn}")
    print(f"  gripper_step: {args.gripper_step}")
    print(f"  left_gripper_init: {args.left_gripper_init}")
    print(f"  right_gripper_init: {args.right_gripper_init}")
    print(f"  joint_step: {args.joint_step}")
    print(f"  max_joint_step: {args.max_joint_step}")
    print(f"  log_file: {resolve_log_path(args.log_file)}")
    print(
        "[关节估计说明] 控制台只记录本次会话中最后一次命令位置。"
        "刚连接时关节真实姿态未知，不会自动假设当前就在 home。"
    )


def print_menu(state: ConsoleState) -> None:
    print()
    print("================ Mantis 救援/收尾控制台 ================")
    print(
        "当前记录夹爪位置："
        f"left={format_gripper_value(state.left_gripper_est)}, "
        f"right={format_gripper_value(state.right_gripper_est)}"
    )
    print(f"左臂关节估计：{format_joint_estimates(state.left_joint_est)}")
    print(f"右臂关节估计：{format_joint_estimates(state.right_joint_est)}")
    print()
    print("[夹爪]")
    print("1  右夹爪松一点")
    print("2  右夹爪紧一点")
    print("3  左夹爪松一点")
    print("4  左夹爪紧一点")
    print("5  打开右夹爪")
    print("6  打开左夹爪")
    print("7  打开双夹爪")
    print()
    print("[机械臂]")
    print("8  右臂 home")
    print("9  左臂 home")
    print("10 双臂 home")
    print()
    print("[单关节小步退限位]")
    print("11 右臂单关节小步移动")
    print("12 左臂单关节小步移动")
    print()
    print("[关机前收尾]")
    print("13 右臂关机前垂直安全姿态 pre_shutdown_vertical_v2")
    print("14 左臂关机前垂直安全姿态 pre_shutdown_vertical_v2")
    print("15 双臂关机前垂直安全姿态 pre_shutdown_vertical_v2")
    print("16 预览关机前垂直安全姿态参数，不执行动作")
    print()
    print("[系统]")
    print("s  尝试 robot.stop()")
    print("q  退出并 disconnect")


def confirm_action(prompt: str) -> bool:
    user_input = input(f"{prompt}\n输入 y 或 Y 执行，其他任意输入取消：").strip()
    return user_input.lower() == "y"


def get_arm(robot: Mantis, side: str):
    return robot.left_arm if side == "left" else robot.right_arm


def get_gripper(robot: Mantis, side: str):
    return robot.left_gripper if side == "left" else robot.right_gripper


def get_joint_estimates(state: ConsoleState, side: str) -> list[Optional[float]]:
    return state.left_joint_est if side == "left" else state.right_joint_est


def set_joint_estimates(
    state: ConsoleState,
    side: str,
    values: list[Optional[float]],
) -> None:
    if side == "left":
        state.left_joint_est = list(values)
    else:
        state.right_joint_est = list(values)


def set_gripper_estimate(state: ConsoleState, side: str, value: float) -> None:
    value = clamp(value, GRIPPER_MIN, GRIPPER_MAX)
    if side == "left":
        state.left_gripper_est = value
    else:
        state.right_gripper_est = value


def get_gripper_estimate(state: ConsoleState, side: str) -> float:
    return state.left_gripper_est if side == "left" else state.right_gripper_est


def side_label(side: str) -> str:
    return "左" if side == "left" else "右"


def apply_gripper_adjustment(
    robot: Mantis,
    state: ConsoleState,
    side: str,
    delta: float,
) -> tuple[float, bool]:
    current = get_gripper_estimate(state, side)
    target = clamp(current + delta, GRIPPER_MIN, GRIPPER_MAX)
    if target == current:
        print(f"{side_label(side)}夹爪已经在当前方向的边界位置，未发送新命令。")
        return target, False
    get_gripper(robot, side).set_position(target, block=True)
    set_gripper_estimate(state, side, target)
    print(f"{side_label(side)}夹爪命令位置更新为 {target:.2f}")
    if delta > 0.0:
        print("请继续观察物体是否有滑脱风险。")
    else:
        print("请继续观察杯壁/壶身是否受压过大。")
    return target, True


def run_open_gripper(
    robot: Mantis,
    state: ConsoleState,
    args: argparse.Namespace,
    menu_choice: str,
    side: str,
) -> None:
    label = side_label(side)
    if not confirm_action(
        f"[高风险提醒] 即将打开{label}夹爪。"
        "本命令使用 set_position(1.00) 打开夹爪，不调用 SDK open()。"
        "如果当前夹着杯子或拉花壶，请先确认物体已由桌面或人工支撑。"
    ):
        print("已取消打开夹爪。")
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action="open_gripper",
            side=side,
            status="cancelled",
            notes="open_method=set_position_not_open",
        )
        return

    print("本命令使用 set_position(1.00) 打开夹爪，不调用 SDK open()。")
    get_gripper(robot, side).set_position(GRIPPER_MAX, block=True)
    set_gripper_estimate(state, side, GRIPPER_MAX)
    print(f"{label}夹爪已打开。")
    append_log_row(
        args,
        state,
        menu_choice=menu_choice,
        action="open_gripper",
        side=side,
        command_value=f"{GRIPPER_MAX:.2f}",
        status="success",
        notes="open_method=set_position_not_open",
    )


def run_open_both_grippers(
    robot: Mantis,
    state: ConsoleState,
    args: argparse.Namespace,
) -> None:
    if not confirm_action(
        "[高风险提醒] 即将打开双夹爪。"
        "本命令使用 set_position(1.00) 打开夹爪，不调用 SDK open()。"
        "如果当前夹着杯子或拉花壶，请先确认物体已由桌面或人工支撑。"
    ):
        print("已取消打开双夹爪。")
        append_log_row(
            args,
            state,
            menu_choice="7",
            action="open_gripper",
            side="both",
            status="cancelled",
            notes="open_method=set_position_not_open",
        )
        return

    print("本命令使用 set_position(1.00) 打开夹爪，不调用 SDK open()。")
    robot.left_gripper.set_position(GRIPPER_MAX, block=False)
    robot.right_gripper.set_position(GRIPPER_MAX, block=False)
    robot.wait([robot.left_gripper.joint_name, robot.right_gripper.joint_name])
    state.left_gripper_est = GRIPPER_MAX
    state.right_gripper_est = GRIPPER_MAX
    print("双夹爪已打开。")
    append_log_row(
        args,
        state,
        menu_choice="7",
        action="open_gripper",
        side="both",
        command_value=f"{GRIPPER_MAX:.2f}",
        status="success",
        notes="open_method=set_position_not_open",
    )


def run_home(
    robot: Mantis,
    state: ConsoleState,
    args: argparse.Namespace,
    menu_choice: str,
    side: str,
) -> None:
    label = side_label(side)
    if not confirm_action(
        f"[动作确认] 即将执行{label}臂 home。"
        "请确认周围安全，且手上物体已经稳定支撑。"
    ):
        print("已取消 home。")
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action="arm_home",
            side=side,
            status="cancelled",
        )
        return

    get_arm(robot, side).home(block=True)
    set_joint_estimates(state, side, [0.0] * NUM_ARM_JOINTS)
    print(f"{label}臂已执行 home。")
    append_log_row(
        args,
        state,
        menu_choice=menu_choice,
        action="arm_home",
        side=side,
        command_value="[0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]",
        status="success",
    )


def run_home_both(robot: Mantis, state: ConsoleState, args: argparse.Namespace) -> None:
    if not confirm_action(
        "[动作确认] 即将执行双臂 home。"
        "请确认周围安全，且手上物体已经稳定支撑。"
    ):
        print("已取消双臂 home。")
        append_log_row(
            args,
            state,
            menu_choice="10",
            action="arm_home",
            side="both",
            status="cancelled",
        )
        return

    robot.left_arm.home(block=False)
    robot.right_arm.home(block=False)
    robot.wait(robot.left_arm.joint_names + robot.right_arm.joint_names)
    state.left_joint_est = [0.0] * NUM_ARM_JOINTS
    state.right_joint_est = [0.0] * NUM_ARM_JOINTS
    print("双臂已执行 home。")
    append_log_row(
        args,
        state,
        menu_choice="10",
        action="arm_home",
        side="both",
        command_value="[0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]",
        status="success",
    )


def print_joint_table(arm, estimates: list[Optional[float]], side: str) -> None:
    print(f"{side_label(side)}臂关节列表：")
    for index, (joint_name, joint_label) in enumerate(JOINT_INFOS):
        lower, upper = arm.get_limit(index)
        estimate = format_optional_value(estimates[index])
        print(
            f"  {index}  {joint_name:<15} {joint_label:<6} "
            f"limit=[{lower:.3f}, {upper:.3f}] est={estimate}"
        )


def prompt_joint_index() -> Optional[int]:
    user_input = input("请输入关节索引 0-6，或输入 q 取消：").strip().lower()
    if user_input == "q":
        return None
    try:
        index = int(user_input)
    except ValueError:
        print("关节索引输入无效。")
        return -1
    if not 0 <= index < NUM_ARM_JOINTS:
        print("关节索引必须在 0 到 6 之间。")
        return -1
    return index


def prompt_current_estimate(
    known_estimate: Optional[float],
    lower: float,
    upper: float,
) -> Optional[float]:
    while True:
        prompt = (
            f"请输入该关节当前估计角度 rad（建议范围 {lower:.3f} 到 {upper:.3f}）"
        )
        if known_estimate is not None:
            prompt += f"，直接回车使用 {known_estimate:.3f}"
        prompt += "；输入 q 取消："
        user_input = input(prompt).strip().lower()
        if user_input == "q":
            return None
        if user_input == "" and known_estimate is not None:
            return known_estimate
        try:
            value = float(user_input)
        except ValueError:
            print("请输入有效数字。")
            continue
        if value < lower or value > upper:
            print(
                f"输入值 {value:.3f} 超出该关节限位范围 [{lower:.3f}, {upper:.3f}]，"
                "请重新输入。"
            )
            continue
        return value


def prompt_direction() -> Optional[int]:
    while True:
        user_input = input("请输入方向：+ 表示正向小步，- 表示负向小步，q 取消：").strip()
        if user_input == "q":
            return None
        if user_input == "+":
            return 1
        if user_input == "-":
            return -1
        print("方向输入无效，请输入 +、- 或 q。")


def prompt_joint_step(args: argparse.Namespace) -> Optional[float]:
    while True:
        user_input = input(
            f"请输入步长 rad，直接回车使用默认 {args.joint_step:.3f}，"
            f"最大允许 {args.max_joint_step:.3f}；输入 q 取消："
        ).strip().lower()
        if user_input == "q":
            return None
        if user_input == "":
            return args.joint_step
        try:
            value = float(user_input)
        except ValueError:
            print("请输入有效数字。")
            continue
        if value <= 0.0:
            print("步长必须大于 0。")
            continue
        if value > args.max_joint_step:
            print(
                f"步长 {value:.3f} 超过允许上限 {args.max_joint_step:.3f}，"
                "请重新输入。"
            )
            continue
        return value


def run_single_joint_step(
    robot: Mantis,
    state: ConsoleState,
    args: argparse.Namespace,
    menu_choice: str,
    side: str,
) -> None:
    arm = get_arm(robot, side)
    estimates = get_joint_estimates(state, side)
    print_joint_table(arm, estimates, side)

    joint_index = prompt_joint_index()
    if joint_index is None:
        print("已取消单关节小步移动。")
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action="single_joint_step",
            side=side,
            status="cancelled",
        )
        return
    if joint_index < 0:
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action="single_joint_step",
            side=side,
            status="invalid_input",
            error="invalid_joint_index",
        )
        return

    lower, upper = arm.get_limit(joint_index)
    current_estimate = prompt_current_estimate(estimates[joint_index], lower, upper)
    if current_estimate is None:
        print("已取消单关节小步移动。")
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action="single_joint_step",
            side=side,
            joint_index=str(joint_index),
            joint_name=JOINT_INFOS[joint_index][0],
            status="cancelled",
        )
        return

    direction = prompt_direction()
    if direction is None:
        print("已取消单关节小步移动。")
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action="single_joint_step",
            side=side,
            joint_index=str(joint_index),
            joint_name=JOINT_INFOS[joint_index][0],
            status="cancelled",
        )
        return

    step = prompt_joint_step(args)
    if step is None:
        print("已取消单关节小步移动。")
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action="single_joint_step",
            side=side,
            joint_index=str(joint_index),
            joint_name=JOINT_INFOS[joint_index][0],
            status="cancelled",
        )
        return

    raw_target = current_estimate + direction * step
    clamped_target = clamp(raw_target, lower, upper)
    joint_name, joint_label = JOINT_INFOS[joint_index]
    print(
        f"{side_label(side)}臂 {joint_name} / {joint_label}: "
        f"current_est={current_estimate:.3f}, step={step:.3f}, raw_target={raw_target:.3f}, "
        f"clamped_target={clamped_target:.3f}"
    )
    if clamped_target != raw_target:
        print("提示：目标值已被关节限位裁剪。")
    if abs(clamped_target - current_estimate) < 1e-9:
        print("目标值与当前估计相同，未发送新命令。")
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action="single_joint_step",
            side=side,
            joint_index=str(joint_index),
            joint_name=joint_name,
            step_value=f"{step:.3f}",
            command_value=f"{clamped_target:.3f}",
            status="skipped",
            notes="target_equals_current_estimate",
        )
        return

    if not confirm_action(
        "[动作确认] 即将执行单关节小步移动。"
        "请确认这是朝远离危险/限位的方向，且周围没有碰撞风险。"
    ):
        print("已取消单关节小步移动。")
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action="single_joint_step",
            side=side,
            joint_index=str(joint_index),
            joint_name=joint_name,
            step_value=f"{step:.3f}",
            command_value=f"{clamped_target:.3f}",
            status="cancelled",
        )
        return

    arm.set_joint(joint_index, clamped_target, block=True)
    estimates[joint_index] = clamped_target
    print(f"{side_label(side)}臂 {joint_name} 已移动到 {clamped_target:.3f} rad。")
    append_log_row(
        args,
        state,
        menu_choice=menu_choice,
        action="single_joint_step",
        side=side,
        joint_index=str(joint_index),
        joint_name=joint_name,
        step_value=f"{step:.3f}",
        command_value=f"{clamped_target:.3f}",
        status="success",
        notes=(
            f"operator_current_est={current_estimate:.3f}; "
            f"raw_target={raw_target:.3f}; limit=[{lower:.3f}, {upper:.3f}]"
        ),
    )


def print_pre_shutdown_vertical_pose_for_side(
    args: argparse.Namespace,
    side: str,
    pose: dict[str, Optional[float]],
) -> None:
    print(f"{side_label(side)}臂 pre_shutdown_vertical_v2 目标值：")
    shoulder_source, elbow_source = get_pre_shutdown_vertical_v2_pitch_sources(args)
    print(
        "  来源："
        f"shoulder_pitch={shoulder_source}, elbow_pitch={elbow_source}"
    )
    has_missing_joint = False
    critical_missing = []
    for index, (joint_name, joint_label) in enumerate(JOINT_INFOS):
        value = pose.get(joint_name)
        value_text = format_optional_value(value)
        if value is None:
            has_missing_joint = True
            if joint_name in PRE_SHUTDOWN_VERTICAL_V2_CRITICAL_JOINTS:
                critical_missing.append(joint_name)
        print(
            f"  {index}  {joint_name:<15} {joint_label:<6} "
            f"target={value_text}"
        )
    if critical_missing:
        print(
            "  [禁止执行] 关键关节尚未标定："
            f"{', '.join(critical_missing)}"
        )
    elif has_missing_joint:
        print("  [提示] 存在非关键关节目标为 None，执行时会跳过这些关节。")
    else:
        print("  [可执行] 当前姿态未发现缺失值，仍需先单臂空载实机验证。")


def print_pre_shutdown_vertical_pose(args: argparse.Namespace) -> None:
    shoulder_pitch = get_pre_shutdown_vertical_v2_shoulder_pitch(args)
    elbow_pitch = get_pre_shutdown_vertical_v2_elbow_pitch(args)
    shoulder_source, elbow_source = get_pre_shutdown_vertical_v2_pitch_sources(args)
    print("[关机前垂直安全姿态预览] pre_shutdown_vertical_v2")
    print(
        f"当前实际使用：shoulder_pitch={shoulder_pitch:.3f} ({shoulder_source}), "
        f"elbow_pitch={elbow_pitch:.3f} ({elbow_source})"
    )
    print("方向说明：")
    print("- shoulder_pitch 越小，大臂越向下；")
    print("- elbow_pitch 越大，小臂越向下；")
    print("- 当前主要通过减小 shoulder_pitch 进一步提高垂直程度。")
    print(
        "如果仍不够垂直，下一步优先尝试更小的 shoulder_pitch，例如 -0.05 或 -0.10；"
    )
    print(
        "不要优先继续增大 elbow_pitch，因为 elbow_pitch 当前已经接近上限。"
    )
    print_pre_shutdown_vertical_pose_for_side(
        args,
        "right",
        resolve_pre_shutdown_vertical_v2_pose(args, "right"),
    )
    print_pre_shutdown_vertical_pose_for_side(
        args,
        "left",
        resolve_pre_shutdown_vertical_v2_pose(args, "left"),
    )


def confirm_pre_shutdown_action(side: str) -> bool:
    print("即将执行关机前垂直安全姿态 pre_shutdown_vertical_v2。")
    print()
    print("请确认：")
    print("1. 当前没有夹持杯子、拉花壶或其他物体；")
    print("2. 机械臂周围无障碍；")
    print("3. 桌面、身体、咖啡机不会被碰撞；")
    print("4. 这是新的更垂直候选姿态，必须先单臂空载测试；")
    print("5. 真正危险情况请使用物理急停；")
    print("6. 如果不确定，请不要输入 y。")
    if side == "both":
        print()
        print("注意：本次将左右臂同时执行关机前垂直安全姿态。")
        print("请确认：")
        print("1. 双臂之间不会互相碰撞；")
        print("2. 双臂附近没有杯子、拉花壶、咖啡机、桌面边缘等障碍；")
        print("3. 当前不是夹持物体状态；")
        print("4. 如果不确定，请先分别执行菜单 13 和 14；")
        print("5. 输入 y 才执行，其他输入取消。")
    print()
    print("输入 y 或 Y：执行当前关机前姿态动作")
    print("输入 Enter、n、N、q、Q 或其他任意内容：取消当前动作，返回菜单")
    answer = input("确认执行请输入 y，其他输入取消：").strip()
    return answer.lower() == "y"


def validate_pre_shutdown_pose(
    arm,
    pose: dict[str, Optional[float]],
    arm_name: str,
) -> None:
    missing_critical = [
        joint_name
        for joint_name in PRE_SHUTDOWN_VERTICAL_V2_CRITICAL_JOINTS
        if pose.get(joint_name) is None
    ]
    if missing_critical:
        raise ValueError(
            f"{arm_name} pre_shutdown_vertical_v2 姿态尚未标定，"
            f"关键关节缺失: {', '.join(missing_critical)}"
        )

    for index, (joint_name, _) in enumerate(JOINT_INFOS):
        value = pose.get(joint_name)
        if value is None:
            continue
        lower, upper = arm.get_limit(index)
        if not lower <= value <= upper:
            raise ValueError(
                f"{arm_name} {joint_name}={value:.3f} 超出限位 "
                f"[{lower:.3f}, {upper:.3f}]"
            )


def apply_arm_joint_pose(
    arm,
    pose: dict[str, Optional[float]],
    arm_name: str,
) -> None:
    validate_pre_shutdown_pose(arm, pose, arm_name)
    critical_joint_names = list(PRE_SHUTDOWN_VERTICAL_V2_CRITICAL_JOINTS)
    critical_joint_serial_names = []

    for joint_name in critical_joint_names:
        value = pose.get(joint_name)
        if value is None:
            continue
        joint_label = dict(JOINT_INFOS)[joint_name]
        print(
            f"{arm_name} {joint_name} / {joint_label} -> {value:.3f} rad "
            "(关键关节，同步下发)"
        )
        getattr(arm, f"set_{joint_name}")(value, block=False)
        critical_joint_serial_names.append(joint_name)

    if critical_joint_serial_names:
        arm.wait()

    for joint_name, joint_label in JOINT_INFOS:
        if joint_name in PRE_SHUTDOWN_VERTICAL_V2_CRITICAL_JOINTS:
            continue
        value = pose.get(joint_name)
        if value is None:
            print(f"{arm_name} {joint_name} / {joint_label} 为 None，已跳过。")
            continue
        print(f"{arm_name} {joint_name} / {joint_label} -> {value:.3f} rad")
        getattr(arm, f"set_{joint_name}")(value, block=True)


def apply_pre_shutdown_vertical_pose(
    robot: Mantis,
    state: ConsoleState,
    args: argparse.Namespace,
    side: str,
) -> str:
    if side == "right":
        pose = resolve_pre_shutdown_vertical_v2_pose(args, "right")
        apply_arm_joint_pose(robot.right_arm, pose, "右臂")
        set_joint_estimates(state, "right", pose_dict_to_joint_estimates(pose))
        return format_pose_dict(pose)

    if side == "left":
        pose = resolve_pre_shutdown_vertical_v2_pose(args, "left")
        apply_arm_joint_pose(robot.left_arm, pose, "左臂")
        set_joint_estimates(state, "left", pose_dict_to_joint_estimates(pose))
        return format_pose_dict(pose)

    if side == "both":
        print("双臂执行顺序：左右臂关键关节同时开始放下。")
        right_pose = resolve_pre_shutdown_vertical_v2_pose(args, "right")
        left_pose = resolve_pre_shutdown_vertical_v2_pose(args, "left")
        validate_pre_shutdown_pose(robot.right_arm, right_pose, "右臂")
        validate_pre_shutdown_pose(robot.left_arm, left_pose, "左臂")

        print("同时下发左右臂 shoulder_pitch / elbow_pitch 关键关节。")
        robot.right_arm.set_shoulder_pitch(right_pose["shoulder_pitch"], block=False)
        robot.right_arm.set_elbow_pitch(right_pose["elbow_pitch"], block=False)
        robot.left_arm.set_shoulder_pitch(left_pose["shoulder_pitch"], block=False)
        robot.left_arm.set_elbow_pitch(left_pose["elbow_pitch"], block=False)
        robot.wait(
            [
                robot.right_arm.joint_names[0],
                robot.right_arm.joint_names[3],
                robot.left_arm.joint_names[0],
                robot.left_arm.joint_names[3],
            ]
        )

        for joint_name in (
            "shoulder_yaw",
            "shoulder_roll",
            "wrist_roll",
            "wrist_pitch",
            "wrist_yaw",
        ):
            joint_label = dict(JOINT_INFOS)[joint_name]
            right_value = right_pose.get(joint_name)
            if right_value is not None:
                print(f"右臂 {joint_name} / {joint_label} -> {right_value:.3f} rad")
                getattr(robot.right_arm, f"set_{joint_name}")(right_value, block=True)
            else:
                print(f"右臂 {joint_name} / {joint_label} 为 None，已跳过。")

            left_value = left_pose.get(joint_name)
            if left_value is not None:
                print(f"左臂 {joint_name} / {joint_label} -> {left_value:.3f} rad")
                getattr(robot.left_arm, f"set_{joint_name}")(left_value, block=True)
            else:
                print(f"左臂 {joint_name} / {joint_label} 为 None，已跳过。")

        set_joint_estimates(state, "right", pose_dict_to_joint_estimates(right_pose))
        set_joint_estimates(state, "left", pose_dict_to_joint_estimates(left_pose))
        right_value = format_pose_dict(right_pose)
        left_value = format_pose_dict(left_pose)
        return f"right=({right_value}); left=({left_value})"

    raise ValueError(f"不支持的 side: {side}")


def prompt_pre_shutdown_observation() -> str:
    return input(
        "请输入本次关机前姿态观察记录：\n"
        "例如：是否更垂直、是否先抬起再放下、是否碰撞、是否贴身体、是否适合断电前使用，可直接回车跳过："
    ).strip()


def format_pre_shutdown_vertical_v2_value(
    args: argparse.Namespace,
    side: str,
) -> str:
    if side == "right":
        return format_pose_dict(resolve_pre_shutdown_vertical_v2_pose(args, "right"))
    if side == "left":
        return format_pose_dict(resolve_pre_shutdown_vertical_v2_pose(args, "left"))
    if side == "both":
        right_value = format_pose_dict(resolve_pre_shutdown_vertical_v2_pose(args, "right"))
        left_value = format_pose_dict(resolve_pre_shutdown_vertical_v2_pose(args, "left"))
        return f"right=({right_value}); left=({left_value})"
    raise ValueError(f"不支持的 side: {side}")


def run_pre_shutdown_vertical_pose(
    robot: Mantis,
    state: ConsoleState,
    args: argparse.Namespace,
    menu_choice: str,
    side: str,
) -> None:
    action_map = {
        "right": "right_pre_shutdown_vertical_v2",
        "left": "left_pre_shutdown_vertical_v2",
        "both": "both_pre_shutdown_vertical_v2",
    }
    target_map = {
        "right": "right_arm",
        "left": "left_arm",
        "both": "both_arms",
    }
    planned_value = format_pre_shutdown_vertical_v2_value(args, side)

    print_pre_shutdown_vertical_pose(args)
    if not confirm_pre_shutdown_action(side):
        print("已取消当前关机前姿态动作。")
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action=action_map[side],
            target=target_map[side],
            value=planned_value,
            side=side,
            status="skipped",
            notes="operator_cancelled",
        )
        return

    try:
        command_value = apply_pre_shutdown_vertical_pose(robot, state, args, side)
    except Exception as exc:
        error_message = f"{type(exc).__name__}: {exc}"
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action=action_map[side],
            target=target_map[side],
            value=planned_value,
            side=side,
            status="failed",
            error=error_message,
        )
        raise

    print(f"{target_map[side]} 已执行 pre_shutdown_vertical_v2。")
    notes = prompt_pre_shutdown_observation()
    append_log_row(
        args,
        state,
        menu_choice=menu_choice,
        action=action_map[side],
        target=target_map[side],
        value=command_value,
        side=side,
        command_value=command_value,
        status="success",
        notes=notes,
    )


def run_console(robot: Mantis, state: ConsoleState, args: argparse.Namespace) -> None:
    while True:
        print_menu(state)
        choice = input("请输入菜单编号：").strip().lower()

        if choice == "q":
            print("准备断开连接并退出。")
            append_log_row(
                args,
                state,
                menu_choice="q",
                action="disconnect_request",
                side="",
                status="success",
            )
            return

        if choice == "s":
            robot.stop()
            print("已尝试调用 robot.stop()。")
            print("[提示] 当前 SDK 的 robot.stop() 只会停止底盘，不会主动停止手臂/夹爪轨迹。")
            append_log_row(
                args,
                state,
                menu_choice="s",
                action="robot_stop",
                side="",
                status="success",
                notes="current_sdk_stop_only_stops_chassis",
            )
            continue

        try:
            if choice == "1":
                target, moved = apply_gripper_adjustment(
                    robot, state, "right", args.gripper_step
                )
                append_log_row(
                    args,
                    state,
                    menu_choice="1",
                    action="gripper_adjust",
                    side="right",
                    step_value=f"{args.gripper_step:.3f}",
                    command_value=f"{target:.3f}",
                    status="success" if moved else "skipped",
                    notes="loosen",
                )
            elif choice == "2":
                target, moved = apply_gripper_adjustment(
                    robot, state, "right", -args.gripper_step
                )
                append_log_row(
                    args,
                    state,
                    menu_choice="2",
                    action="gripper_adjust",
                    side="right",
                    step_value=f"{args.gripper_step:.3f}",
                    command_value=f"{target:.3f}",
                    status="success" if moved else "skipped",
                    notes="tighten",
                )
            elif choice == "3":
                target, moved = apply_gripper_adjustment(
                    robot, state, "left", args.gripper_step
                )
                append_log_row(
                    args,
                    state,
                    menu_choice="3",
                    action="gripper_adjust",
                    side="left",
                    step_value=f"{args.gripper_step:.3f}",
                    command_value=f"{target:.3f}",
                    status="success" if moved else "skipped",
                    notes="loosen",
                )
            elif choice == "4":
                target, moved = apply_gripper_adjustment(
                    robot, state, "left", -args.gripper_step
                )
                append_log_row(
                    args,
                    state,
                    menu_choice="4",
                    action="gripper_adjust",
                    side="left",
                    step_value=f"{args.gripper_step:.3f}",
                    command_value=f"{target:.3f}",
                    status="success" if moved else "skipped",
                    notes="tighten",
                )
            elif choice == "5":
                run_open_gripper(robot, state, args, "5", "right")
            elif choice == "6":
                run_open_gripper(robot, state, args, "6", "left")
            elif choice == "7":
                run_open_both_grippers(robot, state, args)
            elif choice == "8":
                run_home(robot, state, args, "8", "right")
            elif choice == "9":
                run_home(robot, state, args, "9", "left")
            elif choice == "10":
                run_home_both(robot, state, args)
            elif choice == "11":
                run_single_joint_step(robot, state, args, "11", "right")
            elif choice == "12":
                run_single_joint_step(robot, state, args, "12", "left")
            elif choice == "13":
                run_pre_shutdown_vertical_pose(robot, state, args, "13", "right")
            elif choice == "14":
                run_pre_shutdown_vertical_pose(robot, state, args, "14", "left")
            elif choice == "15":
                run_pre_shutdown_vertical_pose(robot, state, args, "15", "both")
            elif choice == "16":
                print_pre_shutdown_vertical_pose(args)
            else:
                print("菜单输入无效，请重新输入。")
        except Exception as exc:
            error_message = f"{type(exc).__name__}: {exc}"
            print(f"执行失败: {error_message}")
            append_log_row(
                args,
                state,
                menu_choice=choice,
                action="menu_action",
                side="",
                status="failed",
                error=error_message,
            )


def main() -> None:
    args = parse_args()
    validate_args(args)
    print_safety_notice(args)

    if args.preview_pre_shutdown_pose:
        print_pre_shutdown_vertical_pose(args)
        if not args.print_connection_config:
            return

    state = ConsoleState(
        session_id=now_iso(),
        left_gripper_est=args.left_gripper_init,
        right_gripper_est=args.right_gripper_init,
    )

    robot: Optional[Mantis] = None
    try:
        effective_profile = select_connection_profile(args, script_name=__file__)
        args.effective_conn_profile = effective_profile
        args.conn_profile = effective_profile
        robot = connect_robot_with_selector(args, script_name=__file__)
        if robot is None:
            print("仅打印连接配置，未连接机器人，也未进入控制台。")
            return

        append_log_row(
            args,
            state,
            menu_choice="session",
            action="connect",
            side="",
            status="success",
        )
        run_console(robot, state, args)
    except SystemExit as exc:
        message = str(exc).strip() or "连接流程结束"
        print(message)
        append_log_row(
            args,
            state,
            menu_choice="session",
            action="session_setup",
            side="",
            status="skipped" if "用户取消" in message else "failed",
            error=message if "用户取消" not in message else "",
            notes=message,
        )
    except Exception as exc:
        error_message = f"{type(exc).__name__}: {exc}"
        print(f"脚本执行失败: {error_message}")
        append_log_row(
            args,
            state,
            menu_choice="session",
            action="session_setup",
            side="",
            status="failed",
            error=error_message,
        )
    except KeyboardInterrupt:
        print("\n检测到 Ctrl-C，准备断开连接退出。")
        append_log_row(
            args,
            state,
            menu_choice="interrupt",
            action="keyboard_interrupt",
            side="",
            status="interrupted",
        )
    finally:
        if robot is not None:
            try:
                robot.disconnect()
                append_log_row(
                    args,
                    state,
                    menu_choice="session",
                    action="disconnect",
                    side="",
                    status="success",
                )
            except Exception as exc:
                print(f"断开连接时忽略异常: {exc}")


if __name__ == "__main__":
    main()
