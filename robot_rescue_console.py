from __future__ import annotations

import argparse
import csv
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

# 保守初版关机前收尾姿态。该姿态仍需要在空载和可控环境下先验证。
PRE_SHUTDOWN_SAFE_POSE = [0.30, 0.00, 0.00, 0.90, 0.00, 0.00, 0.00]

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
]


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


def append_log_row(
    args: argparse.Namespace,
    state: ConsoleState,
    *,
    menu_choice: str,
    action: str,
    status: str,
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
    need_header = not log_path.exists() or log_path.stat().st_size == 0

    row = {
        "timestamp": now_iso(),
        "session_id": state.session_id,
        "conn_profile": getattr(args, "effective_conn_profile", args.conn_profile),
        "real_ip": args.real_ip,
        "sn": args.sn,
        "robot_version": getattr(args, "robot_version", ""),
        "menu_choice": menu_choice,
        "action": action,
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
    print("- 关机前安全姿态需要实机验证，不要在未确认环境下执行。")
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
    print("13 右臂关机前安全姿态")
    print("14 左臂关机前安全姿态")
    print("15 双臂关机前安全姿态")
    print()
    print("[系统]")
    print("s  尝试 robot.stop()")
    print("q  退出并 disconnect")


def confirm_yes(prompt: str) -> bool:
    user_input = input(f"{prompt}\n输入 YES 继续，其他任意输入取消：").strip()
    return user_input == "YES"


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
    if not confirm_yes(
        f"[高风险提醒] 即将打开{label}夹爪。"
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
        )
        return

    get_gripper(robot, side).open(block=True)
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
    )


def run_open_both_grippers(
    robot: Mantis,
    state: ConsoleState,
    args: argparse.Namespace,
) -> None:
    if not confirm_yes(
        "[高风险提醒] 即将打开双夹爪。"
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
        )
        return

    robot.left_gripper.open(block=False)
    robot.right_gripper.open(block=False)
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
    )


def run_home(
    robot: Mantis,
    state: ConsoleState,
    args: argparse.Namespace,
    menu_choice: str,
    side: str,
) -> None:
    label = side_label(side)
    if not confirm_yes(
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
    if not confirm_yes(
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

    if not confirm_yes(
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


def print_shutdown_pose_preview(side: str) -> None:
    print(f"{side_label(side)}臂关机前安全姿态（待实机验证）目标值：")
    for index, (joint_name, joint_label) in enumerate(JOINT_INFOS):
        print(
            f"  {index}  {joint_name:<15} {joint_label:<6} "
            f"target={PRE_SHUTDOWN_SAFE_POSE[index]:.3f}"
        )


def run_shutdown_pose(
    robot: Mantis,
    state: ConsoleState,
    args: argparse.Namespace,
    menu_choice: str,
    side: str,
) -> None:
    print_shutdown_pose_preview(side)
    if not confirm_yes(
        f"[高风险提醒] 即将执行{side_label(side)}臂关机前安全姿态。"
        "该姿态是保守初版，需要先在空载且已确认环境下验证。"
    ):
        print("已取消关机前安全姿态。")
        append_log_row(
            args,
            state,
            menu_choice=menu_choice,
            action="shutdown_safe_pose",
            side=side,
            status="cancelled",
        )
        return

    get_arm(robot, side).set_joints(PRE_SHUTDOWN_SAFE_POSE, block=True)
    set_joint_estimates(state, side, PRE_SHUTDOWN_SAFE_POSE)
    print(f"{side_label(side)}臂已移动到关机前安全姿态。")
    append_log_row(
        args,
        state,
        menu_choice=menu_choice,
        action="shutdown_safe_pose",
        side=side,
        command_value=format_joint_estimates(PRE_SHUTDOWN_SAFE_POSE),
        status="success",
        notes="pose_requires_real_robot_validation",
    )


def run_shutdown_pose_both(
    robot: Mantis,
    state: ConsoleState,
    args: argparse.Namespace,
) -> None:
    print_shutdown_pose_preview("left")
    print_shutdown_pose_preview("right")
    if not confirm_yes(
        "[高风险提醒] 即将执行双臂关机前安全姿态。"
        "该姿态是保守初版，需要先在空载且已确认环境下验证。"
    ):
        print("已取消双臂关机前安全姿态。")
        append_log_row(
            args,
            state,
            menu_choice="15",
            action="shutdown_safe_pose",
            side="both",
            status="cancelled",
        )
        return

    robot.left_arm.set_joints(PRE_SHUTDOWN_SAFE_POSE, block=False)
    robot.right_arm.set_joints(PRE_SHUTDOWN_SAFE_POSE, block=False)
    robot.wait(robot.left_arm.joint_names + robot.right_arm.joint_names)
    state.left_joint_est = list(PRE_SHUTDOWN_SAFE_POSE)
    state.right_joint_est = list(PRE_SHUTDOWN_SAFE_POSE)
    print("双臂已移动到关机前安全姿态。")
    append_log_row(
        args,
        state,
        menu_choice="15",
        action="shutdown_safe_pose",
        side="both",
        command_value=format_joint_estimates(PRE_SHUTDOWN_SAFE_POSE),
        status="success",
        notes="pose_requires_real_robot_validation",
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
                run_shutdown_pose(robot, state, args, "13", "right")
            elif choice == "14":
                run_shutdown_pose(robot, state, args, "14", "left")
            elif choice == "15":
                run_shutdown_pose_both(robot, state, args)
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
