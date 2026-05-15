from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import subprocess
import time
from typing import Optional

from connection_selector import (
    add_connection_args,
    connect_robot_with_selector,
    select_connection_profile,
)
from mantis import Mantis


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_LOG_FILE = "docs/latte_tool_grasp_test_log.csv"
CSV_FIELDNAMES = [
    "timestamp",
    "mode",
    "conn_profile",
    "real_ip",
    "sn",
    "right_gripper_pos",
    "left_gripper_pos",
    "right_final_gripper_pos",
    "left_final_gripper_pos",
    "hold_seconds",
    "status",
    "error",
    "duration_s",
    "notes",
    "git_branch",
    "git_commit",
]


class UserAbort(Exception):
    """Raised when the operator chooses to stop before a risky action."""


@dataclass
class GripTuneResult:
    status: str
    position: Optional[float]
    holding: bool


@dataclass
class HoldResult:
    status: str
    holding: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mantis 咖啡杯/拉花壶抓取与静态持稳测试工具（不执行倒奶/拉花，不运行完整 coffee 流程）"
    )
    parser.add_argument(
        "--mode",
        choices=("right-cup", "left-pitcher", "both-static"),
        default="right-cup",
        help="测试模式：right-cup 只测右手抓杯；left-pitcher 只测左手抓拉花壶；both-static 双手同时持物静态观察",
    )
    parser.add_argument(
        "--right-gripper-pos",
        type=float,
        default=0.80,
        help="右手抓咖啡杯时建议的目标夹爪位置",
    )
    parser.add_argument(
        "--left-gripper-pos",
        type=float,
        default=0.75,
        help="左手抓拉花壶时建议的目标夹爪位置",
    )
    parser.add_argument(
        "--right-gripper-start-pos",
        type=float,
        default=0.90,
        help="右手抓咖啡杯的分步试探起始夹爪位置",
    )
    parser.add_argument(
        "--left-gripper-start-pos",
        type=float,
        default=0.90,
        help="左手抓拉花壶的分步试探起始夹爪位置",
    )
    parser.add_argument(
        "--gripper-step",
        type=float,
        default=0.05,
        help="每次收紧或放松夹爪的步长",
    )
    parser.add_argument(
        "--min-safe-gripper-pos",
        type=float,
        default=0.60,
        help="建议的最小安全夹爪位置；低于该值时会要求高风险二次确认",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=3.0,
        help="抓住物体后的静态观察时长（秒）",
    )
    parser.add_argument(
        "--ask-notes",
        action="store_true",
        help="测试结束后提示输入观察备注，并写入 CSV 日志",
    )
    parser.add_argument(
        "--notes",
        default="",
        help="本次测试备注",
    )
    parser.add_argument(
        "--log-file",
        default=DEFAULT_LOG_FILE,
        help="CSV 测试日志路径，默认 docs/latte_tool_grasp_test_log.csv",
    )
    add_connection_args(parser, default_profile="interactive")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not 0.0 <= args.right_gripper_pos <= 1.0:
        raise ValueError("--right-gripper-pos 必须在 0.0 到 1.0 之间")
    if not 0.0 <= args.left_gripper_pos <= 1.0:
        raise ValueError("--left-gripper-pos 必须在 0.0 到 1.0 之间")
    if not 0.0 <= args.right_gripper_start_pos <= 1.0:
        raise ValueError("--right-gripper-start-pos 必须在 0.0 到 1.0 之间")
    if not 0.0 <= args.left_gripper_start_pos <= 1.0:
        raise ValueError("--left-gripper-start-pos 必须在 0.0 到 1.0 之间")
    if not 0.0 <= args.min_safe_gripper_pos <= 1.0:
        raise ValueError("--min-safe-gripper-pos 必须在 0.0 到 1.0 之间")
    if args.gripper_step <= 0.0:
        raise ValueError("--gripper-step 必须大于 0")
    if args.right_gripper_start_pos < args.right_gripper_pos:
        raise ValueError("--right-gripper-start-pos 应大于等于 --right-gripper-pos")
    if args.left_gripper_start_pos < args.left_gripper_pos:
        raise ValueError("--left-gripper-start-pos 应大于等于 --left-gripper-pos")
    if args.hold_seconds < 0.0:
        raise ValueError("--hold-seconds 不能为负数")


def confirm_or_exit(message: str) -> None:
    print(message)
    user_input = input("按 Enter 继续，输入 q / Q 退出：").strip().lower()
    if user_input == "q":
        raise UserAbort("用户取消执行")


def print_safety_notice(args: argparse.Namespace) -> None:
    print("=" * 72)
    print("安全提示：")
    print("- 当前脚本会控制真实机器人或仿真 Bridge；")
    print("- 本脚本只用于杯子/拉花壶抓取与持稳测试；")
    print("- 第一次测试必须空杯、空壶；")
    print("- 不要放热咖啡、热水或牛奶；")
    print("- 陶瓷杯可能被夹坏；")
    print("- 第一次测试不要使用 0.6；")
    print("- 建议从 0.85 或 0.80 附近开始试探；")
    print("- 每次收紧都要观察杯壁是否受压、变形、打滑；")
    print("- 若发现受压，立即输入 o 打开夹爪释放；")
    print("- 请清空机器人周围障碍物；")
    print("- 请确认杯子和拉花壶放置稳定；")
    print("- 请准备物理急停；")
    print("- 当前脚本不是稳定产品，只是宣传 demo 前的实机调试工具。")
    print("=" * 72)
    print("[测试配置]")
    print(f"  mode: {args.mode}")
    print(f"  right_gripper_pos: {args.right_gripper_pos}")
    print(f"  left_gripper_pos: {args.left_gripper_pos}")
    print(f"  right_gripper_start_pos: {args.right_gripper_start_pos}")
    print(f"  left_gripper_start_pos: {args.left_gripper_start_pos}")
    print(f"  gripper_step: {args.gripper_step}")
    print(f"  min_safe_gripper_pos: {args.min_safe_gripper_pos}")
    print(f"  hold_seconds: {args.hold_seconds}")
    print(f"  conn_profile: {args.conn_profile}")
    print(f"  real_ip: {args.real_ip}")
    print(f"  sn: {args.sn}")
    print(f"  log_file: {resolve_log_path(args.log_file)}")
    print(f"  notes: {args.notes}")
    print_risky_gripper_targets(args)


def resolve_log_path(log_file: str) -> Path:
    path = Path(log_file).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def get_git_value(command: list[str]) -> str:
    try:
        result = subprocess.run(
            command,
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout.strip()
    except Exception:
        return ""


def get_git_info() -> tuple[str, str]:
    branch = get_git_value(["git", "rev-parse", "--abbrev-ref", "HEAD"])
    commit = get_git_value(["git", "rev-parse", "--short", "HEAD"])
    return branch, commit


def combine_notes(*parts: str) -> str:
    cleaned = [part.strip() for part in parts if part and part.strip()]
    return " | ".join(cleaned)


def prompt_for_notes() -> str:
    return input("请输入本次抓取测试观察，可直接回车跳过：").strip()


def append_log_row(
    args: argparse.Namespace,
    status: str,
    error: str,
    duration_s: float,
    notes: str,
) -> None:
    git_branch, git_commit = get_git_info()
    log_path = resolve_log_path(args.log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    upgrade_log_header_if_needed(log_path)
    need_header = not log_path.exists() or log_path.stat().st_size == 0
    row = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        "mode": args.mode,
        "conn_profile": getattr(args, "effective_conn_profile", args.conn_profile),
        "real_ip": args.real_ip,
        "sn": args.sn,
        "right_gripper_pos": args.right_gripper_pos,
        "left_gripper_pos": args.left_gripper_pos,
        "right_final_gripper_pos": format_gripper_pos(getattr(args, "right_final_gripper_pos", None)),
        "left_final_gripper_pos": format_gripper_pos(getattr(args, "left_final_gripper_pos", None)),
        "hold_seconds": args.hold_seconds,
        "status": status,
        "error": error,
        "duration_s": f"{duration_s:.3f}",
        "notes": notes,
        "git_branch": git_branch,
        "git_commit": git_commit,
    }
    with log_path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        if need_header:
            writer.writeheader()
        writer.writerow(row)


def upgrade_log_header_if_needed(log_path: Path) -> None:
    if not log_path.exists() or log_path.stat().st_size == 0:
        return

    with log_path.open("r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames == CSV_FIELDNAMES:
            return
        rows = [{field: row.get(field, "") for field in CSV_FIELDNAMES} for row in reader]

    with log_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def format_gripper_pos(value: Optional[float]) -> str:
    if value is None:
        return ""
    return f"{value:.2f}"


def print_risky_gripper_targets(args: argparse.Namespace) -> None:
    if args.right_gripper_pos <= args.min_safe_gripper_pos:
        print(
            f"[高风险提示] --right-gripper-pos={args.right_gripper_pos:.2f} 低于或等于建议安全值 "
            f"{args.min_safe_gripper_pos:.2f}。"
        )
    if args.left_gripper_pos <= args.min_safe_gripper_pos:
        print(
            f"[高风险提示] --left-gripper-pos={args.left_gripper_pos:.2f} 低于或等于建议安全值 "
            f"{args.min_safe_gripper_pos:.2f}。"
        )


def set_gripper_position(gripper, side_name: str, position: float) -> None:
    position = max(0.0, min(1.0, position))
    gripper.set_position(position)
    print(f"{side_name} 当前夹爪位置: {position:.2f}")


def confirm_high_risk_tightening(
    side_name: str,
    proposed_pos: float,
    min_safe_pos: float,
) -> bool:
    print(
        f"[高风险提示] {side_name} 即将收紧到 {proposed_pos:.2f}，"
        f"已低于建议安全值 {min_safe_pos:.2f}。"
    )
    print("请确认杯壁/壶身没有受压、变形或明显打滑风险。")
    user_input = input("如确认继续，请输入 YES；其他任意输入取消本次收紧：").strip()
    return user_input == "YES"


def prompt_release_choice(side_name: str) -> str:
    return input(
        f"{side_name} 当前可能仍在夹持物体。输入 o 立即打开夹爪，其他任意输入保持当前状态退出："
    ).strip().lower()


def interactive_grip_tune(
    gripper,
    side_name: str,
    start_pos: float,
    target_pos: float,
    step: float,
    min_safe_pos: float,
) -> GripTuneResult:
    current_pos = max(0.0, min(1.0, start_pos))
    set_gripper_position(gripper, side_name, current_pos)
    print(
        f"{side_name} 进入分步试探夹持。建议目标值 {target_pos:.2f}，"
        f"建议不要默认一路收紧到 0.60。"
    )

    while True:
        user_input = input(
            "输入 Enter/c 接受当前夹持，t 再收紧一步，l 放松一步，o 立即打开释放，q 退出，force 强制越过目标继续收紧："
        ).strip().lower()

        if user_input in ("", "c"):
            return GripTuneResult(status="accepted", position=current_pos, holding=True)

        if user_input == "t":
            if current_pos <= target_pos:
                print(
                    f"{side_name} 已经到达建议目标值 {target_pos:.2f}。"
                    "如确需继续低于目标值收紧，请输入 force。"
                )
                continue
            proposed_pos = max(target_pos, current_pos - step)
            if proposed_pos < min_safe_pos and not confirm_high_risk_tightening(side_name, proposed_pos, min_safe_pos):
                continue
            current_pos = proposed_pos
            set_gripper_position(gripper, side_name, current_pos)
            continue

        if user_input == "force":
            proposed_pos = max(0.0, current_pos - step)
            if proposed_pos < min_safe_pos and not confirm_high_risk_tightening(side_name, proposed_pos, min_safe_pos):
                continue
            current_pos = proposed_pos
            set_gripper_position(gripper, side_name, current_pos)
            continue

        if user_input == "l":
            proposed_pos = min(1.0, current_pos + step)
            if proposed_pos == current_pos:
                print(f"{side_name} 已经是最松位置附近，不能再放松。")
                continue
            current_pos = proposed_pos
            set_gripper_position(gripper, side_name, current_pos)
            continue

        if user_input == "o":
            gripper.open()
            print(f"{side_name} 已立即打开夹爪释放。")
            return GripTuneResult(status="released", position=None, holding=False)

        if user_input == "q":
            release_choice = prompt_release_choice(side_name)
            if release_choice == "o":
                gripper.open()
                print(f"{side_name} 已打开夹爪后退出当前流程。")
                return GripTuneResult(status="quit", position=None, holding=False)
            print(f"{side_name} 保持当前状态退出当前流程。")
            return GripTuneResult(status="quit", position=current_pos, holding=True)

        print("输入无效，请重新输入。")


def hold_with_release_prompt(
    gripper,
    seconds: float,
    label: str,
    observations: list[str],
) -> HoldResult:
    while True:
        user_input = input(
            f"{label}。按 Enter 开始保持观察，输入 o 打开夹爪释放，输入 q 退出当前流程："
        ).strip().lower()
        if user_input == "":
            break
        if user_input == "o":
            gripper.open()
            print(f"{label} 已立即打开夹爪释放。")
            return HoldResult(status="released", holding=False)
        if user_input == "q":
            release_choice = prompt_release_choice(label)
            if release_choice == "o":
                gripper.open()
                print(f"{label} 已打开夹爪后退出当前流程。")
                return HoldResult(status="quit", holding=False)
            print(f"{label} 保持当前状态退出当前流程。")
            return HoldResult(status="quit", holding=True)
        print("输入无效，请重新输入。")

    if seconds > 0.0:
        print(f"{label}，开始保持 {seconds:.1f} 秒供观察...")
        remaining = seconds
        while remaining > 0.0:
            print(f"{label} 观察中，剩余 {remaining:.1f} 秒...")
            sleep_seconds = min(0.5, remaining)
            time.sleep(sleep_seconds)
            remaining = max(0.0, remaining - sleep_seconds)
    else:
        print(f"{label}，未额外停留。")
    for item in observations:
        print(f"- {item}")
    return HoldResult(status="held", holding=True)


def move_right_arm_to_cup_grasp_pose(robot: Mantis) -> None:
    print("右手移动到咖啡杯抓取姿态附近...")
    robot.right_arm.set_shoulder_pitch(0.7, block=False)
    robot.right_arm.set_shoulder_roll(-0.42, block=False)
    robot.right_arm.set_wrist_roll(0.1, block=True)
    robot.right_arm.set_elbow_pitch(1.0, block=False)
    robot.right_arm.set_wrist_pitch(0.1)
    time.sleep(0.3)


def release_right_cup(robot: Mantis) -> None:
    print("右手打开夹爪，释放咖啡杯...")
    robot.right_gripper.open()
    time.sleep(0.3)
    print("右手已释放咖啡杯。为避免未知碰撞，当前不自动回 home，请按现场情况人工处理。")


def grasp_right_cup(robot: Mantis, args: argparse.Namespace, release_at_end: bool = True) -> None:
    print("右手打开夹爪，准备抓咖啡杯...")
    robot.right_gripper.open()
    time.sleep(0.3)
    confirm_or_exit("右手即将靠近杯子。请确认空杯放置稳定、夹持区域无遮挡。")
    move_right_arm_to_cup_grasp_pose(robot)
    print("右手已到抓杯姿态附近，请观察并确认杯子位置。")
    confirm_or_exit("右手即将闭合夹爪抓取咖啡杯。请确认不会压杯口，也不会碰撞周围物体。")
    grip_result = interactive_grip_tune(
        robot.right_gripper,
        "右手咖啡杯",
        args.right_gripper_start_pos,
        args.right_gripper_pos,
        args.gripper_step,
        args.min_safe_gripper_pos,
    )
    args.right_holding = grip_result.holding
    args.right_final_gripper_pos = grip_result.position
    if grip_result.status == "released":
        raise UserAbort("右手咖啡杯已立即释放，停止当前流程")
    if grip_result.status == "quit":
        raise UserAbort("用户在右手抓杯调参阶段退出流程")
    print(f"右手咖啡杯最终夹爪位置: {grip_result.position:.2f}")
    hold_result = hold_with_release_prompt(
        robot.right_gripper,
        args.hold_seconds,
        "右手已抓住咖啡杯",
        [
            "请观察是否滑动。",
            "请观察杯子是否倾斜。",
            "请观察夹爪是否压杯口或压杯壁不稳。",
        ],
    )
    args.right_holding = hold_result.holding
    if hold_result.status == "released":
        raise UserAbort("右手咖啡杯在观察前已释放，停止当前流程")
    if hold_result.status == "quit":
        raise UserAbort("用户在右手抓杯观察阶段退出流程")
    if release_at_end and args.right_holding:
        confirm_or_exit("右手即将释放咖啡杯。请确认杯子已被桌面稳定支撑。")
        release_right_cup(robot)
        args.right_holding = False


def move_left_arm_to_pitcher_grasp_pose(robot: Mantis) -> None:
    print("左手移动到拉花壶抓取姿态附近...")
    robot.left_arm.set_shoulder_yaw(0.3, block=False)
    robot.left_arm.set_wrist_roll(-0.4, block=False)
    robot.left_arm.set_shoulder_roll(-0.76, block=True)
    robot.left_arm.set_shoulder_pitch(0.8, block=False)
    robot.left_arm.set_elbow_pitch(0.8, block=False)
    robot.left_arm.set_wrist_roll(0.1, block=True)
    robot.left_arm.set_elbow_pitch(1.35, block=False)
    robot.left_arm.set_shoulder_pitch(0.85, block=True)
    time.sleep(0.3)


def release_left_pitcher(robot: Mantis) -> None:
    print("左手打开夹爪，释放拉花壶...")
    robot.left_gripper.open()
    time.sleep(0.3)
    print("左手已释放拉花壶。为避免未知碰撞，当前不自动回 home，请按现场情况人工处理。")


def grasp_left_pitcher(robot: Mantis, args: argparse.Namespace, release_at_end: bool = True) -> None:
    print("左手打开夹爪，准备抓拉花壶...")
    robot.left_gripper.open()
    time.sleep(0.3)
    confirm_or_exit("左手即将靠近拉花壶。请确认空壶放置稳定、壶嘴方向清楚可见。")
    move_left_arm_to_pitcher_grasp_pose(robot)
    print("左手已到抓壶姿态附近，请观察并确认拉花壶位置。")
    confirm_or_exit("左手即将闭合夹爪抓取拉花壶。请确认夹持位置合适，且不会碰撞壶身。")
    grip_result = interactive_grip_tune(
        robot.left_gripper,
        "左手拉花壶",
        args.left_gripper_start_pos,
        args.left_gripper_pos,
        args.gripper_step,
        args.min_safe_gripper_pos,
    )
    args.left_holding = grip_result.holding
    args.left_final_gripper_pos = grip_result.position
    if grip_result.status == "released":
        raise UserAbort("左手拉花壶已立即释放，停止当前流程")
    if grip_result.status == "quit":
        raise UserAbort("用户在左手抓壶调参阶段退出流程")
    print(f"左手拉花壶最终夹爪位置: {grip_result.position:.2f}")
    hold_result = hold_with_release_prompt(
        robot.left_gripper,
        args.hold_seconds,
        "左手已抓住拉花壶",
        [
            "请观察是否滑动。",
            "请观察壶嘴方向是否合理。",
            "请观察夹爪是否夹在合适位置。",
            "请观察夹爪是否会碰壶身。",
        ],
    )
    args.left_holding = hold_result.holding
    if hold_result.status == "released":
        raise UserAbort("左手拉花壶在观察前已释放，停止当前流程")
    if hold_result.status == "quit":
        raise UserAbort("用户在左手抓壶观察阶段退出流程")
    if release_at_end and args.left_holding:
        confirm_or_exit("左手即将释放拉花壶。请确认拉花壶已被桌面稳定支撑。")
        release_left_pitcher(robot)
        args.left_holding = False


def run_right_cup_mode(robot: Mantis, args: argparse.Namespace) -> None:
    print("[模式] right-cup：测试右手抓咖啡杯")
    grasp_right_cup(robot, args, release_at_end=True)


def run_left_pitcher_mode(robot: Mantis, args: argparse.Namespace) -> None:
    print("[模式] left-pitcher：测试左手抓拉花壶")
    grasp_left_pitcher(robot, args, release_at_end=True)


def run_both_static_mode(robot: Mantis, args: argparse.Namespace) -> None:
    print("[模式] both-static：双手同时持物静态观察")
    grasp_right_cup(robot, args, release_at_end=False)
    if not args.right_holding:
        raise UserAbort("右手咖啡杯未保持夹持，停止 both-static 流程")
    confirm_or_exit("右手抓杯完成。请确认杯子没有被夹坏、没有滑动，再继续左手抓壶。")
    grasp_left_pitcher(robot, args, release_at_end=False)
    if not args.left_holding:
        raise UserAbort("左手拉花壶未保持夹持，停止 both-static 流程")
    confirm_or_exit("双手已完成抓取，即将进入静态持物观察。请确认两侧周围安全。")
    hold_result = hold_with_release_prompt(
        robot.left_gripper,
        args.hold_seconds,
        "双手静态持物观察中",
        [
            "请观察右手咖啡杯是否稳定。",
            "请观察左手拉花壶是否稳定。",
            "请观察双手之间是否有碰撞风险。",
            "请观察壶嘴方向是否大致朝向杯口。",
            "请判断是否适合进入下一步 latte_pour_tune.py --mode with-right-cup。",
        ],
    )
    if hold_result.status == "released":
        args.left_holding = False
        raise UserAbort("双手静态观察前已释放左手拉花壶，停止当前流程")
    if hold_result.status == "quit":
        args.left_holding = hold_result.holding
        raise UserAbort("用户在双手静态观察前退出流程")
    confirm_or_exit("即将先释放左手拉花壶，再释放右手咖啡杯。请确认桌面支撑稳定、周围安全。")
    release_left_pitcher(robot)
    args.left_holding = False
    release_right_cup(robot)
    args.right_holding = False


def run_mode(robot: Mantis, args: argparse.Namespace) -> None:
    if args.mode == "right-cup":
        run_right_cup_mode(robot, args)
        return
    if args.mode == "left-pitcher":
        run_left_pitcher_mode(robot, args)
        return
    if args.mode == "both-static":
        run_both_static_mode(robot, args)
        return
    raise ValueError(f"不支持的模式: {args.mode}")


def format_error(exc: BaseException) -> str:
    message = str(exc).strip()
    if message:
        return f"{type(exc).__name__}: {message}"
    return type(exc).__name__


def maybe_prompt_for_notes(args: argparse.Namespace, existing_notes: str) -> str:
    if not args.ask_notes:
        return existing_notes
    try:
        return combine_notes(existing_notes, prompt_for_notes())
    except KeyboardInterrupt:
        print("\n备注输入被中断，保留已有 notes。")
        return existing_notes


def maybe_warn_and_release_remaining_holds(robot: Mantis, args: argparse.Namespace) -> None:
    if getattr(args, "right_holding", False):
        print("[警告] 当前脚本结束时右手可能仍在夹持物体。")
        print("不要直接移动机器人。")
        print("请先确认物体是否已经由桌面稳定支撑。")
        print("如需释放，请运行对应释放动作或人工处理。")
        release_choice = input("检测到右手可能仍夹持物体，是否现在打开右夹爪？输入 o 打开，其他键保持：").strip().lower()
        if release_choice == "o":
            robot.right_gripper.open()
            args.right_holding = False
            print("右手夹爪已打开。")

    if getattr(args, "left_holding", False):
        print("[警告] 当前脚本结束时左手可能仍在夹持物体。")
        print("不要直接移动机器人。")
        print("请先确认物体是否已经由桌面稳定支撑。")
        print("如需释放，请运行对应释放动作或人工处理。")
        release_choice = input("检测到左手可能仍夹持物体，是否现在打开左夹爪？输入 o 打开，其他键保持：").strip().lower()
        if release_choice == "o":
            robot.left_gripper.open()
            args.left_holding = False
            print("左手夹爪已打开。")


def main() -> None:
    args = parse_args()
    robot: Optional[Mantis] = None
    should_log_trial = not args.print_connection_config
    start_time = time.monotonic()
    status = "failed"
    error_message = ""
    combined_notes = args.notes
    args.right_holding = False
    args.left_holding = False
    args.right_final_gripper_pos = None
    args.left_final_gripper_pos = None

    try:
        validate_args(args)
        print_safety_notice(args)
        if not args.print_connection_config:
            confirm_or_exit("即将进入连接流程。请确认连接目标、现场环境和物理急停都已准备好。")
        effective_profile = select_connection_profile(args, script_name=__file__)
        args.effective_conn_profile = effective_profile
        if not args.print_connection_config:
            print(f"已选择连接模式: {effective_profile}")
        args.conn_profile = effective_profile
        robot = connect_robot_with_selector(args, script_name=__file__)
        if robot is None:
            status = "skipped"
            print("仅打印连接配置，未连接机器人，也未执行测试动作。")
            return

        run_mode(robot, args)
        status = "success"
        print("抓取测试流程执行结束。")
    except UserAbort as exc:
        status = "skipped"
        error_message = str(exc)
        print("用户取消，脚本退出。")
    except KeyboardInterrupt as exc:
        status = "interrupted"
        error_message = format_error(exc)
        print("\n检测到 Ctrl-C，脚本中止。")
    except SystemExit as exc:
        message = str(exc).strip()
        if args.print_connection_config or "用户取消" in message:
            status = "skipped"
            error_message = message
            print(message or "连接流程已结束。")
        else:
            status = "failed"
            error_message = message or "SystemExit"
            print(f"执行失败: {error_message}")
    except Exception as exc:
        status = "failed"
        error_message = format_error(exc)
        print(f"执行失败: {error_message}")
    finally:
        if robot is not None:
            try:
                maybe_warn_and_release_remaining_holds(robot, args)
            except Exception as exc:
                print(f"处理剩余夹持状态时忽略异常: {exc}")
            try:
                robot.disconnect()
            except Exception as exc:
                print(f"断开连接时忽略异常: {exc}")
        if getattr(args, "right_holding", False) or getattr(args, "left_holding", False):
            print("[醒目提示] 脚本结束时仍可能有夹爪在夹持物体。")
            print("请不要直接移动机器人。")
            print("请先确认物体由桌面稳定支撑，再决定是否释放。")
        duration_s = time.monotonic() - start_time
        if should_log_trial:
            combined_notes = maybe_prompt_for_notes(args, combined_notes)
            try:
                append_log_row(
                    args=args,
                    status=status,
                    error=error_message,
                    duration_s=duration_s,
                    notes=combined_notes,
                )
                print(f"测试记录已追加到: {resolve_log_path(args.log_file)}")
            except Exception as exc:
                print(f"写入测试日志失败: {exc}")


if __name__ == "__main__":
    main()
