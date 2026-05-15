from __future__ import annotations

import argparse
import csv
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
        default=0.6,
        help="右手抓咖啡杯时的夹爪位置",
    )
    parser.add_argument(
        "--left-gripper-pos",
        type=float,
        default=0.6,
        help="左手抓拉花壶时的夹爪位置",
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
    print("- 请清空机器人周围障碍物；")
    print("- 请确认杯子和拉花壶放置稳定；")
    print("- 请准备物理急停；")
    print("- 当前脚本不是稳定产品，只是宣传 demo 前的实机调试工具。")
    print("=" * 72)
    print("[测试配置]")
    print(f"  mode: {args.mode}")
    print(f"  right_gripper_pos: {args.right_gripper_pos}")
    print(f"  left_gripper_pos: {args.left_gripper_pos}")
    print(f"  hold_seconds: {args.hold_seconds}")
    print(f"  conn_profile: {args.conn_profile}")
    print(f"  real_ip: {args.real_ip}")
    print(f"  sn: {args.sn}")
    print(f"  log_file: {resolve_log_path(args.log_file)}")
    print(f"  notes: {args.notes}")


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
    need_header = not log_path.exists() or log_path.stat().st_size == 0
    row = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        "mode": args.mode,
        "conn_profile": getattr(args, "effective_conn_profile", args.conn_profile),
        "real_ip": args.real_ip,
        "sn": args.sn,
        "right_gripper_pos": args.right_gripper_pos,
        "left_gripper_pos": args.left_gripper_pos,
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


def wait_for_observation(seconds: float, label: str, observations: list[str]) -> None:
    if seconds > 0.0:
        print(f"{label}，保持 {seconds:.1f} 秒供观察...")
        time.sleep(seconds)
    else:
        print(f"{label}，未额外停留。")
    for item in observations:
        print(f"- {item}")


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
    robot.right_gripper.set_position(args.right_gripper_pos)
    wait_for_observation(
        args.hold_seconds,
        "右手已抓住咖啡杯",
        [
            "请观察是否滑动。",
            "请观察杯子是否倾斜。",
            "请观察夹爪是否压杯口或压杯壁不稳。",
        ],
    )
    if release_at_end:
        confirm_or_exit("右手即将释放咖啡杯。请确认杯子已被桌面稳定支撑。")
        release_right_cup(robot)


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
    robot.left_gripper.set_position(args.left_gripper_pos)
    wait_for_observation(
        args.hold_seconds,
        "左手已抓住拉花壶",
        [
            "请观察是否滑动。",
            "请观察壶嘴方向是否合理。",
            "请观察夹爪是否夹在合适位置。",
            "请观察夹爪是否会碰壶身。",
        ],
    )
    if release_at_end:
        confirm_or_exit("左手即将释放拉花壶。请确认拉花壶已被桌面稳定支撑。")
        release_left_pitcher(robot)


def run_right_cup_mode(robot: Mantis, args: argparse.Namespace) -> None:
    print("[模式] right-cup：测试右手抓咖啡杯")
    grasp_right_cup(robot, args, release_at_end=True)


def run_left_pitcher_mode(robot: Mantis, args: argparse.Namespace) -> None:
    print("[模式] left-pitcher：测试左手抓拉花壶")
    grasp_left_pitcher(robot, args, release_at_end=True)


def run_both_static_mode(robot: Mantis, args: argparse.Namespace) -> None:
    print("[模式] both-static：双手同时持物静态观察")
    grasp_right_cup(robot, args, release_at_end=False)
    grasp_left_pitcher(robot, args, release_at_end=False)
    confirm_or_exit("双手已完成抓取，即将进入静态持物观察。请确认两侧周围安全。")
    wait_for_observation(
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
    confirm_or_exit("即将先释放左手拉花壶，再释放右手咖啡杯。请确认桌面支撑稳定、周围安全。")
    release_left_pitcher(robot)
    release_right_cup(robot)


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


def main() -> None:
    args = parse_args()
    robot: Optional[Mantis] = None
    should_log_trial = not args.print_connection_config
    start_time = time.monotonic()
    status = "failed"
    error_message = ""
    combined_notes = args.notes

    try:
        validate_args(args)
        print_safety_notice(args)
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
                robot.disconnect()
            except Exception as exc:
                print(f"断开连接时忽略异常: {exc}")
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
