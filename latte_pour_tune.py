import argparse
import csv
from datetime import datetime
from pathlib import Path
import subprocess
import sys
import time
from typing import Optional

from connection_selector import add_connection_args, connect_robot_with_selector
from mantis import Mantis


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_IP = "192.168.1.151"
DEFAULT_SN = "BW_3N5CRT22"
DEFAULT_LOG_FILE = "docs/latte_pour_tuning_log.csv"
CSV_FIELDNAMES = [
    "timestamp",
    "trial_name",
    "mode",
    "sn",
    "wrist_roll_max",
    "shoulder_roll_center",
    "shoulder_roll_amp",
    "elbow_pitch_center",
    "elbow_pitch_amp",
    "sway_count",
    "step_sleep",
    "status",
    "error",
    "duration_s",
    "notes",
    "git_branch",
    "git_commit",
]


# 左手调试起始姿态。该值来自 coffee.py 中“左手拿起奶杯后、进入倒奶前”
# 的局部关节动作（约第 109-125 行），这里只保留倒奶段调试所需的保守关节值，
# 不等价于完整拿奶杯流程。
LEFT_START_SHOULDER_PITCH = -0.50
LEFT_START_SHOULDER_ROLL = 0.58
LEFT_START_ELBOW_PITCH = -0.58
LEFT_START_WRIST_PITCH = -0.45
LEFT_START_WRIST_ROLL = 1.10

# 左手简化拉花 demo 参数。这里是关节空间示教参数，不是真正基于杯口坐标的拉花算法。
LEFT_POUR_SHOULDER_PITCH = -0.60
LEFT_POUR_WRIST_ROLL_PREP = 1.35
LEFT_POUR_WRIST_ROLL_MAX = 1.55
LEFT_POUR_WRIST_ROLL_PREP_DELTA = LEFT_POUR_WRIST_ROLL_MAX - LEFT_POUR_WRIST_ROLL_PREP
# 当前拉花摆动仍然是关节空间 demo。

# LEFT_SWING_CYCLES 表示完整左右摆动次数；6 个 cycle 对应 12 个半程目标点。
# 如果 RViz/实机上摆动仍不明显，优先增大 LEFT_SWING_DELAY 或扩大 shoulder_roll 左右差值。
# 如果实机上有碰撞风险，优先减小 shoulder_roll 幅度。
LEFT_SWING_SHOULDER_ROLL_CENTER = 0.625
LEFT_SWING_SHOULDER_ROLL_AMP = 0.075
LEFT_SWING_ELBOW_PITCH_CENTER = -0.50
LEFT_SWING_ELBOW_PITCH_AMP = 0.05
LEFT_SWING_CYCLES = 6
LEFT_SWING_DELAY = 0.45

# 右手静态接奶测试位姿。该值复用 coffee.py 中“右手后撤后等待左手倒奶”
# 的局部动作（约第 80-83、112-116 行），仅用于双臂相对位置联调，
# 不是完整咖啡流程。
RIGHT_RECEIVE_SHOULDER_PITCH = 0.0
RIGHT_RECEIVE_SHOULDER_YAW = 0.0
RIGHT_RECEIVE_SHOULDER_ROLL = 0.70
RIGHT_RECEIVE_ELBOW_PITCH = 0.0
RIGHT_RECEIVE_WRIST_ROLL = 0.30
RIGHT_RECEIVE_WRIST_PITCH = -0.50
RIGHT_RECEIVE_WRIST_YAW = -0.70


class UserAbort(Exception):
    """Raised when the operator cancels before or during the tune flow."""


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    if argv is None:
        argv = sys.argv[1:]
    parser = argparse.ArgumentParser(
        description="Mantis 倒奶/拉花段分模式调试工具（仅关节空间 demo，不运行完整 coffee 流程）"
    )
    parser.add_argument(
        "--mode",
        default="left-only",
        choices=("left-only", "with-right-cup"),
        help="调试模式：left-only 只调左手；with-right-cup 加入右手接奶位姿联调",
    )
    add_connection_args(parser, default_profile="interactive")
    parser.add_argument(
        "--wrist-roll-max",
        type=float,
        default=LEFT_POUR_WRIST_ROLL_MAX,
        help="倒奶阶段最大 wrist_roll 角度",
    )
    parser.add_argument(
        "--shoulder-roll-center",
        type=float,
        default=LEFT_SWING_SHOULDER_ROLL_CENTER,
        help="左右摆动的 shoulder_roll 中心值",
    )
    parser.add_argument(
        "--shoulder-roll-amp",
        type=float,
        default=LEFT_SWING_SHOULDER_ROLL_AMP,
        help="左右摆动的 shoulder_roll 幅度",
    )
    parser.add_argument(
        "--elbow-pitch-center",
        type=float,
        default=LEFT_SWING_ELBOW_PITCH_CENTER,
        help="左右摆动的 elbow_pitch 中心值",
    )
    parser.add_argument(
        "--elbow-pitch-amp",
        type=float,
        default=LEFT_SWING_ELBOW_PITCH_AMP,
        help="左右摆动的 elbow_pitch 微调幅度",
    )
    parser.add_argument(
        "--sway-count",
        type=int,
        default=LEFT_SWING_CYCLES,
        help="完整左右摆动周期数",
    )
    parser.add_argument(
        "--step-sleep",
        type=float,
        default=LEFT_SWING_DELAY,
        help="每个半程目标点后的停留时间（秒）",
    )
    parser.add_argument(
        "--trial-name",
        default="manual_trial",
        help="本次调试实验名称，用于打印和记录",
    )
    parser.add_argument(
        "--log-file",
        default=DEFAULT_LOG_FILE,
        help="CSV 实验记录文件路径，默认写入 docs/latte_pour_tuning_log.csv",
    )
    parser.add_argument(
        "--notes",
        default="",
        help="本次实验备注，例如：少量水测试，观察是否入杯",
    )
    parser.add_argument(
        "--ask-notes",
        action="store_true",
        help="动作执行后提示输入本次实验观察，并写入 CSV 日志",
    )
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="跳过人工确认，仅保留打印提示",
    )
    return parser.parse_args(argv)


def print_global_safety_banner() -> None:
    print("=" * 72)
    print("危险提示：当前脚本会控制真实机器人。")
    print("第一次建议空壶、空杯。")
    print("第二次建议少量水。")
    print("最后再用牛奶/奶泡。")
    print("请清空机器人周围障碍物。")
    print("请确认杯子、奶壶、咖啡机不会被碰撞。")
    print("请准备物理急停。")
    print("当前脚本不是稳定产品，只是宣传 demo 前的动作调试工具。")
    print("本脚本不包含抓杯/拿壶流程，请先人工确认夹持状态。")
    print("=" * 72)


def print_mode_safety(mode: str) -> None:
    if mode == "left-only":
        print("[模式] left-only")
        print("- 这是实机左手倒奶/拉花动作包络调试。")
        print("- 建议第一次空壶测试。")
        print("- 不验证右手杯口相对位置。")
        print("- 请确认周围安全。")
        print("- 请准备物理急停。")
        return

    if mode == "with-right-cup":
        print("[模式] with-right-cup")
        print("- 这是左右手相对位置联调。")
        print("- 右手会移动到接奶位置。")
        print("- 左手会执行倒奶/拉花动作。")
        print("- 不执行完整 coffee 流程。")
        print("- 建议第一次空杯、空壶测试。")
        print("- 第二次少量水测试。")
        print("- 最后再用牛奶/奶泡。")
        print("- 请确认右手夹持杯子安全。")
        print("- 请准备物理急停。")
        return

    raise SystemExit(f"不支持的模式: {mode}")


def confirm_or_exit(message: str, skip_confirm: bool) -> None:
    if skip_confirm:
        print(f"[跳过确认] {message}")
        return

    print(message)
    user_input = input("按 Enter 继续，输入 q 退出: ").strip().lower()
    if user_input == "q":
        raise UserAbort("用户取消执行")


def validate_tuning_args(args: argparse.Namespace) -> None:
    if args.sway_count < 1:
        raise ValueError("--sway-count 必须大于等于 1")
    if args.step_sleep < 0.0:
        raise ValueError("--step-sleep 不能为负数")
    if args.shoulder_roll_amp < 0.0:
        raise ValueError("--shoulder-roll-amp 不能为负数")
    if args.elbow_pitch_amp < 0.0:
        raise ValueError("--elbow-pitch-amp 不能为负数")


def resolve_log_path(log_file: str) -> Path:
    path = Path(log_file).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def combine_notes(*parts: str) -> str:
    cleaned = [part.strip() for part in parts if part and part.strip()]
    return " | ".join(cleaned)


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


def get_git_context() -> tuple[str, str]:
    branch = get_git_value(["git", "rev-parse", "--abbrev-ref", "HEAD"])
    commit = get_git_value(["git", "rev-parse", "--short", "HEAD"])
    return branch, commit


def format_error(exc: BaseException) -> str:
    message = str(exc).strip()
    if message:
        return f"{type(exc).__name__}: {message}"
    return type(exc).__name__


def prompt_for_notes() -> str:
    return input("请输入本次实验观察，可直接回车跳过：").strip()


def append_trial_log(
    args: argparse.Namespace,
    status: str,
    error: str,
    duration_s: float,
    notes: str,
    git_branch: str,
    git_commit: str,
) -> None:
    log_path = resolve_log_path(args.log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    need_header = not log_path.exists() or log_path.stat().st_size == 0
    row = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        "trial_name": args.trial_name,
        "mode": args.mode,
        "sn": args.sn,
        "wrist_roll_max": args.wrist_roll_max,
        "shoulder_roll_center": args.shoulder_roll_center,
        "shoulder_roll_amp": args.shoulder_roll_amp,
        "elbow_pitch_center": args.elbow_pitch_center,
        "elbow_pitch_amp": args.elbow_pitch_amp,
        "sway_count": args.sway_count,
        "step_sleep": args.step_sleep,
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


def print_trial_config(args: argparse.Namespace) -> None:
    print("[实验配置]")
    print(f"  trial_name: {args.trial_name}")
    print(f"  mode: {args.mode}")
    print(f"  conn_profile: {args.conn_profile}")
    print(f"  real_ip: {args.real_ip}")
    print(f"  sim_ip: {args.sim_ip}")
    print(f"  sn: {args.sn}")
    print(f"  wrist_roll_max: {args.wrist_roll_max}")
    print(f"  shoulder_roll_center: {args.shoulder_roll_center}")
    print(f"  shoulder_roll_amp: {args.shoulder_roll_amp}")
    print(f"  elbow_pitch_center: {args.elbow_pitch_center}")
    print(f"  elbow_pitch_amp: {args.elbow_pitch_amp}")
    print(f"  sway_count: {args.sway_count}")
    print(f"  step_sleep: {args.step_sleep}")
    print(f"  log_file: {resolve_log_path(args.log_file)}")
    print(f"  notes: {args.notes}")
    print("本次参数与运行结果会自动追加到 CSV；如需整理总结，可再同步到 docs/latte_pour_tuning_log.md")


def go_to_latte_start_pose(robot: Mantis) -> None:
    print("左手进入拉花前调试起始姿态...")
    robot.left_arm.set_wrist_pitch(LEFT_START_WRIST_PITCH, block=False)
    robot.left_arm.set_shoulder_pitch(LEFT_START_SHOULDER_PITCH, block=False)
    robot.left_arm.set_elbow_pitch(-0.60, block=True)
    robot.left_arm.set_shoulder_roll(LEFT_START_SHOULDER_ROLL, block=False)
    robot.left_arm.set_wrist_roll(LEFT_START_WRIST_ROLL, block=True)
    robot.left_arm.set_elbow_pitch(LEFT_START_ELBOW_PITCH, block=True)
    time.sleep(0.3)


def move_right_arm_to_receive_milk_pose(robot: Mantis) -> None:
    print("右手进入接奶测试位姿...")
    robot.right_arm.set_shoulder_pitch(RIGHT_RECEIVE_SHOULDER_PITCH, block=False)
    robot.right_arm.set_shoulder_yaw(RIGHT_RECEIVE_SHOULDER_YAW, block=False)
    robot.right_arm.set_elbow_pitch(RIGHT_RECEIVE_ELBOW_PITCH, block=False)
    robot.right_arm.set_shoulder_roll(0.60, block=False)
    robot.right_arm.set_wrist_pitch(-0.30, block=True)
    robot.right_arm.set_wrist_yaw(RIGHT_RECEIVE_WRIST_YAW, block=False)
    robot.right_arm.set_wrist_roll(RIGHT_RECEIVE_WRIST_ROLL, block=False)
    robot.right_arm.set_wrist_pitch(RIGHT_RECEIVE_WRIST_PITCH, block=False)
    robot.right_arm.set_shoulder_roll(RIGHT_RECEIVE_SHOULDER_ROLL, block=True)
    time.sleep(0.3)
    print("右手已到接奶测试位姿，请观察杯口高度、夹持稳定性和周围间隙。")


def latte_art_pour_demo(robot: Mantis, args: argparse.Namespace) -> None:
    print("开始执行简化倒奶/拉花 demo...")
    wrist_roll_prep = max(0.0, args.wrist_roll_max - LEFT_POUR_WRIST_ROLL_PREP_DELTA)
    swing_left = (
        args.shoulder_roll_center - args.shoulder_roll_amp,
        args.elbow_pitch_center - args.elbow_pitch_amp,
    )
    swing_right = (
        args.shoulder_roll_center + args.shoulder_roll_amp,
        args.elbow_pitch_center + args.elbow_pitch_amp,
    )

    # 进入接近原始倒奶姿态，再逐步增大 wrist_roll 进入倒奶角。
    robot.left_arm.set_shoulder_pitch(LEFT_POUR_SHOULDER_PITCH, block=False)
    robot.left_arm.set_elbow_pitch(args.elbow_pitch_center - 0.02, block=False)
    robot.left_arm.set_shoulder_roll(args.shoulder_roll_center, block=False)
    robot.left_arm.set_wrist_roll(wrist_roll_prep, block=True)
    time.sleep(0.3)

    robot.left_arm.set_wrist_roll(args.wrist_roll_max, block=True)
    time.sleep(0.3)

    for cycle_idx in range(args.sway_count):
        print(f"拉花摆动 {cycle_idx + 1}/{args.sway_count}: left")
        robot.left_arm.set_shoulder_roll(swing_left[0], block=False)
        robot.left_arm.set_elbow_pitch(swing_left[1], block=True)
        time.sleep(args.step_sleep)

        print(f"拉花摆动 {cycle_idx + 1}/{args.sway_count}: right")
        robot.left_arm.set_shoulder_roll(swing_right[0], block=False)
        robot.left_arm.set_elbow_pitch(swing_right[1], block=True)
        time.sleep(args.step_sleep)

    # 逐步停止倒奶，避免直接从最大倒奶角回摆。
    robot.left_arm.set_shoulder_roll(swing_left[0], block=False)
    robot.left_arm.set_elbow_pitch(args.elbow_pitch_center - 0.02, block=False)
    robot.left_arm.set_wrist_roll(1.10, block=True)
    time.sleep(0.3)

    robot.left_arm.set_wrist_roll(0.25, block=True)
    robot.left_arm.set_shoulder_roll(0.25, block=True)
    time.sleep(0.3)


def recover_left_arm_after_pour(robot: Mantis) -> None:
    print("左手执行倒奶段后的局部恢复...")
    robot.left_arm.set_elbow_pitch(-0.35, block=False)
    robot.left_arm.set_wrist_roll(0.0, block=False)
    robot.left_arm.set_shoulder_roll(0.15, block=True)
    robot.left_arm.set_shoulder_pitch(-0.30, block=True)
    time.sleep(0.3)


def main() -> None:
    args = parse_args()
    robot: Optional[Mantis] = None
    should_log_trial = not args.print_connection_config
    git_branch, git_commit = get_git_context()
    start_time = time.monotonic()
    status = "failed"
    error_message = ""
    combined_notes = args.notes

    try:
        validate_tuning_args(args)
        print_global_safety_banner()
        print_mode_safety(args.mode)
        print_trial_config(args)
        robot = connect_robot_with_selector(args, script_name=__file__)
        if robot is None:
            status = "config-only"
            return

        if args.mode == "with-right-cup":
            move_right_arm_to_receive_milk_pose(robot)
            confirm_or_exit(
                "右手已到接奶测试位姿。请确认杯子安全、杯口无遮挡，再继续左手动作。",
                args.no_confirm,
            )

        go_to_latte_start_pose(robot)
        confirm_or_exit("左手即将执行倒奶/拉花动作，请确认奶壶轨迹范围内没有风险。", args.no_confirm)
        latte_art_pour_demo(robot, args)
        recover_left_arm_after_pour(robot)
        if args.ask_notes:
            combined_notes = combine_notes(combined_notes, prompt_for_notes())
        status = "success"
        print("调试流程执行结束。")
    except UserAbort:
        status = "skipped"
        print("用户取消，脚本退出。")
    except KeyboardInterrupt as exc:
        status = "interrupted"
        error_message = format_error(exc)
        print("\n检测到 Ctrl-C，脚本中止。")
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
            try:
                append_trial_log(
                    args=args,
                    status=status,
                    error=error_message,
                    duration_s=duration_s,
                    notes=combined_notes,
                    git_branch=git_branch,
                    git_commit=git_commit,
                )
                print(f"实验记录已追加到: {resolve_log_path(args.log_file)}")
            except Exception as exc:
                print(f"写入实验日志失败: {exc}")


if __name__ == "__main__":
    main()
