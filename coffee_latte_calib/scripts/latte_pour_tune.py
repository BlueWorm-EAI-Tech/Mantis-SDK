import argparse
import csv
from datetime import datetime
from pathlib import Path
import subprocess
import sys
import time
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from connection_selector import add_connection_args, connect_robot_with_selector
from mantis import Mantis


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_IP = "192.168.1.151"
DEFAULT_SN = "BW_3N5CRT22"
DEFAULT_LOG_FILE = "docs/latte_pour_tuning_log.csv"
CONSERVATIVE_REACH_PROFILE = "conservative"
CURRENT_REACH_PROFILE = "current"
CUSTOM_REACH_PROFILE = "custom"
REACH_PROFILE_CHOICES = (
    CONSERVATIVE_REACH_PROFILE,
    CURRENT_REACH_PROFILE,
    CUSTOM_REACH_PROFILE,
)
CSV_FIELDNAMES = [
    "timestamp",
    "trial_name",
    "mode",
    "reach_profile",
    "sn",
    "wrist_roll_max",
    "shoulder_roll_center",
    "shoulder_roll_amp",
    "elbow_pitch_center",
    "elbow_pitch_amp",
    "sway_count",
    "step_sleep",
    "left_start_shoulder_pitch",
    "left_pour_shoulder_pitch",
    "right_receive_shoulder_roll",
    "right_receive_elbow_pitch",
    "right_receive_wrist_yaw",
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

LEFT_RECOVER_SHOULDER_PITCH = -0.30
LEFT_RECOVER_SHOULDER_ROLL = 0.15
LEFT_RECOVER_ELBOW_PITCH = -0.35

CONSERVATIVE_LEFT_START_SHOULDER_PITCH = -0.30
CONSERVATIVE_LEFT_START_SHOULDER_ROLL = 0.45
CONSERVATIVE_LEFT_START_ELBOW_PITCH = -0.45
CONSERVATIVE_LEFT_START_WRIST_PITCH = -0.30
CONSERVATIVE_LEFT_START_WRIST_ROLL = 0.80

CONSERVATIVE_LEFT_POUR_SHOULDER_PITCH = -0.35
CONSERVATIVE_LEFT_POUR_WRIST_ROLL_PREP = 1.05
CONSERVATIVE_LEFT_POUR_WRIST_ROLL_MAX = 1.25
CONSERVATIVE_LEFT_RECOVER_SHOULDER_PITCH = -0.20
CONSERVATIVE_LEFT_RECOVER_SHOULDER_ROLL = 0.10
CONSERVATIVE_LEFT_RECOVER_ELBOW_PITCH = -0.25

CONSERVATIVE_LEFT_SWING_SHOULDER_ROLL_CENTER = 0.50
CONSERVATIVE_LEFT_SWING_SHOULDER_ROLL_AMP = 0.03
CONSERVATIVE_LEFT_SWING_ELBOW_PITCH_CENTER = -0.42
CONSERVATIVE_LEFT_SWING_ELBOW_PITCH_AMP = 0.02
CONSERVATIVE_LEFT_SWING_CYCLES = 2
CONSERVATIVE_LEFT_SWING_DELAY = 0.60

CONSERVATIVE_RIGHT_RECEIVE_SHOULDER_PITCH = 0.0
CONSERVATIVE_RIGHT_RECEIVE_SHOULDER_YAW = 0.0
CONSERVATIVE_RIGHT_RECEIVE_SHOULDER_ROLL = 0.50
CONSERVATIVE_RIGHT_RECEIVE_ELBOW_PITCH = -0.20
CONSERVATIVE_RIGHT_RECEIVE_WRIST_ROLL = 0.20
CONSERVATIVE_RIGHT_RECEIVE_WRIST_PITCH = -0.30
CONSERVATIVE_RIGHT_RECEIVE_WRIST_YAW = -0.40

PROFILE_TUNING_FIELDS = (
    "wrist_roll_max",
    "shoulder_roll_center",
    "shoulder_roll_amp",
    "elbow_pitch_center",
    "elbow_pitch_amp",
    "sway_count",
    "step_sleep",
    "left_start_shoulder_pitch",
    "left_start_shoulder_roll",
    "left_start_elbow_pitch",
    "left_start_wrist_pitch",
    "left_start_wrist_roll",
    "left_pour_shoulder_pitch",
    "left_pour_wrist_roll_prep",
    "left_recover_shoulder_pitch",
    "left_recover_shoulder_roll",
    "left_recover_elbow_pitch",
    "right_receive_shoulder_pitch",
    "right_receive_shoulder_yaw",
    "right_receive_shoulder_roll",
    "right_receive_elbow_pitch",
    "right_receive_wrist_roll",
    "right_receive_wrist_pitch",
    "right_receive_wrist_yaw",
)

CURRENT_PROFILE_DEFAULTS = {
    "wrist_roll_max": LEFT_POUR_WRIST_ROLL_MAX,
    "shoulder_roll_center": LEFT_SWING_SHOULDER_ROLL_CENTER,
    "shoulder_roll_amp": LEFT_SWING_SHOULDER_ROLL_AMP,
    "elbow_pitch_center": LEFT_SWING_ELBOW_PITCH_CENTER,
    "elbow_pitch_amp": LEFT_SWING_ELBOW_PITCH_AMP,
    "sway_count": LEFT_SWING_CYCLES,
    "step_sleep": LEFT_SWING_DELAY,
    "left_start_shoulder_pitch": LEFT_START_SHOULDER_PITCH,
    "left_start_shoulder_roll": LEFT_START_SHOULDER_ROLL,
    "left_start_elbow_pitch": LEFT_START_ELBOW_PITCH,
    "left_start_wrist_pitch": LEFT_START_WRIST_PITCH,
    "left_start_wrist_roll": LEFT_START_WRIST_ROLL,
    "left_pour_shoulder_pitch": LEFT_POUR_SHOULDER_PITCH,
    "left_recover_shoulder_pitch": LEFT_RECOVER_SHOULDER_PITCH,
    "left_recover_shoulder_roll": LEFT_RECOVER_SHOULDER_ROLL,
    "left_recover_elbow_pitch": LEFT_RECOVER_ELBOW_PITCH,
    "right_receive_shoulder_pitch": RIGHT_RECEIVE_SHOULDER_PITCH,
    "right_receive_shoulder_yaw": RIGHT_RECEIVE_SHOULDER_YAW,
    "right_receive_shoulder_roll": RIGHT_RECEIVE_SHOULDER_ROLL,
    "right_receive_elbow_pitch": RIGHT_RECEIVE_ELBOW_PITCH,
    "right_receive_wrist_roll": RIGHT_RECEIVE_WRIST_ROLL,
    "right_receive_wrist_pitch": RIGHT_RECEIVE_WRIST_PITCH,
    "right_receive_wrist_yaw": RIGHT_RECEIVE_WRIST_YAW,
}

CONSERVATIVE_PROFILE_DEFAULTS = {
    "wrist_roll_max": CONSERVATIVE_LEFT_POUR_WRIST_ROLL_MAX,
    "shoulder_roll_center": CONSERVATIVE_LEFT_SWING_SHOULDER_ROLL_CENTER,
    "shoulder_roll_amp": CONSERVATIVE_LEFT_SWING_SHOULDER_ROLL_AMP,
    "elbow_pitch_center": CONSERVATIVE_LEFT_SWING_ELBOW_PITCH_CENTER,
    "elbow_pitch_amp": CONSERVATIVE_LEFT_SWING_ELBOW_PITCH_AMP,
    "sway_count": CONSERVATIVE_LEFT_SWING_CYCLES,
    "step_sleep": CONSERVATIVE_LEFT_SWING_DELAY,
    "left_start_shoulder_pitch": CONSERVATIVE_LEFT_START_SHOULDER_PITCH,
    "left_start_shoulder_roll": CONSERVATIVE_LEFT_START_SHOULDER_ROLL,
    "left_start_elbow_pitch": CONSERVATIVE_LEFT_START_ELBOW_PITCH,
    "left_start_wrist_pitch": CONSERVATIVE_LEFT_START_WRIST_PITCH,
    "left_start_wrist_roll": CONSERVATIVE_LEFT_START_WRIST_ROLL,
    "left_pour_shoulder_pitch": CONSERVATIVE_LEFT_POUR_SHOULDER_PITCH,
    "left_pour_wrist_roll_prep": CONSERVATIVE_LEFT_POUR_WRIST_ROLL_PREP,
    "left_recover_shoulder_pitch": CONSERVATIVE_LEFT_RECOVER_SHOULDER_PITCH,
    "left_recover_shoulder_roll": CONSERVATIVE_LEFT_RECOVER_SHOULDER_ROLL,
    "left_recover_elbow_pitch": CONSERVATIVE_LEFT_RECOVER_ELBOW_PITCH,
    "right_receive_shoulder_pitch": CONSERVATIVE_RIGHT_RECEIVE_SHOULDER_PITCH,
    "right_receive_shoulder_yaw": CONSERVATIVE_RIGHT_RECEIVE_SHOULDER_YAW,
    "right_receive_shoulder_roll": CONSERVATIVE_RIGHT_RECEIVE_SHOULDER_ROLL,
    "right_receive_elbow_pitch": CONSERVATIVE_RIGHT_RECEIVE_ELBOW_PITCH,
    "right_receive_wrist_roll": CONSERVATIVE_RIGHT_RECEIVE_WRIST_ROLL,
    "right_receive_wrist_pitch": CONSERVATIVE_RIGHT_RECEIVE_WRIST_PITCH,
    "right_receive_wrist_yaw": CONSERVATIVE_RIGHT_RECEIVE_WRIST_YAW,
}


class ConfigurationError(Exception):
    """Raised when user-provided tuning or safety flags are invalid."""


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
        "--reach-profile",
        choices=REACH_PROFILE_CHOICES,
        default=CONSERVATIVE_REACH_PROFILE,
        help="动作包络档位：conservative 默认更保守，current 保留旧参数，custom 仅接受命令行显式传入值",
    )
    parser.add_argument(
        "--wrist-roll-max",
        type=float,
        default=None,
        help="倒奶阶段最大 wrist_roll 角度；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--left-start-shoulder-pitch",
        type=float,
        default=None,
        help="左手起始姿态 shoulder_pitch；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--left-start-shoulder-roll",
        type=float,
        default=None,
        help="左手起始姿态 shoulder_roll；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--left-start-elbow-pitch",
        type=float,
        default=None,
        help="左手起始姿态 elbow_pitch；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--left-start-wrist-pitch",
        type=float,
        default=None,
        help="左手起始姿态 wrist_pitch；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--left-start-wrist-roll",
        type=float,
        default=None,
        help="左手起始姿态 wrist_roll；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--left-pour-shoulder-pitch",
        type=float,
        default=None,
        help="左手倒奶准备姿态 shoulder_pitch；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--left-pour-wrist-roll-prep",
        type=float,
        default=None,
        help="左手倒奶前 wrist_roll 预备角；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--left-recover-shoulder-pitch",
        type=float,
        default=None,
        help="左手倒奶后恢复姿态 shoulder_pitch；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--left-recover-shoulder-roll",
        type=float,
        default=None,
        help="左手倒奶后恢复姿态 shoulder_roll；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--left-recover-elbow-pitch",
        type=float,
        default=None,
        help="左手倒奶后恢复姿态 elbow_pitch；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--right-receive-shoulder-pitch",
        type=float,
        default=None,
        help="右手接奶观察位 shoulder_pitch；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--right-receive-shoulder-yaw",
        type=float,
        default=None,
        help="右手接奶观察位 shoulder_yaw；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--right-receive-shoulder-roll",
        type=float,
        default=None,
        help="右手接奶观察位 shoulder_roll；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--right-receive-elbow-pitch",
        type=float,
        default=None,
        help="右手接奶观察位 elbow_pitch；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--right-receive-wrist-roll",
        type=float,
        default=None,
        help="右手接奶观察位 wrist_roll；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--right-receive-wrist-pitch",
        type=float,
        default=None,
        help="右手接奶观察位 wrist_pitch；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--right-receive-wrist-yaw",
        type=float,
        default=None,
        help="右手接奶观察位 wrist_yaw；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--shoulder-roll-center",
        type=float,
        default=None,
        help="左右摆动的 shoulder_roll 中心值；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--shoulder-roll-amp",
        type=float,
        default=None,
        help="左右摆动的 shoulder_roll 幅度；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--elbow-pitch-center",
        type=float,
        default=None,
        help="左右摆动的 elbow_pitch 中心值；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--elbow-pitch-amp",
        type=float,
        default=None,
        help="左右摆动的 elbow_pitch 微调幅度；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--sway-count",
        type=int,
        default=None,
        help="完整左右摆动周期数；默认由 --reach-profile 决定",
    )
    parser.add_argument(
        "--step-sleep",
        type=float,
        default=None,
        help="每个半程目标点后的停留时间（秒）；默认由 --reach-profile 决定",
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
    parser.add_argument(
        "--allow-no-confirm-real",
        action="store_true",
        help="仅在你明确接受实机风险时使用，允许 --no-confirm 在真实/可能真实连接下生效",
    )
    parser.add_argument(
        "--allow-risky-pose",
        action="store_true",
        help="允许超过建议动作包络边界，但仍会打印高风险提示",
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
    print("如果听到电机异响，立刻停止当前测试。")
    print("不要继续重复执行同一组参数，先切回 conservative profile。")
    print("shoulder 关节会带动整条手臂，风险最高。")
    print("左右臂不要默认按镜像关系理解。")
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


def apply_reach_profile_defaults(args: argparse.Namespace) -> None:
    if args.reach_profile == CONSERVATIVE_REACH_PROFILE:
        profile_defaults = CONSERVATIVE_PROFILE_DEFAULTS
    elif args.reach_profile == CURRENT_REACH_PROFILE:
        profile_defaults = CURRENT_PROFILE_DEFAULTS
    elif args.reach_profile == CUSTOM_REACH_PROFILE:
        profile_defaults = {}
    else:
        raise ConfigurationError(f"不支持的 reach profile: {args.reach_profile}")

    for field_name, field_value in profile_defaults.items():
        if getattr(args, field_name) is None:
            setattr(args, field_name, field_value)

    if args.left_pour_wrist_roll_prep is None and args.reach_profile == CURRENT_REACH_PROFILE:
        args.left_pour_wrist_roll_prep = max(
            0.0,
            args.wrist_roll_max - LEFT_POUR_WRIST_ROLL_PREP_DELTA,
        )

    missing_fields = [field_name for field_name in PROFILE_TUNING_FIELDS if getattr(args, field_name) is None]
    if missing_fields:
        missing_flags = ", ".join(f"--{field_name.replace('_', '-')}" for field_name in missing_fields)
        raise ConfigurationError(
            "以下参数尚未提供，custom profile 需要显式传入："
            f"{missing_flags}"
        )


def resolve_skip_confirm(args: argparse.Namespace) -> bool:
    if not args.no_confirm:
        return False

    if args.print_connection_config:
        return True

    if not args.allow_no_confirm_real:
        raise ConfigurationError(
            "实机模式下不允许直接 --no-confirm，请移除该参数或显式传入 --allow-no-confirm-real"
        )

    print("[强警告] 已显式允许实机跳过确认，请确认现场有人值守且物理急停可立即触达。")
    return True


def warn_risky_value(args: argparse.Namespace, option_name: str, value: float, recommendation: str) -> None:
    if not args.allow_risky_pose:
        raise ConfigurationError(
            f"{option_name}={value} 超出建议边界（{recommendation}）。"
            "如确需继续，请显式传入 --allow-risky-pose。"
        )
    print(
        f"[高风险提示] {option_name}={value} 超出建议边界（{recommendation}），"
        "已显式传入 --allow-risky-pose，请保持物理急停就绪。"
    )


def validate_tuning_args(args: argparse.Namespace) -> None:
    if args.sway_count < 1:
        raise ConfigurationError("--sway-count 必须大于等于 1")
    if args.step_sleep < 0.15:
        raise ConfigurationError("--step-sleep 不能小于 0.15")
    if args.shoulder_roll_amp < 0.0:
        raise ConfigurationError("--shoulder-roll-amp 不能为负数")
    if args.elbow_pitch_amp < 0.0:
        raise ConfigurationError("--elbow-pitch-amp 不能为负数")
    if args.left_pour_wrist_roll_prep < 0.0:
        raise ConfigurationError("--left-pour-wrist-roll-prep 不能为负数")
    if args.left_pour_wrist_roll_prep > args.wrist_roll_max:
        raise ConfigurationError("--left-pour-wrist-roll-prep 不能大于 --wrist-roll-max")
    if args.wrist_roll_max > 1.55:
        warn_risky_value(args, "--wrist-roll-max", args.wrist_roll_max, "建议不超过 1.55")
    if args.shoulder_roll_amp > 0.08:
        warn_risky_value(args, "--shoulder-roll-amp", args.shoulder_roll_amp, "建议不超过 0.08")
    if args.elbow_pitch_amp > 0.06:
        warn_risky_value(args, "--elbow-pitch-amp", args.elbow_pitch_amp, "建议不超过 0.06")
    if args.sway_count > 8:
        warn_risky_value(args, "--sway-count", float(args.sway_count), "建议不超过 8")
    if args.left_pour_shoulder_pitch < -0.60:
        raise ConfigurationError("--left-pour-shoulder-pitch 不能小于 -0.60")
    if args.left_start_shoulder_pitch < -0.60:
        raise ConfigurationError("--left-start-shoulder-pitch 不能小于 -0.60")
    if args.right_receive_shoulder_roll > 0.75:
        raise ConfigurationError("--right-receive-shoulder-roll 不能大于 0.75")
    if abs(args.right_receive_wrist_yaw) > 0.80:
        raise ConfigurationError("--right-receive-wrist-yaw 的绝对值不能大于 0.80")


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
        "reach_profile": args.reach_profile,
        "sn": args.sn,
        "wrist_roll_max": args.wrist_roll_max,
        "shoulder_roll_center": args.shoulder_roll_center,
        "shoulder_roll_amp": args.shoulder_roll_amp,
        "elbow_pitch_center": args.elbow_pitch_center,
        "elbow_pitch_amp": args.elbow_pitch_amp,
        "sway_count": args.sway_count,
        "step_sleep": args.step_sleep,
        "left_start_shoulder_pitch": args.left_start_shoulder_pitch,
        "left_pour_shoulder_pitch": args.left_pour_shoulder_pitch,
        "right_receive_shoulder_roll": args.right_receive_shoulder_roll,
        "right_receive_elbow_pitch": args.right_receive_elbow_pitch,
        "right_receive_wrist_yaw": args.right_receive_wrist_yaw,
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
    print(f"  reach_profile: {args.reach_profile}")
    print(f"  conn_profile: {args.conn_profile}")
    print(f"  real_ip: {args.real_ip}")
    print(f"  sn: {args.sn}")
    print(
        "  left_start_pose: "
        f"shoulder_pitch={args.left_start_shoulder_pitch}, "
        f"shoulder_roll={args.left_start_shoulder_roll}, "
        f"elbow_pitch={args.left_start_elbow_pitch}, "
        f"wrist_pitch={args.left_start_wrist_pitch}, "
        f"wrist_roll={args.left_start_wrist_roll}"
    )
    print(
        "  left_pour_pose: "
        f"shoulder_pitch={args.left_pour_shoulder_pitch}, "
        f"wrist_roll_prep={args.left_pour_wrist_roll_prep}, "
        f"recover_shoulder_pitch={args.left_recover_shoulder_pitch}, "
        f"recover_shoulder_roll={args.left_recover_shoulder_roll}, "
        f"recover_elbow_pitch={args.left_recover_elbow_pitch}"
    )
    print(
        "  right_receive_pose: "
        f"shoulder_pitch={args.right_receive_shoulder_pitch}, "
        f"shoulder_yaw={args.right_receive_shoulder_yaw}, "
        f"shoulder_roll={args.right_receive_shoulder_roll}, "
        f"elbow_pitch={args.right_receive_elbow_pitch}, "
        f"wrist_roll={args.right_receive_wrist_roll}, "
        f"wrist_pitch={args.right_receive_wrist_pitch}, "
        f"wrist_yaw={args.right_receive_wrist_yaw}"
    )
    print(f"  wrist_roll_max: {args.wrist_roll_max}")
    print(f"  shoulder_roll_center: {args.shoulder_roll_center}")
    print(f"  shoulder_roll_amp: {args.shoulder_roll_amp}")
    print(f"  elbow_pitch_center: {args.elbow_pitch_center}")
    print(f"  elbow_pitch_amp: {args.elbow_pitch_amp}")
    print(f"  sway_count: {args.sway_count}")
    print(f"  step_sleep: {args.step_sleep}")
    print(f"  skip_confirm: {args.skip_confirm}")
    print(f"  log_file: {resolve_log_path(args.log_file)}")
    print(f"  notes: {args.notes}")
    print("本次参数与运行结果会自动追加到 CSV；如需整理总结，可再同步到 docs/latte_pour_tuning_log.md")


def go_to_latte_start_pose(robot: Mantis, args: argparse.Namespace) -> None:
    print("左手进入拉花前调试起始姿态...")
    robot.left_arm.set_wrist_pitch(args.left_start_wrist_pitch, block=False)
    robot.left_arm.set_shoulder_pitch(args.left_start_shoulder_pitch, block=False)
    robot.left_arm.set_shoulder_roll(args.left_start_shoulder_roll, block=False)
    robot.left_arm.set_wrist_roll(args.left_start_wrist_roll, block=False)
    robot.left_arm.set_elbow_pitch(args.left_start_elbow_pitch, block=True)
    time.sleep(0.3)


def move_right_arm_to_receive_milk_pose(robot: Mantis, args: argparse.Namespace) -> None:
    print("右手进入接奶测试位姿...")
    robot.right_arm.set_shoulder_pitch(args.right_receive_shoulder_pitch, block=False)
    robot.right_arm.set_shoulder_yaw(args.right_receive_shoulder_yaw, block=False)
    robot.right_arm.set_elbow_pitch(args.right_receive_elbow_pitch, block=False)
    robot.right_arm.set_wrist_yaw(args.right_receive_wrist_yaw, block=False)
    robot.right_arm.set_wrist_roll(args.right_receive_wrist_roll, block=False)
    robot.right_arm.set_wrist_pitch(args.right_receive_wrist_pitch, block=False)
    robot.right_arm.set_shoulder_roll(args.right_receive_shoulder_roll, block=True)
    time.sleep(0.3)
    print("右手已到接奶测试位姿，请观察杯口高度、夹持稳定性和周围间隙。")


def latte_art_pour_demo(robot: Mantis, args: argparse.Namespace) -> None:
    print("开始执行简化倒奶/拉花 demo...")
    swing_left = (
        args.shoulder_roll_center - args.shoulder_roll_amp,
        args.elbow_pitch_center - args.elbow_pitch_amp,
    )
    swing_right = (
        args.shoulder_roll_center + args.shoulder_roll_amp,
        args.elbow_pitch_center + args.elbow_pitch_amp,
    )

    # 进入接近原始倒奶姿态，再逐步增大 wrist_roll 进入倒奶角。
    robot.left_arm.set_shoulder_pitch(args.left_pour_shoulder_pitch, block=False)
    robot.left_arm.set_elbow_pitch(args.elbow_pitch_center, block=False)
    robot.left_arm.set_shoulder_roll(args.shoulder_roll_center, block=False)
    robot.left_arm.set_wrist_roll(args.left_pour_wrist_roll_prep, block=True)
    time.sleep(0.3)

    confirm_or_exit(
        "即将增大 wrist_roll 进入倒奶角，如听到异响请立即物理急停。",
        args.skip_confirm,
    )
    robot.left_arm.set_wrist_roll(args.wrist_roll_max, block=True)
    time.sleep(0.3)

    if args.reach_profile == CURRENT_REACH_PROFILE:
        confirm_or_exit(
            "当前使用 current profile，开始摆动前请再次确认周围安全且肩关节无异常负载。",
            args.skip_confirm,
        )

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
    robot.left_arm.set_shoulder_roll(args.left_recover_shoulder_roll, block=False)
    robot.left_arm.set_elbow_pitch(args.left_recover_elbow_pitch, block=False)
    robot.left_arm.set_wrist_roll(args.left_pour_wrist_roll_prep, block=True)
    time.sleep(0.3)


def recover_left_arm_after_pour(robot: Mantis, args: argparse.Namespace) -> None:
    print("左手执行倒奶段后的局部恢复...")
    robot.left_arm.set_elbow_pitch(args.left_recover_elbow_pitch, block=False)
    robot.left_arm.set_wrist_roll(0.0, block=False)
    robot.left_arm.set_shoulder_roll(args.left_recover_shoulder_roll, block=True)
    robot.left_arm.set_shoulder_pitch(args.left_recover_shoulder_pitch, block=True)
    time.sleep(0.3)


def print_abort_safety_guidance() -> None:
    print("安全提示：当前机械臂可能停在拉花/接奶姿态。")
    print("不要直接断电。")
    print("如果机械臂伸得过长或电机异响，请优先使用物理急停或运行 robot_rescue_console.py。")
    print("不要让夹持杯子/奶壶的手臂悬空断电。")


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
        apply_reach_profile_defaults(args)
        args.skip_confirm = resolve_skip_confirm(args)
        validate_tuning_args(args)
        print_global_safety_banner()
        print_mode_safety(args.mode)
        print_trial_config(args)
        robot = connect_robot_with_selector(args, script_name=__file__)
        if robot is None:
            status = "config-only"
            return

        if args.mode == "with-right-cup":
            confirm_or_exit(
                "右手即将进入接奶测试位姿，该动作可能导致手臂伸长，请确认周围安全。",
                args.skip_confirm,
            )
            move_right_arm_to_receive_milk_pose(robot, args)
            confirm_or_exit(
                "右手已到接奶测试位姿。请确认杯子安全、杯口无遮挡，再继续左手动作。",
                args.skip_confirm,
            )

        confirm_or_exit(
            "左手即将进入拉花起始姿态，该动作可能导致肩关节负载增加，请确认。",
            args.skip_confirm,
        )
        go_to_latte_start_pose(robot, args)
        latte_art_pour_demo(robot, args)
        recover_left_arm_after_pour(robot, args)
        if args.ask_notes:
            combined_notes = combine_notes(combined_notes, prompt_for_notes())
        status = "success"
        print("调试流程执行结束。")
    except ConfigurationError as exc:
        status = "failed"
        error_message = format_error(exc)
        print(str(exc))
    except UserAbort:
        status = "skipped"
        print("用户取消，脚本退出。")
    except KeyboardInterrupt as exc:
        status = "interrupted"
        error_message = format_error(exc)
        print("\n检测到 Ctrl-C，脚本中止。")
        print_abort_safety_guidance()
    except Exception as exc:
        status = "failed"
        error_message = format_error(exc)
        print(f"执行失败: {error_message}")
        print_abort_safety_guidance()
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
