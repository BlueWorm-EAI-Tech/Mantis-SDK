"""
警告：该脚本可能控制真实机器人。
第一次运行必须使用 --dry-run。
实机测试必须先完成双臂关节方向映射。
第一次实机测试不要放杯子、奶壶、液体或咖啡机障碍物。
必须有人看护急停。
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable

from connection_selector import add_connection_args, connect_robot_with_selector


DEFAULT_ROBOT_VERSION = "3.0"
DEFAULT_STAGE_PAUSE_SECONDS = 0.5
CONSERVATIVE_LATTE_REACH_PROFILE = "conservative"
CURRENT_LATTE_REACH_PROFILE = "current"
CUSTOM_LATTE_REACH_PROFILE = "custom"
LATTE_REACH_PROFILE_CHOICES = (
    CONSERVATIVE_LATTE_REACH_PROFILE,
    CURRENT_LATTE_REACH_PROFILE,
    CUSTOM_LATTE_REACH_PROFILE,
)
CURRENT_LATTE_WRIST_ROLL_PREP_DELTA = 0.20
LATTE_CONFIG_FIELDS = (
    "wrist_roll_max",
    "shoulder_roll_center",
    "shoulder_roll_amp",
    "elbow_pitch_center",
    "elbow_pitch_amp",
    "sway_count",
    "step_sleep",
    "left_pour_shoulder_pitch",
    "left_pour_wrist_roll_prep",
    "left_recover_shoulder_pitch",
    "left_recover_shoulder_roll",
    "left_recover_elbow_pitch",
)
CONSERVATIVE_LATTE_PROFILE_DEFAULTS = {
    "wrist_roll_max": 1.25,
    "shoulder_roll_center": 0.50,
    "shoulder_roll_amp": 0.03,
    "elbow_pitch_center": -0.42,
    "elbow_pitch_amp": 0.02,
    "sway_count": 2,
    "step_sleep": 0.60,
    "left_pour_shoulder_pitch": -0.35,
    "left_pour_wrist_roll_prep": 1.05,
    "left_recover_shoulder_pitch": -0.20,
    "left_recover_shoulder_roll": 0.10,
    "left_recover_elbow_pitch": -0.25,
}
CURRENT_LATTE_PROFILE_DEFAULTS = {
    "wrist_roll_max": 1.55,
    "shoulder_roll_center": 0.625,
    "shoulder_roll_amp": 0.075,
    "elbow_pitch_center": -0.50,
    "elbow_pitch_amp": 0.05,
    "sway_count": 6,
    "step_sleep": 0.45,
    "left_pour_shoulder_pitch": -0.60,
    "left_recover_shoulder_pitch": -0.30,
    "left_recover_shoulder_roll": 0.15,
    "left_recover_elbow_pitch": -0.35,
}

STAGE_ORDER = [
    "prepare_head",
    "right_hand_grasp_cup",
    "right_hand_move_to_coffee_machine",
    "left_hand_press_button",
    "right_hand_retreat_after_coffee",
    "left_hand_grasp_milk_pitcher",
    "left_hand_move_to_pour_pose",
    "left_hand_pour_milk",
    "return_home",
]


class _DryRunCallable:
    def __getattr__(self, _name: str):
        return self

    def __call__(self, *args, **kwargs):
        return None


class DryRunRobot:
    def __init__(self) -> None:
        stub = _DryRunCallable()
        self.head = stub
        self.left_arm = stub
        self.right_arm = stub
        self.left_gripper = stub
        self.right_gripper = stub

    def wait(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def disconnect(self) -> None:
        return None


@dataclass
class LattePourConfig:
    reach_profile: str = CONSERVATIVE_LATTE_REACH_PROFILE
    wrist_roll_max: float = 1.25
    shoulder_roll_center: float = 0.50
    shoulder_roll_amp: float = 0.03
    elbow_pitch_center: float = -0.42
    elbow_pitch_amp: float = 0.02
    sway_count: int = 2
    step_sleep: float = 0.60
    left_pour_shoulder_pitch: float = -0.35
    left_pour_wrist_roll_prep: float = 1.05
    left_recover_shoulder_pitch: float = -0.20
    left_recover_shoulder_roll: float = 0.10
    left_recover_elbow_pitch: float = -0.25


@dataclass
class ReplayContext:
    dry_run: bool
    yes: bool
    logger: logging.Logger
    latte_config: LattePourConfig
    sleep_fn: Callable[[float], None] = time.sleep
    action_counter: int = 0
    current_stage: str = ""
    stage_pause_seconds: float = DEFAULT_STAGE_PAUSE_SECONDS
    stage_notes: dict[str, str] = field(default_factory=dict)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mantis 咖啡流程安全复现脚本")
    add_connection_args(parser, default_profile="interactive")
    parser.set_defaults(robot_version=DEFAULT_ROBOT_VERSION)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印阶段和动作计划，不连接机器人，不执行动作",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="允许连接机器人并执行实机动作；未显式传入时默认按 dry-run 处理",
    )
    parser.add_argument(
        "--i-understand-real-robot-risk",
        action="store_true",
        help="二次确认已知晓实机风险；与 --execute 同时传入才允许执行",
    )
    parser.add_argument(
        "--stage",
        choices=[*STAGE_ORDER, "all"],
        default="all",
        help="选择要执行的阶段，默认 all",
    )
    parser.add_argument(
        "--log-file",
        help="日志文件路径；不指定时自动写入 logs/coffee_replay_safe_YYYYmmdd_HHMMSS.log",
    )
    parser.add_argument(
        "--latte-reach-profile",
        choices=LATTE_REACH_PROFILE_CHOICES,
        default=CONSERVATIVE_LATTE_REACH_PROFILE,
        help="拉花/倒奶动作包络档位，默认 conservative",
    )
    parser.add_argument(
        "--latte-wrist-roll-max",
        type=float,
        default=None,
        help="左手倒奶阶段最大 wrist_roll；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-shoulder-roll-center",
        type=float,
        default=None,
        help="左手拉花摆动中心 shoulder_roll；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-shoulder-roll-amp",
        type=float,
        default=None,
        help="左手拉花摆动 shoulder_roll 幅度；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-elbow-pitch-center",
        type=float,
        default=None,
        help="左手拉花摆动中心 elbow_pitch；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-elbow-pitch-amp",
        type=float,
        default=None,
        help="左手拉花摆动 elbow_pitch 幅度；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-sway-count",
        type=int,
        default=None,
        help="左手左右摆动次数；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-step-sleep",
        type=float,
        default=None,
        help="左右摆动之间的停顿时间；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-left-pour-shoulder-pitch",
        type=float,
        default=None,
        help="左手倒奶预备 shoulder_pitch；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-left-pour-wrist-roll-prep",
        type=float,
        default=None,
        help="左手进入倒奶角前的 wrist_roll 预备值；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-left-recover-shoulder-pitch",
        type=float,
        default=None,
        help="左手收束阶段 shoulder_pitch；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-left-recover-shoulder-roll",
        type=float,
        default=None,
        help="左手收束阶段 shoulder_roll；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-left-recover-elbow-pitch",
        type=float,
        default=None,
        help="左手收束阶段 elbow_pitch；默认由 --latte-reach-profile 决定",
    )
    parser.add_argument(
        "--latte-allow-risky-pose",
        action="store_true",
        help="允许超过建议边界的拉花/倒奶关节参数；默认不允许",
    )
    return parser.parse_args()


def build_latte_pour_config(args: argparse.Namespace) -> LattePourConfig:
    if args.latte_reach_profile == CONSERVATIVE_LATTE_REACH_PROFILE:
        config_values = dict(CONSERVATIVE_LATTE_PROFILE_DEFAULTS)
    elif args.latte_reach_profile == CURRENT_LATTE_REACH_PROFILE:
        config_values = dict(CURRENT_LATTE_PROFILE_DEFAULTS)
    elif args.latte_reach_profile == CUSTOM_LATTE_REACH_PROFILE:
        config_values = {}
    else:
        raise SystemExit(f"不支持的 latte reach profile: {args.latte_reach_profile}")

    override_map = {
        "wrist_roll_max": args.latte_wrist_roll_max,
        "shoulder_roll_center": args.latte_shoulder_roll_center,
        "shoulder_roll_amp": args.latte_shoulder_roll_amp,
        "elbow_pitch_center": args.latte_elbow_pitch_center,
        "elbow_pitch_amp": args.latte_elbow_pitch_amp,
        "sway_count": args.latte_sway_count,
        "step_sleep": args.latte_step_sleep,
        "left_pour_shoulder_pitch": args.latte_left_pour_shoulder_pitch,
        "left_pour_wrist_roll_prep": args.latte_left_pour_wrist_roll_prep,
        "left_recover_shoulder_pitch": args.latte_left_recover_shoulder_pitch,
        "left_recover_shoulder_roll": args.latte_left_recover_shoulder_roll,
        "left_recover_elbow_pitch": args.latte_left_recover_elbow_pitch,
    }
    for field_name, value in override_map.items():
        if value is not None:
            config_values[field_name] = value

    if (
        args.latte_reach_profile == CURRENT_LATTE_REACH_PROFILE
        and "left_pour_wrist_roll_prep" not in config_values
    ):
        config_values["left_pour_wrist_roll_prep"] = max(
            0.0,
            float(config_values["wrist_roll_max"]) - CURRENT_LATTE_WRIST_ROLL_PREP_DELTA,
        )

    missing_fields = [field_name for field_name in LATTE_CONFIG_FIELDS if field_name not in config_values]
    if missing_fields:
        missing_flags = ", ".join(f"--latte-{field_name.replace('_', '-')}" for field_name in missing_fields)
        raise SystemExit(
            "以下拉花参数尚未提供，custom profile 需要显式传入："
            f"{missing_flags}"
        )

    config = LattePourConfig(
        reach_profile=args.latte_reach_profile,
        wrist_roll_max=float(config_values["wrist_roll_max"]),
        shoulder_roll_center=float(config_values["shoulder_roll_center"]),
        shoulder_roll_amp=float(config_values["shoulder_roll_amp"]),
        elbow_pitch_center=float(config_values["elbow_pitch_center"]),
        elbow_pitch_amp=float(config_values["elbow_pitch_amp"]),
        sway_count=int(config_values["sway_count"]),
        step_sleep=float(config_values["step_sleep"]),
        left_pour_shoulder_pitch=float(config_values["left_pour_shoulder_pitch"]),
        left_pour_wrist_roll_prep=float(config_values["left_pour_wrist_roll_prep"]),
        left_recover_shoulder_pitch=float(config_values["left_recover_shoulder_pitch"]),
        left_recover_shoulder_roll=float(config_values["left_recover_shoulder_roll"]),
        left_recover_elbow_pitch=float(config_values["left_recover_elbow_pitch"]),
    )
    validate_latte_pour_config(config, allow_risky_pose=args.latte_allow_risky_pose)
    return config


def validate_latte_pour_config(
    config: LattePourConfig,
    allow_risky_pose: bool = False,
) -> None:
    def reject_risky_value(option_name: str, value: float, recommendation: str) -> None:
        if not allow_risky_pose:
            raise SystemExit(
                f"{option_name}={value} 超出建议边界（{recommendation}）。"
                "如确需继续，请显式传入 --latte-allow-risky-pose。"
            )

    if config.sway_count < 1:
        raise SystemExit("--latte-sway-count 必须大于等于 1")
    if config.step_sleep < 0.15:
        raise SystemExit("--latte-step-sleep 不能小于 0.15")
    if config.shoulder_roll_amp < 0.0:
        raise SystemExit("--latte-shoulder-roll-amp 不能为负数")
    if config.elbow_pitch_amp < 0.0:
        raise SystemExit("--latte-elbow-pitch-amp 不能为负数")
    if config.left_pour_wrist_roll_prep < 0.0:
        raise SystemExit("--latte-left-pour-wrist-roll-prep 不能为负数")
    if config.left_pour_wrist_roll_prep > config.wrist_roll_max:
        raise SystemExit("--latte-left-pour-wrist-roll-prep 不能大于 --latte-wrist-roll-max")
    if config.wrist_roll_max > 1.55:
        reject_risky_value("--latte-wrist-roll-max", config.wrist_roll_max, "建议不超过 1.55")
    if config.shoulder_roll_amp > 0.08:
        reject_risky_value("--latte-shoulder-roll-amp", config.shoulder_roll_amp, "建议不超过 0.08")
    if config.elbow_pitch_amp > 0.06:
        reject_risky_value("--latte-elbow-pitch-amp", config.elbow_pitch_amp, "建议不超过 0.06")
    if config.sway_count > 8:
        reject_risky_value("--latte-sway-count", float(config.sway_count), "建议不超过 8")
    if config.left_pour_shoulder_pitch < -0.60:
        raise SystemExit("--latte-left-pour-shoulder-pitch 不能小于 -0.60")


def build_logger(log_file: Path) -> logging.Logger:
    log_file.parent.mkdir(parents=True, exist_ok=True)

    logger = logging.getLogger("coffee_replay_safe")
    logger.setLevel(logging.INFO)
    logger.propagate = False

    formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")

    if logger.handlers:
        logger.handlers.clear()

    file_handler = logging.FileHandler(log_file, encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)
    return logger


def format_invocation(args: tuple, kwargs: dict) -> str:
    parts = [repr(arg) for arg in args]
    parts.extend(f"{key}={value!r}" for key, value in kwargs.items())
    return ", ".join(parts)


def next_action_id(ctx: ReplayContext) -> int:
    ctx.action_counter += 1
    return ctx.action_counter


def call(
    ctx: ReplayContext,
    description: str,
    callable_obj: Callable,
    *args,
    sdk_call: str,
    **kwargs,
):
    action_id = next_action_id(ctx)
    invocation = format_invocation(args, kwargs)
    dry_run_prefix = "DRY-RUN " if ctx.dry_run else ""
    logger = ctx.logger
    logger.info(
        "[%s] %sAction %03d | %s | %s(%s)",
        ctx.current_stage,
        dry_run_prefix,
        action_id,
        description,
        sdk_call,
        invocation,
    )
    if kwargs.get("block") is False:
        logger.info("[%s] Action %03d 使用 block=False", ctx.current_stage, action_id)
    if ctx.dry_run:
        return None
    return callable_obj(*args, **kwargs)


def sleep_step(ctx: ReplayContext, seconds: float, reason: str = "") -> None:
    action_id = next_action_id(ctx)
    suffix = f" | {reason}" if reason else ""
    dry_run_prefix = "DRY-RUN " if ctx.dry_run else ""
    ctx.logger.info(
        "[%s] %sAction %03d | sleep(%.2f)%s",
        ctx.current_stage,
        dry_run_prefix,
        action_id,
        seconds,
        suffix,
    )
    if not ctx.dry_run:
        ctx.sleep_fn(seconds)


def wait_step(ctx: ReplayContext, robot, reason: str) -> None:
    action_id = next_action_id(ctx)
    dry_run_prefix = "DRY-RUN " if ctx.dry_run else ""
    ctx.logger.info(
        "[%s] %sAction %03d | robot.wait() | %s",
        ctx.current_stage,
        dry_run_prefix,
        action_id,
        reason,
    )
    if not ctx.dry_run:
        robot.wait()


def stage_pause(ctx: ReplayContext) -> None:
    sleep_step(ctx, ctx.stage_pause_seconds, reason="阶段结束后观察停稳")


def confirm_stage(ctx: ReplayContext, stage_name: str, meta: dict[str, str], robot) -> None:
    ctx.logger.info("即将执行阶段：%s", stage_name)
    ctx.logger.info("该阶段涉及部件：%s", meta["components"])
    ctx.logger.info("风险提示：%s", meta["risk"])
    if ctx.dry_run:
        ctx.logger.info("dry-run 模式：跳过人工确认，不连接机器人，不执行动作。")
        return
    if ctx.yes:
        ctx.logger.info("--yes 已启用：跳过该阶段人工确认。")
        return
    user_input = input("确认周围安全后按 Enter 继续，输入 q 退出：").strip().lower()
    if user_input == "q":
        ctx.logger.warning("用户在阶段 %s 执行前取消。", stage_name)
        if robot is not None:
            try:
                robot.stop()
                ctx.logger.info("已尝试调用 robot.stop()")
            except Exception:
                ctx.logger.exception("调用 robot.stop() 失败")
        raise SystemExit("用户取消阶段执行")


def finalize_stage(ctx: ReplayContext, robot) -> None:
    wait_step(ctx, robot, reason="阶段结束统一等待，兜底处理 block=False 动作")
    stage_pause(ctx)


def prepare_head(robot, ctx: ReplayContext) -> None:
    call(
        ctx,
        "头部下视到咖啡流程观察位",
        robot.head.look_down,
        0.5,
        block=True,
        sdk_call="robot.head.look_down",
    )


def right_hand_grasp_cup(robot, ctx: ReplayContext) -> None:
    call(ctx, "右夹爪初始化为闭合", robot.right_gripper.close, sdk_call="robot.right_gripper.close")
    call(ctx, "左夹爪初始化为闭合", robot.left_gripper.close, sdk_call="robot.left_gripper.close")
    call(ctx, "右夹爪打开准备抓杯", robot.right_gripper.open, sdk_call="robot.right_gripper.open")
    call(
        ctx,
        "右肩俯仰到抓杯预备位",
        robot.right_arm.set_shoulder_pitch,
        0.7,
        block=False,
        sdk_call="robot.right_arm.set_shoulder_pitch",
    )
    call(
        ctx,
        "右肩翻滚到抓杯预备位",
        robot.right_arm.set_shoulder_roll,
        -0.42,
        block=False,
        sdk_call="robot.right_arm.set_shoulder_roll",
    )
    call(
        ctx,
        "右腕翻滚对齐杯把方向",
        robot.right_arm.set_wrist_roll,
        0.1,
        block=True,
        sdk_call="robot.right_arm.set_wrist_roll",
    )
    call(
        ctx,
        "右肘下探接近杯子",
        robot.right_arm.set_elbow_pitch,
        1.0,
        block=False,
        sdk_call="robot.right_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "右腕俯仰微调抓取姿态",
        robot.right_arm.set_wrist_pitch,
        0.1,
        block=True,
        sdk_call="robot.right_arm.set_wrist_pitch",
    )
    sleep_step(ctx, 1.0, reason="保留原始抓杯前等待")
    call(
        ctx,
        "右夹爪收至抓杯位置",
        robot.right_gripper.set_position,
        0.6,
        sdk_call="robot.right_gripper.set_position",
    )
    sleep_step(ctx, 1.0, reason="保留原始抓杯后等待")
    call(
        ctx,
        "右肘抬起准备离开杯架",
        robot.right_arm.set_elbow_pitch,
        0.6,
        block=False,
        sdk_call="robot.right_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "右肩俯仰抬杯离桌",
        robot.right_arm.set_shoulder_pitch,
        0.6,
        block=True,
        sdk_call="robot.right_arm.set_shoulder_pitch",
    )


def right_hand_move_to_coffee_machine(robot, ctx: ReplayContext) -> None:
    call(
        ctx,
        "右肩翻滚离开抓杯位",
        robot.right_arm.set_shoulder_roll,
        0.3,
        sdk_call="robot.right_arm.set_shoulder_roll",
    )
    call(
        ctx,
        "右肩俯仰朝向咖啡机",
        robot.right_arm.set_shoulder_pitch,
        0.7,
        block=False,
        sdk_call="robot.right_arm.set_shoulder_pitch",
    )
    call(
        ctx,
        "右肩翻滚横向送杯到咖啡机",
        robot.right_arm.set_shoulder_roll,
        0.65,
        block=False,
        sdk_call="robot.right_arm.set_shoulder_roll",
    )
    call(
        ctx,
        "右腕翻滚调整接咖啡角度",
        robot.right_arm.set_wrist_roll,
        -0.3,
        sdk_call="robot.right_arm.set_wrist_roll",
    )
    call(
        ctx,
        "右肩俯仰继续送杯到咖啡出口",
        robot.right_arm.set_shoulder_pitch,
        0.98,
        block=False,
        sdk_call="robot.right_arm.set_shoulder_pitch",
    )
    call(
        ctx,
        "右肘俯仰配合送杯到最终接咖啡位",
        robot.right_arm.set_elbow_pitch,
        0.98,
        block=False,
        sdk_call="robot.right_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "右腕翻滚微调接咖啡姿态",
        robot.right_arm.set_wrist_roll,
        -0.68,
        block=False,
        sdk_call="robot.right_arm.set_wrist_roll",
    )
    call(
        ctx,
        "右腕俯仰归零以对齐杯口",
        robot.right_arm.set_wrist_pitch,
        0.0,
        block=True,
        sdk_call="robot.right_arm.set_wrist_pitch",
    )


def left_hand_press_button(robot, ctx: ReplayContext) -> None:
    call(
        ctx,
        "左肩俯仰抬起接近咖啡机按键",
        robot.left_arm.set_shoulder_pitch,
        0.5,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_pitch",
    )
    call(
        ctx,
        "左肘前探准备按键",
        robot.left_arm.set_elbow_pitch,
        -0.1,
        block=True,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左肘轻推执行按键",
        robot.left_arm.set_elbow_pitch,
        -0.03,
        block=True,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左肘回撤离开按键",
        robot.left_arm.set_elbow_pitch,
        -0.1,
        block=True,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(ctx, "左臂返回 home", robot.left_arm.home, sdk_call="robot.left_arm.home")
    sleep_step(ctx, 0.5, reason="保留原始按键后等待")


def right_hand_retreat_after_coffee(robot, ctx: ReplayContext) -> None:
    call(ctx, "右臂从咖啡机位置回撤到 home", robot.right_arm.home, sdk_call="robot.right_arm.home")
    call(
        ctx,
        "右肩翻滚切到后续接奶中间姿态",
        robot.right_arm.set_shoulder_roll,
        0.6,
        block=False,
        sdk_call="robot.right_arm.set_shoulder_roll",
    )
    call(
        ctx,
        "右腕俯仰切到后续接奶中间姿态",
        robot.right_arm.set_wrist_pitch,
        -0.3,
        block=True,
        sdk_call="robot.right_arm.set_wrist_pitch",
    )


def left_hand_grasp_milk_pitcher(robot, ctx: ReplayContext) -> None:
    call(ctx, "左夹爪打开准备抓奶壶", robot.left_gripper.open, sdk_call="robot.left_gripper.open")
    call(
        ctx,
        "左肩偏航朝向奶壶",
        robot.left_arm.set_shoulder_yaw,
        0.3,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_yaw",
    )
    call(
        ctx,
        "左腕翻滚对齐奶壶把手",
        robot.left_arm.set_wrist_roll,
        -0.4,
        block=False,
        sdk_call="robot.left_arm.set_wrist_roll",
    )
    call(
        ctx,
        "左肩翻滚进入抓壶预备位",
        robot.left_arm.set_shoulder_roll,
        -0.76,
        block=True,
        sdk_call="robot.left_arm.set_shoulder_roll",
    )
    call(
        ctx,
        "左肩俯仰抬起到抓壶路径",
        robot.left_arm.set_shoulder_pitch,
        0.8,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_pitch",
    )
    call(
        ctx,
        "左肘俯仰接近奶壶",
        robot.left_arm.set_elbow_pitch,
        0.8,
        block=False,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左腕翻滚微调抓壶姿态",
        robot.left_arm.set_wrist_roll,
        0.1,
        block=True,
        sdk_call="robot.left_arm.set_wrist_roll",
    )
    call(
        ctx,
        "左肘下探进入抓壶深度",
        robot.left_arm.set_elbow_pitch,
        1.35,
        block=False,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左肩俯仰微调抓壶对位",
        robot.left_arm.set_shoulder_pitch,
        0.85,
        block=True,
        sdk_call="robot.left_arm.set_shoulder_pitch",
    )
    sleep_step(ctx, 1.0, reason="保留原始抓壶前等待")
    call(
        ctx,
        "左夹爪收至抓壶位置",
        robot.left_gripper.set_position,
        0.6,
        sdk_call="robot.left_gripper.set_position",
    )
    sleep_step(ctx, 1.0, reason="保留原始抓壶后等待")


def left_hand_move_to_pour_pose(robot, ctx: ReplayContext) -> None:
    # 该阶段按原始 coffee.py 顺序保守拆分：先把左手抓壶后的高度与朝向调整到倒奶预备位，
    # 再把右手送到接奶位置，不在这里执行明显倒奶动作。
    call(
        ctx,
        "左肘抬起带离奶壶取放位",
        robot.left_arm.set_elbow_pitch,
        0.9,
        block=True,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左肩俯仰过渡到倒奶预备姿态",
        robot.left_arm.set_shoulder_pitch,
        0.2,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_pitch",
    )
    call(
        ctx,
        "左肘俯仰过渡到倒奶预备姿态",
        robot.left_arm.set_elbow_pitch,
        0.4,
        block=True,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左臂按原流程回到中间过渡姿态",
        robot.left_arm.home,
        block=False,
        sdk_call="robot.left_arm.home",
    )
    call(
        ctx,
        "左腕俯仰切到倒奶预备角度",
        robot.left_arm.set_wrist_pitch,
        -0.45,
        block=True,
        sdk_call="robot.left_arm.set_wrist_pitch",
    )
    call(
        ctx,
        "右腕偏航对准接奶方向",
        robot.right_arm.set_wrist_yaw,
        -0.7,
        block=False,
        sdk_call="robot.right_arm.set_wrist_yaw",
    )
    call(
        ctx,
        "右腕俯仰切到接奶角度",
        robot.right_arm.set_wrist_pitch,
        -0.5,
        block=False,
        sdk_call="robot.right_arm.set_wrist_pitch",
    )
    call(
        ctx,
        "右腕翻滚调整杯口姿态",
        robot.right_arm.set_wrist_roll,
        0.3,
        block=False,
        sdk_call="robot.right_arm.set_wrist_roll",
    )
    call(
        ctx,
        "右肩翻滚切到接奶位",
        robot.right_arm.set_shoulder_roll,
        0.7,
        block=False,
        sdk_call="robot.right_arm.set_shoulder_roll",
    )
    sleep_step(ctx, 1.0, reason="保留原始倒奶前等待")


def left_hand_pour_milk(robot, ctx: ReplayContext) -> None:
    config = ctx.latte_config
    swing_left = (
        round(config.shoulder_roll_center - config.shoulder_roll_amp, 4),
        round(config.elbow_pitch_center - config.elbow_pitch_amp, 4),
    )
    swing_right = (
        round(config.shoulder_roll_center + config.shoulder_roll_amp, 4),
        round(config.elbow_pitch_center + config.elbow_pitch_amp, 4),
    )

    ctx.logger.info(
        "[%s] latte profile=%s | wrist_roll_max=%.2f | shoulder_roll_center=%.2f | "
        "shoulder_roll_amp=%.2f | elbow_pitch_center=%.2f | elbow_pitch_amp=%.2f | "
        "sway_count=%d | step_sleep=%.2f",
        ctx.current_stage,
        config.reach_profile,
        config.wrist_roll_max,
        config.shoulder_roll_center,
        config.shoulder_roll_amp,
        config.elbow_pitch_center,
        config.elbow_pitch_amp,
        config.sway_count,
        config.step_sleep,
    )

    call(
        ctx,
        "左肩俯仰切入保守倒奶预备姿态",
        robot.left_arm.set_shoulder_pitch,
        config.left_pour_shoulder_pitch,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_pitch",
    )
    call(
        ctx,
        "左肘俯仰对齐保守摆动中心",
        robot.left_arm.set_elbow_pitch,
        config.elbow_pitch_center,
        block=False,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左肩翻滚对齐保守摆动中心",
        robot.left_arm.set_shoulder_roll,
        config.shoulder_roll_center,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_roll",
    )
    call(
        ctx,
        "左腕翻滚进入保守倒奶预备角",
        robot.left_arm.set_wrist_roll,
        config.left_pour_wrist_roll_prep,
        block=True,
        sdk_call="robot.left_arm.set_wrist_roll",
    )
    sleep_step(ctx, 0.3, reason="保守倒奶预备后短暂停顿")
    call(
        ctx,
        "左腕翻滚增大到保守倒奶角",
        robot.left_arm.set_wrist_roll,
        config.wrist_roll_max,
        block=True,
        sdk_call="robot.left_arm.set_wrist_roll",
    )
    sleep_step(ctx, 0.3, reason="进入保守倒奶角后短暂停顿")

    for cycle_idx in range(config.sway_count):
        call(
            ctx,
            f"左手保守拉花第 {cycle_idx + 1}/{config.sway_count} 次左摆 shoulder_roll",
            robot.left_arm.set_shoulder_roll,
            swing_left[0],
            block=False,
            sdk_call="robot.left_arm.set_shoulder_roll",
        )
        call(
            ctx,
            f"左手保守拉花第 {cycle_idx + 1}/{config.sway_count} 次左摆 elbow_pitch",
            robot.left_arm.set_elbow_pitch,
            swing_left[1],
            block=True,
            sdk_call="robot.left_arm.set_elbow_pitch",
        )
        sleep_step(
            ctx,
            config.step_sleep,
            reason=f"左手保守拉花第 {cycle_idx + 1}/{config.sway_count} 次左摆停顿",
        )
        call(
            ctx,
            f"左手保守拉花第 {cycle_idx + 1}/{config.sway_count} 次右摆 shoulder_roll",
            robot.left_arm.set_shoulder_roll,
            swing_right[0],
            block=False,
            sdk_call="robot.left_arm.set_shoulder_roll",
        )
        call(
            ctx,
            f"左手保守拉花第 {cycle_idx + 1}/{config.sway_count} 次右摆 elbow_pitch",
            robot.left_arm.set_elbow_pitch,
            swing_right[1],
            block=True,
            sdk_call="robot.left_arm.set_elbow_pitch",
        )
        sleep_step(
            ctx,
            config.step_sleep,
            reason=f"左手保守拉花第 {cycle_idx + 1}/{config.sway_count} 次右摆停顿",
        )

    call(
        ctx,
        "左肩翻滚进入局部收束位",
        robot.left_arm.set_shoulder_roll,
        config.left_recover_shoulder_roll,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_roll",
    )
    call(
        ctx,
        "左肘俯仰进入局部收束位",
        robot.left_arm.set_elbow_pitch,
        config.left_recover_elbow_pitch,
        block=False,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左腕翻滚回到倒奶预备角以减小回摆",
        robot.left_arm.set_wrist_roll,
        config.left_pour_wrist_roll_prep,
        block=True,
        sdk_call="robot.left_arm.set_wrist_roll",
    )
    sleep_step(ctx, 0.3, reason="局部收束后短暂停顿")
    call(
        ctx,
        "左腕翻滚回到中间角度",
        robot.left_arm.set_wrist_roll,
        0.0,
        block=False,
        sdk_call="robot.left_arm.set_wrist_roll",
    )
    call(
        ctx,
        "左肩翻滚保持在收束角度",
        robot.left_arm.set_shoulder_roll,
        config.left_recover_shoulder_roll,
        block=True,
        sdk_call="robot.left_arm.set_shoulder_roll",
    )
    call(
        ctx,
        "左肩俯仰回到倒奶后收束角度",
        robot.left_arm.set_shoulder_pitch,
        config.left_recover_shoulder_pitch,
        block=True,
        sdk_call="robot.left_arm.set_shoulder_pitch",
    )
    sleep_step(ctx, 0.3, reason="保守倒奶动作完成后观察停稳")


def return_home(robot, ctx: ReplayContext) -> None:
    # 该阶段按原始 coffee.py 顺序保守拆分：先按原流程回杯，再按原流程回奶壶。
    call(ctx, "右臂先回 home 准备放杯", robot.right_arm.home, sdk_call="robot.right_arm.home")
    call(
        ctx,
        "右肩俯仰回到放杯预备位",
        robot.right_arm.set_shoulder_pitch,
        0.7,
        block=False,
        sdk_call="robot.right_arm.set_shoulder_pitch",
    )
    call(
        ctx,
        "右肩翻滚回到放杯预备位",
        robot.right_arm.set_shoulder_roll,
        -0.42,
        block=False,
        sdk_call="robot.right_arm.set_shoulder_roll",
    )
    call(
        ctx,
        "右腕翻滚回到放杯预备位",
        robot.right_arm.set_wrist_roll,
        0.1,
        block=True,
        sdk_call="robot.right_arm.set_wrist_roll",
    )
    call(
        ctx,
        "右肘下探回放杯深度",
        robot.right_arm.set_elbow_pitch,
        1.0,
        block=False,
        sdk_call="robot.right_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "右腕俯仰回到放杯姿态",
        robot.right_arm.set_wrist_pitch,
        0.1,
        block=True,
        sdk_call="robot.right_arm.set_wrist_pitch",
    )
    sleep_step(ctx, 1.0, reason="保留原始放杯前等待")
    call(ctx, "右夹爪打开放杯", robot.right_gripper.open, sdk_call="robot.right_gripper.open")
    sleep_step(ctx, 1.0, reason="保留原始放杯后等待")
    call(ctx, "右臂回 home", robot.right_arm.home, sdk_call="robot.right_arm.home")
    sleep_step(ctx, 1.0, reason="保留原始右臂回 home 后等待")
    call(ctx, "左臂先回 home 准备放奶壶", robot.left_arm.home, sdk_call="robot.left_arm.home")
    call(
        ctx,
        "左肩偏航回到放壶预备位",
        robot.left_arm.set_shoulder_yaw,
        0.3,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_yaw",
    )
    call(
        ctx,
        "左腕翻滚回到放壶预备位",
        robot.left_arm.set_wrist_roll,
        -0.4,
        block=False,
        sdk_call="robot.left_arm.set_wrist_roll",
    )
    call(
        ctx,
        "左肩翻滚回到放壶预备位",
        robot.left_arm.set_shoulder_roll,
        -0.76,
        block=True,
        sdk_call="robot.left_arm.set_shoulder_roll",
    )
    call(
        ctx,
        "左肩俯仰回到放壶路径",
        robot.left_arm.set_shoulder_pitch,
        0.8,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_pitch",
    )
    call(
        ctx,
        "左肘俯仰回到放壶路径",
        robot.left_arm.set_elbow_pitch,
        0.8,
        block=False,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左腕翻滚微调放壶姿态",
        robot.left_arm.set_wrist_roll,
        0.1,
        block=True,
        sdk_call="robot.left_arm.set_wrist_roll",
    )
    call(
        ctx,
        "左肘下探回放壶深度",
        robot.left_arm.set_elbow_pitch,
        1.35,
        block=False,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左肩俯仰微调放壶对位",
        robot.left_arm.set_shoulder_pitch,
        0.85,
        block=True,
        sdk_call="robot.left_arm.set_shoulder_pitch",
    )
    sleep_step(ctx, 1.0, reason="保留原始放壶前等待")
    call(ctx, "左夹爪打开放奶壶", robot.left_gripper.open, sdk_call="robot.left_gripper.open")
    call(
        ctx,
        "左肘抬起离开放壶位",
        robot.left_arm.set_elbow_pitch,
        0.9,
        block=True,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左肩俯仰回到放壶后过渡位",
        robot.left_arm.set_shoulder_pitch,
        0.2,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_pitch",
    )
    call(
        ctx,
        "左肘俯仰回到放壶后过渡位",
        robot.left_arm.set_elbow_pitch,
        0.4,
        block=True,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左臂最终回 home",
        robot.left_arm.home,
        block=False,
        sdk_call="robot.left_arm.home",
    )
    sleep_step(ctx, 1.0, reason="保留原始流程结束等待")


def get_stage_definitions():
    return {
        "prepare_head": {
            "func": prepare_head,
            "components": "head",
            "risk": "头部下视，需确认前方无遮挡并避免误判为整机静止。",
        },
        "right_hand_grasp_cup": {
            "func": right_hand_grasp_cup,
            "components": "right_arm / right_gripper / left_gripper",
            "risk": "右手接近杯子抓取，需确认杯位、桌边和夹爪周围无人手靠近。",
        },
        "right_hand_move_to_coffee_machine": {
            "func": right_hand_move_to_coffee_machine,
            "components": "right_arm / right_gripper",
            "risk": "右手横向送杯到咖啡机，注意肩部扫掠范围和咖啡机前方碰撞风险。",
        },
        "left_hand_press_button": {
            "func": left_hand_press_button,
            "components": "left_arm",
            "risk": "左手接近按键区域，需确认咖啡机面板和周围障碍物安全。",
        },
        "right_hand_retreat_after_coffee": {
            "func": right_hand_retreat_after_coffee,
            "components": "right_arm / right_gripper",
            "risk": "右手带杯回撤，注意杯体晃动和肩腕回撤路径。",
        },
        "left_hand_grasp_milk_pitcher": {
            "func": left_hand_grasp_milk_pitcher,
            "components": "left_arm / left_gripper",
            "risk": "左手接近奶壶抓取，需确认奶壶位置稳定且周围无遮挡。",
        },
        "left_hand_move_to_pour_pose": {
            "func": left_hand_move_to_pour_pose,
            "components": "left_arm / right_arm / left_gripper / right_gripper",
            "risk": "双臂进入倒奶预备姿态，需重点关注双臂互相干涉和杯壶间距。",
        },
        "left_hand_pour_milk": {
            "func": left_hand_pour_milk,
            "components": "left_arm / left_gripper / right_arm / right_gripper",
            "risk": "左手执行保守拉花/倒奶动作，风险主要来自 wrist_roll 倒奶角、shoulder_roll 摆动以及双臂间距。",
        },
        "return_home": {
            "func": return_home,
            "components": "left_arm / right_arm / left_gripper / right_gripper",
            "risk": "回放杯与回放奶壶都在本阶段执行，需确认路径、桌面和末端物体状态。",
        },
    }


def resolve_stage_sequence(stage: str) -> list[str]:
    if stage == "all":
        return STAGE_ORDER.copy()
    return [stage]


def run_stage(stage_name: str, stage_meta: dict[str, str], robot, ctx: ReplayContext) -> None:
    ctx.current_stage = stage_name
    confirm_stage(ctx, stage_name, stage_meta, robot)
    ctx.logger.info("阶段开始：%s", stage_name)
    stage_meta["func"](robot, ctx)
    finalize_stage(ctx, robot)
    ctx.logger.info("阶段结束：%s", stage_name)


def ensure_execution_allowed(args: argparse.Namespace) -> bool:
    if args.execute and args.dry_run:
        raise SystemExit("参数冲突：--dry-run 与 --execute 不能同时使用")
    if not args.execute:
        return False
    if not args.i_understand_real_robot_risk:
        raise SystemExit("拒绝执行：实机模式必须同时传入 --execute 和 --i-understand-real-robot-risk")
    return True


def build_log_path(raw_path: str | None) -> Path:
    if raw_path:
        return Path(raw_path)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"coffee_replay_safe_{timestamp}.log"


def main() -> int:
    args = parse_args()
    execute = ensure_execution_allowed(args)
    dry_run = not execute
    log_path = build_log_path(args.log_file)
    logger = build_logger(log_path)
    latte_config = build_latte_pour_config(args)

    ctx = ReplayContext(
        dry_run=dry_run,
        yes=args.yes,
        logger=logger,
        latte_config=latte_config,
        stage_notes={},
    )

    stage_definitions = get_stage_definitions()
    selected_stages = resolve_stage_sequence(args.stage)

    logger.info("=== coffee_replay_safe 启动 ===")
    logger.info("DEFAULT_ROBOT_VERSION=%s", DEFAULT_ROBOT_VERSION)
    logger.info("log_file=%s", log_path)
    logger.info("dry_run=%s", dry_run)
    logger.info("execute=%s", execute)
    logger.info("selected_stage=%s", args.stage)
    logger.info("resolved_stages=%s", ", ".join(selected_stages))
    logger.info("latte_profile=%s", latte_config.reach_profile)
    logger.info(
        "latte_motion=wrist_roll_max=%.2f, shoulder_roll_center=%.2f, shoulder_roll_amp=%.2f, "
        "elbow_pitch_center=%.2f, elbow_pitch_amp=%.2f, sway_count=%d, step_sleep=%.2f",
        latte_config.wrist_roll_max,
        latte_config.shoulder_roll_center,
        latte_config.shoulder_roll_amp,
        latte_config.elbow_pitch_center,
        latte_config.elbow_pitch_amp,
        latte_config.sway_count,
        latte_config.step_sleep,
    )
    logger.info(
        "latte_recover=left_pour_shoulder_pitch=%.2f, left_pour_wrist_roll_prep=%.2f, "
        "left_recover_shoulder_pitch=%.2f, left_recover_shoulder_roll=%.2f, left_recover_elbow_pitch=%.2f",
        latte_config.left_pour_shoulder_pitch,
        latte_config.left_pour_wrist_roll_prep,
        latte_config.left_recover_shoulder_pitch,
        latte_config.left_recover_shoulder_roll,
        latte_config.left_recover_elbow_pitch,
    )
    if args.latte_allow_risky_pose:
        logger.warning("已启用 --latte-allow-risky-pose，请确认现场急停与双臂安全间隙。")

    if dry_run:
        logger.info("当前为 dry-run：不会连接机器人，不会执行任何 SDK 动作。")
    else:
        logger.warning("当前为 execute：将尝试连接机器人并执行实机动作。")

    robot = DryRunRobot() if dry_run else None
    try:
        if execute:
            robot = connect_robot_with_selector(args, script_name="coffee_replay_safe.py")
            if robot is None:
                logger.warning("连接流程未返回机器人实例，脚本提前结束。")
                return 0

        for stage_name in selected_stages:
            run_stage(stage_name, stage_definitions[stage_name], robot, ctx)

        logger.info("全部请求阶段执行完成。")
        return 0
    except KeyboardInterrupt:
        logger.warning("收到 KeyboardInterrupt，中止执行。")
        return 130
    except SystemExit as exc:
        logger.warning("脚本退出：%s", exc)
        raise
    except Exception as exc:
        logger.exception("执行过程中出现异常：%s", exc)
        return 1
    finally:
        if execute and robot is not None:
            try:
                robot.stop()
                logger.info("finally: 已尝试调用 robot.stop()")
            except Exception:
                logger.exception("finally: 调用 robot.stop() 失败")
            try:
                robot.disconnect()
                logger.info("finally: 已尝试调用 robot.disconnect()")
            except Exception:
                logger.exception("finally: 调用 robot.disconnect() 失败")


if __name__ == "__main__":
    raise SystemExit(main())
