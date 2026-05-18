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
class ReplayContext:
    dry_run: bool
    yes: bool
    logger: logging.Logger
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
    return parser.parse_args()


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
    # TODO: 后续可替换为拉花轨迹执行器。
    call(
        ctx,
        "左肩俯仰切入倒奶动作",
        robot.left_arm.set_shoulder_pitch,
        -0.6,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_pitch",
    )
    call(
        ctx,
        "左肘俯仰切入倒奶动作",
        robot.left_arm.set_elbow_pitch,
        -0.6,
        block=True,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左肩翻滚增大壶身倾斜",
        robot.left_arm.set_shoulder_roll,
        0.65,
        block=False,
        sdk_call="robot.left_arm.set_shoulder_roll",
    )
    call(
        ctx,
        "左腕翻滚执行大角度倒奶",
        robot.left_arm.set_wrist_roll,
        1.7,
        block=False,
        sdk_call="robot.left_arm.set_wrist_roll",
    )
    call(
        ctx,
        "左肘俯仰微调倒奶轨迹",
        robot.left_arm.set_elbow_pitch,
        -0.5,
        block=True,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左肘俯仰回到原始收束角度",
        robot.left_arm.set_elbow_pitch,
        -0.6,
        block=False,
        sdk_call="robot.left_arm.set_elbow_pitch",
    )
    call(
        ctx,
        "左腕翻滚回到中间角度",
        robot.left_arm.set_wrist_roll,
        0.0,
        block=True,
        sdk_call="robot.left_arm.set_wrist_roll",
    )
    call(
        ctx,
        "左肩翻滚回到中间角度",
        robot.left_arm.set_shoulder_roll,
        0.0,
        block=True,
        sdk_call="robot.left_arm.set_shoulder_roll",
    )


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
            "risk": "左手执行大角度倒奶动作，是当前流程中风险最高的阶段之一。",
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

    ctx = ReplayContext(
        dry_run=dry_run,
        yes=args.yes,
        logger=logger,
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
