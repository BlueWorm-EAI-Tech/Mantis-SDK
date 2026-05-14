"""Shared connection selection helpers for Mantis scripts."""

from __future__ import annotations

import argparse
from pathlib import Path

from mantis import Mantis


DEFAULT_REAL_IP = "192.168.1.151"
DEFAULT_SN = "BW_3N5CRT22"

INTERACTIVE_PROFILE = "interactive"
REAL_IP_PROFILE = "real-ip"
REAL_SN_PROFILE = "real-sn"

PROFILE_CHOICES = (
    INTERACTIVE_PROFILE,
    REAL_IP_PROFILE,
    REAL_SN_PROFILE,
)


def add_connection_args(
    parser: argparse.ArgumentParser,
    default_profile: str = REAL_IP_PROFILE,
) -> argparse.ArgumentParser:
    """Register shared connection-related CLI arguments."""
    parser.add_argument(
        "--conn-profile",
        choices=PROFILE_CHOICES,
        default=default_profile,
        help=(
            "连接模式：interactive 启动时选择；real-ip 按 IP 连接指定 Mantis Bridge；"
            "real-sn 按 SN 连接机器人"
        ),
    )
    parser.add_argument(
        "--real-ip",
        "--ip",
        dest="real_ip",
        default=DEFAULT_REAL_IP,
        help=f"按 IP 连接的 Mantis Bridge 地址，默认 {DEFAULT_REAL_IP}；--ip 作为兼容别名保留",
    )
    parser.add_argument(
        "--sn",
        default=DEFAULT_SN,
        help=f"机器人 SN，默认 {DEFAULT_SN}",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="若连接模式已确定，则跳过最终 Enter/q 确认；interactive 模式仍会先选择模式",
    )
    parser.add_argument(
        "--print-connection-config",
        action="store_true",
        help="只打印当前连接配置，不连接机器人，也不执行后续动作",
    )
    return parser


def select_connection_profile(args: argparse.Namespace, script_name: str = "") -> str:
    """Resolve the effective connection profile."""
    profile = getattr(args, "conn_profile", INTERACTIVE_PROFILE)
    if profile != INTERACTIVE_PROFILE:
        return profile

    if getattr(args, "print_connection_config", False):
        return INTERACTIVE_PROFILE

    script_label = _script_label(script_name)
    while True:
        print("=" * 72)
        print(f"当前脚本名: {script_label}")
        print("请选择连接模式：")
        print(f"1) real-ip：按 IP 连接，默认 {args.real_ip}")
        print(f"2) real-sn：按 SN 连接，默认 {args.sn}")
        print("q) 退出")
        user_input = input("请输入选择 [1/2/q]: ").strip().lower()
        if user_input == "1":
            return REAL_IP_PROFILE
        if user_input == "2":
            return REAL_SN_PROFILE
        if user_input == "q":
            raise SystemExit("用户取消连接")
        print("输入无效，请重新选择。")


def connect_robot_with_selector(
    args: argparse.Namespace,
    script_name: str = "",
):
    """Create and connect a robot according to the selected profile."""
    profile = select_connection_profile(args, script_name)

    if profile == INTERACTIVE_PROFILE:
        _print_interactive_preview(args, script_name)
        return None

    _print_connection_banner(profile, args, script_name)

    if getattr(args, "print_connection_config", False):
        print("仅打印连接配置：不会连接机器人，不会执行后续动作。")
        return None

    if not getattr(args, "yes", False):
        user_input = input("确认以上连接目标无误后按 Enter 继续，输入 q 退出：").strip().lower()
        if user_input == "q":
            raise SystemExit("用户取消连接")

    robot = Mantis(**_build_mantis_kwargs(args))
    if profile == REAL_IP_PROFILE:
        ok = robot.connect(ip=args.real_ip)
    elif profile == REAL_SN_PROFILE:
        ok = robot.connect(sn=args.sn)
    else:
        raise SystemExit(f"不支持的连接模式: {profile}")

    if ok is False:
        raise SystemExit("连接失败，停止测试")
    return robot


def _build_mantis_kwargs(args: argparse.Namespace) -> dict[str, str]:
    mantis_kwargs: dict[str, str] = {}
    robot_version = getattr(args, "robot_version", "")
    if robot_version:
        mantis_kwargs["robot_version"] = robot_version
    return mantis_kwargs


def _print_interactive_preview(args: argparse.Namespace, script_name: str) -> None:
    script_label = _script_label(script_name)
    print("=" * 72)
    print(f"当前脚本名: {script_label}")
    print("当前连接模式: interactive")
    print("当前连接目标: 交互选择（尚未实际选择具体模式）")
    print("是否真实机器人: 待用户选择")
    print("检测到 --print-connection-config，跳过交互式输入，仅展示可选目标：")
    print(f"- real-ip -> {args.real_ip}（按 IP 连接指定 Mantis Bridge）")
    print(f"- real-sn -> {args.sn}（可能连接真实机器人）")
    print("如需直接打印某一模式配置，可追加 --conn-profile real-ip/real-sn。")


def _print_connection_banner(
    profile: str,
    args: argparse.Namespace,
    script_name: str,
) -> None:
    script_label = _script_label(script_name)
    print("=" * 72)
    print(f"当前脚本名: {script_label}")
    print(f"当前连接模式: {profile}")

    if profile == REAL_IP_PROFILE:
        print(f"当前连接目标 IP: {args.real_ip}")
        print("是否真实机器人: 取决于该 IP 指向的 Bridge")
        print("说明: 该模式会按 IP 连接指定的 Mantis Bridge。")
        print("请确认该 IP 是当前要连接的实机或仿真 Bridge。")
        print("如果该 IP 指向真实机器人，请清空周围障碍物并准备物理急停。")
        return

    if profile == REAL_SN_PROFILE:
        print(f"当前连接目标: 真实机器人 SN {args.sn}")
        print("是否真实机器人: 可能是")
        print("警告: 这可能连接真实机器人。")
        print("请清空机器人周围障碍物并确认周围安全。")
        print("请准备物理急停。")
        return

    raise SystemExit(f"不支持的连接模式: {profile}")


def _script_label(script_name: str) -> str:
    if not script_name:
        return "<unknown script>"
    return Path(script_name).name
