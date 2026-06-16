from __future__ import annotations

import argparse
from contextlib import contextmanager
from typing import Iterator

from mantis import Mantis


def add_robot_arguments(
    parser: argparse.ArgumentParser,
    *,
    default_robot_version: str = "2.0",
    require_target: bool = True,
) -> None:
    parser.add_argument("--ip", help="Robot IP address.")
    parser.add_argument("--sn", help="Robot serial number.")
    parser.add_argument("--port", type=int, default=7447, help="Zenoh router port.")
    parser.add_argument(
        "--robot-version",
        choices=("2.0", "3.0"),
        default=default_robot_version,
        help="Target robot generation.",
    )
    verify_group = parser.add_mutually_exclusive_group()
    verify_group.add_argument(
        "--verify",
        dest="verify",
        action="store_true",
        help="Wait for robot status during connect.",
    )
    verify_group.add_argument(
        "--no-verify",
        dest="verify",
        action="store_false",
        help="Skip robot status verification during connect.",
    )
    parser.set_defaults(verify=True)
    if require_target:
        parser.epilog = "Pass either --ip or --sn. Examples never use a hard-coded robot target."


def validate_robot_target(args: argparse.Namespace) -> None:
    if not getattr(args, "ip", None) and not getattr(args, "sn", None):
        raise SystemExit("Please pass --ip or --sn before running a robot motion example.")


@contextmanager
def connected_robot(args: argparse.Namespace, *, require_target: bool = True) -> Iterator[Mantis]:
    if require_target:
        validate_robot_target(args)

    robot = Mantis(
        ip=getattr(args, "ip", None),
        sn=getattr(args, "sn", None),
        port=getattr(args, "port", 7447),
        robot_version=getattr(args, "robot_version", "2.0"),
    )
    robot.connect(verify=getattr(args, "verify", True))
    try:
        yield robot
    finally:
        robot.disconnect()


def add_common_motion_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--non-blocking",
        action="store_true",
        help="Send motions with block=False where the example supports it.",
    )


def print_robot_identity(robot: Mantis) -> None:
    print(f"connected={robot.is_connected}")
    print(f"robot_sn={robot.robot_sn}")
    print(f"robot_ip={robot.robot_ip}")
    print(f"robot_version={robot.robot_version}")
