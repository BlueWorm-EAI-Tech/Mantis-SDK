from __future__ import annotations

import argparse

from examples.common import (
    add_arm_motion_profile_arguments,
    add_common_motion_arguments,
    add_robot_arguments,
    arm_motion_profile_kwargs,
    connected_robot,
)


ABS_POSES = {
    "left": (0.50, 0.10, 0.20, 0.0, 0.0, 0.0),
    "right": (0.50, -0.10, 0.20, 0.0, 0.0, 0.0),
}
REL_DELTA = (0.02, 0.00, 0.02, 0.0, 0.0, 0.0)


def _selected_arms(robot, side):
    if side == "both":
        return [("left", robot.left_arm), ("right", robot.right_arm)]
    if side == "left":
        return [("left", robot.left_arm)]
    return [("right", robot.right_arm)]


def _wait_for_arms(robot, arms):
    joint_names = []
    for _, arm in arms:
        joint_names.extend(arm.joint_names)
    robot.wait(joint_names)


def _home_arms(robot, arms, block, **motion_profile_kwargs):
    if block and len(arms) == 1:
        _, arm = arms[0]
        arm.home(block=True, **motion_profile_kwargs)
        return
    for _, arm in arms:
        arm.home(block=False, **motion_profile_kwargs)
    _wait_for_arms(robot, arms)


def main() -> int:
    parser = argparse.ArgumentParser(description="Send robot-side IK pose commands.")
    add_robot_arguments(parser)
    add_common_motion_arguments(parser)
    add_arm_motion_profile_arguments(parser)
    parser.add_argument("--side", choices=("left", "right", "both"), default="left")
    parser.add_argument("--mode", choices=("abs", "rel", "both"), default="rel")
    args = parser.parse_args()
    block = not args.non_blocking
    profile_kwargs = arm_motion_profile_kwargs(args)

    with connected_robot(args) as robot:
        arms = _selected_arms(robot, args.side)
        if not robot.supports_ik:
            raise SystemExit(f"robot_version={robot.robot_version} does not support IK pose commands")

        print("IK is solved on the robot ROS side. SDK only publishes pose commands.")
        print(f"side={args.side}, mode={args.mode}, block={block}")

        if args.mode in ("abs", "both"):
            print("Sending absolute pose command")
            for side, arm in arms:
                arm.ik(
                    *ABS_POSES[side],
                    block=block and len(arms) == 1,
                    abs=True,
                    **profile_kwargs,
                )
            if not block or len(arms) > 1:
                _wait_for_arms(robot, arms)

        if args.mode in ("rel", "both"):
            print("Sending small relative pose command")
            for _, arm in arms:
                arm.ik(
                    *REL_DELTA,
                    block=block and len(arms) == 1,
                    abs=False,
                    **profile_kwargs,
                )
            if not block or len(arms) > 1:
                _wait_for_arms(robot, arms)

        _home_arms(robot, arms, block=True, **profile_kwargs)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
