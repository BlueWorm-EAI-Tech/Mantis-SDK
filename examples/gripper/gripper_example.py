from __future__ import annotations

import argparse

from examples.common import add_common_motion_arguments, add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(description="Demonstrate gripper presets and position control.")
    add_robot_arguments(parser)
    add_common_motion_arguments(parser)
    args = parser.parse_args()
    block = not args.non_blocking

    with connected_robot(args) as robot:
        robot.left_gripper.set_speed(2.0)
        robot.right_gripper.set_speed(2.0)

        robot.left_gripper.open(block=block)
        robot.right_gripper.open(block=block)
        robot.left_gripper.half_open(block=block)
        robot.right_gripper.half_open(block=block)
        robot.left_gripper.set_position(0.7, block=block)
        robot.right_gripper.set_position(0.3, block=block)

        if args.non_blocking:
            robot.wait([robot.left_gripper.joint_name, robot.right_gripper.joint_name])

        print(f"left={robot.left_gripper.position:.2f}")
        print(f"right={robot.right_gripper.position:.2f}")

        robot.left_gripper.close(block=True)
        robot.right_gripper.close(block=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
