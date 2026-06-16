from __future__ import annotations

import argparse

from examples.common import add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compact multi-step workflow using arms, grippers, head, and home."
    )
    add_robot_arguments(parser, default_robot_version="3.0")
    args = parser.parse_args()

    with connected_robot(args) as robot:
        robot.home(block=True)
        robot.head.look_down(angle=0.2, block=True)

        robot.right_gripper.open(block=True)
        robot.right_arm.set_shoulder_pitch(0.3, block=False)
        robot.right_arm.set_shoulder_roll(-0.2, block=False)
        robot.right_arm.set_elbow_pitch(0.3, block=True)
        robot.right_gripper.set_position(0.6, block=True)

        robot.left_gripper.open(block=True)
        robot.left_arm.set_shoulder_pitch(0.25, block=False)
        robot.left_arm.set_elbow_pitch(0.2, block=True)
        robot.left_gripper.set_position(0.6, block=True)

        robot.right_arm.home(block=False)
        robot.left_arm.home(block=False)
        robot.head.center(block=False)
        robot.wait()
        robot.home(block=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
