from __future__ import annotations

import argparse

from examples.common import add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(description="Move one arm through a small named-joint pose.")
    add_robot_arguments(parser, default_robot_version="3.0")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    args = parser.parse_args()

    with connected_robot(args) as robot:
        arm = robot.left_arm if args.side == "left" else robot.right_arm
        robot.home(block=True)

        arm.set_shoulder_pitch(-0.3, block=False)
        arm.set_shoulder_yaw(0.2, block=False)
        arm.set_shoulder_roll(-0.2, block=False)
        arm.set_wrist_roll(0.2, block=False)
        arm.set_wrist_yaw(0.2, block=False)
        arm.set_wrist_pitch(0.1, block=False)
        arm.set_elbow_pitch(-0.2, block=True)

        print(f"positions={arm.positions}")
        arm.home(block=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
