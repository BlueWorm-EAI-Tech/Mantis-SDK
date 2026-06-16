from __future__ import annotations

import argparse

from examples.common import add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(description="Send robot-side IK pose commands.")
    add_robot_arguments(parser)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--mode", choices=("abs", "rel", "both"), default="rel")
    args = parser.parse_args()

    with connected_robot(args) as robot:
        arm = robot.left_arm if args.side == "left" else robot.right_arm
        if not robot.supports_ik:
            raise SystemExit(f"robot_version={robot.robot_version} does not support IK pose commands")

        print("IK is solved on the robot ROS side. SDK only publishes pose commands.")

        if args.mode in ("abs", "both"):
            print("Sending absolute pose command")
            arm.ik(0.30, 0.10, 0.20, 0.0, 0.0, 0.0, block=True, abs=True)

        if args.mode in ("rel", "both"):
            print("Sending small relative pose command")
            arm.ik(0.02, 0.00, 0.02, 0.0, 0.0, 0.0, block=True, abs=False)

        arm.home(block=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
