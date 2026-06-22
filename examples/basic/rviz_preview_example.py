from __future__ import annotations

import argparse

from examples.common import add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(description="Small RViz preview style motion.")
    add_robot_arguments(parser)
    args = parser.parse_args()

    with connected_robot(args) as robot:
        robot.left_arm.set_shoulder_pitch(-0.2, block=True)
        robot.right_arm.set_shoulder_pitch(-0.2, block=True)
        robot.head.look_left(0.2, block=True)
        robot.home(block=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
