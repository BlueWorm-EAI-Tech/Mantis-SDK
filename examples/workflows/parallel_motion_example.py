from __future__ import annotations

import argparse
import time

from examples.common import add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(description="Demonstrate block=False parallel motion.")
    add_robot_arguments(parser)
    args = parser.parse_args()

    with connected_robot(args) as robot:
        robot.home(block=True)

        start = time.time()
        robot.left_arm.set_shoulder_pitch(-0.2, block=False)
        robot.right_arm.set_shoulder_pitch(-0.2, block=False)
        robot.head.look_left(0.2, block=False)
        robot.waist.up(0.02, block=False)
        robot.left_gripper.half_open(block=False)
        robot.right_gripper.half_open(block=False)
        robot.wait()

        print(f"parallel_elapsed={time.time() - start:.2f}s")
        print(f"is_any_moving={robot.is_any_moving}")
        robot.home(block=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
