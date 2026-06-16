from __future__ import annotations

import argparse

from examples.common import add_common_motion_arguments, add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(description="Demonstrate waist height control.")
    add_robot_arguments(parser)
    add_common_motion_arguments(parser)
    args = parser.parse_args()
    block = not args.non_blocking

    with connected_robot(args) as robot:
        robot.waist.set_speed(0.05)
        print(f"limits={robot.waist.limits}")

        robot.waist.home(block=True)
        robot.waist.up(0.03, block=block)
        robot.waist.down(0.03, block=block)
        robot.waist.move(0.02, block=block)
        robot.waist.set_height(0.0, block=block)

        if args.non_blocking:
            robot.waist.wait()

        print(f"height={robot.waist.height:.3f}")
        print(f"is_moving={robot.waist.is_moving}")
        robot.waist.home(block=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
