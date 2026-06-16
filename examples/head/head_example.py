from __future__ import annotations

import argparse

from examples.common import add_common_motion_arguments, add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(description="Demonstrate head pose and look helpers.")
    add_robot_arguments(parser)
    add_common_motion_arguments(parser)
    args = parser.parse_args()
    block = not args.non_blocking

    with connected_robot(args) as robot:
        robot.head.set_speed(1.0)
        print(f"limits={robot.head.limits}")

        robot.head.look_left(0.2, block=block)
        robot.head.look_right(0.2, block=block)
        robot.head.look_up(0.1, block=block)
        robot.head.look_down(0.1, block=block)
        robot.head.set_pitch(0.0, block=block)
        robot.head.set_yaw(0.1, block=block)
        robot.head.set_pose(pitch=0.0, yaw=0.0, block=block)

        if args.non_blocking:
            robot.head.wait()

        print(f"pitch={robot.head.pitch:.3f}")
        print(f"yaw={robot.head.yaw:.3f}")
        print(f"is_moving={robot.head.is_moving}")
        robot.head.center(block=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
