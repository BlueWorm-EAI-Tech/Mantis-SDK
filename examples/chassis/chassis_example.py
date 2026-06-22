from __future__ import annotations

import argparse

from examples.common import add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(description="Demonstrate safe chassis distance and angle commands.")
    add_robot_arguments(parser)
    parser.add_argument(
        "--mode",
        choices=("basic", "speed", "move", "non-blocking", "all"),
        default="basic",
    )
    args = parser.parse_args()

    with connected_robot(args) as robot:
        robot.chassis.set_friction(linear=1.2, angular=1.2)
        robot.chassis.set_default_speed(linear=0.2, angular=0.5)

        if args.mode in ("basic", "all"):
            robot.chassis.forward(0.10)
            robot.chassis.backward(0.10)
            robot.chassis.strafe_left(0.05)
            robot.chassis.strafe_right(0.05)
            robot.chassis.turn_left(15)
            robot.chassis.turn_right(15)

        if args.mode in ("speed", "all"):
            robot.chassis.forward(0.10, speed=0.1)
            robot.chassis.turn_left(15, speed=0.4)

        if args.mode in ("move", "all"):
            robot.chassis.move(x=0.10, y=0.05, angle=10, linear_speed=0.1, angular_speed=0.4)

        if args.mode in ("non-blocking", "all"):
            robot.chassis.forward(0.10, speed=0.1, block=False)
            robot.chassis.wait()

        robot.chassis.stop()
        print(f"is_moving={robot.chassis.is_moving}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
