from __future__ import annotations

import argparse

from examples.common import add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(description="Demonstrate Mantis 3.0 waist bend control.")
    add_robot_arguments(parser, default_robot_version="3.0")
    args = parser.parse_args()

    with connected_robot(args) as robot:
        if robot.robot_version != "3.0":
            raise SystemExit("Waist bend is only available with --robot-version 3.0")

        robot.waist.set_speed(0.05)
        robot.waist.set_bend_speed(0.2)
        robot.waist.home(block=True)
        robot.waist.set_height(-0.03, block=True)
        robot.waist.bend_forward(0.15, block=True)
        robot.waist.bend_backward(0.05, block=True)
        robot.waist.set_bend(0.0, block=True)

        print(f"height={robot.waist.height:.3f}")
        print(f"bend_angle={robot.waist.bend_angle:.3f}")
        robot.waist.home(block=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
