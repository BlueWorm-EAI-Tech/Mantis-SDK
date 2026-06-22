from __future__ import annotations

import argparse

from examples.common import add_common_motion_arguments, add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(description="Demonstrate arm joint control.")
    add_robot_arguments(parser)
    add_common_motion_arguments(parser)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    args = parser.parse_args()

    block = not args.non_blocking

    with connected_robot(args) as robot:
        arm = robot.left_arm if args.side == "left" else robot.right_arm
        print(f"side={arm.side}")
        print(f"joint_names={arm.joint_names}")
        print(f"limits={arm.limits}")
        print(f"shoulder_pitch_limit={arm.get_limit(0)}")

        arm.set_speed(0.8)
        arm.set_shoulder_pitch(0.2, block=block)
        arm.set_shoulder_yaw(0.1, block=block)
        arm.set_shoulder_roll(0.1, block=block)
        arm.set_elbow_pitch(0.2, block=block)
        arm.set_wrist_roll(0.1, block=block)
        arm.set_wrist_pitch(0.1, block=block)
        arm.set_wrist_yaw(0.1, block=block)

        arm.set_joint(0, 0.0, block=block)
        arm.set_joints([0.1, 0.05, 0.0, 0.2, 0.0, 0.0, 0.0], block=block)

        if args.non_blocking:
            arm.wait()

        print(f"positions={arm.positions}")
        print(f"is_moving={arm.is_moving}")
        arm.home(block=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
