"""
实机连接测试
测试与真实机器人的通信（通过 Python 桥接节点）

使用前请确保:
1. 机器人端启动 Python 桥接节点: ros2 run bw_sdk_bridge sdk_bridge
2. 机器人端 ROS2 节点已启动并订阅相应话题
"""

import argparse

from connection_selector import add_connection_args, connect_robot_with_selector


DEFAULT_ROBOT_VERSION = "3.0"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mantis 全关节方向测试副本")
    add_connection_args(parser, default_profile="interactive")
    parser.set_defaults(robot_version=DEFAULT_ROBOT_VERSION)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print("=== Mantis 全关节方向测试 ===\n")
    print(f"当前机器人版本: {DEFAULT_ROBOT_VERSION}")

    robot = None
    try:
        robot = connect_robot_with_selector(args, script_name=__file__)
        if robot is None:
            return
        robot.home()
        robot.right_arm.set_shoulder_pitch(-0.7, block=False)
        robot.right_arm.set_shoulder_yaw(0.6, block=False)
        robot.right_arm.set_shoulder_roll(-0.4)
        robot.right_arm.set_wrist_roll(0.5)
        robot.right_arm.set_wrist_yaw(0.5)
        robot.right_arm.set_wrist_pitch(0.5)
        robot.right_arm.set_elbow_pitch(-0.5)
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    main()
