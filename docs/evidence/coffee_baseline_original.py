"""
实机连接测试
测试与真实机器人的通信（通过 Python 桥接节点）

使用前请确保:
1. 机器人端启动 Python 桥接节点: ros2 run bw_sdk_bridge sdk_bridge
2. 机器人端 ROS2 节点已启动并订阅相应话题
"""

import argparse
import time

from connection_selector import add_connection_args, connect_robot_with_selector


DEFAULT_ROBOT_VERSION = "3.0"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mantis 全关节方向测试")
    add_connection_args(parser, default_profile="interactive")
    parser.set_defaults(robot_version=DEFAULT_ROBOT_VERSION)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print("=== Mantis 全关节方向测试 ===\n")
    print(f"当前机器人版本: {DEFAULT_ROBOT_VERSION}")
    print("当前仍是完整咖啡流程脚本。")

    robot = None
    try:
        robot = connect_robot_with_selector(args, script_name="coffee.py")
        if robot is None:
            return

        robot.head.look_down(angle=0.5, block=True)
        for i in range(200):
            print(f"第{i}次测试")
            robot.right_gripper.close()
            robot.left_gripper.close()

            robot.right_gripper.open()
            robot.right_arm.set_shoulder_pitch(0.7, block=False)
            robot.right_arm.set_shoulder_roll(-0.42, block=False)
            robot.right_arm.set_wrist_roll(0.1, block=True)

            robot.right_arm.set_elbow_pitch(1.0, block=False)
            robot.right_arm.set_wrist_pitch(0.1)
            time.sleep(1)
            robot.right_gripper.set_position(0.6)
            time.sleep(1)

            robot.right_arm.set_elbow_pitch(0.6, block=False)
            robot.right_arm.set_shoulder_pitch(0.6, block=True)

            robot.right_arm.set_shoulder_roll(0.3)

            robot.right_arm.set_shoulder_pitch(0.7, block=False)
            robot.right_arm.set_shoulder_roll(0.65, block=False)
            robot.right_arm.set_wrist_roll(-0.3)

            robot.right_arm.set_shoulder_pitch(0.98, block=False)
            robot.right_arm.set_elbow_pitch(0.98, block=False)
            robot.right_arm.set_wrist_roll(-0.68, block=False)
            robot.right_arm.set_wrist_pitch(0)

            robot.left_arm.set_shoulder_pitch(0.5, block=False)
            robot.left_arm.set_elbow_pitch(-0.1, block=True)

            robot.left_arm.set_elbow_pitch(-0.03, block=True)
            robot.left_arm.set_elbow_pitch(-0.1, block=True)
            robot.left_arm.home()
            time.sleep(0.5)

            robot.right_arm.home()
            robot.right_arm.set_shoulder_roll(0.6, block=False)
            robot.right_arm.set_wrist_pitch(-0.3)

            robot.left_gripper.open()

            robot.left_arm.set_shoulder_yaw(0.3, block=False)
            robot.left_arm.set_wrist_roll(-0.4, block=False)
            robot.left_arm.set_shoulder_roll(-0.76, block=True)

            robot.left_arm.set_shoulder_pitch(0.8, block=False)
            robot.left_arm.set_elbow_pitch(0.8, block=False)
            robot.left_arm.set_wrist_roll(0.1, block=True)

            robot.left_arm.set_elbow_pitch(1.35, block=False)
            robot.left_arm.set_shoulder_pitch(0.85, block=True)

            time.sleep(1)
            robot.left_gripper.set_position(0.6)
            time.sleep(1)

            robot.left_arm.set_elbow_pitch(0.9, block=True)

            robot.left_arm.set_shoulder_pitch(0.2, block=False)
            robot.left_arm.set_elbow_pitch(0.4, block=True)

            robot.left_arm.home(block=False)
            robot.left_arm.set_wrist_pitch(-0.45, block=True)

            robot.right_arm.set_wrist_yaw(-0.7, block=False)
            robot.right_arm.set_wrist_pitch(-0.5, block=False)
            robot.right_arm.set_wrist_roll(0.3, block=False)
            robot.right_arm.set_shoulder_roll(0.7, block=False)
            time.sleep(1)

            robot.left_arm.set_shoulder_pitch(-0.6, block=False)
            robot.left_arm.set_elbow_pitch(-0.6, block=True)

            robot.left_arm.set_shoulder_roll(0.65, block=False)
            robot.left_arm.set_wrist_roll(1.7, block=False)
            robot.left_arm.set_elbow_pitch(-0.5, block=True)

            robot.left_arm.set_elbow_pitch(-0.6, block=False)
            robot.left_arm.set_wrist_roll(0.0)

            robot.left_arm.set_shoulder_roll(0.0)

            robot.right_arm.home()
            robot.right_arm.set_shoulder_pitch(0.7, block=False)
            robot.right_arm.set_shoulder_roll(-0.42, block=False)
            robot.right_arm.set_wrist_roll(0.1, block=True)

            robot.right_arm.set_elbow_pitch(1.0, block=False)
            robot.right_arm.set_wrist_pitch(0.1)
            time.sleep(1)
            robot.right_gripper.open()
            time.sleep(1)

            robot.right_arm.home()
            time.sleep(1)

            robot.left_arm.home()
            robot.left_arm.set_shoulder_yaw(0.3, block=False)
            robot.left_arm.set_wrist_roll(-0.4, block=False)
            robot.left_arm.set_shoulder_roll(-0.76, block=True)

            robot.left_arm.set_shoulder_pitch(0.8, block=False)
            robot.left_arm.set_elbow_pitch(0.8, block=False)
            robot.left_arm.set_wrist_roll(0.1, block=True)

            robot.left_arm.set_elbow_pitch(1.35, block=False)
            robot.left_arm.set_shoulder_pitch(0.85, block=True)

            time.sleep(1)
            robot.left_gripper.open()
            robot.left_arm.set_elbow_pitch(0.9, block=True)

            robot.left_arm.set_shoulder_pitch(0.2, block=False)
            robot.left_arm.set_elbow_pitch(0.4, block=True)

            robot.left_arm.home(block=False)
            time.sleep(1)
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    main()
