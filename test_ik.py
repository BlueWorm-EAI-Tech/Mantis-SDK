"""
实机连接测试
测试与真实机器人的通信（通过 Python 桥接节点）

使用前请确保:
1. 机器人端启动 Python 桥接节点: ros2 run bw_sdk_bridge sdk_bridge
2. 机器人端 ROS2 节点已启动并订阅相应话题
"""
import argparse

from mantis import Mantis


DEFAULT_ROBOT_IP = "192.168.1.151"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mantis IK 测试")
    parser.add_argument(
        "--ip",
        default=DEFAULT_ROBOT_IP,
        help="目标机器人 IP，默认 192.168.1.151",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print("=== Mantis IK 测试 ===\n")
    print(f"当前按 IP 连接: {args.ip}")

    robot = None
    try:
        robot = Mantis()
        ok = robot.connect(ip=args.ip)
        if not ok:
            raise SystemExit("连接失败，停止测试")
        robot.left_arm.ik(0.0, 0.5, 0.0, 0, 0, 0, abs=True)
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    main()


