"""
RViz 预览测试
在 RViz 中预览机器人动作（带平滑）

使用前请确保:
1. 启动仿真环境: ros2 launch bw_sim2real sdk_sim.launch.py
   （该 launch 文件会启动 sdk_bridge 节点）
"""

import argparse
import time

from mantis import Mantis


DEFAULT_ROBOT_IP = "192.168.1.151"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mantis RViz 预览测试")
    parser.add_argument(
        "--ip",
        default=DEFAULT_ROBOT_IP,
        help="目标机器人 IP，默认 192.168.1.151",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    print("=== Mantis 连接测试 ===\n")
    print(f"当前按 IP 连接: {args.ip}")

    robot = None
    try:
        robot = Mantis()
        ok = robot.connect(ip=args.ip)
        if not ok:
            raise SystemExit("连接失败，停止测试")
        while 1:
            print("1. 左臂抬起")
            robot.left_arm.set_shoulder_pitch(-0.5)
            time.sleep(2)  # 等待动作完成

            print("2. 回到初始位置")
            robot.left_arm.home()
            robot.right_arm.home()
            time.sleep(2)  # 等待动作完成
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass
   


if __name__ == "__main__":
    main()
