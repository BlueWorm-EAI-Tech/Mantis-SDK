"""
实机连接测试
测试与真实机器人的通信（通过 Python 桥接节点）

使用前请确保:
1. 机器人端启动 Python 桥接节点: ros2 run bw_sdk_bridge sdk_bridge
2. 机器人端 ROS2 节点已启动并订阅相应话题
"""
import argparse
import time

from mantis import Mantis


DEFAULT_ROBOT_IP = "192.168.1.151"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mantis 全关节方向测试")
    parser.add_argument(
        "--ip",
        default=DEFAULT_ROBOT_IP,
        help="目标机器人 IP，默认 192.168.1.151",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    print("=== Mantis 全关节方向测试 ===\n")
    print(f"当前按 IP 连接: {args.ip}")

    robot = None
    try:
        robot = Mantis()
        ok = robot.connect(ip=args.ip)
        if not ok:
            raise SystemExit("连接失败，停止测试")
        robot.home()
    
        # 大角度极限测试
        max_angle = 0.5 # 极限角度（如有安全限制可调整）
        while 1:
            # ==================== 左臂测试 ====================
            print("\n--- 左臂测试 ---")
            for joint, func in [
                ("shoulder_pitch", robot.left_arm.set_shoulder_pitch),
                ("shoulder_yaw", robot.left_arm.set_shoulder_yaw),
                ("shoulder_roll", robot.left_arm.set_shoulder_roll),
                ("elbow_pitch", robot.left_arm.set_elbow_pitch),
            ]:
                print(f"左臂 {joint} +max...")
                func(max_angle)
                time.sleep(1.5)
                print(f"左臂 {joint} -max...")
                func(-max_angle)
                time.sleep(1.5)
                robot.left_arm.home()
                time.sleep(1)

            # ==================== 右臂测试 ====================
            print("\n--- 右臂测试 ---")
            for joint, func in [
                ("shoulder_pitch", robot.right_arm.set_shoulder_pitch),
                ("shoulder_yaw", robot.right_arm.set_shoulder_yaw),
                ("shoulder_roll", robot.right_arm.set_shoulder_roll),
                ("elbow_pitch", robot.right_arm.set_elbow_pitch),
            ]:
                print(f"右臂 {joint} +max...")
                func(max_angle)
                time.sleep(1.5)
                print(f"右臂 {joint} -max...")
                func(-max_angle)
                time.sleep(1.5)
                robot.right_arm.home()
                time.sleep(1)

            print("\n✅ 全关节极限测试完成！")
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass

if __name__ == "__main__":
    main()
