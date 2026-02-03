"""
实机连接测试
测试与真实机器人的通信（通过 Python 桥接节点）

使用前请确保:
1. 机器人端启动 Python 桥接节点: ros2 run bw_sdk_bridge sdk_bridge
2. 机器人端 ROS2 节点已启动并订阅相应话题
"""
# 两点半开始

from mantis import Mantis
import time

def main():
    print("=== Mantis 全关节方向测试 ===\n")
    
    robot = Mantis(ip="192.168.1.111")
    robot.connect(verify=True)
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

if __name__ == "__main__":
    main()
