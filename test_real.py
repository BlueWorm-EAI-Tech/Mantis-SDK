"""
实机连接测试
测试与真实机器人的通信（通过 Python 桥接节点）

使用前请确保:
1. 机器人端启动 Python 桥接节点: ros2 run bw_sdk_bridge sdk_bridge
2. 机器人端 ROS2 节点已启动并订阅相应话题
"""

from mantis import Mantis
import time

def main():
    print("=== Mantis 全关节方向测试 ===\n")
    
    robot = Mantis(ip="192.168.1.111")
    robot.connect(verify=False)
    
    
    # 小角度测试，便于观察方向
    angle = 0.3  # 约 17 度
    
    try:
        # ==================== 左臂测试 ====================
        # print("\n--- 左臂测试 ---")
        
        # print("左臂 shoulder_pitch (肩部俯仰)...")
        # robot.left_arm.set_shoulder_pitch(-angle)
        # time.sleep(1.5)
        # robot.left_arm.home()
        # time.sleep(1)
        
        # print("左臂 shoulder_yaw (肩部偏航)...")
        # robot.left_arm.set_shoulder_yaw(angle)
        # time.sleep(1.5)
        # robot.left_arm.home()
        # time.sleep(1)
        
        # print("左臂 shoulder_roll (肩部翻滚)...")
        # robot.left_arm.set_shoulder_roll(angle)
        # time.sleep(1.5)
        # robot.left_arm.home()
        # time.sleep(1)
        
        # print("左臂 elbow_pitch (肘部俯仰)...")
        # robot.left_arm.set_elbow_pitch(angle)
        # time.sleep(1.5)
        # robot.left_arm.home()
        # time.sleep(1)
        
        # print("左臂 wrist_roll (腕部翻滚)...")
        # robot.left_arm.set_wrist_roll(angle)
        # time.sleep(1.5)
        # robot.left_arm.home()
        # time.sleep(1)
        
        # print("左臂 wrist_pitch (腕部俯仰)...")
        # robot.left_arm.set_wrist_pitch(angle)
        # time.sleep(1.5)
        # robot.left_arm.home()
        # time.sleep(1)
        
        # print("左臂 wrist_yaw (腕部偏航)...")
        # robot.left_arm.set_wrist_yaw(angle)
        # time.sleep(1.5)
        # robot.left_arm.home()
        # time.sleep(1)
        
        # # ==================== 右臂测试 ====================
        # print("\n--- 右臂测试 ---")
        
        # print("右臂 shoulder_pitch (肩部俯仰)...")
        # robot.right_arm.set_shoulder_pitch(-angle)
        # time.sleep(1.5)
        # robot.right_arm.home()
        # time.sleep(1)
        
        # print("右臂 shoulder_yaw (肩部偏航)...")
        # robot.right_arm.set_shoulder_yaw(angle)
        # time.sleep(1.5)
        # robot.right_arm.home()
        # time.sleep(1)
        
        # print("右臂 shoulder_roll (肩部翻滚)...")
        # robot.right_arm.set_shoulder_roll(angle)
        # time.sleep(1.5)
        # robot.right_arm.home()
        # time.sleep(1)
        
        # print("右臂 elbow_pitch (肘部俯仰)...")
        # robot.right_arm.set_elbow_pitch(angle)
        # time.sleep(1.5)
        # robot.right_arm.home()
        # time.sleep(1)
        
        # print("右臂 wrist_roll (腕部翻滚)...")
        # robot.right_arm.set_wrist_roll(angle)
        # time.sleep(1.5)
        # robot.right_arm.home()
        # time.sleep(1)
        
        # print("右臂 wrist_pitch (腕部俯仰)...")
        # robot.right_arm.set_wrist_pitch(angle)
        # time.sleep(1.5)
        # robot.right_arm.home()
        # time.sleep(1)
        
        # print("右臂 wrist_yaw (腕部偏航)...")
        # robot.right_arm.set_wrist_yaw(angle)
        # time.sleep(1.5)
        # robot.right_arm.home()
        # time.sleep(1)
        
        # # ==================== 头部测试 ====================
        # print("\n--- 头部测试 ---")
        
        # print("头部 pitch (俯仰)...")
        # robot.head.set_pitch(0.15)
        # time.sleep(1.5)
        # robot.head.center()
        # time.sleep(1)
        
        # print("头部 yaw (偏航)...")
        # robot.head.set_yaw(angle)
        # time.sleep(1.5)
        # robot.head.center()
        # time.sleep(1)
        
        # ==================== 夹爪测试 ====================
        print("\n--- 夹爪测试 ---")
        
        print("左夹爪打开...")
        robot.left_gripper.open()
        time.sleep(1.5)
        robot.left_gripper.close()
        time.sleep(1)
        
        print("右夹爪打开...")
        robot.right_gripper.open()
        time.sleep(1.5)
        robot.right_gripper.close()
        time.sleep(1)
        
        print("\n✅ 全关节测试完成！")
        
    except KeyboardInterrupt:
        print("\n⚠️ 测试中断")
    finally:
        robot.home()
        time.sleep(1)
        robot.disconnect()

if __name__ == "__main__":
    main()
