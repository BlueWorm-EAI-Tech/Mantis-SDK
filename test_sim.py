"""
纯仿真预览测试
在 RViz 中预览机器人动作（带平滑）

使用前请确保:
1. 启动纯仿真环境: ros2 launch bw_sim2real sdk_sim.launch.py
2. 启动 zenoh 桥接: ~/zenoh_ros2/zenoh-bridge-ros2dds -d 99
"""

from mantis import Mantis
import time
import math

def main():
    print("=== Mantis 仿真预览测试（带平滑）===\n")
    
    with Mantis(sim=True) as robot:
        # 可选：调整平滑参数
        # robot.set_smoothing(alpha=0.1, rate=100)  # 默认值：平滑
        # robot.set_smoothing(alpha=0.3, rate=100)  # 更快响应
        # robot.set_smoothing(alpha=0.05, rate=100)  # 更平滑
        
        print("开始演示动作（关节运动会平滑过渡）...\n")
        
        # 1. 双臂抬起
        print("1. 双臂抬起")
        robot.left_arm.set_shoulder_pitch(-0.5)
        robot.right_arm.set_shoulder_pitch(-0.5)
        time.sleep(1)
        
        # 2. 手臂展开
        print("2. 手臂展开")
        robot.left_arm.set_shoulder_roll(0.5)
        robot.right_arm.set_shoulder_roll(-0.5)
        time.sleep(1)
        
        # 3. 弯曲手肘
        print("3. 弯曲手肘")
        robot.left_arm.set_elbow_pitch(1.0)
        robot.right_arm.set_elbow_pitch(1.0)
        time.sleep(1)
        
        # 4. 头部环顾
        print("4. 头部环顾")
        robot.head.look_left(0.5)
        time.sleep(0.5)
        robot.head.look_right(0.5)
        time.sleep(0.5)
        robot.head.center()
        time.sleep(0.5)
        
        # 5. 挥手动作
        print("5. 挥手动作")
        for _ in range(3):
            robot.left_arm.set_wrist_yaw(0.5)
            time.sleep(0.2)
            robot.left_arm.set_wrist_yaw(-0.5)
            time.sleep(0.2)
        robot.left_arm.set_wrist_yaw(0.0)
        
        # 6. 回到初始位置
        print("6. 回到初始位置")
        robot.left_arm.home()
        robot.right_arm.home()
        time.sleep(1)
        
        print("\n演示完成！")


if __name__ == "__main__":
    main()
