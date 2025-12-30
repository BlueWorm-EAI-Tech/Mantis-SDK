#!/usr/bin/env python3
"""
Mantis 机器人控制示例

使用前:
    1. pip install eclipse-zenoh
    2. 机器人启动: ~/zenoh_ros2/zenoh-bridge-ros2dds -d 0
"""

import time
import math
from mantis_sdk import Mantis


def example_arm_control():
    """手臂控制示例"""
    print("\n" + "="*50)
    print("示例: 手臂控制")
    print("="*50)
    
    with Mantis() as robot:
        # 左臂抬起
        print(">>> 左臂肩膀抬起 45°...")
        robot.left_arm.set_shoulder_pitch(math.radians(45))
        time.sleep(1.5)
        
        # 右臂抬起
        print(">>> 右臂肩膀抬起 45°...")
        robot.right_arm.set_shoulder_pitch(math.radians(45))
        time.sleep(1.5)
        
        # 设置左臂全部关节
        print(">>> 左臂设置多个关节...")
        robot.left_arm.set_joints([
            math.radians(60),   # shoulder_pitch
            math.radians(-15),  # shoulder_yaw
            math.radians(0),    # shoulder_roll
            math.radians(45),   # elbow_pitch
            math.radians(0),    # wrist_roll
            math.radians(0),    # wrist_pitch
            math.radians(0),    # wrist_yaw
        ])
        time.sleep(1.5)
        
        # 回零位
        print(">>> 回零位...")
        robot.left_arm.home()
        robot.right_arm.home()
        time.sleep(1.0)


def example_gripper_control():
    """夹爪控制示例"""
    print("\n" + "="*50)
    print("示例: 夹爪控制")
    print("="*50)
    
    with Mantis() as robot:
        print(">>> 双手张开...")
        robot.left_gripper.open()
        robot.right_gripper.open()
        time.sleep(1.0)
        
        print(">>> 双手半开...")
        robot.left_gripper.half_open()
        robot.right_gripper.half_open()
        time.sleep(1.0)
        
        print(">>> 双手闭合...")
        robot.left_gripper.close()
        robot.right_gripper.close()
        time.sleep(1.0)
        
        print(">>> 左开右闭...")
        robot.left_gripper.open()
        robot.right_gripper.close()
        time.sleep(1.0)


def example_head_control():
    """头部控制示例"""
    print("\n" + "="*50)
    print("示例: 头部控制")
    print("="*50)
    
    with Mantis() as robot:
        print(">>> 向左看...")
        robot.head.look_left()
        time.sleep(1.0)
        
        print(">>> 向右看...")
        robot.head.look_right()
        time.sleep(1.0)
        
        print(">>> 向下看...")
        robot.head.look_down()
        time.sleep(1.0)
        
        print(">>> 向上看...")
        robot.head.look_up()
        time.sleep(1.0)
        
        print(">>> 回中...")
        robot.head.center()
        time.sleep(0.5)


def example_chassis_control():
    """底盘控制示例"""
    print("\n" + "="*50)
    print("示例: 底盘控制")
    print("="*50)
    
    with Mantis() as robot:
        print(">>> 前进...")
        robot.chassis.forward(0.1)
        time.sleep(2.0)
        
        print(">>> 左转...")
        robot.chassis.turn_left(0.3)
        time.sleep(2.0)
        
        print(">>> 停止...")
        robot.chassis.stop()


def example_wave():
    """挥手动作"""
    print("\n" + "="*50)
    print("示例: 挥手动作")
    print("="*50)
    
    with Mantis() as robot:
        # 准备姿势
        print(">>> 抬起右臂...")
        robot.right_arm.set_joints([
            math.radians(90),   # shoulder_pitch - 抬起
            math.radians(-30),  # shoulder_yaw
            math.radians(0),
            math.radians(45),   # elbow_pitch - 弯曲
            math.radians(0),
            math.radians(0),
            math.radians(0),
        ])
        time.sleep(1.0)
        
        # 挥手
        print(">>> 挥手...")
        for _ in range(4):
            robot.right_arm.set_wrist_yaw(math.radians(30))
            time.sleep(0.25)
            robot.right_arm.set_wrist_yaw(math.radians(-30))
            time.sleep(0.25)
        
        # 放下
        print(">>> 放下手臂...")
        robot.right_arm.home()
        time.sleep(1.0)


def example_smooth_trajectory():
    """平滑轨迹示例"""
    print("\n" + "="*50)
    print("示例: 平滑轨迹")
    print("="*50)
    
    with Mantis() as robot:
        print(">>> 右臂画圆...")
        
        duration = 4.0
        freq = 50
        steps = int(duration * freq)
        
        for i in range(steps):
            t = i / steps * 2 * math.pi
            
            robot.right_arm.set_joints([
                math.radians(60 + 20 * math.sin(t)),    # shoulder_pitch
                math.radians(15 * math.cos(t)),         # shoulder_yaw
                0,
                math.radians(30 + 20 * math.sin(t)),    # elbow
                0, 0, 0
            ])
            time.sleep(1.0 / freq)
        
        print(">>> 归零...")
        robot.right_arm.home()


def main():
    print("\n" + "#"*50)
    print("#" + " "*15 + "Mantis SDK 示例" + " "*16 + "#")
    print("#"*50)
    
    print("\n选择示例:")
    print("  1. 手臂控制")
    print("  2. 夹爪控制")
    print("  3. 头部控制")
    print("  4. 底盘控制")
    print("  5. 挥手动作")
    print("  6. 平滑轨迹")
    print("  0. 全部运行")
    print("  q. 退出")
    
    while True:
        choice = input("\n请选择 (0-6, q): ").strip()
        
        if choice == 'q':
            break
        elif choice == '0':
            example_arm_control()
            example_gripper_control()
            example_head_control()
            example_wave()
            example_smooth_trajectory()
        elif choice == '1':
            example_arm_control()
        elif choice == '2':
            example_gripper_control()
        elif choice == '3':
            example_head_control()
        elif choice == '4':
            example_chassis_control()
        elif choice == '5':
            example_wave()
        elif choice == '6':
            example_smooth_trajectory()
        else:
            print("无效选择")
    
    print("\n再见！")


if __name__ == "__main__":
    main()
