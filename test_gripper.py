"""
夹爪控制测试
在 RViz 中预览夹爪动作

使用前请确保:
1. 启动纯仿真环境: ros2 launch bw_sim2real sdk_sim.launch.py
2. 启动 zenoh 桥接: ~/zenoh_ros2/zenoh-bridge-ros2dds -d 99
"""

from mantis import Mantis
import time

def main():
    print("=== Mantis 夹爪控制测试 ===\n")
    
    with Mantis(sim=True) as robot:
        print("开始演示夹爪动作...\n")
        
        # 1. 完全张开
        print("1. 完全张开")
        robot.left_gripper.open()
        robot.right_gripper.open()
        time.sleep(1)
        
        # 2. 完全闭合
        print("2. 完全闭合")
        robot.left_gripper.close()
        robot.right_gripper.close()
        time.sleep(1)
        
        # 3. 半开
        print("3. 半开")
        robot.left_gripper.half_open()
        robot.right_gripper.half_open()
        time.sleep(1)
        
        # 4. 设置具体位置 (0.0 - 1.0)
        print("4. 渐变测试")
        for i in range(11):
            pos = i / 10.0
            print(f"位置: {pos:.1f}")
            robot.left_gripper.set_position(pos)
            robot.right_gripper.set_position(pos)
            time.sleep(0.2)
            
        print("\n演示完成！")

if __name__ == "__main__":
    main()
