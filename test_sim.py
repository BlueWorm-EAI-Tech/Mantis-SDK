"""
RViz 预览测试
在 RViz 中预览机器人动作（带平滑）

使用前请确保:
1. 启动仿真环境: ros2 launch bw_sim2real sdk_sim.launch.py
   （该 launch 文件会启动 sdk_bridge 节点）
"""

from mantis import Mantis
import time
import math

def main():
    print("=== Mantis 连接测试 ===\n")
    robot = Mantis(ip="192.168.1.151")
    # verify=False 跳过验证，因为本地测试没有 joint_states_fdb 话题
    robot.connect(verify=False)
    while 1:
        print("1. 左臂抬起")
        robot.left_arm.set_shoulder_pitch(-0.5)
        time.sleep(2)  # 等待动作完成

        print("2. 回到初始位置")
        robot.left_arm.home()
        robot.right_arm.home()
        time.sleep(2)  # 等待动作完成
    
    print("\n演示完成！")
    robot.disconnect()
   


if __name__ == "__main__":
    main()
