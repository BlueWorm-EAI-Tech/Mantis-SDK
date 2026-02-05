"""
实机连接测试
测试与真实机器人的通信（通过 Python 桥接节点）

使用前请确保:
1. 机器人端启动 Python 桥接节点: ros2 run bw_sdk_bridge sdk_bridge
2. 机器人端 ROS2 节点已启动并订阅相应话题
"""
# 两点半开始
import time
    # robot.right_arm.set_shoulder_pitch(0.7, block=False) 大臂前后
    # robot.right_arm.set_shoulder_yaw(0.6, block=False) 大臂内外
    # robot.right_arm.set_shoulder_roll(-0.4) 大臂左右
    # robot.right_arm.set_wrist_roll(0.5) 小臂中轴旋转
    # robot.right_arm.set_wrist_yaw(0.5) 手腕左右
    # robot.right_arm.set_wrist_pitch(0.5) 手腕上下
    # robot.right_arm.set_elbow_pitch(-0.5) 手肘上下
    
import sys
import threading
from mantis import Mantis
import time
import sys
import threading
from mantis import Mantis
import time
print("=== Mantis 全关节方向测试 ===\n")

robot = Mantis(ip="192.168.1.111")
if not robot.connect(verify=True):
    sys.exit(1) 
robot.left_arm.ik(0.0, 0.5, 0.0, 0, 0, 0, abs=True)

# robot.disconnect()




