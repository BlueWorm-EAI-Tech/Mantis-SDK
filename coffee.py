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
    
# 右手拿杯子
robot.right_gripper.open()
robot.right_arm.set_shoulder_pitch(0.7, block=False)
robot.right_arm.set_shoulder_roll(-0.42, block=False)
robot.right_arm.set_wrist_roll(0.1, block=True)

robot.right_arm.set_elbow_pitch(1.0, block=False)
robot.right_arm.set_wrist_pitch(0.1)
time.sleep(2)
robot.right_gripper.set_position(0.6) # 默认 block=True
time.sleep(2)

# 右手接咖啡
robot.right_arm.set_elbow_pitch(0.6, block=False)
robot.right_arm.set_shoulder_pitch(0.6, block=True)

robot.right_arm.set_shoulder_roll(0.3)

robot.right_arm.set_shoulder_pitch(0.7, block=False)
robot.right_arm.set_shoulder_roll(0.65, block=False)
robot.right_arm.set_wrist_roll(-0.3)


robot.right_arm.set_shoulder_pitch(0.98, block=False)
robot.right_arm.set_elbow_pitch(0.98, block=False)
robot.right_arm.set_wrist_roll(-0.68, block=False)
robot.right_arm.set_wrist_pitch(0)
time.sleep(2)


# 左手点击屏幕选择咖啡
# 抬手
robot.left_arm.set_shoulder_pitch(0.5, block=False)
robot.left_arm.set_elbow_pitch(-0.1, block=True)

# 点击
robot.left_arm.set_elbow_pitch(-0.03, block=True)
robot.left_arm.set_elbow_pitch(-0.1, block=True)
robot.left_arm.home()
time.sleep(2)

# 接完咖啡右手后撤
robot.right_arm.home()
robot.right_arm.set_shoulder_roll(0.6, block=False)
robot.right_arm.set_wrist_pitch(-0.3)
time.sleep(2)


# # 左手拿牛奶
robot.left_gripper.open()

robot.left_arm.home()
robot.left_arm.set_shoulder_yaw(0.3, block=False)
robot.left_arm.set_wrist_roll(-0.4, block=False)
robot.left_arm.set_shoulder_roll(-0.76, block=True)

robot.left_arm.set_shoulder_pitch(0.8, block=False)
robot.left_arm.set_elbow_pitch(0.8, block=False)
robot.left_arm.set_wrist_roll(0.1, block=True)

robot.left_arm.set_elbow_pitch(1.35, block=False)
robot.left_arm.set_shoulder_pitch(0.85, block=True)

time.sleep(2)
robot.left_gripper.set_position(0.6) # 默认 block=True
time.sleep(2)

# # 拿起牛奶
robot.left_arm.set_elbow_pitch(0.9, block=True)

robot.left_arm.set_shoulder_pitch(0.2, block=False)
robot.left_arm.set_elbow_pitch(0.4, block=True)

robot.left_arm.home(block=False)
robot.left_arm.set_wrist_pitch(-0.45, block=True)
time.sleep(2)


# 右手来接咖啡
robot.right_arm.set_wrist_yaw(-0.7, block=False)
robot.right_arm.set_wrist_pitch(-0.5, block=False) 
robot.right_arm.set_wrist_roll(0.3, block=False)
robot.right_arm.set_shoulder_roll(0.7, block=False)
time.sleep(2)

# 左手倒牛奶
robot.left_arm.set_shoulder_pitch(-0.6, block=False)
robot.left_arm.set_elbow_pitch(-0.6, block=True)

robot.left_arm.set_shoulder_roll(0.65, block=False)
robot.left_arm.set_wrist_roll(1.7, block=False)
robot.left_arm.set_elbow_pitch(-0.5, block=True)

time.sleep(1.0)
robot.left_arm.set_elbow_pitch(-0.6, block=False)
robot.left_arm.set_wrist_roll(0.0)

robot.left_arm.set_shoulder_roll(0.0)
time.sleep(2)

# 接完咖啡复位
robot.right_arm.home()
robot.right_arm.set_shoulder_pitch(0.7, block=False)
robot.right_arm.set_shoulder_roll(-0.42, block=False)
robot.right_arm.set_wrist_roll(0.1, block=True)

robot.right_arm.set_elbow_pitch(1.0, block=False)
robot.right_arm.set_wrist_pitch(0.1)
time.sleep(2)
robot.right_gripper.open()
time.sleep(2)

robot.right_arm.home()
time.sleep(2)

# # 左手复位
robot.left_arm.home()
robot.left_arm.set_shoulder_yaw(0.3, block=False)
robot.left_arm.set_wrist_roll(-0.4, block=False)
robot.left_arm.set_shoulder_roll(-0.76, block=True)

robot.left_arm.set_shoulder_pitch(0.8, block=False)
robot.left_arm.set_elbow_pitch(0.8, block=False)
robot.left_arm.set_wrist_roll(0.1, block=True)

robot.left_arm.set_elbow_pitch(1.35, block=False)
robot.left_arm.set_shoulder_pitch(0.85, block=True)

time.sleep(2)
robot.left_gripper.open()
robot.left_arm.set_elbow_pitch(0.9, block=True)

robot.left_arm.set_shoulder_pitch(0.2, block=False)
robot.left_arm.set_elbow_pitch(0.4, block=True)

robot.left_arm.home(block=False)
time.sleep(2)


robot.disconnect()




