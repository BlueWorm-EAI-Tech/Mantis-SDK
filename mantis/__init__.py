"""
Mantis Robot SDK - Zenoh 二次开发接口

让客户无需安装 ROS2，通过 Zenoh 协议直接控制 Mantis 机器人。

安装依赖:
    pip install eclipse-zenoh

使用示例:
    from mantis_sdk import Mantis
    
    robot = Mantis()
    robot.connect()
    
    # 控制手臂
    robot.left_arm.set_joints([0.0] * 7)
    robot.right_arm.set_joints([0.0] * 7)
    
    # 控制夹爪
    robot.left_gripper.set_position(0.5)
    robot.right_gripper.set_position(0.5)
    
    # 控制头部
    robot.head.set_pose(pitch=0.0, yaw=0.0)
    
    # 控制底盘
    robot.chassis.set_velocity(vx=0.1, vy=0.0, omega=0.0)
    
    robot.disconnect()
"""

from .mantis import Mantis
from .arm import Arm
from .gripper import Gripper
from .head import Head
from .chassis import Chassis
from .constants import *

__version__ = "1.0.0"
__author__ = "BlueWorm-EAI-Tech"

__all__ = [
    "Mantis",
    "Arm", 
    "Gripper",
    "Head",
    "Chassis",
    "JOINT_NAMES",
    "LEFT_ARM_JOINTS",
    "RIGHT_ARM_JOINTS",
]
