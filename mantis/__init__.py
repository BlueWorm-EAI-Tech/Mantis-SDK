"""
Mantis Robot SDK - Zenoh 二次开发接口
=====================================

让客户无需安装 ROS2，通过 Zenoh 协议直接控制 Mantis 机器人。

Features:
    - 双臂控制：7自由度机械臂，支持单关节/多关节设置
    - 夹爪控制：左右夹爪开合
    - 头部控制：俯仰/偏航两自由度
    - 底盘控制：全向移动底盘
    - 平滑运动：自动平滑插值
    - 关节限位：自动限制在安全范围内

Installation:
    .. code-block:: bash
    
        pip install eclipse-zenoh
        pip install mantis-sdk

Quick Start:
    .. code-block:: python
    
        from mantis import Mantis
        
        # 连接机器人
        with Mantis(ip="192.168.1.100") as robot:
            robot.left_arm.set_shoulder_pitch(-0.5)
            robot.right_arm.set_joints([0.0] * 7)
            robot.head.look_left()
            robot.left_gripper.open()
        
        # 本地调试（同一局域网）
        with Mantis() as robot:
            robot.left_arm.set_shoulder_pitch(-0.5)

Modules:
    - :class:`Mantis`: 主控制类
    - :class:`Arm`: 手臂控制
    - :class:`Gripper`: 夹爪控制
    - :class:`Head`: 头部控制
    - :class:`Chassis`: 底盘控制

Note:
    使用前需启动机器人端的 Python 桥接节点::
    
        ros2 run bw_sdk_bridge sdk_bridge
"""

from .mantis import Mantis
from .arm import Arm
from .gripper import Gripper
from .head import Head
from .waist import Waist
from .chassis import Chassis
from .constants import *

#: SDK 版本号
__version__ = "1.3.4"

#: 作者
__author__ = "BlueWorm-EAI-Tech"

#: 版本发布日期
__release_date__ = "2025-12-30"

__all__ = [
    "Mantis",
    "Arm", 
    "Gripper",
    "Head",
    "Waist",
    "Chassis",
    "JOINT_NAMES",
    "LEFT_ARM_JOINTS",
    "RIGHT_ARM_JOINTS",
]
