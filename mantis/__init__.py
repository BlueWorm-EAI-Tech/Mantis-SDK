"""
Mantis Robot SDK - Zenoh 二次开发接口
=====================================

让客户无需安装 ROS2，通过 Zenoh 协议直接控制 Mantis 机器人。

Features:
    - 双臂控制：7自由度机械臂，支持单关节/多关节设置
    - 夹爪控制：左右夹爪开合
    - 头部控制：俯仰/偏航两自由度
    - 底盘控制：全向移动底盘
    - 仿真预览：RViz 实时预览（带平滑）
    - 关节限位：自动限制在安全范围内

Installation:
    .. code-block:: bash
    
        pip install eclipse-zenoh
        pip install mantis-sdk

Quick Start:
    .. code-block:: python
    
        from mantis import Mantis
        
        # 实机控制
        with Mantis(ip="192.168.1.100") as robot:
            robot.left_arm.set_shoulder_pitch(-0.5)
            robot.right_arm.set_joints([0.0] * 7)
            robot.head.look_left()
            robot.left_gripper.open()
        
        # 仿真预览
        with Mantis(sim=True) as robot:
            robot.left_arm.set_shoulder_pitch(-0.5)

Modules:
    - :class:`Mantis`: 主控制类
    - :class:`Arm`: 手臂控制
    - :class:`Gripper`: 夹爪控制
    - :class:`Head`: 头部控制
    - :class:`Chassis`: 底盘控制

Note:
    仿真模式需要先启动 ROS2 仿真环境::
    
        ros2 launch bw_sim2real sdk_sim.launch.py
        zenoh-bridge-ros2dds -d 99
"""

from .mantis import Mantis
from .arm import Arm
from .gripper import Gripper
from .head import Head
from .chassis import Chassis
from .constants import *

#: SDK 版本号
__version__ = "1.0.0"

#: 作者
__author__ = "BlueWorm-EAI-Tech"

#: 版本发布日期
__release_date__ = "2025-12-30"

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
