"""
头部控制模块
============

提供 Mantis 机器人头部的控制接口。头部有 2 个自由度：俯仰和偏航。

支持阻塞/非阻塞模式，允许头部与其他部件并行运动。

Example:
    .. code-block:: python
    
        from mantis import Mantis
        
        with Mantis(sim=True) as robot:
            # 阻塞模式（默认）
            robot.head.look_left()
            
            # 非阻塞模式（与手臂并行）
            robot.head.look_left(block=False)
            robot.left_arm.set_shoulder_pitch(-0.5, block=False)
"""

import time
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .mantis import Mantis

from .constants import HEAD_LIMITS


# 动作定义: (方法名, 参数名, 符号, 默认值, 说明)
_LOOK_ACTIONS = [
    ("look_left",  "yaw",   1,  0.5, "向左看"),
    ("look_right", "yaw",  -1,  0.5, "向右看"),
    ("look_up",    "pitch", -1, 0.3, "向上看"),
    ("look_down",  "pitch", 1,  0.3, "向下看"),
]


def _make_look_action(attr: str, sign: int, default: float, doc: str):
    """工厂函数：生成 look_xxx 方法。"""
    def action(self, angle: float = default, block: bool = True):
        """执行头部动作。
        
        Args:
            angle: 角度大小（弧度），默认使用预设值
            block: 是否阻塞等待完成，默认 True
        """
        old_value = getattr(self, f"_{attr}")
        new_value = sign * abs(angle)
        setattr(self, f"_{attr}", new_value)
        self._apply_limits()
        
        self._robot._publish_head()
        self._execute_motion(block)
        
    action.__doc__ = f"""{doc}。
    
    Args:
        angle: 角度大小（弧度），默认 {default}
        block: 是否阻塞等待完成，默认 True
    """
    return action


class Head:
    """头部控制类。
    
    头部有 2 个自由度：
    
    ========  ==============  ==================
    轴        中文名          范围 (rad)
    ========  ==============  ==================
    pitch     俯仰            -0.7 ~ 0.2
    yaw       偏航            -1.57 ~ 1.57
    ========  ==============  ==================
    
    支持阻塞/非阻塞模式：
        - block=True（默认）：等待运动完成后返回
        - block=False：立即返回，运动在后台执行
    
    Attributes:
        pitch: 当前俯仰角（弧度）
        yaw: 当前偏航角（弧度）
        is_moving: 是否正在运动中
    
    Example:
        .. code-block:: python
        
            # 阻塞模式
            robot.head.look_left(0.5)
            
            # 非阻塞模式
            robot.head.look_left(0.5, block=False)
            robot.head.wait()  # 等待完成
    """
    
    #: 默认头部速度 (rad/s)
    DEFAULT_SPEED = 1.0
    
    def __init__(self, robot: "Mantis"):
        """初始化头部控制器。"""
        self._robot = robot
        self._pitch = 0.0
        self._yaw = 0.0
        self._limits = HEAD_LIMITS
        self._speed = self.DEFAULT_SPEED
    
    @property
    def pitch(self) -> float:
        """当前俯仰角（弧度）。"""
        return self._pitch
    
    @property
    def yaw(self) -> float:
        """当前偏航角（弧度）。"""
        return self._yaw

    @property
    def limits(self) -> dict:
        """限位字典。"""
        return self._limits.copy()
    
    def set_speed(self, speed: float):
        """设置头部运动速度。
        
        Args:
            speed: 速度 (rad/s)，范围 0.1-3.0
        """
        self._speed = max(0.1, min(3.0, abs(speed)))
    
    def _clamp(self, attr: str, value: float) -> float:
        """限制值在限位范围内。"""
        lower, upper = self._limits[attr]
        return max(lower, min(upper, value))
    
    def _apply_limits(self):
        """应用限位。"""
        self._pitch = self._clamp("pitch", self._pitch)
        self._yaw = self._clamp("yaw", self._yaw)
    
    def _execute_motion(self, block: bool):
        """执行运动。"""
        if block:
            self.wait()
    
    def wait(self):
        """等待当前运动完成。"""
        self._robot.wait(['head'])
    
    @property
    def is_moving(self) -> bool:
        """是否正在运动中。"""
        return self._robot.is_moving(['head'])
    
    def set_pose(self, pitch: float = None, yaw: float = None, clamp: bool = True, block: bool = True):
        """设置头部姿态。
        
        Args:
            pitch: 俯仰角（弧度），范围 -0.7 ~ 0.2
            yaw: 偏航角（弧度），范围 -1.57 ~ 1.57
            clamp: 是否自动限制在限位范围内，默认 True
            block: 是否阻塞等待完成，默认 True
        """
        
        if pitch is not None:
            self._pitch = self._clamp("pitch", pitch) if clamp else pitch
        if yaw is not None:
            self._yaw = self._clamp("yaw", yaw) if clamp else yaw
        
        self._robot._publish_head()
        self._execute_motion(block)
    
    def set_pitch(self, angle: float, clamp: bool = True, block: bool = True):
        """设置俯仰角。
        
        Args:
            angle: 目标角度（弧度）
            clamp: 是否限位
            block: 是否阻塞
        """
        self.set_pose(pitch=angle, clamp=clamp, block=block)
    
    def set_yaw(self, angle: float, clamp: bool = True, block: bool = True):
        """设置偏航角。
        
        Args:
            angle: 目标角度（弧度）
            clamp: 是否限位
            block: 是否阻塞
        """
        self.set_pose(yaw=angle, clamp=clamp, block=block)
    
    def center(self, block: bool = True):
        """回中（俯仰和偏航都归零）。
        
        Args:
            block: 是否阻塞等待完成，默认 True
        """
        self.set_pose(0.0, 0.0, block=block)
    
    def __repr__(self) -> str:
        """返回头部的字符串表示。"""
        status = "运动中" if self._is_moving else "停止"
        return f"Head({status}, pitch={self._pitch:.2f}, yaw={self._yaw:.2f})"


# 动态生成 look_xxx 方法
for name, attr, sign, default, doc in _LOOK_ACTIONS:
    setattr(Head, name, _make_look_action(attr, sign, default, doc))
