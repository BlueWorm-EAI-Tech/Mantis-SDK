"""
腰部控制模块
============

提供 Mantis 机器人腰部的控制接口。腰部是 prismatic（直线移动）关节。

支持阻塞/非阻塞模式，允许腰部与其他部件并行运动。

Example:
    .. code-block:: python
    
        from mantis import Mantis
        
        with Mantis(sim=True) as robot:
            # 阻塞模式（默认）
            robot.waist.set_height(0.1)
            
            # 非阻塞模式（与手臂并行）
            robot.waist.up(block=False)
            robot.left_arm.set_shoulder_pitch(-0.5, block=False)
"""

import time
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .mantis import Mantis


#: 腰部限位 (lower, upper)，单位：米
WAIST_LIMITS = (-0.62, 0.24)


class Waist:
    """腰部控制类。
    
    腰部是 prismatic（直线移动）关节，控制机器人上半身的高度。
    
    位置范围：-0.62m ~ 0.24m
    
    - 负值：下降
    - 正值：上升
    - 0.0：默认高度
    
    支持阻塞/非阻塞模式：
        - block=True（默认）：等待运动完成后返回
        - block=False：立即返回，运动在后台执行
    
    Attributes:
        height: 当前高度（米）
        is_moving: 是否正在运动中
    
    Example:
        .. code-block:: python
        
            # 阻塞模式
            robot.waist.set_height(0.1)
            
            # 非阻塞模式
            robot.waist.up(block=False)
            robot.waist.wait()
    """
    
    #: 默认移动速度 (m/s)
    DEFAULT_SPEED = 0.1
    
    def __init__(self, robot: "Mantis"):
        """初始化腰部控制器。"""
        self._robot = robot
        self._height = 0.0
        self._limits = WAIST_LIMITS
        self._speed = self.DEFAULT_SPEED
    
    @property
    def height(self) -> float:
        """当前腰部高度（米）。"""
        return self._height
    
    @property
    def limits(self) -> tuple:
        """限位元组 (lower, upper)。"""
        return self._limits
    
    def set_speed(self, speed: float):
        """设置移动速度。
        
        Args:
            speed: 速度 (m/s)，范围 0.01-0.5
        """
        self._speed = max(0.01, min(0.5, abs(speed)))
    
    def _clamp(self, value: float) -> float:
        """限制值在限位范围内。"""
        lower, upper = self._limits
        return max(lower, min(upper, value))
    
    def _execute_motion(self, block: bool):
        """执行运动。"""
        if block:
            self.wait()
    
    def wait(self):
        """等待当前运动完成。"""
        self._robot.wait(['waist'])
    
    @property
    def is_moving(self) -> bool:
        """是否正在运动中。"""
        return self._robot.is_moving(['waist'])
    
    def set_height(self, height: float, clamp: bool = True, block: bool = True):
        """设置腰部高度。
        
        Args:
            height: 目标高度（米），范围 -0.62m ~ 0.24m
            clamp: 是否自动限制在限位范围内，默认 True
            block: 是否阻塞等待完成，默认 True
        """
        old_height = self._height
        self._height = self._clamp(height) if clamp else height
        
        self._robot._publish_waist()
        self._execute_motion(block)
    
    def up(self, delta: float = 0.05, block: bool = True):
        """安全上升（相对移动）。
        
        Args:
            delta: 上升距离（米），默认 0.05m (5cm)
            block: 是否阻塞等待完成，默认 True
        """
        self.move(abs(delta), block=block)
    
    def down(self, delta: float = 0.05, block: bool = True):
        """安全下降（相对移动）。
        
        Args:
            delta: 下降距离（米），默认 0.05m (5cm)
            block: 是否阻塞等待完成，默认 True
        """
        self.move(-abs(delta), block=block)
    
    def home(self, block: bool = True):
        """回到零位（默认高度）。
        
        Args:
            block: 是否阻塞等待完成，默认 True
        """
        self.set_height(0.0, block=block)
    
    def move(self, delta: float, block: bool = True):
        """相对移动。
        
        Args:
            delta: 相对位移（米），正值上升，负值下降
            block: 是否阻塞等待完成，默认 True
        """
        self.set_height(self._height + delta, block=block)
    
    def __repr__(self) -> str:
        """返回腰部的字符串表示。"""
        status = "运动中" if self._is_moving else "停止"
        return f"Waist({status}, height={self._height:.3f}m)"
