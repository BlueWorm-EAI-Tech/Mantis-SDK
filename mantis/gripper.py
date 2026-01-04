"""
夹爪控制模块
============

提供 Mantis 机器人夹爪的控制接口。夹爪位置使用 0.0-1.0 归一化表示。

支持阻塞/非阻塞模式，允许夹爪与其他部件并行运动。

Example:
    .. code-block:: python
    
        from mantis import Mantis
        
        with Mantis(sim=True) as robot:
            # 阻塞模式（默认）
            robot.left_gripper.open()
            
            # 非阻塞模式（双手并行）
            robot.left_gripper.open(block=False)
            robot.right_gripper.close(block=False)
"""

import time
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .mantis import Mantis


# 预设位置: (方法名, 位置值, 说明)
_PRESETS = [
    ("open",      1.0, "完全张开"),
    ("close",     0.0, "完全闭合"),
    ("half_open", 0.5, "半开"),
]


def _make_preset(pos: float, doc: str):
    """工厂函数：生成预设位置方法。"""
    def method(self, block: bool = True):
        """执行预设动作。
        
        Args:
            block: 是否阻塞等待完成，默认 True
        """
        self.set_position(pos, block=block)
    method.__doc__ = f"""{doc}。
    
    Args:
        block: 是否阻塞等待完成，默认 True
    """
    return method


class Gripper:
    """夹爪控制类。
    
    夹爪位置使用归一化值表示：
    
    - ``0.0``: 完全闭合
    - ``0.5``: 半开
    - ``1.0``: 完全张开
    
    支持阻塞/非阻塞模式：
        - block=True（默认）：等待运动完成后返回
        - block=False：立即返回，运动在后台执行
    
    Attributes:
        side: 夹爪侧别 ('left' 或 'right')
        position: 当前位置 (0.0-1.0)
        is_moving: 是否正在运动中
    
    Example:
        .. code-block:: python
        
            # 阻塞模式
            robot.left_gripper.open()
            
            # 非阻塞模式（双手同时）
            robot.left_gripper.open(block=False)
            robot.right_gripper.open(block=False)
    """
    
    #: 默认夹爪速度 (单位/s，0-1 范围)
    DEFAULT_SPEED = 2.0
    
    def __init__(self, robot: "Mantis", side: str):
        """初始化夹爪控制器。"""
        if side not in ("left", "right"):
            raise ValueError("side 必须是 'left' 或 'right'")
        self._robot = robot
        self._side = side
        self._position = 0.0
        self._is_moving = False
        self._speed = self.DEFAULT_SPEED
    
    @property
    def side(self) -> str:
        """夹爪侧别。"""
        return self._side
    
    @property
    def position(self) -> float:
        """当前夹爪位置 (0.0-1.0)。"""
        return self._position
    
    @property
    def is_moving(self) -> bool:
        """是否正在运动中。"""
        return self._is_moving
    
    def set_speed(self, speed: float):
        """设置夹爪速度。
        
        Args:
            speed: 速度 (单位/s)，范围 0.5-5.0
        """
        self._speed = max(0.5, min(5.0, abs(speed)))
    
    def _execute_motion(self, duration: float, block: bool):
        """执行运动。"""
        if duration < 0.01:
            return
        
        self._is_moving = True
        
        if block:
            time.sleep(duration)
            self._is_moving = False
        else:
            def _delayed_stop():
                time.sleep(duration)
                self._is_moving = False
            threading.Thread(target=_delayed_stop, daemon=True).start()
    
    def wait(self):
        """等待当前运动完成。"""
        while self._is_moving:
            time.sleep(0.01)
    
    def set_position(self, position: float, block: bool = True):
        """设置夹爪位置。
        
        Args:
            position: 目标位置 (0.0-1.0)
            block: 是否阻塞等待完成，默认 True
        """
        old_position = self._position
        self._position = max(0.0, min(1.0, position))
        
        duration = abs(self._position - old_position) / self._speed
        
        self._robot._publish_grippers()
        self._execute_motion(duration, block)
    
    def __repr__(self) -> str:
        """返回夹爪的字符串表示。"""
        status = "运动中" if self._is_moving else "停止"
        return f"Gripper('{self._side}', {status}, pos={self._position:.2f})"


# 动态生成预设方法
for name, pos, doc in _PRESETS:
    setattr(Gripper, name, _make_preset(pos, doc))
