"""
夹爪控制模块
============

提供 Mantis 机器人夹爪的控制接口。夹爪位置使用 0.0-1.0 归一化表示。

Example:
    .. code-block:: python
    
        from mantis import Mantis
        
        with Mantis(sim=True) as robot:
            # 设置具体位置
            robot.left_gripper.set_position(0.5)
            
            # 使用预设方法
            robot.left_gripper.open()
            robot.right_gripper.close()
            robot.left_gripper.half_open()
"""

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
    """工厂函数：生成预设位置方法。
    
    Args:
        pos: 预设位置值 (0.0-1.0)
        doc: 方法说明
        
    Returns:
        Callable: 生成的预设方法
    """
    def method(self):
        """执行预设动作。"""
        self.set_position(pos)
    method.__doc__ = f"""{doc}。
    
    将夹爪设置到 {pos} 位置。
    """
    return method


class Gripper:
    """夹爪控制类。
    
    夹爪位置使用归一化值表示：
    
    - ``0.0``: 完全闭合
    - ``0.5``: 半开
    - ``1.0``: 完全张开
    
    Attributes:
        side: 夹爪侧别 ('left' 或 'right')
        position: 当前位置 (0.0-1.0)
    
    Example:
        .. code-block:: python
        
            # 设置具体位置
            robot.left_gripper.set_position(0.7)
            
            # 使用预设
            robot.left_gripper.open()      # 完全张开
            robot.left_gripper.close()     # 完全闭合
            robot.left_gripper.half_open() # 半开
    """
    
    def __init__(self, robot: "Mantis", side: str):
        """初始化夹爪控制器。
        
        Args:
            robot: Mantis 机器人实例
            side: 夹爪侧别，'left' 或 'right'
            
        Raises:
            ValueError: 如果 side 不是 'left' 或 'right'
        """
        if side not in ("left", "right"):
            raise ValueError("side 必须是 'left' 或 'right'")
        self._robot = robot
        self._side = side
        self._position = 0.0
    
    @property
    def side(self) -> str:
        """夹爪侧别。
        
        Returns:
            str: 'left' 或 'right'
        """
        return self._side
    
    @property
    def position(self) -> float:
        """当前夹爪位置。
        
        Returns:
            float: 位置值 (0.0-1.0)
        """
        return self._position
    
    def set_position(self, position: float):
        """设置夹爪位置。
        
        Args:
            position: 目标位置 (0.0-1.0)，自动限制在有效范围内
            
        Example:
            .. code-block:: python
            
                robot.left_gripper.set_position(0.5)  # 半开
                robot.right_gripper.set_position(1.0) # 完全张开
        """
        self._position = max(0.0, min(1.0, position))
        self._robot._publish_grippers()
    
    def __repr__(self) -> str:
        """返回夹爪的字符串表示。"""
        return f"Gripper('{self._side}', pos={self._position:.2f})"


# 动态生成预设方法
for name, pos, doc in _PRESETS:
    setattr(Gripper, name, _make_preset(pos, doc))
