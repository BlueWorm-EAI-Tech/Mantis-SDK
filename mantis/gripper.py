"""
夹爪控制模块
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
    """工厂函数：生成预设位置方法"""
    def method(self):
        self.set_position(pos)
    method.__doc__ = doc
    return method


class Gripper:
    """夹爪控制类: 0.0(闭合) ~ 1.0(张开)"""
    
    def __init__(self, robot: "Mantis", side: str):
        if side not in ("left", "right"):
            raise ValueError("side 必须是 'left' 或 'right'")
        self._robot = robot
        self._side = side
        self._position = 0.0
    
    @property
    def side(self) -> str:
        return self._side
    
    @property
    def position(self) -> float:
        return self._position
    
    def set_position(self, position: float):
        """设置夹爪位置 (0.0~1.0)"""
        self._position = max(0.0, min(1.0, position))
        self._robot._publish_grippers()
    
    def __repr__(self):
        return f"Gripper('{self._side}', pos={self._position:.2f})"


# 动态生成预设方法
for name, pos, doc in _PRESETS:
    setattr(Gripper, name, _make_preset(pos, doc))
