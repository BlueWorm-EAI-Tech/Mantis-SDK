"""
头部控制模块
"""

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
    """工厂函数：生成 look_xxx 方法"""
    def action(self, angle: float = default):
        setattr(self, f"_{attr}", sign * abs(angle))
        self._apply_limits()
        self._robot._publish_head()
    action.__doc__ = doc
    return action


class Head:
    """
    头部控制类：俯仰(pitch) + 偏航(yaw)
    
    限位:
        pitch: -0.7 ~ 0.2 rad
        yaw:   -1.57 ~ 1.57 rad
    """
    
    def __init__(self, robot: "Mantis"):
        self._robot = robot
        self._pitch = 0.0
        self._yaw = 0.0
        self._limits = HEAD_LIMITS
    
    @property
    def pitch(self) -> float:
        return self._pitch
    
    @property
    def yaw(self) -> float:
        return self._yaw
    
    @property
    def limits(self) -> dict:
        """返回限位 {'pitch': (lower, upper), 'yaw': (lower, upper)}"""
        return self._limits.copy()
    
    def _clamp(self, attr: str, value: float) -> float:
        """限制值在限位范围内"""
        lower, upper = self._limits[attr]
        return max(lower, min(upper, value))
    
    def _apply_limits(self):
        """应用限位"""
        self._pitch = self._clamp("pitch", self._pitch)
        self._yaw = self._clamp("yaw", self._yaw)
    
    def set_pose(self, pitch: float = None, yaw: float = None, clamp: bool = True):
        """
        设置头部姿态（弧度）
        
        Args:
            pitch: 俯仰角（-0.7 ~ 0.2）
            yaw: 偏航角（-1.57 ~ 1.57）
            clamp: 是否自动限制在限位范围内（默认 True）
        """
        if pitch is not None:
            self._pitch = self._clamp("pitch", pitch) if clamp else pitch
        if yaw is not None:
            self._yaw = self._clamp("yaw", yaw) if clamp else yaw
        self._robot._publish_head()
    
    def set_pitch(self, angle: float, clamp: bool = True):
        """设置俯仰角"""
        self.set_pose(pitch=angle, clamp=clamp)
    
    def set_yaw(self, angle: float, clamp: bool = True):
        """设置偏航角"""
        self.set_pose(yaw=angle, clamp=clamp)
    
    def center(self):
        """回中"""
        self.set_pose(0.0, 0.0)
    
    def __repr__(self):
        return f"Head(pitch={self._pitch:.2f}, yaw={self._yaw:.2f})"


# 动态生成 look_xxx 方法
for name, attr, sign, default, doc in _LOOK_ACTIONS:
    setattr(Head, name, _make_look_action(attr, sign, default, doc))
