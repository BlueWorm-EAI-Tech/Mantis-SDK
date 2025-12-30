"""
头部控制模块
============

提供 Mantis 机器人头部的控制接口。头部有 2 个自由度：俯仰和偏航。

Example:
    .. code-block:: python
    
        from mantis import Mantis
        
        with Mantis(sim=True) as robot:
            # 设置具体角度
            robot.head.set_pose(pitch=0.1, yaw=0.5)
            
            # 使用便捷方法
            robot.head.look_left()
            robot.head.look_up()
            robot.head.center()
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
    """工厂函数：生成 look_xxx 方法。
    
    Args:
        attr: 属性名 ('pitch' 或 'yaw')
        sign: 方向符号 (1 或 -1)
        default: 默认角度值
        doc: 方法说明
        
    Returns:
        Callable: 生成的动作方法
    """
    def action(self, angle: float = default):
        """执行头部动作。
        
        Args:
            angle: 角度大小（弧度），默认使用预设值
        """
        setattr(self, f"_{attr}", sign * abs(angle))
        self._apply_limits()
        self._robot._publish_head()
    action.__doc__ = f"""{doc}。
    
    Args:
        angle: 角度大小（弧度），默认 {default}
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
    
    Attributes:
        pitch: 当前俯仰角（弧度）
        yaw: 当前偏航角（弧度）
        limits: 限位字典
    
    Example:
        .. code-block:: python
        
            # 直接设置角度
            robot.head.set_pose(pitch=0.1, yaw=0.5)
            robot.head.set_pitch(-0.3)
            robot.head.set_yaw(1.0)
            
            # 便捷方法
            robot.head.look_left(0.5)   # 向左看 0.5 rad
            robot.head.look_right()     # 向右看（默认角度）
            robot.head.look_up()
            robot.head.look_down()
            robot.head.center()         # 回中
    """
    
    def __init__(self, robot: "Mantis"):
        """初始化头部控制器。
        
        Args:
            robot: Mantis 机器人实例
        """
        self._robot = robot
        self._pitch = 0.0
        self._yaw = 0.0
        self._limits = HEAD_LIMITS
    
    @property
    def pitch(self) -> float:
        """当前俯仰角（弧度）。
        
        Returns:
            float: 俯仰角，正值低头，负值抬头
        """
        return self._pitch
    
    @property
    def yaw(self) -> float:
        """当前偏航角（弧度）。
        
        Returns:
            float: 偏航角，正值左转，负值右转
        """
        return self._yaw
    
    @property
    def limits(self) -> dict:
        """限位字典。
        
        Returns:
            dict: {'pitch': (lower, upper), 'yaw': (lower, upper)}
        """
        return self._limits.copy()
    
    def _clamp(self, attr: str, value: float) -> float:
        """限制值在限位范围内。"""
        lower, upper = self._limits[attr]
        return max(lower, min(upper, value))
    
    def _apply_limits(self):
        """应用限位。"""
        self._pitch = self._clamp("pitch", self._pitch)
        self._yaw = self._clamp("yaw", self._yaw)
    
    def set_pose(self, pitch: float = None, yaw: float = None, clamp: bool = True):
        """设置头部姿态。
        
        Args:
            pitch: 俯仰角（弧度），范围 -0.7 ~ 0.2
            yaw: 偏航角（弧度），范围 -1.57 ~ 1.57
            clamp: 是否自动限制在限位范围内，默认 True
            
        Example:
            .. code-block:: python
            
                robot.head.set_pose(pitch=0.1, yaw=0.5)
                robot.head.set_pose(pitch=-0.3)  # 只设置俯仰
        """
        if pitch is not None:
            self._pitch = self._clamp("pitch", pitch) if clamp else pitch
        if yaw is not None:
            self._yaw = self._clamp("yaw", yaw) if clamp else yaw
        self._robot._publish_head()
    
    def set_pitch(self, angle: float, clamp: bool = True):
        """设置俯仰角。
        
        Args:
            angle: 目标角度（弧度），范围 -0.7 ~ 0.2
            clamp: 是否自动限制在限位范围内，默认 True
        """
        self.set_pose(pitch=angle, clamp=clamp)
    
    def set_yaw(self, angle: float, clamp: bool = True):
        """设置偏航角。
        
        Args:
            angle: 目标角度（弧度），范围 -1.57 ~ 1.57
            clamp: 是否自动限制在限位范围内，默认 True
        """
        self.set_pose(yaw=angle, clamp=clamp)
    
    def center(self):
        """回中（俯仰和偏航都归零）。"""
        self.set_pose(0.0, 0.0)
    
    def __repr__(self) -> str:
        """返回头部的字符串表示。"""
        return f"Head(pitch={self._pitch:.2f}, yaw={self._yaw:.2f})"


# 动态生成 look_xxx 方法
for name, attr, sign, default, doc in _LOOK_ACTIONS:
    setattr(Head, name, _make_look_action(attr, sign, default, doc))
