"""
底盘控制模块
============

提供 Mantis 机器人底盘的控制接口。底盘支持全向移动（前后、左右、旋转）。

Note:
    仿真模式下底盘控制暂不支持预览。

Example:
    .. code-block:: python
    
        from mantis import Mantis
        
        with Mantis(ip="192.168.1.100") as robot:
            # 设置具体速度
            robot.chassis.set_velocity(vx=0.1, vy=0.0, omega=0.0)
            
            # 使用便捷方法
            robot.chassis.forward(0.2)
            robot.chassis.turn_left(0.5)
            robot.chassis.stop()
"""

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .mantis import Mantis


# 移动动作: (方法名, vx系数, vy系数, omega系数, 默认速度, 说明)
_MOVE_ACTIONS = [
    ("forward",      1,  0,  0, 0.1, "前进"),
    ("backward",    -1,  0,  0, 0.1, "后退"),
    ("strafe_left",  0,  1,  0, 0.1, "左移"),
    ("strafe_right", 0, -1,  0, 0.1, "右移"),
    ("turn_left",    0,  0,  1, 0.3, "左转"),
    ("turn_right",   0,  0, -1, 0.3, "右转"),
]


def _make_move_action(vx_sign, vy_sign, omega_sign, default_speed, doc):
    """工厂函数：生成移动方法。
    
    Args:
        vx_sign: 前后速度符号
        vy_sign: 左右速度符号
        omega_sign: 旋转速度符号
        default_speed: 默认速度值
        doc: 方法说明
        
    Returns:
        Callable: 生成的移动方法
    """
    def action(self, speed: float = default_speed):
        """执行移动动作。
        
        Args:
            speed: 速度大小（m/s 或 rad/s）
        """
        s = abs(speed)
        self.set_velocity(
            vx=vx_sign * s if vx_sign else 0.0,
            vy=vy_sign * s if vy_sign else 0.0,
            omega=omega_sign * s if omega_sign else 0.0
        )
    action.__doc__ = f"""{doc}。
    
    Args:
        speed: 速度大小，默认 {default_speed}
    """
    return action


class Chassis:
    """底盘控制类。
    
    底盘支持三个方向的运动：
    
    ========  ==============  ========
    参数      含义            单位
    ========  ==============  ========
    vx        前后速度        m/s
    vy        左右速度        m/s
    omega     旋转速度        rad/s
    ========  ==============  ========
    
    - ``vx > 0``: 前进
    - ``vx < 0``: 后退
    - ``vy > 0``: 左移
    - ``vy < 0``: 右移
    - ``omega > 0``: 左转
    - ``omega < 0``: 右转
    
    Attributes:
        vx: 当前前后速度 (m/s)
        vy: 当前左右速度 (m/s)
        omega: 当前旋转速度 (rad/s)
    
    Example:
        .. code-block:: python
        
            # 设置具体速度
            robot.chassis.set_velocity(vx=0.1, omega=0.2)
            
            # 便捷方法
            robot.chassis.forward(0.2)     # 前进
            robot.chassis.backward()       # 后退（默认速度）
            robot.chassis.strafe_left()    # 左移
            robot.chassis.turn_right(0.5)  # 右转
            robot.chassis.stop()           # 停止
    """
    
    def __init__(self, robot: "Mantis"):
        """初始化底盘控制器。
        
        Args:
            robot: Mantis 机器人实例
        """
        self._robot = robot
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
    
    @property
    def vx(self) -> float:
        """当前前后速度 (m/s)。
        
        Returns:
            float: 正值前进，负值后退
        """
        return self._vx
    
    @property
    def vy(self) -> float:
        """当前左右速度 (m/s)。
        
        Returns:
            float: 正值左移，负值右移
        """
        return self._vy
    
    @property
    def omega(self) -> float:
        """当前旋转速度 (rad/s)。
        
        Returns:
            float: 正值左转，负值右转
        """
        return self._omega
    
    def set_velocity(self, vx: float = None, vy: float = None, omega: float = None):
        """设置底盘速度。
        
        Args:
            vx: 前后速度 (m/s)，正值前进
            vy: 左右速度 (m/s)，正值左移
            omega: 旋转速度 (rad/s)，正值左转
            
        Example:
            .. code-block:: python
            
                robot.chassis.set_velocity(vx=0.1)          # 只设置前后
                robot.chassis.set_velocity(vx=0.1, omega=0.2)  # 边走边转
        """
        if vx is not None:
            self._vx = vx
        if vy is not None:
            self._vy = vy
        if omega is not None:
            self._omega = omega
        self._robot._publish_chassis()
    
    def stop(self):
        """停止所有运动。"""
        self.set_velocity(0.0, 0.0, 0.0)
    
    def __repr__(self) -> str:
        """返回底盘的字符串表示。"""
        return f"Chassis(vx={self._vx:.2f}, vy={self._vy:.2f}, ω={self._omega:.2f})"


# 动态生成移动方法
for name, vx, vy, omega, speed, doc in _MOVE_ACTIONS:
    setattr(Chassis, name, _make_move_action(vx, vy, omega, speed, doc))
