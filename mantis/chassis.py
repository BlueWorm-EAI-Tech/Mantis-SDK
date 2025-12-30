"""
底盘控制模块
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
    """工厂函数：生成移动方法"""
    def action(self, speed: float = default_speed):
        s = abs(speed)
        self.set_velocity(
            vx=vx_sign * s if vx_sign else 0.0,
            vy=vy_sign * s if vy_sign else 0.0,
            omega=omega_sign * s if omega_sign else 0.0
        )
    action.__doc__ = doc
    return action


class Chassis:
    """底盘控制类: vx(前后) + vy(左右) + omega(旋转)"""
    
    def __init__(self, robot: "Mantis"):
        self._robot = robot
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
    
    @property
    def vx(self) -> float:
        return self._vx
    
    @property
    def vy(self) -> float:
        return self._vy
    
    @property
    def omega(self) -> float:
        return self._omega
    
    def set_velocity(self, vx: float = None, vy: float = None, omega: float = None):
        """设置底盘速度 (m/s, rad/s)"""
        if vx is not None:
            self._vx = vx
        if vy is not None:
            self._vy = vy
        if omega is not None:
            self._omega = omega
        self._robot._publish_chassis()
    
    def stop(self):
        """停止"""
        self.set_velocity(0.0, 0.0, 0.0)
    
    def __repr__(self):
        return f"Chassis(vx={self._vx:.2f}, vy={self._vy:.2f}, ω={self._omega:.2f})"


# 动态生成移动方法
for name, vx, vy, omega, speed, doc in _MOVE_ACTIONS:
    setattr(Chassis, name, _make_move_action(vx, vy, omega, speed, doc))
