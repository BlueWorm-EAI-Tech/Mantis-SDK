"""
腰部控制模块
============

提供 Mantis 机器人腰部的控制接口。

- `2.0`: 腰部为 prismatic（直线移动）关节，控制上半身高度
- `3.0`: 额外支持上半身前后弯腰的绝对角度控制

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

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .mantis import Mantis


#: 2.0 腰部限位 (lower, upper)，单位：米
WAIST_LIMITS = (-0.62, 0.24)

#: 3.0 滑台限位，协议绝对高度范围 600mm ~ 1000mm，SDK 暴露为相对默认 900mm 的米制位移。
WAIST_LIMITS_3_0 = (-0.3, 0.1)

import math


#: 3.0 弯腰角度限位（弧度）
#:
#: SDK 对外统一使用弧度语义；底层协议需要的单位转换由后端桥接层处理。
#: - 负值：前倾 / 弯腰
#: - 正值：后仰
WAIST_BEND_LIMITS = (math.radians(-90.0), math.radians(5.0))


class Waist:
    """腰部控制类。
    
    `2.0` 下腰部是 prismatic（直线移动）关节，控制机器人上半身的高度。
    `3.0` 下额外支持上半身前后弯腰的绝对角度控制。
    
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

    #: 3.0 默认弯腰最大角速度 (rad/s)
    DEFAULT_BEND_SPEED = math.radians(20.0)
    
    def __init__(self, robot: "Mantis"):
        """初始化腰部控制器。"""
        self._robot = robot
        self._height = 0.0
        self._limits = WAIST_LIMITS
        self._speed = self.DEFAULT_SPEED
        self._bend_angle = 0.0
        self._bend_speed = self.DEFAULT_BEND_SPEED
    
    @property
    def height(self) -> float:
        """当前腰部高度（米）。"""
        return self._height
    
    @property
    def limits(self) -> tuple:
        """限位元组 (lower, upper)。"""
        return self._limits

    @property
    def bend_angle(self) -> float:
        """当前 3.0 弯腰角度（弧度）。"""
        return self._bend_angle
    
    def set_speed(self, speed: float):
        """设置滑台最大移动速度。
        
        Args:
            speed: 滑台最大速度 (m/s)，范围 0.01-0.5
        """
        self._speed = max(0.01, min(0.5, abs(speed)))

    def set_bend_speed(self, speed: float):
        """设置 3.0 前后弯腰最大角速度。

        Args:
            speed: 弯腰最大角速度 (rad/s)，范围 0.01-2.0
        """
        self._ensure_bend_supported()
        self._bend_speed = max(0.01, min(2.0, abs(speed)))
    
    def _clamp(self, value: float) -> float:
        """限制值在限位范围内。"""
        lower, upper = WAIST_LIMITS_3_0 if self._robot.robot_version == "3.0" else self._limits
        return max(lower, min(upper, value))

    def _clamp_bend_angle(self, value: float) -> float:
        """限制 3.0 弯腰角度到允许范围。"""
        lower, upper = WAIST_BEND_LIMITS
        return max(lower, min(upper, value))

    def _ensure_bend_supported(self) -> None:
        """校验当前机器人版本是否支持 3.0 弯腰控制。"""
        if self._robot.robot_version == "3.0":
            return
        raise NotImplementedError(f"{self._robot.robot_version} 当前 SDK 不支持腰部前后弯腰控制")
    
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
        self._height = self._clamp(height) if clamp else height
        
        self._robot._publish_waist()
        self._execute_motion(block)

    def set_bend(self, angle: float, clamp: bool = True, block: bool = True):
        """设置 3.0 上半身前后弯腰角度。

        Args:
            angle: 目标弯腰角（弧度）。
                负值表示前倾，正值表示后仰。
            clamp: 是否自动限制到有效范围，默认 True
            block: 是否阻塞等待完成，默认 True
        """
        self._ensure_bend_supported()
        self._bend_angle = self._clamp_bend_angle(angle) if clamp else float(angle)
        self._robot._publish_waist_angle()
        self._execute_motion(block)

    def bend_forward(self, angle: float = 0.3, block: bool = True):
        """3.0 上半身向前弯腰。"""
        self.set_bend(-abs(angle), block=block)

    def bend_backward(self, angle: float = 0.2, block: bool = True):
        """3.0 上半身向后仰。"""
        self.set_bend(abs(angle), block=block)
    
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
        if self._robot.robot_version == "3.0":
            self._bend_angle = 0.0
            self._robot._publish_waist_angle()
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
        return (
            f"Waist(height={self._height:.3f}m, "
            f"bend_angle={self._bend_angle:.3f}rad)"
        )
