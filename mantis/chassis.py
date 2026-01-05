"""
底盘控制模块
============

提供 Mantis 机器人底盘的控制接口。底盘支持全向移动（前后、左右、旋转）。

安全设计：
    - 所有运动命令必须指定距离或角度，运动完成后自动停止
    - 不提供持续速度控制，避免代码异常导致机器人失控
    - 支持自定义速度，但必须同时指定运动量

Example:
    .. code-block:: python
    
        from mantis import Mantis
        
        with Mantis(ip="192.168.1.100") as robot:
            # 前进 0.5 米
            robot.chassis.forward(0.5)
            
            # 左转 90 度
            robot.chassis.turn_left(90)
            
            # 自定义速度前进
            robot.chassis.forward(1.0, speed=0.2)
            
            # 组合运动：边走边转
            robot.chassis.move(x=0.5, y=0.2, angle=45)
"""

import time
import math
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from .mantis import Mantis


class Chassis:
    """底盘控制类（基于距离/角度的安全控制）。
    
    所有运动命令都需要指定目标距离或角度，运动完成后自动停止。
    这种设计确保即使程序异常退出，机器人也会在完成当前运动后停止。
    
    默认速度：
        - 线速度: 0.1 m/s
        - 角速度: 0.5 rad/s (约 28.6 °/s)
    
    Example:
        .. code-block:: python
        
            # 基本运动
            robot.chassis.forward(0.5)      # 前进 0.5 米
            robot.chassis.backward(0.3)     # 后退 0.3 米
            robot.chassis.strafe_left(0.2)  # 左移 0.2 米
            robot.chassis.turn_left(90)     # 左转 90 度
            
            # 自定义速度
            robot.chassis.forward(1.0, speed=0.2)  # 0.2m/s 前进 1 米
            
            # 组合运动
            robot.chassis.move(x=0.5, y=0.2, angle=45)
    """
    
    #: 默认线速度 (m/s)
    DEFAULT_LINEAR_SPEED = 1.0
    
    #: 默认角速度 (rad/s)
    DEFAULT_ANGULAR_SPEED = 0.3
    
    #: 最大线速度 (m/s)
    MAX_LINEAR_SPEED = 3.0
    
    #: 最大角速度 (rad/s)
    MAX_ANGULAR_SPEED = 2.0
    
    #: 默认摩擦补偿系数（线速度）
    DEFAULT_LINEAR_FRICTION = 1.0
    
    #: 默认摩擦补偿系数（角速度）
    DEFAULT_ANGULAR_FRICTION = 1.0
    
    def __init__(self, robot: "Mantis"):
        """初始化底盘控制器。
        
        Args:
            robot: Mantis 机器人实例
        """
        self._robot = robot
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
        self._default_linear_speed = self.DEFAULT_LINEAR_SPEED
        self._default_angular_speed = self.DEFAULT_ANGULAR_SPEED
        self._linear_friction = self.DEFAULT_LINEAR_FRICTION
        self._angular_friction = self.DEFAULT_ANGULAR_FRICTION
        self._is_moving = False
    
    @property
    def is_moving(self) -> bool:
        """是否正在运动中。"""
        return self._is_moving
    
    def set_friction(self, linear: float = None, angular: float = None):
        """设置摩擦补偿系数。
        
        系数越大，运动时间越长，用于补偿地面摩擦力导致的距离/角度损失。
        
        Args:
            linear: 线性运动摩擦补偿系数，默认 1.0，建议范围 1.0-3.0
            angular: 旋转运动摩擦补偿系数，默认 1.0，建议范围 1.0-3.0
            
        Example:
            .. code-block:: python
            
                # 地面摩擦力大，需要更长时间才能走到目标距离
                robot.chassis.set_friction(linear=1.5, angular=2.0)
        """
        if linear is not None:
            self._linear_friction = max(0.5, min(5.0, abs(linear)))
        if angular is not None:
            self._angular_friction = max(0.5, min(5.0, abs(angular)))
    
    def set_default_speed(self, linear: float = None, angular: float = None):
        """设置默认速度。
        
        Args:
            linear: 默认线速度 (m/s)，范围 0.01-0.5
            angular: 默认角速度 (rad/s)，范围 0.1-1.0
            
        Example:
            .. code-block:: python
            
                robot.chassis.set_default_speed(linear=0.15, angular=0.8)
        """
        if linear is not None:
            self._default_linear_speed = max(0.01, min(self.MAX_LINEAR_SPEED, abs(linear)))
        if angular is not None:
            self._default_angular_speed = max(0.1, min(self.MAX_ANGULAR_SPEED, abs(angular)))
    
    def forward(self, distance: float, speed: float = None, block: bool = True):
        """前进指定距离。
        
        Args:
            distance: 前进距离 (米)，必须为正数
            speed: 速度 (m/s)，默认使用 default_linear_speed
            block: 是否阻塞等待完成，默认 True
            
        Example:
            .. code-block:: python
            
                robot.chassis.forward(0.5)           # 前进 0.5 米
                robot.chassis.forward(1.0, speed=0.2)  # 以 0.2m/s 前进 1 米
        """
        self._move_linear(abs(distance), 0, speed, block)
    
    def backward(self, distance: float, speed: float = None, block: bool = True):
        """后退指定距离。
        
        Args:
            distance: 后退距离 (米)，必须为正数
            speed: 速度 (m/s)，默认使用 default_linear_speed
            block: 是否阻塞等待完成，默认 True
        """
        self._move_linear(-abs(distance), 0, speed, block)
    
    def strafe_left(self, distance: float, speed: float = None, block: bool = True):
        """左移指定距离。
        
        Args:
            distance: 左移距离 (米)，必须为正数
            speed: 速度 (m/s)，默认使用 default_linear_speed
            block: 是否阻塞等待完成，默认 True
        """
        self._move_linear(0, abs(distance), speed, block)
    
    def strafe_right(self, distance: float, speed: float = None, block: bool = True):
        """右移指定距离。
        
        Args:
            distance: 右移距离 (米)，必须为正数
            speed: 速度 (m/s)，默认使用 default_linear_speed
            block: 是否阻塞等待完成，默认 True
        """
        self._move_linear(0, -abs(distance), speed, block)
    
    def turn_left(self, degrees: float, speed: float = None, block: bool = True):
        """左转指定角度。
        
        Args:
            degrees: 左转角度 (度)，必须为正数
            speed: 角速度 (rad/s)，默认使用 default_angular_speed
            block: 是否阻塞等待完成，默认 True
            
        Example:
            .. code-block:: python
            
                robot.chassis.turn_left(90)   # 左转 90 度
                robot.chassis.turn_left(180)  # 左转 180 度
        """
        self._rotate(abs(degrees), speed, block)
    
    def turn_right(self, degrees: float, speed: float = None, block: bool = True):
        """右转指定角度。
        
        Args:
            degrees: 右转角度 (度)，必须为正数
            speed: 角速度 (rad/s)，默认使用 default_angular_speed
            block: 是否阻塞等待完成，默认 True
        """
        self._rotate(-abs(degrees), speed, block)
    
    def move(self, x: float = 0, y: float = 0, angle: float = 0, 
             linear_speed: float = None, angular_speed: float = None,
             block: bool = True):
        """组合运动：先平移再旋转。
        
        Args:
            x: 前后移动距离 (米)，正值前进，负值后退
            y: 左右移动距离 (米)，正值左移，负值右移
            angle: 旋转角度 (度)，正值左转，负值右转
            linear_speed: 线速度 (m/s)
            angular_speed: 角速度 (rad/s)
            block: 是否阻塞等待完成，默认 True
            
        Example:
            .. code-block:: python
            
                # 前进 0.5m，左移 0.2m，左转 45 度
                robot.chassis.move(x=0.5, y=0.2, angle=45)
        """
        # 先执行平移
        if x != 0 or y != 0:
            self._move_linear(x, y, linear_speed, block=True)
        
        # 再执行旋转
        if angle != 0:
            self._rotate(angle, angular_speed, block)
    
    def stop(self):
        """立即停止所有运动。"""
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
        self._is_moving = False
        self._robot._publish_chassis()
    
    # ==================== 内部方法 ====================
    
    def _move_linear(self, dx: float, dy: float, speed: float = None, block: bool = True):
        """执行线性移动。
        
        Args:
            dx: X 方向距离 (前后)
            dy: Y 方向距离 (左右)
            speed: 速度 (m/s)
            block: 是否阻塞
        """
        distance = math.sqrt(dx * dx + dy * dy)
        if distance < 0.001:  # 距离太小，忽略
            return
        
        # 计算速度
        spd = speed if speed is not None else self._default_linear_speed
        spd = max(0.01, min(self.MAX_LINEAR_SPEED, abs(spd)))
        
        # 计算各分量速度
        self._vx = (dx / distance) * spd
        self._vy = (dy / distance) * spd
        self._omega = 0.0
        
        # 计算运动时间（应用摩擦补偿系数）
        duration = distance / spd * self._linear_friction
        
        self._execute_motion(duration, block)
    
    def _rotate(self, degrees: float, speed: float = None, block: bool = True):
        """执行旋转。
        
        Args:
            degrees: 角度 (度)，正值左转
            speed: 角速度 (rad/s)
            block: 是否阻塞
        """
        radians = math.radians(degrees)
        if abs(radians) < 0.001:  # 角度太小，忽略
            return
        
        # 计算速度
        spd = speed if speed is not None else self._default_angular_speed
        spd = max(0.1, min(self.MAX_ANGULAR_SPEED, abs(spd)))
        
        # 设置角速度方向
        self._vx = 0.0
        self._vy = 0.0
        self._omega = spd if degrees > 0 else -spd
        
        # 计算运动时间（应用摩擦补偿系数）
        duration = abs(radians) / spd * self._angular_friction
        
        self._execute_motion(duration, block)
    
    def _execute_motion(self, duration: float, block: bool):
        """执行运动。
        
        Args:
            duration: 运动时长 (秒)
            block: 是否阻塞等待完成
        """
        self._is_moving = True
        self._robot._publish_chassis()
        
        if block:
            time.sleep(duration)
            self.stop()
        else:
            # 非阻塞模式：启动定时器在后台停止
            import threading
            def _delayed_stop():
                time.sleep(duration)
                if self._is_moving:  # 检查是否已被手动停止
                    self.stop()
            threading.Thread(target=_delayed_stop, daemon=True).start()
    
    def __repr__(self) -> str:
        """返回底盘的字符串表示。"""
        status = "运动中" if self._is_moving else "停止"
        return f"Chassis({status}, vx={self._vx:.2f}, vy={self._vy:.2f}, ω={self._omega:.2f})"
