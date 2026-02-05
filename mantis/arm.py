"""
手臂控制模块
============

提供 Mantis 机器人手臂的控制接口。每只手臂有 7 个自由度。

支持阻塞/非阻塞模式，允许双臂或多关节并行运动。

Example:
    .. code-block:: python
    
        from mantis import Mantis
        
        with Mantis(sim=True) as robot:
            # 阻塞模式（默认）：等待运动完成
            robot.left_arm.set_shoulder_pitch(-0.5)
            
            # 非阻塞模式：立即返回，可并行执行
            robot.left_arm.set_shoulder_pitch(-0.5, block=False)
            robot.right_arm.set_shoulder_pitch(-0.5, block=False)
            robot.wait()  # 等待所有运动完成
"""

import time
import threading
import math
import numpy as np
from typing import List, Tuple, TYPE_CHECKING
from .constants import (
    LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, NUM_ARM_JOINTS,
    LEFT_ARM_LIMITS, RIGHT_ARM_LIMITS,
    SERIAL_TO_URDF_MAP, JOINT_DIRECTION_MAP
)

if TYPE_CHECKING:
    from .mantis import Mantis


# 关节定义: (索引, 方法名后缀, 中文说明)
JOINT_DEFS = [
    (0, "shoulder_pitch", "肩俯仰"),
    (1, "shoulder_yaw",   "肩偏航"),
    (2, "shoulder_roll",  "肩翻滚"),
    (3, "elbow_pitch",    "肘俯仰"),
    (4, "wrist_roll",     "腕翻滚"),
    (5, "wrist_pitch",    "腕俯仰"),
    (6, "wrist_yaw",      "腕偏航"),
]


def _make_joint_setter(index: int, doc: str):
    """工厂函数：生成单关节设置方法。
    
    Args:
        index: 关节索引 (0-6)
        doc: 关节中文说明
        
    Returns:
        Callable: 生成的 setter 方法
    """
    def setter(self, angle: float, block: bool = True):
        """设置关节角度。
        
        Args:
            angle: 目标角度（弧度）
            block: 是否阻塞等待完成，默认 True
        """
        self.set_joint(index, angle, block=block)
    setter.__doc__ = f"""设置{doc}角度。
    
    Args:
        angle: 目标角度（弧度）
        block: 是否阻塞等待完成，默认 True
    """
    return setter


class Arm:
    """手臂控制类。
    
    每只手臂有 7 个关节，按索引顺序为:
    
    ======  ================  ==============  ==================
    索引    方法名            中文名          典型范围 (rad)
    ======  ================  ==============  ==================
    0       shoulder_pitch    肩俯仰          -2.61 ~ 0.78
    1       shoulder_yaw      肩偏航          -0.213 ~ 2.029
    2       shoulder_roll     肩翻滚          -1.57 ~ 1.57
    3       elbow_pitch       肘俯仰          -0.78 ~ 1.57
    4       wrist_roll        腕翻滚          -1.57 ~ 1.57
    5       wrist_pitch       腕俯仰          -0.52 ~ 0.52
    6       wrist_yaw         腕偏航          -1.57 ~ 1.57
    ======  ================  ==============  ==================
    
    支持阻塞/非阻塞模式：
        - block=True（默认）：等待运动完成后返回
        - block=False：立即返回，运动在后台执行
    
    Attributes:
        side: 手臂侧别 ('left' 或 'right')
        joint_names: 关节名称列表
        positions: 当前关节位置列表
        limits: 关节限位列表
        is_moving: 是否正在运动中
    
    Example:
        .. code-block:: python
        
            # 阻塞模式（等待完成）
            robot.left_arm.set_shoulder_pitch(-0.5)
            
            # 非阻塞模式（并行运动）
            robot.left_arm.set_joints([...], block=False)
            robot.right_arm.set_joints([...], block=False)
            robot.left_arm.wait()   # 等待左臂完成
            robot.right_arm.wait()  # 等待右臂完成
    """
    
    #: 默认关节速度 (rad/s)
    DEFAULT_JOINT_SPEED = 1.0
    
    def __init__(self, robot: "Mantis", side: str):
        """初始化手臂控制器。
        
        Args:
            robot: Mantis 机器人实例
            side: 手臂侧别，'left' 或 'right'
            
        Raises:
            ValueError: 如果 side 不是 'left' 或 'right'
        """
        if side not in ("left", "right"):
            raise ValueError("side 必须是 'left' 或 'right'")
        
        self._robot = robot
        self._side = side
        self._joint_names = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        self._limits = LEFT_ARM_LIMITS if side == "left" else RIGHT_ARM_LIMITS
        self._positions = [0.0] * NUM_ARM_JOINTS
        self._target_positions = [0.0] * NUM_ARM_JOINTS
        self._joint_speed = self.DEFAULT_JOINT_SPEED
    
    @property
    def side(self) -> str:
        """手臂侧别。"""
        return self._side
    
    @property
    def joint_names(self) -> List[str]:
        """关节名称列表。"""
        return self._joint_names.copy()
    
    @property
    def positions(self) -> List[float]:
        """当前关节位置。"""
        return self._positions.copy()
    
    @property
    def limits(self) -> List[Tuple[float, float]]:
        """关节限位列表。"""
        return self._limits.copy()
    
    def set_speed(self, speed: float):
        """设置关节运动速度。
        
        Args:
            speed: 关节速度 (rad/s)，范围 0.1-3.0
        """
        self._joint_speed = max(0.1, min(3.0, abs(speed)))
    
    def get_limit(self, index: int) -> Tuple[float, float]:
        """获取指定关节的限位。"""
        if not 0 <= index < NUM_ARM_JOINTS:
            raise ValueError(f"index 必须在 0-{NUM_ARM_JOINTS-1} 之间")
        return self._limits[index]
    
    def _clamp(self, index: int, value: float) -> float:
        """限制值在关节限位范围内。"""
        lower, upper = self._limits[index]
        return max(lower, min(upper, value))
    
    def set_joints(self, positions: List[float], clamp: bool = True, block: bool = True):
        """设置所有关节角度。
        
        Args:
            positions: 7 个关节角度（弧度）
            clamp: 是否自动限制在限位范围内，默认 True
            block: 是否阻塞等待完成，默认 True
            
        Example:
            .. code-block:: python
            
                # 阻塞模式
                robot.left_arm.set_joints([0.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0])
                
                # 非阻塞模式（双臂并行）
                robot.left_arm.set_joints([...], block=False)
                robot.right_arm.set_joints([...], block=False)
        """
        if len(positions) != NUM_ARM_JOINTS:
            raise ValueError(f"positions 长度必须为 {NUM_ARM_JOINTS}")
        
        # 计算目标位置
        if clamp:
            new_positions = [self._clamp(i, p) for i, p in enumerate(positions)]
        else:
            new_positions = list(positions)
        
        # 更新位置
        self._target_positions = new_positions
        self._positions = new_positions
        
        # 同步 IK Solver 目标点 (防止混合使用 IK 和手动控制时目标点漂移)
        # 注意: 即使是 IK 调用的 set_joints，这里再次同步也是安全的 (重置为解算结果)
        self._robot.sync_ik_with_commands()
        
        self._robot._publish_joints()
        
        # 执行运动
        self._execute_motion(block)
    
    def set_joint(self, index: int, position: float, clamp: bool = True, block: bool = True):
        """设置单个关节角度。
        
        Args:
            index: 关节索引 (0-6)
            position: 目标角度（弧度）
            clamp: 是否自动限制在限位范围内，默认 True
            block: 是否阻塞等待完成，默认 True
        """
        if not 0 <= index < NUM_ARM_JOINTS:
            raise ValueError(f"index 必须在 0-{NUM_ARM_JOINTS-1} 之间")
        
        if clamp:
            position = self._clamp(index, position)
        
        # 更新位置
        self._positions[index] = position
        self._target_positions[index] = position

        # 同步 IK Solver 目标点 (防止混合使用 IK 和手动控制时目标点漂移)
        self._robot.sync_ik_with_commands()

        self._robot._publish_joints()
        
        # 执行运动
        self._execute_motion(block)
    
    def _execute_motion(self, block: bool):
        """执行运动。"""
        # 这里仅用于非阻塞模式下的即时返回
        if block:
            self.wait()

    def wait(self):
        """等待当前运动完成。"""
        self._robot.wait(self.joint_names)
    
    @property
    def is_moving(self) -> bool:
        """是否正在运动中（基于整机状态）。"""
        return self._robot.is_moving(self.joint_names)

    def home(self, block: bool = True):
        """回到零位。
        
        Args:
            block: 是否阻塞等待完成，默认 True
        """
        self.set_joints([0.0] * NUM_ARM_JOINTS, block=block)

    def ik(self, x: float, y: float, z: float, 
                     roll: float, pitch: float, yaw: float, 
                     block: bool = True, abs: bool = True):
        """Move arm end-effector to target pose.
        
        Uses Inverse Kinematics to find joint angles.
        
        Args:
            x, y, z: Target position (meters) or Delta position (if abs=False)
            roll, pitch, yaw: Target orientation (radians) or Delta orientation (if abs=False)
            block: Whether to wait for motion to complete
            abs: Whether to use absolute coordinates (default), False for relative/incremental
            
        Raises:
            RuntimeError: If IK fails or solution is invalid
        """
        if abs:
            # Construct Rotation Matrix (RPY -> Matrix)
            # R = Rz(yaw) * Ry(pitch) * Rx(roll)
            cr = math.cos(roll); sr = math.sin(roll)
            cp = math.cos(pitch); sp = math.sin(pitch)
            cy = math.cos(yaw); sy = math.sin(yaw)
            
            Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
            Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            
            R = Rz @ Ry @ Rx
            
            # Construct Transformation Matrix
            T_target = np.eye(4)
            T_target[:3, :3] = R
            T_target[:3, 3] = [x, y, z]
            
            # Get current poses for both arms (needed for IK solver to hold other arm still)
            try:
                T_cur_left, T_cur_right = self._robot.compute_fk()
            except Exception as e:
                raise RuntimeError(f"Failed to compute FK: {e}")
                
            # Prepare IK inputs
            if self._side == 'left':
                T_l = T_target
                T_r = T_cur_right
            else:
                T_l = T_cur_left
                T_r = T_target
            # Solve IK
            try:
                q_sol = self._robot.solve_ik_abs(T_l, T_r)
            except Exception as e:
                raise RuntimeError(f"IK solver failed: {e}")

        else: # mode == 'rel'
            # Construct Delta Vector [dx, dy, dz, droll, dpitch, dyaw]
            delta = np.array([x, y, z, roll, pitch, yaw])
            zeros = np.zeros(6)
            
            if self._side == 'left':
                d_l = delta
                d_r = zeros
            else:
                d_l = zeros
                d_r = delta
                
            # Solve IK (Incremental)
            try:
                q_sol = self._robot.solve_ik_rel(d_l, d_r)
            except Exception as e:
                raise RuntimeError(f"Incremental IK solver failed: {e}")
            
        # Map solution to this arm's joints
        solver = self._robot._get_ik_solver()
        solver_names = solver.get_joint_names()
        solution_dict = dict(zip(solver_names, q_sol))
        
        target_positions = []
        for name in self._joint_names:
            urdf_name = SERIAL_TO_URDF_MAP.get(name)
            if not urdf_name:
                raise RuntimeError(f"Mapping not found for joint {name}")
                
            if urdf_name not in solution_dict:
                 raise RuntimeError(f"Joint {urdf_name} not in IK solution")
            
            # Apply direction correction (Solver (URDF) -> SDK (Serial))
            # q_serial = q_urdf * direction (since direction is +/- 1)
            direction = JOINT_DIRECTION_MAP.get(name, 1.0)
            val = solution_dict[urdf_name] * direction
            target_positions.append(val)
            
        # Move
        self.set_joints(target_positions, block=block)

    
    def __repr__(self) -> str:
        """返回手臂的字符串表示。"""
        status = "运动中" if self.is_moving else "停止"
        return f"Arm(side='{self._side}', {status})"


# 动态生成各关节的 set_xxx 方法
for idx, name, doc in JOINT_DEFS:
    setattr(Arm, f"set_{name}", _make_joint_setter(idx, doc))
