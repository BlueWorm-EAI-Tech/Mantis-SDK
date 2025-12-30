"""
手臂控制模块
============

提供 Mantis 机器人手臂的控制接口。每只手臂有 7 个自由度。

Example:
    .. code-block:: python
    
        from mantis import Mantis
        
        with Mantis(sim=True) as robot:
            # 设置单个关节
            robot.left_arm.set_shoulder_pitch(-0.5)
            
            # 设置多个关节
            robot.left_arm.set_joints([0.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0])
            
            # 回到零位
            robot.left_arm.home()
"""

from typing import List, Tuple, TYPE_CHECKING
from .constants import (
    LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, NUM_ARM_JOINTS,
    LEFT_ARM_LIMITS, RIGHT_ARM_LIMITS
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
    def setter(self, angle: float):
        """设置关节角度。
        
        Args:
            angle: 目标角度（弧度）
        """
        self.set_joint(index, angle)
    setter.__doc__ = f"""设置{doc}角度。
    
    Args:
        angle: 目标角度（弧度）
    """
    return setter


class Arm:
    """手臂控制类。
    
    每只手臂有 7 个关节，按索引顺序为:
    
    ======  ================  ==============  ==================
    索引    方法名            中文名          典型范围 (rad)
    ======  ================  ==============  ==================
    0       shoulder_pitch    肩俯仰          -2.61 ~ 0.78
    1       shoulder_yaw      肩偏航          0.08 ~ 1.04 (左臂)
    2       shoulder_roll     肩翻滚          -1.57 ~ 1.57
    3       elbow_pitch       肘俯仰          -0.78 ~ 1.57
    4       wrist_roll        腕翻滚          -1.57 ~ 1.57
    5       wrist_pitch       腕俯仰          -0.52 ~ 0.52
    6       wrist_yaw         腕偏航          -1.57 ~ 1.57
    ======  ================  ==============  ==================
    
    Attributes:
        side: 手臂侧别 ('left' 或 'right')
        joint_names: 关节名称列表
        positions: 当前关节位置列表
        limits: 关节限位列表
    
    Example:
        .. code-block:: python
        
            # 通过索引设置
            robot.left_arm.set_joint(0, -0.5)
            
            # 通过方法名设置
            robot.left_arm.set_shoulder_pitch(-0.5)
            robot.left_arm.set_elbow_pitch(1.0)
            
            # 批量设置
            robot.left_arm.set_joints([-0.5, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0])
    """
    
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
    
    @property
    def side(self) -> str:
        """手臂侧别。
        
        Returns:
            str: 'left' 或 'right'
        """
        return self._side
    
    @property
    def joint_names(self) -> List[str]:
        """关节名称列表。
        
        Returns:
            List[str]: 7 个关节的名称
        """
        return self._joint_names.copy()
    
    @property
    def positions(self) -> List[float]:
        """当前关节位置。
        
        Returns:
            List[float]: 7 个关节的当前角度（弧度）
        """
        return self._positions.copy()
    
    @property
    def limits(self) -> List[Tuple[float, float]]:
        """关节限位列表。
        
        Returns:
            List[Tuple[float, float]]: 每个关节的 (下限, 上限)
        """
        return self._limits.copy()
    
    def get_limit(self, index: int) -> Tuple[float, float]:
        """获取指定关节的限位。
        
        Args:
            index: 关节索引 (0-6)
            
        Returns:
            Tuple[float, float]: (下限, 上限) 弧度
            
        Raises:
            ValueError: 如果索引超出范围
        """
        if not 0 <= index < NUM_ARM_JOINTS:
            raise ValueError(f"index 必须在 0-{NUM_ARM_JOINTS-1} 之间")
        return self._limits[index]
    
    def _clamp(self, index: int, value: float) -> float:
        """限制值在关节限位范围内。
        
        Args:
            index: 关节索引
            value: 输入值
            
        Returns:
            float: 限制后的值
        """
        lower, upper = self._limits[index]
        return max(lower, min(upper, value))
    
    def set_joints(self, positions: List[float], clamp: bool = True):
        """设置所有关节角度。
        
        Args:
            positions: 7 个关节角度（弧度）
            clamp: 是否自动限制在限位范围内，默认 True
            
        Raises:
            ValueError: 如果 positions 长度不为 7
            
        Example:
            .. code-block:: python
            
                robot.left_arm.set_joints([0.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0])
        """
        if len(positions) != NUM_ARM_JOINTS:
            raise ValueError(f"positions 长度必须为 {NUM_ARM_JOINTS}")
        if clamp:
            self._positions = [self._clamp(i, p) for i, p in enumerate(positions)]
        else:
            self._positions = list(positions)
        self._robot._publish_joints()
    
    def set_joint(self, index: int, position: float, clamp: bool = True):
        """设置单个关节角度。
        
        Args:
            index: 关节索引 (0-6)
            position: 目标角度（弧度）
            clamp: 是否自动限制在限位范围内，默认 True
            
        Raises:
            ValueError: 如果索引超出范围
            
        Example:
            .. code-block:: python
            
                robot.left_arm.set_joint(0, -0.5)  # 肩俯仰
                robot.left_arm.set_joint(3, 1.0)   # 肘俯仰
        """
        if not 0 <= index < NUM_ARM_JOINTS:
            raise ValueError(f"index 必须在 0-{NUM_ARM_JOINTS-1} 之间")
        if clamp:
            position = self._clamp(index, position)
        self._positions[index] = position
        self._robot._publish_joints()
    
    def home(self):
        """回到零位。
        
        将所有 7 个关节设置为 0.0。
        """
        self.set_joints([0.0] * NUM_ARM_JOINTS)
    
    def __repr__(self) -> str:
        """返回手臂的字符串表示。"""
        return f"Arm(side='{self._side}', positions={self._positions})"


# 动态生成各关节的 set_xxx 方法
for idx, name, doc in JOINT_DEFS:
    setattr(Arm, f"set_{name}", _make_joint_setter(idx, doc))
