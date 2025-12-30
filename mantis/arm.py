"""
手臂控制模块
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
    """工厂函数：生成单关节设置方法"""
    def setter(self, angle: float):
        self.set_joint(index, angle)
    setter.__doc__ = f"设置{doc}角度（弧度）"
    return setter


class Arm:
    """
    手臂控制类
    
    每只手臂有 7 个关节:
        0: shoulder_pitch - 肩俯仰
        1: shoulder_yaw   - 肩偏航
        2: shoulder_roll  - 肩翻滚
        3: elbow_pitch    - 肘俯仰
        4: wrist_roll     - 腕翻滚
        5: wrist_pitch    - 腕俯仰
        6: wrist_yaw      - 腕偏航
    """
    
    def __init__(self, robot: "Mantis", side: str):
        if side not in ("left", "right"):
            raise ValueError("side 必须是 'left' 或 'right'")
        
        self._robot = robot
        self._side = side
        self._joint_names = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        self._limits = LEFT_ARM_LIMITS if side == "left" else RIGHT_ARM_LIMITS
        self._positions = [0.0] * NUM_ARM_JOINTS
    
    @property
    def side(self) -> str:
        """返回手臂侧别: 'left' 或 'right'"""
        return self._side
    
    @property
    def joint_names(self) -> List[str]:
        """返回关节名称列表"""
        return self._joint_names.copy()
    
    @property
    def positions(self) -> List[float]:
        """返回当前关节位置"""
        return self._positions.copy()
    
    @property
    def limits(self) -> List[Tuple[float, float]]:
        """返回关节限位列表 [(lower, upper), ...]"""
        return self._limits.copy()
    
    def get_limit(self, index: int) -> Tuple[float, float]:
        """获取指定关节的限位 (lower, upper)"""
        if not 0 <= index < NUM_ARM_JOINTS:
            raise ValueError(f"index 必须在 0-{NUM_ARM_JOINTS-1} 之间")
        return self._limits[index]
    
    def _clamp(self, index: int, value: float) -> float:
        """限制值在关节限位范围内"""
        lower, upper = self._limits[index]
        return max(lower, min(upper, value))
    
    def set_joints(self, positions: List[float], clamp: bool = True):
        """
        设置所有关节角度（7个，弧度）
        
        Args:
            positions: 7 个关节角度
            clamp: 是否自动限制在限位范围内（默认 True）
        """
        if len(positions) != NUM_ARM_JOINTS:
            raise ValueError(f"positions 长度必须为 {NUM_ARM_JOINTS}")
        if clamp:
            self._positions = [self._clamp(i, p) for i, p in enumerate(positions)]
        else:
            self._positions = list(positions)
        self._robot._publish_joints()
    
    def set_joint(self, index: int, position: float, clamp: bool = True):
        """
        设置单个关节角度
        
        Args:
            index: 关节索引 (0-6)
            position: 角度（弧度）
            clamp: 是否自动限制在限位范围内（默认 True）
        """
        if not 0 <= index < NUM_ARM_JOINTS:
            raise ValueError(f"index 必须在 0-{NUM_ARM_JOINTS-1} 之间")
        if clamp:
            position = self._clamp(index, position)
        self._positions[index] = position
        self._robot._publish_joints()
    
    def home(self):
        """回到零位"""
        self.set_joints([0.0] * NUM_ARM_JOINTS)
    
    def __repr__(self):
        return f"Arm(side='{self._side}', positions={self._positions})"


# 动态生成各关节的 set_xxx 方法
for idx, name, doc in JOINT_DEFS:
    setattr(Arm, f"set_{name}", _make_joint_setter(idx, doc))
