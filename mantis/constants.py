"""
常量定义
"""

# 左臂关节名称（7个）
LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_yaw_joint",
    "left_shoulder_roll_joint",
    "left_elbow_pitch_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]

# 右臂关节名称（7个）
RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_yaw_joint",
    "right_shoulder_roll_joint",
    "right_elbow_pitch_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

# 全部 14 个关节
JOINT_NAMES = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

# 关节数量
NUM_ARM_JOINTS = 7
NUM_TOTAL_JOINTS = 14

# ==================== 关节限位（弧度） ====================
# 来源: mantis.urdf

# 左臂关节限位: (lower, upper)
LEFT_ARM_LIMITS = [
    (-2.61, 0.78),   # shoulder_pitch: L_Shoulder_Pitch_Joint
    (0.08, 1.04),    # shoulder_yaw:   L_Shoulder_Yaw_Joint
    (-1.57, 1.57),   # shoulder_roll:  L_Shoulder_Roll_Joint
    (-0.78, 1.57),   # elbow_pitch:    L_Elbow_Pitch_Joint
    (-1.57, 1.57),   # wrist_roll:     L_Wrist_Roll_Joint
    (-0.52, 0.52),   # wrist_pitch:    L_Wrist_Pitch_Joint
    (-1.57, 1.57),   # wrist_yaw:      L_Wrist_Yaw_Joint
]

# 右臂关节限位: (lower, upper)
RIGHT_ARM_LIMITS = [
    (-2.61, 0.78),   # shoulder_pitch: R_Shoulder_Pitch_Joint
    (-1.04, -0.08),  # shoulder_yaw:   R_Shoulder_Yaw_Joint (注意方向相反)
    (-1.57, 1.57),   # shoulder_roll:  R_Shoulder_Roll_Joint
    (-0.78, 1.57),   # elbow_pitch:    R_Elbow_Pitch_Joint
    (-1.57, 1.57),   # wrist_roll:     R_Wrist_Roll_Joint
    (-0.52, 0.52),   # wrist_pitch:    R_Wrist_Pitch_Joint
    (-1.57, 1.57),   # wrist_yaw:      R_Wrist_Yaw_Joint
]

# 头部限位
HEAD_LIMITS = {
    "pitch": (-0.7, 0.2),   # Head_Joint: 俯仰
    "yaw": (-1.57, 1.57),   # Neck_Joint: 偏航
}

# 夹爪限位 (0.0~1.0 归一化)
GRIPPER_LIMITS = (0.0, 1.0)

# Zenoh 话题名（对应 ROS2 话题）
class Topics:
    JOINT_CMD = "Teleop/joint_angle_solution/smooth"
    GRIPPER = "Teleop/gripper_pos"
    HEAD = "Teleop/head_pose"
    CHASSIS = "Teleop/cmd_vel"
    PELVIS = "Teleop/pelvis_speed"
    JOINT_FEEDBACK = "joint_states_fdb"
    FORCE_FEEDBACK = "force_feedback"
