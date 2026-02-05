"""
常量定义
========

定义 Mantis 机器人的关节名称、限位、话题等常量。

Sections:
    - 关节名称：左臂、右臂的关节命名
    - 关节限位：各关节的安全运动范围
    - URDF 映射：Serial 名称与 URDF 名称的映射
    - Zenoh 话题：通信话题定义
"""

# ==================== 关节名称 ====================

#: 左臂关节名称（7个），按索引顺序
LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_yaw_joint",
    "left_shoulder_roll_joint",
    "left_elbow_pitch_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]

#: 右臂关节名称（7个），按索引顺序
RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_yaw_joint",
    "right_shoulder_roll_joint",
    "right_elbow_pitch_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

#: 全部 14 个手臂关节名称
JOINT_NAMES = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

#: 单臂关节数量
NUM_ARM_JOINTS = 7

#: 双臂关节总数
NUM_TOTAL_JOINTS = 14

# ==================== 关节限位（弧度） ====================
# 数据来源: mantis.urdf

#: 左臂关节限位 (lower, upper)，单位：弧度
#: 
#: 索引对应关系：
#:   0. shoulder_pitch: -2.61 ~ 0.78
#:   1. shoulder_yaw:   -0.213 ~ 2.029
#:   2. shoulder_roll:  -1.57 ~ 1.57
#:   3. elbow_pitch:    -0.78 ~ 1.57
#:   4. wrist_roll:     -1.57 ~ 1.57
#:   5. wrist_pitch:    -0.52 ~ 0.52
#:   6. wrist_yaw:      -1.57 ~ 1.57
LEFT_ARM_LIMITS = [
    (-2.61, 1.5),   # shoulder_pitch: L_Shoulder_Pitch_Joint
    (-0.213, 2.029), # shoulder_yaw:   L_Shoulder_Yaw_Joint
    (-1.57, 1.57),   # shoulder_roll:  L_Shoulder_Roll_Joint
    (-0.78, 1.57),   # elbow_pitch:    L_Elbow_Pitch_Joint
    (-1.57, 1.57),   # wrist_roll:     L_Wrist_Roll_Joint
    (-0.52, 0.52),   # wrist_pitch:    L_Wrist_Pitch_Joint
    (-1.57, 1.57),   # wrist_yaw:      L_Wrist_Yaw_Joint
]

#: 右臂关节限位 (lower, upper)，单位：弧度
#:
#: Note:
#:   右臂 shoulder_yaw 限位与左臂相同（URDF 中轴方向已处理）
RIGHT_ARM_LIMITS = [
    (-2.61, 1.5),   # shoulder_pitch: R_Shoulder_Pitch_Joint
    (-0.213, 2.029), # shoulder_yaw:   R_Shoulder_Yaw_Joint
    (-1.57, 1.57),   # shoulder_roll:  R_Shoulder_Roll_Joint
    (-0.78, 1.57),   # elbow_pitch:    R_Elbow_Pitch_Joint
    (-1.57, 1.57),   # wrist_roll:     R_Wrist_Roll_Joint
    (-0.52, 0.52),   # wrist_pitch:    R_Wrist_Pitch_Joint
    (-1.57, 1.57),   # wrist_yaw:      R_Wrist_Yaw_Joint
]

#: 头部限位，单位：弧度
HEAD_LIMITS = {
    "pitch": (-0.7, 0.9),   # Head_Joint: 俯仰（正值低头）
    "yaw": (-1.57, 1.57),   # Neck_Joint: 偏航（正值左转）
}

#: 夹爪限位（归一化值）
GRIPPER_LIMITS = (0.0, 1.0)

# ==================== URDF 关节名称（仿真用） ====================

#: URDF 左臂关节名称
LEFT_ARM_URDF_JOINTS = [
    "L_Shoulder_Pitch_Joint",
    "L_Shoulder_Yaw_Joint",
    "L_Shoulder_Roll_Joint",
    "L_Elbow_Pitch_Joint",
    "L_Wrist_Roll_Joint",
    "L_Wrist_Pitch_Joint",
    "L_Wrist_Yaw_Joint",
]

#: URDF 右臂关节名称
RIGHT_ARM_URDF_JOINTS = [
    "R_Shoulder_Pitch_Joint",
    "R_Shoulder_Yaw_Joint",
    "R_Shoulder_Roll_Joint",
    "R_Elbow_Pitch_Joint",
    "R_Wrist_Roll_Joint",
    "R_Wrist_Pitch_Joint",
    "R_Wrist_Yaw_Joint",
]

#: 全部 URDF 手臂关节名称（14个）
URDF_ARM_JOINT_NAMES = LEFT_ARM_URDF_JOINTS + RIGHT_ARM_URDF_JOINTS

#: URDF 完整关节列表（26个，用于仿真模式一次性发布所有关节）
#:
#: 包含：腰部、双臂、双手、头部、耳朵、轮子
ALL_URDF_JOINTS = [
    # 腰部
    "Waist_Joint",
    # 左臂 (7)
    "L_Shoulder_Pitch_Joint",
    "L_Shoulder_Yaw_Joint",
    "L_Shoulder_Roll_Joint",
    "L_Elbow_Pitch_Joint",
    "L_Wrist_Roll_Joint",
    "L_Wrist_Pitch_Joint",
    "L_Wrist_Yaw_Joint",
    # 左手 (2)
    "L_Hand_R_Joint",
    "L_Hand_L_Joint",
    # 右臂 (7)
    "R_Shoulder_Pitch_Joint",
    "R_Shoulder_Yaw_Joint",
    "R_Shoulder_Roll_Joint",
    "R_Elbow_Pitch_Joint",
    "R_Wrist_Roll_Joint",
    "R_Wrist_Pitch_Joint",
    "R_Wrist_Yaw_Joint",
    # 右手 (2)
    "R_Hand_R_Joint",
    "R_Hand_L_Joint",
    # 头部 (2)
    "Neck_Joint",
    "Head_Joint",
    # 耳朵 (2)
    "L_Ear_Joint",
    "R_Ear_Joint",
    # 轮子 (3)
    "Wheel_Left_Joint",
    "Wheel_Right_Joint",
    "Wheel_Back_Joint",
]

#: Serial 名称 → URDF 名称映射
SERIAL_TO_URDF_MAP = dict(zip(JOINT_NAMES, URDF_ARM_JOINT_NAMES))

#: 方向修正映射：Serial 名称 → 方向系数
#:
#: SDK 端不做方向修正，方向修正在接收端（sdk_bridge）处理
#: 这样 RViz 和实机可以分别应用不同的方向系数
JOINT_DIRECTION_MAP = {
    # 左臂
    "left_shoulder_pitch_joint": 1,
    "left_shoulder_yaw_joint": 1,
    "left_shoulder_roll_joint": 1,
    "left_elbow_pitch_joint": 1,
    "left_wrist_roll_joint": 1,
    "left_wrist_pitch_joint": 1,
    "left_wrist_yaw_joint": 1,
    # 右臂
    "right_shoulder_pitch_joint": 1,
    "right_shoulder_yaw_joint": 1,
    "right_shoulder_roll_joint": 1,
    "right_elbow_pitch_joint": 1,
    "right_wrist_roll_joint": 1,
    "right_wrist_pitch_joint": 1,
    "right_wrist_yaw_joint": 1,
}

# ==================== Zenoh 话题 ====================

class Topics:
    """Zenoh 话题定义。
    
    所有话题使用纯 Zenoh 通信，通过 Python 桥接节点转发到 ROS2。
    
    Attributes:
        SDK_JOINT_STATES: 关节状态话题（SDK→Python桥接→ROS2）
        SDK_CHASSIS: 底盘速度话题（SDK→Python桥接→ROS2）
        FORCE_FEEDBACK: 力反馈话题（ROS2→Python桥接→SDK）
    """
    
    #: 关节状态（SDK 发布，Python 桥接节点订阅）
    SDK_JOINT_STATES = "sdk/joint_states"
    
    #: 底盘速度命令（SDK 发布，Python 桥接节点订阅）
    SDK_CHASSIS = "sdk/chassis"
    
    #: 系统状态反馈（Python 桥接节点发布，SDK 订阅）
    SYSTEM_STATUS = "sdk/system_status"
    
    #: 力反馈（Python 桥接节点发布，SDK 订阅）
    FORCE_FEEDBACK = "sdk/force_feedback"
