"""CasADi + Pinocchio Mantis 机械臂逆运动学 (IK) 求解器。

此模块是 `mantis_casadi_node` 所需的**最小化**求解器接口。
它是从原始仓库中提取出来的，以便 phase1 可以在 `enable_ik:=true` 的情况下运行，
而无需保留不相关的遗留/调试文件。

约定接口:
- `MantisArmIK().get_initial_fk() -> (T_left, T_right)`: 获取初始的正向运动学 (FK) 位姿。
- `MantisArmIK().solve_ik(T_left, T_right) -> q` (numpy array): 求解给定目标位姿的关节角度。
- `MantisArmIK().get_joint_names() -> list[str]`: 获取机械臂关节名称列表，顺序与 `q` 一致。
"""
from __future__ import annotations

import os
from dataclasses import dataclass

import casadi
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
from ament_index_python.packages import get_package_share_directory

from .constants import LEFT_ARM_URDF_JOINTS, RIGHT_ARM_URDF_JOINTS

@dataclass
class _ReducedRobot:
    """简化的机器人模型数据类。"""
    model: pin.Model
    data: pin.Data


class MantisArmIK:
    """基于阻尼最小二乘法的 IK 求解器封装类。"""

    def __init__(self):
        """初始化 IK 求解器。
        
        主要步骤:
        1. 加载 Mantis 机器人的 URDF 模型。
        2. 构建简化模型 (Reduced Model): 锁定除了手臂以外的所有关节。
        3. 确定左右臂末端执行器 (EE) 的 Frame ID。
        4. 初始化 CasADi 优化问题。
        5. 设置坐标系校准参数。
        """
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        # 获取 URDF 文件路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_dir = os.path.join(current_dir, 'model')
        urdf_path = os.path.join(pkg_dir, 'urdf', 'mantis.urdf')

        # 使用 Pinocchio 加载机器人模型
        robot = pin.RobotWrapper.BuildFromURDF(urdf_path, [pkg_dir, os.path.dirname(urdf_path)])
        full_model = robot.model

        # 构建简化模型: 锁定非手臂关节
        # 白名单包含左右臂的所有关节名称
        whitelist = set(LEFT_ARM_URDF_JOINTS + RIGHT_ARM_URDF_JOINTS)
        joints_to_lock: list[int] = []
        for name in full_model.names:
            if name == 'universe':
                continue
            if name not in whitelist:
                # 获取不在白名单中的关节 ID 并添加到锁定列表
                joints_to_lock.append(full_model.getJointId(name))
        joints_to_lock = sorted(set(joints_to_lock))

        # 构建 Reduced Robot (仅保留手臂关节可动)
        reduced_robot = robot.buildReducedRobot(
            list_of_joints_to_lock=joints_to_lock,
            reference_configuration=pin.neutral(full_model),
        )
        self._robot = _ReducedRobot(model=reduced_robot.model, data=reduced_robot.data)

        # 确定末端执行器 (End-Effector) 的 Frame 名称
        # 使用之前代码中的腕部偏航关节作为 EE
        self._left_ee_frame = 'L_Wrist_Yaw_Joint'
        self._right_ee_frame = 'R_Wrist_Yaw_Joint'
        self._l_id = self._robot.model.getFrameId(self._left_ee_frame)
        self._r_id = self._robot.model.getFrameId(self._right_ee_frame)

        # 初始化关节配置 q 为中立位置 (neutral configuration)
        self._q = pin.neutral(self._robot.model)
        # 计算一次初始的正向运动学
        pin.framesForwardKinematics(self._robot.model, self._robot.data, self._q)

        # 提取关节名称列表 (排除 universe)
        self._joint_names = [
            n for n in list(self._robot.model.names)
            if n != 'universe'
        ]

        # 设置 CasADi 优化器
        self._setup_casadi()
        
        # --- 坐标系校准参数 ---
        # 用于对齐 IK 输入 (交互球) 和机器人实际 EE 坐标系
        # 交互球通常与机器人的视觉 Mesh 对齐，但实际 URDF EE 坐标系可能存在旋转 (例如 90 度)
        # 关系: T_ee = T_marker @ T_calib
        # 这将求解器期望的 T_ee 与输入 T_marker 对齐
        self._T_calib = np.eye(4)
        self._T_calib[:3, :3] = np.array([
            [ 0.0, -1.0,  0.0],  # 旋转矩阵第一行
            [ 1.0,  0.0,  0.0],  # 旋转矩阵第二行
            [ 0.0,  0.0,  1.0]   # 旋转矩阵第三行
        ])
        self._T_calib_inv = np.linalg.inv(self._T_calib)

        # --- 目标位姿状态维护 ---
        # 维护当前的 IK 目标点 (Marker Frame)
        # 初始时与当前 FK 一致
        self._target_T_l, self._target_T_r = self.get_initial_fk()

    def get_joint_names(self) -> list[str]:
        """获取关节名称列表。"""
        return list(self._joint_names)

    def set_config(self, q: np.ndarray):
        """更新内部机器人关节配置。
        
        注意：此方法仅更新用于 IK 求解热启动的关节角度 `_q`，
        **不会** 重置内部维护的目标位姿 `_target_T`。
        如果需要将目标位姿重置为当前关节角对应的位姿，请调用 `reset_targets()`。
        
        Args:
            q (np.ndarray): 关节角度数组。
        
        Raises:
            ValueError: 如果 q 的长度与模型关节数不匹配。
        """
        if len(q) != self._robot.model.nq:
             raise ValueError(f"预期关节数 {self._robot.model.nq}, 实际收到 {len(q)}")
        self._q = np.array(q)
        # 修改：set_config 不再自动重置目标点，以支持命令累积
        # self.reset_targets()

    def reset_targets(self):
        """重置目标位姿为当前关节配置对应的位姿 (Marker Frame)。"""
        self._target_T_l, self._target_T_r = self.get_initial_fk()

    def compute_fk(self, q: np.ndarray = None):
        """计算给定 (或当前) 配置的正向运动学 (FK)。
        
        Args:
            q (np.ndarray, optional): 关节角度。如果为 None，则使用内部存储的 self._q。
            
        Returns:
            (T_l, T_r): 左臂和右臂末端执行器 (EE) 的齐次变换矩阵 (4x4) [相对于 Robot Frame]。
        """
        if q is not None:
            q_eval = np.array(q)
        else:
            q_eval = self._q
        
        # 使用 Pinocchio 计算 FK
        pin.framesForwardKinematics(self._robot.model, self._robot.data, q_eval)
        # 获取左右臂 EE 的位姿
        T_l = self._robot.data.oMf[self._l_id].homogeneous
        T_r = self._robot.data.oMf[self._r_id].homogeneous
        return T_l, T_r

    def get_initial_fk(self):
        """获取初始的 FK 位姿 (经过校准)。
        
        此方法用于在启动时获取 IK Marker 的初始位置。
        它返回的是 **MARKER Frame** 的位姿，而不是原始的 Robot EE Frame。
        转换关系: T_marker = T_ee @ T_calib_inv
        
        Returns:
            (T_l_marker, T_r_marker): 左右臂 Marker 的初始齐次变换矩阵。
        """
        # 计算原始 FK (Robot EE Frame)
        T_l, T_r = self.compute_fk()
        # 应用校准逆变换，转换到 Marker Frame
        return T_l @ self._T_calib_inv, T_r @ self._T_calib_inv

    def _setup_casadi(self):
        """设置 CasADi 优化问题。
        
        构建一个非线性规划 (NLP) 问题来求解 IK。
        目标函数包含:
        1. 位置误差 (Position Error)
        2. 旋转误差 (Rotation Error)
        3. 正则化项 (Regularization): 偏向于 0 位姿
        4. 平滑项 (Smoothness): 偏向于上一帧的解
        """
        # 创建 CasADi 版本的 Pinocchio 模型
        self._cmodel = cpin.Model(self._robot.model)
        self._cdata = self._cmodel.createData()

        # 定义符号变量
        self._cq = casadi.SX.sym('q', self._robot.model.nq, 1)      # 关节角度变量
        self._cTf_l = casadi.SX.sym('tf_l', 4, 4)                   # 左臂目标位姿
        self._cTf_r = casadi.SX.sym('tf_r', 4, 4)                   # 右臂目标位姿

        # 符号计算 FK
        cpin.framesForwardKinematics(self._cmodel, self._cdata, self._cq)

        # 计算误差
        # 位置误差: translation_current - translation_target
        pos_err_l = self._cdata.oMf[self._l_id].translation - self._cTf_l[:3, 3]
        pos_err_r = self._cdata.oMf[self._r_id].translation - self._cTf_r[:3, 3]
        
        # 旋转误差: Log3(R_current * R_target^T) -> 旋转向量误差
        rot_err_l = cpin.log3(self._cdata.oMf[self._l_id].rotation @ self._cTf_l[:3, :3].T)
        rot_err_r = cpin.log3(self._cdata.oMf[self._r_id].rotation @ self._cTf_r[:3, :3].T)

        # 定义误差函数
        self._error_func = casadi.Function(
            'error_func',
            [self._cq, self._cTf_l, self._cTf_r],
            [casadi.vertcat(pos_err_l, pos_err_r, rot_err_l, rot_err_r)],
        )

        # 设置优化器 (Opti stack)
        self._opti = casadi.Opti()
        self._var_q = self._opti.variable(self._robot.model.nq)         # 优化变量: q
        self._var_q_last = self._opti.parameter(self._robot.model.nq)   # 参数: 上一帧 q
        self._param_tf_l = self._opti.parameter(4, 4)                   # 参数: 左臂目标 T
        self._param_tf_r = self._opti.parameter(4, 4)                   # 参数: 右臂目标 T

        # 构建代价函数 (Cost Function)
        errors = self._error_func(self._var_q, self._param_tf_l, self._param_tf_r)
        cost = (
            1000.0 * casadi.sumsqr(errors[:6])             # 位置权重
            + 500.0 * casadi.sumsqr(errors[6:])            # 旋转权重
            + 0.001 * casadi.sumsqr(self._var_q)           # 正则化权重 (避免奇异值)
            + 0.05 * casadi.sumsqr(self._var_q - self._var_q_last) # 平滑权重 (最小化移动量)
        )

        self._opti.minimize(cost)
        
        # 设置关节限位约束
        self._opti.subject_to(
            self._opti.bounded(
                self._robot.model.lowerPositionLimit,
                self._var_q,
                self._robot.model.upperPositionLimit,
            )
        )

        # 配置 IPOPT 求解器选项
        opts = {'ipopt': {'print_level': 0, 'max_iter': 30, 'tol': 1e-4}, 'print_time': False}
        self._opti.solver('ipopt', opts)

    def get_current_marker_pose(self):
        """获取当前 Marker Frame 的位姿 (基于 FK)。
        
        相当于 get_initial_fk，但明确表示获取当前状态。
        
        Returns:
            (T_l_marker, T_r_marker): 左右臂 Marker 的当前齐次变换矩阵。
        """
        return self.get_initial_fk()

    def get_target_pose(self):
        """获取当前维护的目标位姿 (Marker Frame)。
        
        Returns:
            (T_l_target, T_r_target): 左右臂目标点的齐次变换矩阵。
        """
        return self._target_T_l.copy(), self._target_T_r.copy()

    def _apply_delta(self, T: np.ndarray, delta: np.ndarray) -> np.ndarray:
        """应用增量到位姿矩阵。
        
        Args:
            T (np.ndarray): 原始位姿矩阵 (4x4)。
            delta (np.ndarray): 增量 [dx, dy, dz, dr, dp, dy]。
                                translation 为全局增量，rotation 为局部增量。
        
        Returns:
            np.ndarray: 更新后的位姿矩阵。
        """
        T_new = T.copy()
        # 全局平移: pos_new = pos_old + delta_pos
        T_new[:3, 3] += delta[:3]
        
        # 局部旋转: R_new = R_old @ R_delta
        # 注意: Pinocchio rpyToMatrix 通常对应 Euler RPY
        R_delta = pin.utils.rpyToMatrix(delta[3], delta[4], delta[5])
        T_new[:3, :3] = T_new[:3, :3] @ R_delta
        
        return T_new

    def solve_ik_abs(self, T_left: np.ndarray, T_right: np.ndarray) -> np.ndarray:
        """绝对坐标 IK 求解 (更新内部目标点)。
        
        Args:
            T_left (np.ndarray): 左臂绝对目标位姿。
            T_right (np.ndarray): 右臂绝对目标位姿。
            
        Returns:
            np.ndarray: 关节角度 q。
        """
        self._target_T_l = T_left
        self._target_T_r = T_right
        return self.solve_ik(self._target_T_l, self._target_T_r)

    def solve_ik_rel(self, delta_left: np.ndarray, delta_right: np.ndarray) -> np.ndarray:
        """增量 IK 求解 (更新内部目标点)。
        
        基于当前维护的目标位姿叠加增量后求解 IK。
        
        Args:
            delta_left (np.ndarray): 左臂增量 [dx, dy, dz, droll, dpitch, dyaw]。
            delta_right (np.ndarray): 右臂增量 [dx, dy, dz, droll, dpitch, dyaw]。
            
        Returns:
            np.ndarray: 关节角度 q。
        """
        # 基于内部维护的目标点应用增量
        self._target_T_l = self._apply_delta(self._target_T_l, delta_left)
        self._target_T_r = self._apply_delta(self._target_T_r, delta_right)
        
        # 调用绝对 IK 求解
        return self.solve_ik(self._target_T_l, self._target_T_r)

    def solve_ik(self, T_left: np.ndarray, T_right: np.ndarray) -> np.ndarray:
        """求解逆运动学 (IK)。
        
        Args:
            T_left (np.ndarray): 左臂目标位姿 (Marker Frame)。
            T_right (np.ndarray): 右臂目标位姿 (Marker Frame)。
            
        Returns:
            np.ndarray: 求解得到的关节角度数组 (q)。
        """
        # 设置初值 (热启动)
        self._opti.set_initial(self._var_q, self._q)
        self._opti.set_value(self._var_q_last, self._q)
        
        # 应用校准: 将 Marker Frame 目标转换为 Robot EE Frame 目标
        # T_ee = T_marker @ T_calib
        T_l_ee = T_left @ self._T_calib
        T_r_ee = T_right @ self._T_calib
        
        # 设置参数值
        self._opti.set_value(self._param_tf_l, T_l_ee)
        self._opti.set_value(self._param_tf_r, T_r_ee)

        try:
            # 求解
            sol = self._opti.solve()
            q_sol = sol.value(self._var_q)
        except Exception:
            # 如果求解失败，使用当前的调试值 (可能未收敛)
            # print(f"IK Solve Failed: {e}")
            q_sol = self._opti.debug.value(self._var_q)

        # 更新内部状态并返回结果
        q_sol = np.array(q_sol).reshape((-1,))
        self._q = q_sol
        return q_sol
