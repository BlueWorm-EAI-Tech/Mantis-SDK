"""
Mantis 机器人主控制类
======================

提供 Mantis 机器人的统一控制接口，支持实机控制和仿真预览两种模式。

通信协议:
    使用 Zenoh 协议进行通信，无需安装 ROS2。

模式:
    - **实机模式**: 连接真实机器人进行控制
    - **仿真模式**: 在 RViz 中预览机器人动作（带平滑）

Example:
    .. code-block:: python
    
        from mantis import Mantis
        
        # 实机控制
        with Mantis(ip="192.168.1.100") as robot:
            robot.left_arm.set_shoulder_pitch(-0.5)
            robot.head.look_left()
        
        # 仿真预览
        with Mantis(sim=True) as robot:
            robot.left_arm.set_joints([0.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0])
"""

from typing import Optional, Callable
import time
import json
import threading

try:
    import zenoh
except ImportError:
    raise ImportError("请安装 zenoh: pip install eclipse-zenoh")

from .arm import Arm
from .gripper import Gripper
from .head import Head
from .chassis import Chassis
from .cdr import CDREncoder, CDRDecoder
from .constants import (
    Topics, JOINT_NAMES,
    SERIAL_TO_URDF_MAP, JOINT_DIRECTION_MAP,
    ALL_URDF_JOINTS
)


class Mantis:
    """Mantis 机器人主控制类。
    
    提供对 Mantis 机器人的统一控制接口，包括双臂、夹爪、头部和底盘。
    支持实机控制和仿真预览两种模式。
    
    Attributes:
        left_arm (Arm): 左臂控制器
        right_arm (Arm): 右臂控制器
        left_gripper (Gripper): 左夹爪控制器
        right_gripper (Gripper): 右夹爪控制器
        head (Head): 头部控制器
        chassis (Chassis): 底盘控制器
        is_connected (bool): 是否已连接
        is_sim_mode (bool): 是否为仿真模式
    
    Example:
        使用上下文管理器（推荐）::
        
            with Mantis(ip="192.168.1.100") as robot:
                robot.left_arm.set_shoulder_pitch(-0.5)
                robot.head.look_left()
        
        手动管理连接::
        
            robot = Mantis(sim=True)
            robot.connect()
            robot.left_arm.home()
            robot.disconnect()
    
    Note:
        仿真模式需要先启动仿真环境::
        
            ros2 launch bw_sim2real sdk_sim.launch.py
            zenoh-bridge-ros2dds -d 99
    """
    
    #: 默认 Zenoh 端口
    DEFAULT_PORT = 7447
    
    def __init__(self, ip: Optional[str] = None, port: int = None, sim: bool = False):
        """初始化 Mantis 机器人。
        
        Args:
            ip: 机器人 IP 地址，例如 "192.168.1.100"。
                如果为 None，则使用 Zenoh 自动发现（需在同一局域网）。
            port: Zenoh 端口，默认 7447。
            sim: 仿真模式开关。
                - False（默认）: 实机控制模式
                - True: 仿真预览模式，在 RViz 中显示
        
        Example:
            .. code-block:: python
            
                # 指定 IP 连接
                robot = Mantis(ip="192.168.1.100")
                
                # 自动发现（同一局域网）
                robot = Mantis()
                
                # 仿真模式
                robot = Mantis(sim=True)
        """
        if ip:
            p = port or self.DEFAULT_PORT
            self._router = f"tcp/{ip}:{p}"
        else:
            self._router = None
        
        self._sim_mode = sim
        self._session: Optional[zenoh.Session] = None
        self._publishers = {}
        self._subscribers = {}
        self._connected = False
        
        # 创建子模块
        self._left_arm = Arm(self, "left")
        self._right_arm = Arm(self, "right")
        self._left_gripper = Gripper(self, "left")
        self._right_gripper = Gripper(self, "right")
        self._head = Head(self)
        self._chassis = Chassis(self)
        
        # 反馈数据
        self._feedback_callback: Optional[Callable] = None
        
        # 仿真模式：存储所有关节状态（用于完整发布）
        self._sim_joint_states = {name: 0.0 for name in ALL_URDF_JOINTS}
        
        # 仿真平滑参数
        self._sim_target_states = {name: 0.0 for name in ALL_URDF_JOINTS}  # 目标位置
        self._sim_current_states = {name: 0.0 for name in ALL_URDF_JOINTS}  # 当前平滑位置
        self._sim_alpha = 0.1  # EMA 平滑因子，越小越平滑
        self._sim_frequency = 100.0  # 发布频率 Hz
        self._sim_thread: Optional[threading.Thread] = None
        self._sim_running = False
    
    # ==================== 属性访问 ====================
    
    @property
    def left_arm(self) -> Arm:
        """左臂控制器。
        
        Returns:
            Arm: 左臂 7 自由度控制器
        """
        return self._left_arm
    
    @property
    def right_arm(self) -> Arm:
        """右臂控制器。
        
        Returns:
            Arm: 右臂 7 自由度控制器
        """
        return self._right_arm
    
    @property
    def left_gripper(self) -> Gripper:
        """左夹爪控制器。
        
        Returns:
            Gripper: 左夹爪控制器
        """
        return self._left_gripper
    
    @property
    def right_gripper(self) -> Gripper:
        """右夹爪控制器。
        
        Returns:
            Gripper: 右夹爪控制器
        """
        return self._right_gripper
    
    @property
    def head(self) -> Head:
        """头部控制器。
        
        Returns:
            Head: 头部 2 自由度控制器
        """
        return self._head
    
    @property
    def chassis(self) -> Chassis:
        """底盘控制器。
        
        Returns:
            Chassis: 全向底盘控制器
        """
        return self._chassis
    
    @property
    def is_connected(self) -> bool:
        """是否已连接到机器人。
        
        Returns:
            bool: 连接状态
        """
        return self._connected
    
    @property
    def is_sim_mode(self) -> bool:
        """是否为仿真预览模式。
        
        Returns:
            bool: True 为仿真模式，False 为实机模式
        """
        return self._sim_mode
    
    def set_smoothing(self, alpha: float = 0.1, rate: float = 100.0):
        """设置仿真模式的运动平滑参数。
        
        使用 EMA（指数移动平均）算法进行平滑：
        ``current = current + alpha * (target - current)``
        
        Args:
            alpha: 平滑系数 (0.01-1.0)。
                - 0.05: 非常平滑，响应慢
                - 0.1: 平滑（默认）
                - 0.3: 较快响应
                - 1.0: 无平滑，立即到达目标
            rate: 发布频率 (Hz)，默认 100Hz。
        
        Example:
            .. code-block:: python
            
                robot = Mantis(sim=True)
                robot.set_smoothing(alpha=0.2)  # 更快响应
                robot.connect()
        """
        self._sim_alpha = max(0.01, min(1.0, alpha))
        self._sim_frequency = max(10.0, min(500.0, rate))
    
    # ==================== 连接管理 ====================
    
    def connect(self, timeout: float = 5.0, verify: bool = True) -> bool:
        """连接到机器人。
        
        建立与机器人的 Zenoh 通信连接。实机模式下会验证机器人是否在线，
        仿真模式下跳过验证直接连接。
        
        Args:
            timeout: 连接超时时间（秒），默认 5.0
            verify: 是否验证机器人在线，默认 True。
                仿真模式下此参数被忽略。
            
        Returns:
            bool: 连接是否成功
        
        Raises:
            无异常抛出，失败时返回 False 并打印错误信息。
        
        Example:
            .. code-block:: python
            
                robot = Mantis(ip="192.168.1.100")
                if robot.connect():
                    print("连接成功")
                else:
                    print("连接失败")
        """
        if self._connected:
            return True
        
        # 仿真模式不验证
        if self._sim_mode:
            verify = False
        
        try:
            config = zenoh.Config()
            if self._router:
                config.insert_json5("connect/endpoints", f'["{self._router}"]')
            
            self._session = zenoh.open(config)
            
            # 创建发布者
            if self._sim_mode:
                # 仿真模式：发布到 /joint_states（纯仿真，绕过 input_router）
                self._publishers['sim_joints'] = self._session.declare_publisher(Topics.SIM_JOINT_STATES)
            else:
                # 实机模式：发布到控制话题
                self._publishers['joints'] = self._session.declare_publisher(Topics.JOINT_CMD)
                self._publishers['gripper'] = self._session.declare_publisher(Topics.GRIPPER)
                self._publishers['head'] = self._session.declare_publisher(Topics.HEAD)
                self._publishers['chassis'] = self._session.declare_publisher(Topics.CHASSIS)
                self._publishers['pelvis'] = self._session.declare_publisher(Topics.PELVIS)
            
            # 验证机器人是否在线
            if verify:
                import time
                received = []
                
                def _check_callback(sample):
                    received.append(True)
                
                # 订阅反馈话题检测
                sub = self._session.declare_subscriber(Topics.JOINT_FEEDBACK, _check_callback)
                
                # 等待消息
                start = time.time()
                while time.time() - start < timeout:
                    if received:
                        break
                    time.sleep(0.1)
                
                sub.undeclare()
                
                if not received:
                    self._session.close()
                    self._session = None
                    self._publishers.clear()
                    target = self._router if self._router else "本机 (自动发现)"
                    print(f"❌ 连接超时: 未检测到机器人 ({target})")
                    print("   请检查:")
                    print("   1) zenoh-bridge-ros2dds 是否已启动")
                    print("   2) ROS2 节点是否在发布 /joint_states_fdb")
                    print("   3) Domain ID 是否一致")
                    return False
            
            self._connected = True
            mode_str = "仿真预览" if self._sim_mode else "实机控制"
            print(f"✅ 已连接到 Mantis 机器人 ({mode_str})")
            
            # 仿真模式：启动平滑发布线程
            if self._sim_mode:
                self._start_sim_smooth_thread()
            
            return True
            
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开与机器人的连接。
        
        停止所有运动，关闭 Zenoh 会话，释放资源。
        如果正在运行平滑线程，也会一并停止。
        
        Note:
            使用上下文管理器时会自动调用此方法。
        """
        if self._session:
            # 停止平滑线程
            if self._sim_thread is not None:
                self._sim_running = False
                self._sim_thread.join(timeout=1.0)
                self._sim_thread = None
            
            # 停止运动
            self._chassis.stop()
            
            for pub in self._publishers.values():
                pub.undeclare()
            for sub in self._subscribers.values():
                sub.undeclare()
            
            self._session.close()
            self._session = None
            self._publishers.clear()
            self._subscribers.clear()
            self._connected = False
            print("✅ 已断开连接")
    
    # ==================== 仿真平滑处理 ====================
    
    def _start_sim_smooth_thread(self):
        """启动仿真平滑发布线程。
        
        在仿真模式连接成功后自动调用，启动后台线程进行关节位置平滑插值。
        """
        if self._sim_thread is not None:
            return
        
        self._sim_running = True
        self._sim_thread = threading.Thread(target=self._sim_smooth_loop, daemon=True)
        self._sim_thread.start()
    
    def _sim_smooth_loop(self):
        """平滑插值循环。
        
        使用 EMA 算法逐渐将当前关节状态逼近目标状态，
        并以固定频率发布到仿真话题。
        """
        interval = 1.0 / self._sim_frequency
        
        while self._sim_running:
            # EMA 平滑：current = current + alpha * (target - current)
            for name in ALL_URDF_JOINTS:
                current = self._sim_current_states[name]
                target = self._sim_target_states[name]
                self._sim_current_states[name] = current + self._sim_alpha * (target - current)
            
            # 发布平滑后的状态
            self._publish_sim_state()
            
            time.sleep(interval)
    
    # ==================== 内部发布方法 ====================
    
    def _check_connection(self):
        """检查连接状态。
        
        Raises:
            RuntimeError: 如果未连接到机器人
        """
        if not self._connected:
            raise RuntimeError("未连接到机器人，请先调用 connect()")
    
    def _publish_joints(self):
        """发布手臂关节角度。
        
        将左右臂的关节位置发送到机器人或仿真环境。
        """
        self._check_connection()
        
        # 合并左右臂位置
        positions = self._left_arm._positions + self._right_arm._positions
        
        if self._sim_mode:
            # 仿真模式：更新目标关节状态（由平滑线程逐渐逼近并发布）
            for i, (serial_name, pos) in enumerate(zip(JOINT_NAMES, positions)):
                urdf_name = SERIAL_TO_URDF_MAP.get(serial_name)
                if urdf_name:
                    direction = JOINT_DIRECTION_MAP.get(serial_name, 1)
                    self._sim_target_states[urdf_name] = pos * direction
            # 不再手动调用 _publish_sim_state，由平滑线程处理
        else:
            # 实机模式：直接发送
            payload = CDREncoder.encode_joint_state(
                names=JOINT_NAMES,
                positions=positions
            )
            self._publishers['joints'].put(payload)
    
    def _publish_grippers(self):
        """发布夹爪位置。
        
        将左右夹爪的位置发送到机器人或仿真环境。
        """
        self._check_connection()
        
        if self._sim_mode:
            # 仿真模式：更新目标夹爪关节（由平滑线程逐渐逼近并发布）
            # 夹爪是 prismatic 类型，范围 0.0-0.04 米
            # 用户输入是归一化值 0.0-1.0，需要转换为实际位置
            # 左夹爪：L_Hand_R_Joint 和 L_Hand_L_Joint（同向运动，都是正值）
            # 右夹爪：R_Hand_R_Joint 和 R_Hand_L_Joint（同向运动，都是正值）
            left_pos = self._left_gripper._position * 0.04  # 归一化 -> 实际位置
            right_pos = self._right_gripper._position * 0.04
            self._sim_target_states["L_Hand_R_Joint"] = left_pos
            self._sim_target_states["L_Hand_L_Joint"] = left_pos
            self._sim_target_states["R_Hand_R_Joint"] = right_pos
            self._sim_target_states["R_Hand_L_Joint"] = right_pos
            # 不再手动调用 _publish_sim_state，由平滑线程处理
        else:
            payload = CDREncoder.encode_joint_state(
                names=["left_gripper", "right_gripper"],
                positions=[self._left_gripper._position, self._right_gripper._position]
            )
            self._publishers['gripper'].put(payload)
    
    def _publish_sim_state(self):
        """发布仿真关节状态。
        
        仿真模式专用：将完整的关节状态以 JSON 格式发布，
        由 sdk_bridge_node 转发到 ROS2 的 /joint_states 话题。
        """
        if not self._sim_mode:
            return
        
        # 构建完整的关节状态消息（JSON 格式），使用平滑后的当前位置
        names = ALL_URDF_JOINTS
        positions = [float(self._sim_current_states[name]) for name in names]
        
        msg = {
            'name': names,
            'position': positions,
            'velocity': [],
            'effort': []
        }
        
        payload = json.dumps(msg).encode('utf-8')
        self._publishers['sim_joints'].put(payload)
    
    def _publish_head(self):
        """发布头部姿态。
        
        将头部的俯仰和偏航角度发送到机器人或仿真环境。
        """
        self._check_connection()
        
        if self._sim_mode:
            # 仿真模式：更新目标头部关节（由平滑线程逐渐逼近并发布）
            self._sim_target_states["Neck_Joint"] = self._head._yaw
            self._sim_target_states["Head_Joint"] = self._head._pitch
            # 不再手动调用 _publish_sim_state，由平滑线程处理
        else:
            payload = CDREncoder.encode_joint_state(
                names=["head_pitch", "head_yaw"],
                positions=[self._head._pitch, self._head._yaw]
            )
            self._publishers['head'].put(payload)
    
    def _publish_chassis(self):
        """发布底盘速度。
        
        将底盘的线速度和角速度发送到机器人。
        
        Note:
            仿真模式下底盘控制暂不支持预览。
        """
        self._check_connection()
        
        if self._sim_mode:
            # 仿真模式暂不支持底盘预览
            return
        
        payload = CDREncoder.encode_twist(
            linear_x=self._chassis._vx,
            linear_y=self._chassis._vy,
            linear_z=0.0,
            angular_x=0.0,
            angular_y=0.0,
            angular_z=self._chassis._omega
        )
        self._publishers['chassis'].put(payload)
    
    # ==================== 便捷方法 ====================
    
    def home(self):
        """所有关节回到零位。
        
        将双臂、头部回零，夹爪闭合。
        """
        self._left_arm.home()
        self._right_arm.home()
        self._head.center()
        self._left_gripper.close()
        self._right_gripper.close()
    
    def stop(self):
        """停止所有运动。
        
        立即停止底盘运动。
        """
        if self._connected:
            self._chassis.stop()
    
    def subscribe_feedback(self, callback: Callable):
        """订阅关节状态反馈。
        
        注册回调函数，接收机器人的关节位置反馈。
        
        Args:
            callback: 回调函数，签名为 ``callback(data: dict)``。
                data 包含 'name' (关节名列表) 和 'position' (位置列表) 等字段。
        
        Example:
            .. code-block:: python
            
                def on_feedback(data):
                    print(f"关节: {data['name']}")
                    print(f"位置: {data['position']}")
                
                robot.subscribe_feedback(on_feedback)
        """
        self._check_connection()
        self._feedback_callback = callback
        
        def _on_feedback(sample):
            try:
                data = CDRDecoder.decode_joint_state(sample.payload.to_bytes())
                if self._feedback_callback:
                    self._feedback_callback(data)
            except Exception as e:
                print(f"⚠️ 解析反馈失败: {e}")
        
        self._subscribers['feedback'] = self._session.declare_subscriber(
            Topics.JOINT_FEEDBACK,
            _on_feedback
        )
        print("✅ 已订阅关节反馈")
    
    # ==================== 上下文管理 ====================
    
    def __enter__(self) -> "Mantis":
        """进入上下文管理器。
        
        自动调用 connect() 建立连接。
        
        Returns:
            Mantis: 机器人实例
        
        Raises:
            ConnectionError: 如果连接失败
        """
        if not self.connect():
            raise ConnectionError("无法连接到机器人")
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """退出上下文管理器。
        
        自动停止运动并断开连接。
        """
        if self._connected:
            self.stop()
            self.disconnect()
    
    def __repr__(self) -> str:
        """返回机器人的字符串表示。"""
        status = "已连接" if self._connected else "未连接"
        mode = "仿真" if self._sim_mode else "实机"
        return f"Mantis(status='{status}', mode='{mode}')"
