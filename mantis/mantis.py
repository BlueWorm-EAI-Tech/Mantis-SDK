"""
Mantis 机器人主控制类
======================

提供 Mantis 机器人的统一控制接口。

通信协议:
    使用 Zenoh 协议进行通信，无需安装 ROS2。
    SDK 通过纯 Python Zenoh 发送 JSON 格式数据，
    机器人端通过 Python 桥接节点 (sdk_bridge) 转发到 ROS2。

Example:
    .. code-block:: python
    
        from mantis import Mantis
        
        # 连接机器人
        with Mantis(ip="192.168.1.100") as robot:
            robot.left_arm.set_shoulder_pitch(-0.5)
            robot.head.look_left()
        
        # 本地调试（同一局域网）
        with Mantis() as robot:
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
from .waist import Waist
from .chassis import Chassis
from .constants import (
    Topics, JOINT_NAMES,
    SERIAL_TO_URDF_MAP, JOINT_DIRECTION_MAP,
    ALL_URDF_JOINTS
)


class Mantis:
    """Mantis 机器人主控制类。
    
    提供对 Mantis 机器人的统一控制接口，包括双臂、夹爪、头部和底盘。
    
    Attributes:
        left_arm (Arm): 左臂控制器
        right_arm (Arm): 右臂控制器
        left_gripper (Gripper): 左夹爪控制器
        right_gripper (Gripper): 右夹爪控制器
        head (Head): 头部控制器
        chassis (Chassis): 底盘控制器
        is_connected (bool): 是否已连接
    
    Example:
        使用上下文管理器（推荐）::
        
            with Mantis(ip="192.168.1.100") as robot:
                robot.left_arm.set_shoulder_pitch(-0.5)
                robot.head.look_left()
        
        手动管理连接::
        
            robot = Mantis(ip="192.168.1.100")
            robot.connect()
            robot.left_arm.home()
            robot.disconnect()
    
    Note:
        使用前需启动机器人端的 Python 桥接节点::
        
            ros2 run bw_sdk_bridge sdk_bridge
    """
    
    #: 默认 Zenoh 端口
    DEFAULT_PORT = 7447
    
    def __init__(self, ip: Optional[str] = None, port: int = None):
        """初始化 Mantis 机器人。
        
        Args:
            ip: 机器人 IP 地址，例如 "192.168.1.100"。
                如果为 None，则使用 Zenoh 自动发现（需在同一局域网）。
            port: Zenoh 端口，默认 7447。
        
        Example:
            .. code-block:: python
            
                # 指定 IP 连接
                robot = Mantis(ip="192.168.1.100")
                
                # 自动发现（同一局域网）
                robot = Mantis()
        """
        if ip:
            p = port or self.DEFAULT_PORT
            self._router = f"tcp/{ip}:{p}"
        else:
            self._router = None
        
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
        self._waist = Waist(self)
        self._chassis = Chassis(self)
        
        # 反馈数据
        self._feedback_callback: Optional[Callable] = None
        
        # 存储所有关节状态（用于完整发布）
        self._joint_states = {name: 0.0 for name in ALL_URDF_JOINTS}
        
        # 平滑参数（仿真和实机模式通用）
        self._target_states = {name: 0.0 for name in ALL_URDF_JOINTS}  # 目标位置
        self._current_states = {name: 0.0 for name in ALL_URDF_JOINTS}  # 当前平滑位置
        self._smooth_alpha = 0.1  # EMA 平滑因子，越小越平滑
        self._smooth_frequency = 100.0  # 发布频率 Hz
        self._smooth_enabled = True  # 是否启用平滑
        self._smooth_thread: Optional[threading.Thread] = None
        self._smooth_running = False
        
        # 实机模式：存储各模块的目标状态
        self._real_arm_positions = [0.0] * 14  # 双臂 14 个关节
        self._real_gripper_positions = [0.0, 0.0]  # 左右夹爪
        self._real_head_positions = [0.0, 0.0]  # pitch, yaw
        self._real_waist_position = 0.0  # 腰部高度
    
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
    def waist(self) -> Waist:
        """腰部控制器。
        
        Returns:
            Waist: 腰部升降控制器
        """
        return self._waist
    
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
    
    def set_smoothing(self, alpha: float = 0.1, rate: float = 100.0, enabled: bool = True):
        """设置运动平滑参数（仿真和实机模式通用）。
        
        使用 EMA（指数移动平均）算法进行平滑：
        ``current = current + alpha * (target - current)``
        
        Args:
            alpha: 平滑系数 (0.01-1.0)。
                - 0.05: 非常平滑，响应慢
                - 0.1: 平滑（默认）
                - 0.3: 较快响应
                - 1.0: 无平滑，立即到达目标
            rate: 发布频率 (Hz)，默认 100Hz。
            enabled: 是否启用平滑，默认 True。
        
        Example:
            .. code-block:: python
            
                robot = Mantis(ip="192.168.1.100")
                robot.set_smoothing(alpha=0.2)  # 更快响应
                robot.set_smoothing(enabled=False)  # 禁用平滑
                robot.connect()
        """
        self._smooth_alpha = max(0.01, min(1.0, alpha))
        self._smooth_frequency = max(10.0, min(500.0, rate))
        self._smooth_enabled = enabled
    
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
            self.home()
            return True
        
        try:
            config = zenoh.Config()
            if self._router:
                config.insert_json5("connect/endpoints", f'["{self._router}"]')
            
            self._session = zenoh.open(config)
            
            # 创建发布者（统一使用 JSON 格式，通过 Python 桥接节点转发到 ROS2）
            self._publishers['joints'] = self._session.declare_publisher(Topics.SDK_JOINT_STATES)
            self._publishers['chassis'] = self._session.declare_publisher(Topics.SDK_CHASSIS)
            
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
                    print("   1) 机器人端 sdk_bridge 节点是否已启动")
                    print("   2) ROS2 节点是否在发布 /joint_states_fdb")
                    print("   3) Zenoh 网络是否可达")
                    return False
            
            self._connected = True
            print(f"✅ 已连接到 Mantis 机器人")
            
            # 启动平滑发布线程
            if self._smooth_enabled:
                self._start_smooth_thread()
            
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
            if self._smooth_thread is not None:
                self._smooth_running = False
                self._smooth_thread.join(timeout=1.0)
                self._smooth_thread = None
            
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
    
    # ==================== 平滑处理（仿真和实机通用） ====================
    
    def _start_smooth_thread(self):
        """启动平滑发布线程。
        
        连接成功后自动调用，启动后台线程进行关节位置平滑插值。
        仿真模式和实机模式都会使用此线程。
        """
        if self._smooth_thread is not None:
            return
        
        self._smooth_running = True
        self._smooth_thread = threading.Thread(target=self._smooth_loop, daemon=True)
        self._smooth_thread.start()
    
    def _smooth_loop(self):
        """平滑插值循环。
        
        使用 EMA 算法逐渐将当前关节状态逼近目标状态，
        并以固定频率发布。
        """
        interval = 1.0 / self._smooth_frequency
        
        while self._smooth_running:
            # 平滑各模块的目标状态并发布
            self._smooth_and_publish()
            time.sleep(interval)
    
    def _smooth_and_publish(self):
        """平滑并发布所有控制指令（使用 JSON 格式）。"""
        # 平滑双臂关节
        positions = self._left_arm._positions + self._right_arm._positions
        for i in range(14):
            current = self._real_arm_positions[i]
            target = positions[i]
            self._real_arm_positions[i] = current + self._smooth_alpha * (target - current)
        
        # 平滑夹爪
        left_target = self._left_gripper._position
        right_target = self._right_gripper._position
        self._real_gripper_positions[0] += self._smooth_alpha * (left_target - self._real_gripper_positions[0])
        self._real_gripper_positions[1] += self._smooth_alpha * (right_target - self._real_gripper_positions[1])
        
        # 平滑头部
        pitch_target = self._head._pitch
        yaw_target = self._head._yaw
        self._real_head_positions[0] += self._smooth_alpha * (pitch_target - self._real_head_positions[0])
        self._real_head_positions[1] += self._smooth_alpha * (yaw_target - self._real_head_positions[1])
        
        # 平滑腰部
        waist_target = self._waist._height
        self._real_waist_position += self._smooth_alpha * (waist_target - self._real_waist_position)
        
        # 构建完整的关节状态 JSON（与仿真模式相同格式）
        # 使用 URDF 关节名称
        names = []
        values = []
        
        # 双臂关节 (14个)
        for i, serial_name in enumerate(JOINT_NAMES):
            urdf_name = SERIAL_TO_URDF_MAP.get(serial_name, serial_name)
            direction = JOINT_DIRECTION_MAP.get(serial_name, 1)
            names.append(urdf_name)
            values.append(self._real_arm_positions[i] * direction)
        
        # 夹爪 (4个关节)
        left_grip = self._real_gripper_positions[0] * 0.04  # 归一化 -> 实际位置
        right_grip = self._real_gripper_positions[1] * 0.04
        names.extend(["L_Hand_R_Joint", "L_Hand_L_Joint", "R_Hand_R_Joint", "R_Hand_L_Joint"])
        values.extend([left_grip, left_grip, right_grip, right_grip])
        
        # 头部 (2个关节)
        names.extend(["Head_Joint", "Neck_Joint"])
        values.extend([self._real_head_positions[0], self._real_head_positions[1]])
        
        # 腰部 (1个关节)
        names.append("Waist_Joint")
        values.append(self._real_waist_position)
        
        # 发布 JSON 格式
        msg = {
            'name': names,
            'position': values,
            'velocity': [],
            'effort': []
        }
        payload = json.dumps(msg).encode('utf-8')
        self._publishers['joints'].put(payload)
    
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
        
        将左右臂的关节位置发送到机器人。
        平滑模式下仅更新目标状态，由平滑线程发布。
        """
        self._check_connection()
        
        # 平滑模式：仅更新目标，由平滑线程发布
        if self._smooth_enabled:
            pass  # 目标状态已在子模块中更新，平滑线程会读取
        else:
            # 无平滑模式：直接发送 JSON
            positions = self._left_arm._positions + self._right_arm._positions
            names = []
            values = []
            for i, serial_name in enumerate(JOINT_NAMES):
                urdf_name = SERIAL_TO_URDF_MAP.get(serial_name, serial_name)
                direction = JOINT_DIRECTION_MAP.get(serial_name, 1)
                names.append(urdf_name)
                values.append(positions[i] * direction)
            
            msg = {'name': names, 'position': values, 'velocity': [], 'effort': []}
            self._publishers['joints'].put(json.dumps(msg).encode('utf-8'))
    
    def _publish_grippers(self):
        """发布夹爪位置。
        
        将左右夹爪的位置发送到机器人。
        平滑模式下仅更新目标状态，由平滑线程发布。
        """
        self._check_connection()
        # 夹爪数据由 _smooth_and_publish 统一发送
        pass
    
    def _publish_head(self):
        """发布头部姿态。
        
        将头部的俯仰和偏航角度发送到机器人。
        平滑模式下仅更新目标状态，由平滑线程发布。
        """
        self._check_connection()
        # 头部数据由 _smooth_and_publish 统一发送
        pass
    
    def _publish_waist(self):
        """发布腰部高度。
        
        将腰部的高度发送到机器人。
        平滑模式下仅更新目标状态，由平滑线程发布。
        """
        self._check_connection()
        # 腰部数据由 _smooth_and_publish 统一发送
        pass
    
    def _publish_chassis(self):
        """发布底盘速度。
        
        将底盘的线速度和角速度发送到机器人。
        """
        self._check_connection()
        
        # 统一使用 JSON 格式发送底盘命令
        data = {
            'vx': self._chassis._vx,
            'vy': self._chassis._vy,
            'omega': self._chassis._omega
        }
        self._publishers['chassis'].put(json.dumps(data).encode('utf-8'))
    
    # ==================== 便捷方法 ====================
    
    def home(self, block: bool = True):
        """所有关节回到零位。
        
        将双臂、头部回零，夹爪闭合。
        
        Args:
            block: 是否阻塞等待完成，默认 True
        """
        self._left_arm.home(block=False)
        self._right_arm.home(block=False)
        self._head.center(block=False)
        self._waist.home(block=False)
        self._left_gripper.close(block=False)
        self._right_gripper.close(block=False)
        
        if block:
            self.wait()
    
    def wait(self):
        """等待所有部件运动完成。
        
        阻塞直到所有正在运动的部件都完成运动。
        
        Example:
            .. code-block:: python
            
                # 启动多个非阻塞运动
                robot.left_arm.set_shoulder_pitch(-0.5, block=False)
                robot.right_arm.set_shoulder_pitch(-0.5, block=False)
                robot.head.look_left(block=False)
                
                # 等待全部完成
                robot.wait()
        """
        import time
        while self.is_any_moving:
            time.sleep(0.01)
    
    @property
    def is_any_moving(self) -> bool:
        """是否有任何部件正在运动。
        
        Returns:
            bool: True 如果有部件在运动中
        """
        return (
            self._left_arm.is_moving or
            self._right_arm.is_moving or
            self._head.is_moving or
            self._waist.is_moving or
            self._left_gripper.is_moving or
            self._right_gripper.is_moving or
            self._chassis.is_moving
        )
    
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
                # 解析 JSON 格式的反馈数据
                data = json.loads(sample.payload.to_bytes().decode('utf-8'))
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
