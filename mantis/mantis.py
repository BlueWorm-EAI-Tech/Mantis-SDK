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
            self._target_ip = ip
        else:
            self._router = None
            self._target_ip = None
        
        self._session: Optional[zenoh.Session] = None
        self._publishers = {}
        self._subscribers = {}
        self._connected = False
        self._robot_ip: Optional[str] = None
        
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
        self._status_callback: Optional[Callable] = None
        self._system_status = {}  # 存储最近一次系统状态
        
        # 存储所有关节状态（用于完整发布）
        self._joint_states = {name: 0.0 for name in ALL_URDF_JOINTS}
        self._joint_states = {name: 0.0 for name in ALL_URDF_JOINTS}
        
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
    
    @property
    def robot_ip(self) -> Optional[str]:
        """获取机器人的 IP 地址。
        
        Returns:
            str: 机器人的 IP 地址，如果未连接或获取失败则为 None
        """
        return self._robot_ip
    
    @property
    def system_status(self) -> dict:
        """获取最近一次系统状态。
        
        Returns:
            dict: 包含 system_state, control_source, message, motion_names, motion_states 等字段
        """
        return self._system_status
    
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
        
        target = self._router if self._router else "自动发现模式"
        print(f"⏳ 正在连接 Mantis 机器人 ({target})...")
        
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
                    try:
                        data = json.loads(sample.payload.to_bytes().decode('utf-8'))
                        recv_ip = data.get('ip')
                        
                        if recv_ip:
                            self._robot_ip = recv_ip
                        
                        # 验证 IP
                        if self._target_ip:
                            if recv_ip == self._target_ip:
                                self._system_status = data  # 保存初始状态
                                received.append(True)
                        else:
                            # 自动发现模式，只要收到合法数据即视为在线
                            self._system_status = data  # 保存初始状态
                            received.append(True)
                            
                    except Exception:
                        pass
                
                # 订阅反馈话题检测
                sub = self._session.declare_subscriber(Topics.SYSTEM_STATUS, _check_callback)
                
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
                    if self._target_ip:
                        print(f"   4) 机器人反馈 IP 是否为 {self._target_ip}")
                    return False
            
            self._connected = True
            print(f"✅ 已连接到 Mantis 机器人")
            
            # 自动订阅反馈和状态，用于更新内部状态 (robot_ip, system_status)
            self.subscribe_status(None)
            
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
        直接发送 JSON 数据，不进行平滑插值。
        """
        self._check_connection()
        self._publish_full_state()

    
    def _publish_grippers(self):
        """发布夹爪位置。
        
        将左右夹爪的位置发送到机器人。
        直接发送 JSON 数据，不进行平滑插值。
        """
        self._check_connection()
        
        # 构建夹爪 JSON (复用 joint_states 通道，或者需要单独处理？这里假设所有关节都走 joint_states)
        # 注意：原先 _smooth_and_publish 是把所有关节打包一起发的。
        # 现在分开发布，可能需要确认接收端是否支持部分更新。
        # 为了兼容性，这里我们最好还是打包发所有关节，或者只发变化的。
        # 但 mantis.py 结构是分模块调用的。
        # 
        # 策略：每次调用任何 _publish_xxx，都收集全量状态发送，确保原子性。
        self._publish_full_state()

    def _publish_head(self):
        self._check_connection()
        self._publish_full_state()
    
    def _publish_waist(self):
        self._check_connection()
        self._publish_full_state()

    def _publish_full_state(self):
        """发送所有模块的当前目标状态。确保所有数据均为 float 类型。"""
        # 收集所有模块状态
        arm_positions = self._left_arm._positions + self._right_arm._positions
        
        names = []
        values = []
        
        # 1. 双臂
        for i, serial_name in enumerate(JOINT_NAMES):
            urdf_name = SERIAL_TO_URDF_MAP.get(serial_name, serial_name)
            direction = float(JOINT_DIRECTION_MAP.get(serial_name, 1.0))
            names.append(urdf_name)
            # 强制转换为 float 确保 ROS2 兼容性
            values.append(float(arm_positions[i]) * direction)
            
        # 2. 夹爪
        left_grip = float(self._left_gripper._position) * 0.04
        right_grip = float(self._right_gripper._position) * 0.04
        names.extend(["L_Hand_R_Joint", "L_Hand_L_Joint", "R_Hand_R_Joint", "R_Hand_L_Joint"])
        values.extend([left_grip, left_grip, right_grip, right_grip])
        
        # 3. 头部
        names.extend(["Head_Joint", "Neck_Joint"])
        values.extend([float(self._head._pitch), float(self._head._yaw)])
        
        # 4. 腰部
        names.append("Waist_Joint")
        values.append(float(self._waist._height))
        
        # 构建消息
        msg = {
            'name': names,
            'position': values,
            'velocity': [],
            'effort': []
        }
        
        # 安全性检查：确保 position 中没有非数字值
        if any(v is None for v in values):
            return
            
        # 检查 NaN 或 Inf
        import math
        if any(math.isnan(v) or math.isinf(v) for v in values):
            print(f"⚠️ 警告: 检测到 NaN 或 Inf 数据，跳过发送: {values}")
            return

        self._publishers['joints'].put(json.dumps(msg).encode('utf-8'))

    
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
    
    def wait(self, joint_names: Optional[list] = None):
        """等待部件运动完成。
        
        阻塞直到指定的部件完成运动。
        基于机器人反馈的 system_status 进行判断。
        
        Args:
            joint_names: 要等待的关节名称列表。如果为 None，则等待所有部件。
        
        Example:
            .. code-block:: python
            
                # 启动多个非阻塞运动
                robot.left_arm.set_shoulder_pitch(-0.5, block=False)
                robot.right_arm.set_shoulder_pitch(-0.5, block=False)
                robot.head.look_left(block=False)
                
                # 等待全部完成
                robot.wait()
                
                # 仅等待左臂
                robot.wait(robot.left_arm.joint_names)
        """
        import time
        # 初始等待，确保指令已发送且状态已更新
        # 即使在 100Hz 下，网络传输和 ROS 内部处理也需要时间
        time.sleep(0.01)
        
        # 强等待策略
        wait_start = time.time()
        motion_detected = False
        
        # 阶段 1: 等待运动标志位变 1 (Waiting for motion to START)
        # 增加超时时间到 3.0s，防止长延迟导致漏检
        while time.time() - wait_start < 1:
            if self.is_moving(joint_names):
                motion_detected = True
                break
            time.sleep(0.001)
            
        if not motion_detected:
            # 即使超时，也不立即返回，而是进入停止检测，双重保险
            pass
            
        # 阶段 2: 等待运动标志位变 0 (Waiting for motion to STOP)
        consecutive_stops = 0
        REQUIRED_STOPS = 5  # 增加到 20 次 (20 * 5ms = 100ms)，确保彻底停稳
        
        while True:
            is_moving = self.is_moving(joint_names)
            
            if not is_moving:
                consecutive_stops += 1
            else:
                consecutive_stops = 0
                
            if consecutive_stops >= REQUIRED_STOPS:
                break
                
            time.sleep(0.001)
    
    @property
    def is_any_moving(self) -> bool:
        """是否有任何部件正在运动。
        
        Returns:
            bool: True 如果有部件在运动中
        """
        return self.is_moving()

    def is_moving(self, joint_names: Optional[list] = None) -> bool:
        """指定部件是否正在运动。

        Args:
            joint_names: 关节名称列表。如果为 None，检查所有部件。

        Returns:
            bool: True 如果指定部件中有任何一个在运动中
        """
        if not self._system_status or 'motion_states' not in self._system_status:
            return False
            
        motion_names = self._system_status.get('motion_names', [])
        motion_states = self._system_status.get('motion_states', [])
        
        if not joint_names:
            return any(s == 1 for s in motion_states)
            
        for name in joint_names:
            if name in motion_names:
                idx = motion_names.index(name)
                if idx < len(motion_states) and motion_states[idx] == 1:
                    return True
        return False
    
    def stop(self):
        """停止所有运动。
        
        立即停止底盘运动。
        """
        if self._connected:
            self._chassis.stop()
    
    def subscribe_status(self, callback: Optional[Callable] = None):
        """订阅系统状态反馈。
        
        注册回调函数，接收机器人的系统状态信息（如关节是否在运动）。
        
        Args:
            callback: 回调函数，签名为 ``callback(data: dict)``。
                data 包含 system_state, motion_names, motion_states 等字段。
        """
        self._check_connection()
        self._status_callback = callback
        
        def _on_status(sample):
            try:
                data = json.loads(sample.payload.to_bytes().decode('utf-8'))
                self._system_status = data
                if self._status_callback:
                    self._status_callback(data)
            except Exception as e:
                print(f"⚠️ 解析系统状态失败: {e}")
                
        # 如果已经存在，先取消订阅
        if 'status' in self._subscribers:
            self._subscribers['status'].undeclare()
            
        self._subscribers['status'] = self._session.declare_subscriber(
            Topics.SYSTEM_STATUS,
            _on_status
        )
        if callback:
            print("✅ 已订阅系统状态")
    
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
