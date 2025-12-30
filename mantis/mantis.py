"""
Mantis 机器人主控制类
"""

from typing import Optional, Callable
import time

try:
    import zenoh
except ImportError:
    raise ImportError("请安装 zenoh: pip install eclipse-zenoh")

from .arm import Arm
from .gripper import Gripper
from .head import Head
from .chassis import Chassis
from .cdr import CDREncoder, CDRDecoder
from .constants import Topics, JOINT_NAMES


class Mantis:
    """
    Mantis 机器人主控制类
    
    使用示例:
        from mantis_sdk import Mantis
        
        robot = Mantis()
        robot.connect()
        
        # 控制左臂
        robot.left_arm.set_joints([0.0] * 7)
        robot.left_arm.set_shoulder_pitch(0.5)
        
        # 控制右臂
        robot.right_arm.set_joints([0.0] * 7)
        
        # 控制夹爪
        robot.left_gripper.open()
        robot.right_gripper.close()
        
        # 控制头部
        robot.head.set_pose(pitch=0.0, yaw=0.0)
        robot.head.look_left()
        
        # 控制底盘
        robot.chassis.forward(0.1)
        robot.chassis.stop()
        
        robot.disconnect()
    """
    
    DEFAULT_PORT = 7447
    
    def __init__(self, ip: Optional[str] = None, port: int = None):
        """
        初始化 Mantis 机器人
        
        Args:
            ip: 机器人 IP 地址，例如 "192.168.1.100"
                如果为 None，则使用 Zenoh 自动发现（同一局域网）
            port: Zenoh 端口，默认 7447
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
        self._chassis = Chassis(self)
        
        # 反馈数据
        self._feedback_callback: Optional[Callable] = None
    
    # ==================== 属性访问 ====================
    
    @property
    def left_arm(self) -> Arm:
        """左臂控制器"""
        return self._left_arm
    
    @property
    def right_arm(self) -> Arm:
        """右臂控制器"""
        return self._right_arm
    
    @property
    def left_gripper(self) -> Gripper:
        """左夹爪控制器"""
        return self._left_gripper
    
    @property
    def right_gripper(self) -> Gripper:
        """右夹爪控制器"""
        return self._right_gripper
    
    @property
    def head(self) -> Head:
        """头部控制器"""
        return self._head
    
    @property
    def chassis(self) -> Chassis:
        """底盘控制器"""
        return self._chassis
    
    @property
    def is_connected(self) -> bool:
        """是否已连接"""
        return self._connected
    
    # ==================== 连接管理 ====================
    
    def connect(self, timeout: float = 5.0, verify: bool = True) -> bool:
        """
        连接到机器人
        
        Args:
            timeout: 连接超时时间（秒）
            verify: 是否验证机器人在线（通过订阅反馈话题检测）
            
        Returns:
            是否连接成功
        """
        if self._connected:
            return True
        
        try:
            config = zenoh.Config()
            if self._router:
                config.insert_json5("connect/endpoints", f'["{self._router}"]')
            
            self._session = zenoh.open(config)
            
            # 创建发布者
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
            print("✅ 已连接到 Mantis 机器人")
            return True
            return True
            
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
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
        """检查连接状态"""
        if not self._connected:
            raise RuntimeError("未连接到机器人，请先调用 connect()")
    
    def _publish_joints(self):
        """发布关节角度"""
        self._check_connection()
        
        # 合并左右臂位置
        positions = self._left_arm._positions + self._right_arm._positions
        
        payload = CDREncoder.encode_joint_state(
            names=JOINT_NAMES,
            positions=positions
        )
        self._publishers['joints'].put(payload)
    
    def _publish_grippers(self):
        """发布夹爪位置"""
        self._check_connection()
        
        payload = CDREncoder.encode_joint_state(
            names=["left_gripper", "right_gripper"],
            positions=[self._left_gripper._position, self._right_gripper._position]
        )
        self._publishers['gripper'].put(payload)
    
    def _publish_head(self):
        """发布头部姿态"""
        self._check_connection()
        
        payload = CDREncoder.encode_joint_state(
            names=["head_pitch", "head_yaw"],
            positions=[self._head._pitch, self._head._yaw]
        )
        self._publishers['head'].put(payload)
    
    def _publish_chassis(self):
        """发布底盘速度"""
        self._check_connection()
        
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
        """所有关节回零位"""
        self._left_arm.home()
        self._right_arm.home()
        self._head.center()
        self._left_gripper.close()
        self._right_gripper.close()
    
    def stop(self):
        """停止所有运动"""
        if self._connected:
            self._chassis.stop()
    
    def subscribe_feedback(self, callback: Callable):
        """
        订阅关节反馈
        
        Args:
            callback: 回调函数，接收 dict 包含 'name', 'position' 等字段
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
    
    def __enter__(self):
        if not self.connect():
            raise ConnectionError("无法连接到机器人")
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._connected:
            self.stop()
            self.disconnect()
    
    def __repr__(self):
        status = "已连接" if self._connected else "未连接"
        return f"Mantis(status='{status}')"
