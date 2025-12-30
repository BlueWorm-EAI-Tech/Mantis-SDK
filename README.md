# Mantis Robot SDK

基于 Zenoh 的 Mantis 机器人控制 SDK，**无需安装 ROS2**。

## 安装

```bash
pip install eclipse-zenoh
```

## 快速开始

```python
from mantis_sdk import Mantis

with Mantis(ip="192.168.1.100") as robot:
    robot.left_arm.set_shoulder_pitch(0.5)
    robot.left_gripper.open()
    robot.head.look_up()
    robot.chassis.forward(0.1)
```

## 连接方式

```python
# 方式1：指定 IP 连接
robot = Mantis(ip="192.168.1.100")

# 方式2：自动发现（同一局域网）
robot = Mantis()

# 方式3：指定 IP 和端口
robot = Mantis(ip="192.168.1.100", port=7447)
```

---

## API 概览

### Mantis (主类)

| 属性 | 类型 | 说明 |
|------|------|------|
| `left_arm` | Arm | 左臂控制器 |
| `right_arm` | Arm | 右臂控制器 |
| `left_gripper` | Gripper | 左夹爪控制器 |
| `right_gripper` | Gripper | 右夹爪控制器 |
| `head` | Head | 头部控制器 |
| `chassis` | Chassis | 底盘控制器 |

| 方法 | 说明 |
|------|------|
| `connect(timeout=5.0, verify=True)` | 连接机器人 |
| `disconnect()` | 断开连接 |
| `on_feedback(callback)` | 注册关节反馈回调 |
| `home()` | 所有关节归零 |
| `stop()` | 停止运动 |

### Arm (手臂)

每只手臂 7 个关节，**所有角度自动限制在安全范围内**：

| 索引 | 关节 | 方法 | 左臂限位 (rad) | 右臂限位 (rad) |
|------|------|------|----------------|----------------|
| 0 | 肩俯仰 | `set_shoulder_pitch(angle)` | -2.61 ~ 0.78 | -2.61 ~ 0.78 |
| 1 | 肩偏航 | `set_shoulder_yaw(angle)` | 0.08 ~ 1.04 | -1.04 ~ -0.08 |
| 2 | 肩翻滚 | `set_shoulder_roll(angle)` | -1.57 ~ 1.57 | -1.57 ~ 1.57 |
| 3 | 肘俯仰 | `set_elbow_pitch(angle)` | -0.78 ~ 1.57 | -0.78 ~ 1.57 |
| 4 | 腕翻滚 | `set_wrist_roll(angle)` | -1.57 ~ 1.57 | -1.57 ~ 1.57 |
| 5 | 腕俯仰 | `set_wrist_pitch(angle)` | -0.52 ~ 0.52 | -0.52 ~ 0.52 |
| 6 | 腕偏航 | `set_wrist_yaw(angle)` | -1.57 ~ 1.57 | -1.57 ~ 1.57 |

其他方法：
- `set_joints([j0, j1, j2, j3, j4, j5, j6])` - 设置全部 7 个关节（弧度）
- `set_joint(index, angle)` - 设置单个关节（索引 0-6）
- `get_limit(index)` - 获取指定关节限位 `(lower, upper)`
- `limits` - 获取所有关节限位列表
- `home()` - 回到零位

### Gripper (夹爪)

| 方法 | 说明 |
|------|------|
| `set_position(pos)` | 设置位置 (0.0=闭合, 1.0=张开) |
| `open()` | 完全张开 |
| `close()` | 完全闭合 |
| `half_open()` | 半开 |

### Head (头部)

头部有限位保护：
- **pitch (俯仰)**: -0.7 ~ 0.2 rad
- **yaw (偏航)**: -1.57 ~ 1.57 rad

| 方法 | 说明 |
|------|------|
| `set_pose(pitch, yaw)` | 设置姿态（弧度） |
| `set_pitch(angle)` | 设置俯仰角 |
| `set_yaw(angle)` | 设置偏航角 |
| `look_left(angle=0.5)` | 向左看 |
| `look_right(angle=0.5)` | 向右看 |
| `look_up(angle=0.3)` | 向上看 |
| `look_down(angle=0.3)` | 向下看 |
| `center()` | 回中 |
| `limits` | 获取限位 `{'pitch': (min, max), 'yaw': (min, max)}` |

### Chassis (底盘)

| 方法 | 说明 |
|------|------|
| `set_velocity(vx, vy, omega)` | 设置速度 (m/s, rad/s) |
| `forward(speed=0.1)` | 前进 |
| `backward(speed=0.1)` | 后退 |
| `strafe_left(speed=0.1)` | 左移 |
| `strafe_right(speed=0.1)` | 右移 |
| `turn_left(speed=0.3)` | 左转 |
| `turn_right(speed=0.3)` | 右转 |
| `stop()` | 停止 |

---

## 完整示例

### 1. 手臂控制

```python
from mantis_sdk import Mantis
import time

with Mantis(ip="192.168.1.100") as robot:
    # 设置左臂各关节
    robot.left_arm.set_shoulder_pitch(0.5)   # 肩俯仰
    robot.left_arm.set_shoulder_yaw(0.2)     # 肩偏航
    robot.left_arm.set_shoulder_roll(0.1)    # 肩翻滚
    robot.left_arm.set_elbow_pitch(0.8)      # 肘俯仰
    robot.left_arm.set_wrist_roll(0.0)       # 腕翻滚
    robot.left_arm.set_wrist_pitch(0.3)      # 腕俯仰
    robot.left_arm.set_wrist_yaw(0.0)        # 腕偏航
    time.sleep(2)
    
    # 一次性设置全部关节
    robot.left_arm.set_joints([0.5, 0.2, 0.1, 0.8, 0.0, 0.3, 0.0])
    time.sleep(2)
    
    # 回到零位
    robot.left_arm.home()
    robot.right_arm.home()
    time.sleep(1)
```

### 2. 夹爪控制

```python
from mantis_sdk import Mantis
import time

with Mantis(ip="192.168.1.100") as robot:
    # 张开夹爪
    robot.left_gripper.open()
    robot.right_gripper.open()
    time.sleep(1)
    
    # 半开
    robot.left_gripper.half_open()
    time.sleep(1)
    
    # 闭合
    robot.left_gripper.close()
    robot.right_gripper.close()
    time.sleep(1)
    
    # 自定义位置 (0.0 ~ 1.0)
    robot.left_gripper.set_position(0.7)
    time.sleep(1)
```

### 3. 头部控制

```python
from mantis_sdk import Mantis
import time

with Mantis(ip="192.168.1.100") as robot:
    # 向左看
    robot.head.look_left()
    time.sleep(1)
    
    # 向右看
    robot.head.look_right()
    time.sleep(1)
    
    # 向上看
    robot.head.look_up()
    time.sleep(1)
    
    # 向下看
    robot.head.look_down()
    time.sleep(1)
    
    # 回中
    robot.head.center()
    time.sleep(1)
    
    # 自定义角度
    robot.head.set_pose(pitch=0.2, yaw=-0.3)
    time.sleep(1)
```

### 4. 底盘控制

```python
from mantis_sdk import Mantis
import time

with Mantis(ip="192.168.1.100") as robot:
    # 前进
    robot.chassis.forward(0.1)
    time.sleep(2)
    
    # 后退
    robot.chassis.backward(0.1)
    time.sleep(2)
    
    # 左移
    robot.chassis.strafe_left(0.1)
    time.sleep(2)
    
    # 右移
    robot.chassis.strafe_right(0.1)
    time.sleep(2)
    
    # 左转
    robot.chassis.turn_left(0.3)
    time.sleep(2)
    
    # 右转
    robot.chassis.turn_right(0.3)
    time.sleep(2)
    
    # 停止
    robot.chassis.stop()
    
    # 自定义速度
    robot.chassis.set_velocity(vx=0.1, vy=0.05, omega=0.1)
    time.sleep(2)
    robot.chassis.stop()
```

### 5. 关节反馈

```python
from mantis_sdk import Mantis
import time

def on_feedback(joint_names, positions):
    print(f"关节反馈: {len(positions)} 个关节")
    for name, pos in zip(joint_names, positions):
        print(f"  {name}: {pos:.3f}")

with Mantis(ip="192.168.1.100") as robot:
    # 注册反馈回调
    robot.on_feedback(on_feedback)
    
    # 保持运行，接收反馈
    time.sleep(10)
```

### 6. 综合示例

```python
from mantis_sdk import Mantis
import time

with Mantis(ip="192.168.1.100") as robot:
    print("开始综合演示...")
    
    # 1. 头部环顾
    robot.head.look_left()
    time.sleep(0.5)
    robot.head.look_right()
    time.sleep(0.5)
    robot.head.center()
    
    # 2. 双臂抬起
    robot.left_arm.set_shoulder_pitch(0.5)
    robot.right_arm.set_shoulder_pitch(0.5)
    time.sleep(1)
    
    # 3. 夹爪开合
    robot.left_gripper.open()
    robot.right_gripper.open()
    time.sleep(0.5)
    robot.left_gripper.close()
    robot.right_gripper.close()
    time.sleep(0.5)
    
    # 4. 前进后退
    robot.chassis.forward(0.1)
    time.sleep(1)
    robot.chassis.backward(0.1)
    time.sleep(1)
    robot.chassis.stop()
    
    # 5. 回到初始位置
    robot.left_arm.home()
    robot.right_arm.home()
    robot.head.center()
    
    print("演示完成！")
```

---

## 机器人端配置

启动 Zenoh-ROS2 桥接（根据你的 ROS_DOMAIN_ID 设置 -d 参数）：

```bash
~/zenoh_ros2/zenoh-bridge-ros2dds -d 99
```

## 文件结构

```
mantis/
├── mantis_sdk/
│   ├── __init__.py     # 模块入口
│   ├── mantis.py       # 主控制类
│   ├── arm.py          # 手臂控制
│   ├── gripper.py      # 夹爪控制
│   ├── head.py         # 头部控制
│   ├── chassis.py      # 底盘控制
│   ├── cdr.py          # CDR编解码
│   └── constants.py    # 常量定义
└── README.md
```

## 注意事项

1. **角度单位**：所有角度均为弧度（rad）
2. **速度单位**：线速度 m/s，角速度 rad/s
3. **夹爪范围**：0.0（闭合）到 1.0（张开）
4. **连接超时**：默认 5 秒，可通过 `connect(timeout=10)` 调整

---

**BlueWorm-EAI-Tech**
