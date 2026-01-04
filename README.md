# Mantis Robot SDK

[![PyPI](https://img.shields.io/pypi/v/bw-mantis-sdk.svg)](https://pypi.org/project/bw-mantis-sdk/)
[![Version](https://img.shields.io/badge/version-1.1.0-blue.svg)](./VERSION)
[![Python](https://img.shields.io/badge/python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-MIT-orange.svg)](./LICENSE)

基于 Zenoh 的 Mantis 机器人控制 SDK，**无需安装 ROS2**。

> 2025-12-30 | [Release Notes](./RELEASE_NOTES.md)

## 特性

- 🚀 **无 ROS2 依赖**: 客户端只需 Python + Zenoh
- 🤖 **完整控制**: 双臂、夹爪、头部、腰部、底盘
- 🔒 **安全限位**: 自动限制在 URDF 定义范围内
- 🎯 **仿真预览**: RViz / Gazebo 实时预览（带平滑）
- ⚡ **并行运动**: 阻塞/非阻塞模式，支持多部件同时运动
- 🛡️ **安全底盘**: 基于距离/角度控制，运动完成自动停止
- 📚 **完整文档**: Google 风格 docstring

## 安装

```bash
pip install bw-mantis-sdk
```

## 快速开始

```python
from mantis import Mantis

# 控制实机
with Mantis(ip="192.168.1.100") as robot:
    robot.left_arm.set_shoulder_pitch(-0.5)
    robot.left_gripper.open()
    robot.head.look_up()
    robot.chassis.forward(0.1)
```

## 连接方式

```python
# 方式1：指定 IP 连接实机
robot = Mantis(ip="192.168.1.100")

# 方式2：自动发现（同一局域网）
robot = Mantis()

# 方式3：指定 IP 和端口
robot = Mantis(ip="192.168.1.100", port=7447)

# 方式4：仿真预览模式（在 RViz 中显示）
robot = Mantis(sim=True)
```

## 仿真预览模式

仿真模式可以在 RViz 中预览机器人动作，无需连接实机：

```python
from mantis import Mantis
import time

# 启用仿真模式
with Mantis(sim=True) as robot:
    # 可选：调整平滑参数
    robot.set_smoothing(alpha=0.1)  # 默认值
  
    # 控制手臂（在 RViz 中实时显示）
    robot.left_arm.set_shoulder_pitch(-0.5)
    robot.right_arm.set_elbow_pitch(0.8)
  
    # 控制头部
    robot.head.look_left()
  
    time.sleep(3)
```

**启动仿真环境：**

```bash
# 终端 1: 启动仿真环境（二选一）
cd ~/bw_motion_ws && source install/setup.bash

# RViz 模式（默认，快速预览）
ros2 launch bw_sim2real sdk_sim.launch.py

# Gazebo 模式（物理仿真，底盘可移动）
ros2 launch bw_sim2real sdk_sim.launch.py use_gazebo:=true

# 终端 2: 启动 zenoh 桥接
~/zenoh_ros2/zenoh-bridge-ros2dds -d 99

# 终端 3: 运行 SDK
cd ~/mantis
python test_sim.py
```

> 💡 **提示**: Gazebo 模式下底盘命令会真正移动机器人，RViz 模式仅预览关节。

---

## API 概览

### Mantis (主类)


| 属性            | 类型    | 说明           |
| --------------- | ------- | -------------- |
| `left_arm`      | Arm     | 左臂控制器     |
| `right_arm`     | Arm     | 右臂控制器     |
| `left_gripper`  | Gripper | 左夹爪控制器   |
| `right_gripper` | Gripper | 右夹爪控制器   |
| `head`          | Head    | 头部控制器     |
| `chassis`       | Chassis | 底盘控制器     |
| `is_sim_mode`   | bool    | 是否为仿真模式 |


| 方法                                | 说明             |
| ----------------------------------- | ---------------- |
| `connect(timeout=5.0, verify=True)` | 连接机器人       |
| `disconnect()`                      | 断开连接         |
| `on_feedback(callback)`             | 注册关节反馈回调 |
| `home(block=True)`                  | 所有关节归零     |
| `stop()`                            | 停止所有运动     |
| `wait()`                            | 等待所有部件运动完成 |
| `is_any_moving`                     | 是否有部件在运动中 |

### Arm (手臂)

每只手臂 7 个关节，**所有角度自动限制在安全范围内**：


| 索引 | 关节   | 方法                        | 限位 (rad) |
| ---- | ------ | --------------------------- | ---------- |
| 0    | 肩俯仰 | `set_shoulder_pitch(angle, block=True)` | -2.61 ~ 0.78 |
| 1    | 肩偏航 | `set_shoulder_yaw(angle, block=True)`   | -0.213 ~ 2.029 |
| 2    | 肩翻滚 | `set_shoulder_roll(angle, block=True)`  | -1.57 ~ 1.57 |
| 3    | 肘俯仰 | `set_elbow_pitch(angle, block=True)`    | -0.78 ~ 1.57 |
| 4    | 腕翻滚 | `set_wrist_roll(angle, block=True)`     | -1.57 ~ 1.57 |
| 5    | 腕俯仰 | `set_wrist_pitch(angle, block=True)`    | -0.52 ~ 0.52 |
| 6    | 腕偏航 | `set_wrist_yaw(angle, block=True)`      | -1.57 ~ 1.57 |

其他方法：

- `set_joints(positions, block=True)` - 设置全部 7 个关节（弧度）
- `set_joint(index, angle, block=True)` - 设置单个关节（索引 0-6）
- `get_limit(index)` - 获取指定关节限位 `(lower, upper)`
- `limits` - 获取所有关节限位列表
- `home(block=True)` - 回到零位
- `wait()` - 等待当前运动完成
- `is_moving` - 是否正在运动中
- `set_speed(speed)` - 设置关节速度 (rad/s)

### Gripper (夹爪)

支持阻塞/非阻塞模式：

| 方法                | 说明                          |
| ------------------- | ----------------------------- |
| `set_position(pos, block=True)` | 设置位置 (0.0=闭合, 1.0=张开) |
| `open(block=True)`            | 完全张开                      |
| `close(block=True)`           | 完全闭合                      |
| `half_open(block=True)`       | 半开                          |
| `wait()`            | 等待运动完成                  |
| `is_moving`         | 是否正在运动中                |

### Head (头部)

头部有限位保护，支持阻塞/非阻塞模式：

- **pitch (俯仰)**: -0.7 ~ 0.2 rad
- **yaw (偏航)**: -1.57 ~ 1.57 rad


| 方法                    | 说明                                               |
| ----------------------- | -------------------------------------------------- |
| `set_pose(pitch, yaw, block=True)`  | 设置姿态（弧度）                       |
| `set_pitch(angle, block=True)`      | 设置俯仰角                             |
| `set_yaw(angle, block=True)`        | 设置偏航角                             |
| `look_left(angle=0.5, block=True)`  | 向左看                                 |
| `look_right(angle=0.5, block=True)` | 向右看                                 |
| `look_up(angle=0.3, block=True)`    | 向上看                                 |
| `look_down(angle=0.3, block=True)`  | 向下看                                 |
| `center(block=True)`                | 回中                                   |
| `wait()`              | 等待运动完成                                       |
| `is_moving`           | 是否正在运动中                                     |
| `limits`              | 获取限位`{'pitch': (min, max), 'yaw': (min, max)}` |

### Waist (腰部)

腰部是 prismatic 直线关节，控制上半身高度：

- **高度范围**: -0.62m ~ 0.24m

| 方法                    | 说明                                   |
| ----------------------- | -------------------------------------- |
| `set_height(height, block=True)` | 设置高度（米）                |
| `up(delta=0.05, block=True)`     | 上升指定距离                  |
| `down(delta=0.05, block=True)`   | 下降指定距离                  |
| `home(block=True)`               | 回到默认高度                  |
| `wait()`                | 等待运动完成                           |
| `is_moving`             | 是否正在运动中                         |

### Chassis (底盘)

**安全设计**：所有运动命令必须指定距离或角度，运动完成后自动停止，避免失控。

| 方法                              | 说明                              |
| --------------------------------- | --------------------------------- |
| `forward(distance, speed=0.1, block=True)`    | 前进指定距离（米）    |
| `backward(distance, speed=0.1, block=True)`   | 后退指定距离（米）    |
| `strafe_left(distance, speed=0.1, block=True)`  | 左移指定距离（米）  |
| `strafe_right(distance, speed=0.1, block=True)` | 右移指定距离（米）  |
| `turn_left(angle_deg, speed=0.5, block=True)`   | 左转指定角度（度）  |
| `turn_right(angle_deg, speed=0.5, block=True)`  | 右转指定角度（度）  |
| `move(x, y, angle, block=True)` | 组合运动                          |
| `stop()`                        | 立即停止                          |
| `wait()`                        | 等待运动完成                      |
| `is_moving`                     | 是否正在运动中                    |

**默认速度**：线速度 0.1 m/s，角速度 0.5 rad/s（约 28.6°/s）

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
from mantis import Mantis

with Mantis(ip="192.168.1.100") as robot:
    # 前进 0.5 米（阻塞，完成后自动停止）
    robot.chassis.forward(0.5)
    
    # 后退 0.3 米
    robot.chassis.backward(0.3)
    
    # 左移 0.2 米
    robot.chassis.strafe_left(0.2)
    
    # 左转 90 度
    robot.chassis.turn_left(90)
    
    # 自定义速度前进
    robot.chassis.forward(1.0, speed=0.2)
    
    # 组合运动：边走边转
    robot.chassis.move(x=0.5, y=0.2, angle=45)
```

### 5. 并行运动（非阻塞模式）

使用 `block=False` 让多个部件同时运动：

```python
from mantis import Mantis

with Mantis(sim=True) as robot:
    # ===== 顺序运动（默认，慢但安全）=====
    robot.left_arm.set_shoulder_pitch(-0.5)   # 等待完成
    robot.right_arm.set_shoulder_pitch(-0.5)  # 等待完成
    robot.head.look_left()                    # 等待完成
    # 总耗时 = 三个动作时间之和
    
    # ===== 并行运动（快）=====
    robot.left_arm.set_shoulder_pitch(-0.5, block=False)   # 立即返回
    robot.right_arm.set_shoulder_pitch(-0.5, block=False)  # 立即返回
    robot.head.look_left(block=False)                      # 立即返回
    robot.wait()  # 等待所有部件完成
    # 总耗时 = 最慢动作的时间
    
    # ===== 分组运动 =====
    # 第一组：双臂
    robot.left_arm.set_shoulder_pitch(-0.3, block=False)
    robot.right_arm.set_shoulder_pitch(-0.3, block=False)
    robot.wait()
    
    # 第二组：头部 + 腰部
    robot.head.look_down(block=False)
    robot.waist.up(0.05, block=False)
    robot.wait()
    
    # 第三组：双夹爪
    robot.left_gripper.open(block=False)
    robot.right_gripper.open(block=False)
    robot.wait()
```

### 6. 关节反馈

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

### 7. 综合示例

```python
from mantis import Mantis

with Mantis(ip="192.168.1.100") as robot:
    print("开始综合演示...")
  
    # 1. 头部环顾（阻塞模式）
    robot.head.look_left()
    robot.head.look_right()
    robot.head.center()
  
    # 2. 双臂抬起（并行）
    robot.left_arm.set_shoulder_pitch(-0.5, block=False)
    robot.right_arm.set_shoulder_pitch(-0.5, block=False)
    robot.wait()
  
    # 3. 夹爪开合（并行）
    robot.left_gripper.open(block=False)
    robot.right_gripper.open(block=False)
    robot.wait()
    
    robot.left_gripper.close(block=False)
    robot.right_gripper.close(block=False)
    robot.wait()
  
    # 4. 底盘移动（基于距离，安全）
    robot.chassis.forward(0.3)
    robot.chassis.turn_left(90)
    robot.chassis.backward(0.3)
  
    # 5. 回到初始位置
    robot.home()
  
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
├── mantis/
│   ├── __init__.py     # 模块入口
│   ├── mantis.py       # 主控制类
│   ├── arm.py          # 手臂控制（7自由度）
│   ├── gripper.py      # 夹爪控制
│   ├── head.py         # 头部控制（俯仰/偏航）
│   ├── waist.py        # 腰部控制（升降）
│   ├── chassis.py      # 底盘控制（全向移动）
│   ├── cdr.py          # CDR编解码
│   └── constants.py    # 关节限位常量
├── test_sim.py         # 仿真测试脚本
├── test_parallel_motion.py  # 并行运动测试
└── README.md
```

## 注意事项

1. **角度单位**：所有角度均为弧度（rad），底盘旋转角度为度（°）
2. **速度单位**：线速度 m/s，角速度 rad/s
3. **夹爪范围**：0.0（闭合）到 1.0（张开）
4. **腰部范围**：-0.62m ~ 0.24m
5. **底盘安全**：运动完成后自动停止，无需手动调用 `stop()`
6. **阻塞模式**：默认 `block=True`，使用 `block=False` 实现并行运动
7. **连接超时**：默认 5 秒，可通过 `connect(timeout=10)` 调整

---

**BlueWorm-EAI-Tech**
