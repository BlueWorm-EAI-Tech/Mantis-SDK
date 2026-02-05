# Mantis Robot SDK

[![PyPI](https://img.shields.io/pypi/v/bw-mantis-sdk.svg)](https://pypi.org/project/bw-mantis-sdk/)
[![Version](https://img.shields.io/badge/version-1.3.3-blue.svg)](./VERSION)
[![Python](https://img.shields.io/badge/python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-MIT-orange.svg)](./LICENSE)

基于 Zenoh 的 Mantis 机器人控制 SDK，**无需安装 ROS2**。

> 2026-02-05 | [Release Notes](./RELEASE_NOTES.md)
## 🚨 紧急版本兼容性说明 (CRITICAL COMPATIBILITY NOTICE)

**SDK 版本与机器人端代码版本存在强绑定关系，请务必检查版本匹配！**

| SDK 版本 (Client) | 机器人代码版本 (Robot) | 兼容状态 |
| :--- | :--- | :--- |
| **v1.3.2 及以上** | **≥ 26.2.3.1** | ✅ **兼容 (Compatible)** |
| **v1.3.2 以下** | **< 26.2.3.1** | ✅ **兼容 (Compatible)** |
| v1.3.2 及以上 | < 26.2.3.1 | ❌ **不可用** |
| v1.3.2 以下 | ≥ 26.2.3.1 | ❌ **不可用** |

> ⚠️ **警告**：如果您使用的是 **v1.3.2+** 的 SDK，**必须** 确保机器人端代码已升级到 **26.2.3.1** 或更高版本，否则无法正常通信！
## 特性

- 🚀 **无 ROS2 依赖**: 客户端只需 Python + Zenoh
- 🤖 **完整控制**: 双臂、夹爪、头部、腰部、底盘
- 🦾 **逆运动学 (IK)**: 基于 Pinocchio 的高性能解算，支持绝对位姿与相对增量控制
- 🔒 **安全限位**: 自动限制在 URDF 定义范围内
- 🎯 **统一驱动**: 同一套代码控制 RViz 仿真和实机
- ⚡ **并行运动**: 阻塞/非阻塞模式，支持多部件同时运动
- 🛡️ **安全底盘**: 基于距离/角度控制，运动完成自动停止
- 🎚️ **摩擦补偿**: 可调节摩擦系数，适应不同地面环境
- 📚 **完整文档**: Google 风格 docstring

## 安装

```bash
pip install bw-mantis-sdk
```

### 依赖安装 (IK 功能)

如果需要使用 IK (逆运动学) 功能，请按照以下步骤安装 Pinocchio 依赖：

```bash
# Ensure you have some required installation dependencies 
sudo apt install -qqy lsb-release curl 
# Register the authentication certificate of robotpkg: 
sudo mkdir -p /etc/apt/keyrings 
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
  | sudo tee /etc/apt/keyrings/robotpkg.asc 
# Add robotpkg as source repository to apt: 
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
  | sudo tee /etc/apt/sources.list.d/robotpkg.list 
sudo apt update 
sudo apt install -qqy robotpkg-py310-pinocchio
```

## 快速开始

```python
from mantis import Mantis

# 连接机器人（RViz 仿真或实机均可）
with Mantis(ip="192.168.1.100") as robot:
    robot.left_arm.set_shoulder_pitch(-0.5)
    robot.left_gripper.open()
    robot.head.look_up()
    robot.chassis.forward(0.5)
```

## 连接方式

```python
# 方式1：指定 IP 连接（推荐）
robot = Mantis(ip="192.168.1.100")

# 方式2：自动发现（同一局域网）
robot = Mantis()

# 方式3：指定 IP 和端口
robot = Mantis(ip="192.168.1.100", port=7447)

# 连接时跳过验证（调试用）
robot.connect(verify=False)
```

## 架构说明

```
┌─────────────┐     Zenoh (JSON)      ┌──────────────────┐
│  Mantis SDK │ ──────────────────────▶ │  sdk_rviz_bridge │ ──▶ RViz
│  (Python)   │                        │  (ROS2 节点)     │
└─────────────┘                        └──────────────────┘
       │
       │         Zenoh (JSON)         ┌──────────────────┐
       └─────────────────────────────▶ │  sdk_real_bridge │ ──▶ 实机
                                       │  (ROS2 节点)     │
                                       └──────────────────┘
```

**数据流**：

- SDK 发布 JSON 格式数据到 Zenoh 话题 `sdk/joint_states` 和 `sdk/chassis`
- Bridge 节点订阅 Zenoh 话题，转换为 ROS2 消息发布
- 无需 zenoh-bridge-ros2dds，通讯更稳定

## 机器人端配置

**启动 Bridge 节点**（二选一）：

```bash
# RViz 仿真预览
cd ~/bw_motion_ws && source install/setup.bash
ros2 launch bw_sim2real sdk_sim.launch.py

# 实机控制
# 1) 进入遥操作文件夹
cd ~/bw_system_ws
# 2) 修改.env文件
# 修改文件中的内容不是终端运行
RUN_MODE=sdk
# 3) 运行启动脚本
./run.sh
```

> 💡 **提示**: 不再需要启动 zenoh-bridge-ros2dds

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


| 方法                                | 说明                 |
| ----------------------------------- | -------------------- |
| `connect(timeout=5.0, verify=True)` | 连接机器人           |
| `disconnect()`                      | 断开连接             |
| `on_feedback(callback)`             | 注册关节反馈回调     |
| `home(block=True)`                  | 所有关节归零         |
| `stop()`                            | 停止所有运动         |
| `wait()`                            | 等待所有部件运动完成 |
| `is_any_moving`                     | 是否有部件在运动中   |

### Arm (手臂)

每只手臂 7 个关节，**所有角度自动限制在安全范围内**：


| 索引 | 关节   | 方法                                    | 限位 (rad)     |
| ---- | ------ | --------------------------------------- | -------------- |
| 0    | 肩俯仰 | `set_shoulder_pitch(angle, block=True)` | -2.61 ~ 0.78   |
| 1    | 肩偏航 | `set_shoulder_yaw(angle, block=True)`   | -0.213 ~ 2.029 |
| 2    | 肩翻滚 | `set_shoulder_roll(angle, block=True)`  | -1.57 ~ 1.57   |
| 3    | 肘俯仰 | `set_elbow_pitch(angle, block=True)`    | -0.78 ~ 1.57   |
| 4    | 腕翻滚 | `set_wrist_roll(angle, block=True)`     | -1.57 ~ 1.57   |
| 5    | 腕俯仰 | `set_wrist_pitch(angle, block=True)`    | -0.52 ~ 0.52   |
| 6    | 腕偏航 | `set_wrist_yaw(angle, block=True)`      | -1.57 ~ 1.57   |

其他方法：

- `set_joints(positions, block=True)` - 设置全部 7 个关节（弧度）
- `set_joint(index, angle, block=True)` - 设置单个关节（索引 0-6）
- `get_limit(index)` - 获取指定关节限位 `(lower, upper)`
- `limits` - 获取所有关节限位列表
- `home(block=True)` - 回到零位
- `wait()` - 等待当前运动完成
- `is_moving` - 是否正在运动中
- `set_speed(speed)` - 设置关节速度 (rad/s)
- `ik(x, y, z, roll, pitch, yaw, block=True, abs=True)` - 末端位姿控制 (IK)。
  - `abs=True`: 绝对位姿 (位置: m, 姿态: rad)。会自动重置内部目标点。
  - `abs=False`: 相对增量 (位置: 全局 m, 姿态: 局部 rad)。基于内部维护的目标位姿进行累积，支持连续调用。

### Gripper (夹爪)

支持阻塞/非阻塞模式：


| 方法                            | 说明                          |
| ------------------------------- | ----------------------------- |
| `set_position(pos, block=True)` | 设置位置 (0.0=闭合, 1.0=张开) |
| `open(block=True)`              | 完全张开                      |
| `close(block=True)`             | 完全闭合                      |
| `half_open(block=True)`         | 半开                          |
| `wait()`                        | 等待运动完成                  |
| `is_moving`                     | 是否正在运动中                |

### Head (头部)

头部有限位保护，支持阻塞/非阻塞模式：

- **pitch (俯仰)**: -0.7 ~ 0.2 rad
- **yaw (偏航)**: -1.57 ~ 1.57 rad


| 方法                                | 说明                                               |
| ----------------------------------- | -------------------------------------------------- |
| `set_pose(pitch, yaw, block=True)`  | 设置姿态（弧度）                                   |
| `set_pitch(angle, block=True)`      | 设置俯仰角                                         |
| `set_yaw(angle, block=True)`        | 设置偏航角                                         |
| `look_left(angle=0.5, block=True)`  | 向左看                                             |
| `look_right(angle=0.5, block=True)` | 向右看                                             |
| `look_up(angle=0.3, block=True)`    | 向上看                                             |
| `look_down(angle=0.3, block=True)`  | 向下看                                             |
| `center(block=True)`                | 回中                                               |
| `wait()`                            | 等待运动完成                                       |
| `is_moving`                         | 是否正在运动中                                     |
| `limits`                            | 获取限位`{'pitch': (min, max), 'yaw': (min, max)}` |

### Waist (腰部)

腰部是 prismatic 直线关节，控制上半身高度：

- **高度范围**: -0.62m ~ 0.24m


| 方法                             | 说明           |
| -------------------------------- | -------------- |
| `set_height(height, block=True)` | 设置高度（米） |
| `up(delta=0.05, block=True)`     | 上升指定距离   |
| `down(delta=0.05, block=True)`   | 下降指定距离   |
| `home(block=True)`               | 回到默认高度   |
| `wait()`                         | 等待运动完成   |
| `is_moving`                      | 是否正在运动中 |

### Chassis (底盘)

**安全设计**：所有运动命令必须指定距离或角度，运动完成后自动停止，避免失控。


| 方法                                            | 说明               |
| ----------------------------------------------- | ------------------ |
| `forward(distance, speed=1.5, block=True)`      | 前进指定距离（米） |
| `backward(distance, speed=1.5, block=True)`     | 后退指定距离（米） |
| `strafe_left(distance, speed=1.5, block=True)`  | 左移指定距离（米） |
| `strafe_right(distance, speed=1.5, block=True)` | 右移指定距离（米） |
| `turn_left(angle_deg, speed=0.3, block=True)`   | 左转指定角度（度） |
| `turn_right(angle_deg, speed=0.3, block=True)`  | 右转指定角度（度） |
| `move(x, y, angle, block=True)`                 | 组合运动           |
| `set_friction(linear, angular)`                 | 设置摩擦补偿系数   |
| `stop()`                                        | 立即停止           |
| `wait()`                                        | 等待运动完成       |
| `is_moving`                                     | 是否正在运动中     |

**默认速度**：线速度 1.0 m/s，角速度 0.3 rad/s

**摩擦补偿**：

```python
# 地面摩擦力大，机器人走不到指定距离时，增加摩擦系数
robot.chassis.set_friction(linear=1.5, angular=2.0)
# linear: 线性运动补偿，值越大运动时间越长
# angular: 旋转运动补偿，值越大运动时间越长
```

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

    # IK 控制 (新增)
    # 绝对位姿控制，移动左臂到指定位置 (x, y, z) 和姿态 (roll, pitch, yaw)
    robot.left_arm.ik(0.5, 0.2, 0.3, 0, 0, 0, True)
    
    # 增量控制，移动左臂在目前的位姿上增量
    robot.left_arm.ik(0.5, 0.2, 0.3, 0, 0, 0, False)

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
    robot.chassis.forward(1.0, speed=2.0)
  
    # 组合运动：边走边转
    robot.chassis.move(x=0.5, y=0.2, angle=45)
  
    # 地面摩擦力大时，设置补偿系数
    robot.chassis.set_friction(linear=1.5, angular=2.0)
    robot.chassis.forward(1.0)  # 现在能走到 1 米了
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

**启动 Bridge 节点**（根据使用场景选择）：

```bash
# ===== 实机控制 =====
# 实机控制
# 1) 进入遥操作文件夹
cd ~/bw_system_ws
# 2) 修改.env文件
# 修改文件中的内容不是终端运行
RUN_MODE=sdk
# 3) 运行启动脚本
./run.sh
```

> ⚠️ **注意**: v1.2.0 起不再需要 zenoh-bridge-ros2dds

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
│   ├── ik_solver.py    # ik求解器
│   └── constants.py    # 关节限位常量
├── test_sim.py         # 仿真测试脚本
├── test_real.py        # 实机测试脚本
├── test_chassis.py     # 底盘测试脚本
├── test_gripper.py     # 夹爪测试脚本
└── README.md
```

## 注意事项

1. **角度单位**：所有角度均为弧度（rad），底盘旋转角度为度（°）
2. **速度单位**：线速度 m/s，角速度 rad/s
3. **夹爪范围**：0.0（闭合）到 1.0（张开）
4. **腰部范围**：-0.62m ~ 0.24m
5. **底盘安全**：运动完成后自动停止，无需手动调用 `stop()`
6. **摩擦补偿**：地面摩擦力大时，使用 `set_friction()` 增加运动时间补偿
7. **阻塞模式**：默认 `block=True`，使用 `block=False` 实现并行运动
8. **连接超时**：默认 5 秒，可通过 `connect(timeout=10)` 调整
9. **跳过验证**：调试时可用 `connect(verify=False)` 跳过连接验证

---

**BlueWorm-EAI-Tech**
