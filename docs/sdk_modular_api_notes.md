# Mantis SDK 模块化能力说明

## 3.1 Mantis 主类

`Mantis` 是新版 SDK 的统一入口，负责连接管理、状态订阅、全量状态发布和各子模块装配。

### `Mantis(ip=...)`

- `mantis/mantis.py` 的构造函数支持 `ip` 参数，也支持 `port` 和 `sn`。
- 传入 `ip` 时，SDK 会把它作为目标地址候选，并在 `connect()` 时用于身份解析与连接。
- README 也给出了 `Mantis(ip="192.168.1.100", port=7447)` 的用法。

### `Mantis(sn=...)`

- 构造函数同样支持 `sn` 参数。
- 当前 `coffee.py` 使用的是 `Mantis(sn="BW_3N5CRT22")`，说明新版 SDK 已经允许按机器人编号建立目标选择，而不是只能写固定 IP。

### `connect()`

- `connect()` 定义在 [mantis.py](/home/lanchong/BlueWorm_ws/Mantis-SDK-github/mantis/mantis.py:293)。
- 它做的事情不只是“打开 socket”，而是一个带校验的连接流程：
  - 基于全局 `/sn` 身份话题解析目标机器人。
  - 为 joint/chassis 等控制话题自动拼接 `<SN>/sdk/*` 前缀。
  - 可通过 `verify=True` 订阅 `<SN>/sdk/system_status` 做二次在线校验。
- 这意味着新版 SDK 已经把“发现目标”和“连接目标”统一进了主类。

### `disconnect()`

- `disconnect()` 定义在 [mantis.py](/home/lanchong/BlueWorm_ws/Mantis-SDK-github/mantis/mantis.py:449)。
- 它会停止底盘、取消发布者/订阅者、关闭 Zenoh 会话并清理内部状态。
- 在工程化脚本里，应把它放进 `finally` 或上下文管理器退出路径。

### `home()`

- `home()` 定义在 [mantis.py](/home/lanchong/BlueWorm_ws/Mantis-SDK-github/mantis/mantis.py:588)。
- 它会统一调用：
  - `left_arm.home(block=False)`
  - `right_arm.home(block=False)`
  - `head.center(block=False)`
  - `waist.home(block=False)`
  - `left_gripper.close(block=False)`
  - `right_gripper.close(block=False)`
- 这是一个整机级“回默认位”能力，适合做安全复位或流程初始化。

### `wait()`

- `wait()` 定义在 [mantis.py](/home/lanchong/BlueWorm_ws/Mantis-SDK-github/mantis/mantis.py:606)。
- 它不是简单的固定延时，而是依据 `system_status.motion_states` 轮询等待。
- `joint_names=None` 时等待所有部件，传入关节名列表时可只等待指定部件。
- 对工程化脚本而言，`wait()` 是比 `time.sleep()` 更应优先使用的同步手段。

### `is_any_moving`

- `is_any_moving` 定义在 [mantis.py](/home/lanchong/BlueWorm_ws/Mantis-SDK-github/mantis/mantis.py:668)。
- 它本质上调用 `is_moving()`，用于判断当前是否有任一部件仍处于运动状态。
- 这为“执行下一步前确认全机已停稳”提供了统一接口。

### `subscribe_status()`

- `subscribe_status()` 定义在 [mantis.py](/home/lanchong/BlueWorm_ws/Mantis-SDK-github/mantis/mantis.py:709)。
- 作用是订阅系统状态反馈，把机器人返回的状态保存到 `self._system_status`，也可以把数据转发给业务回调。
- `connect()` 成功后已经自动调用 `subscribe_status(None)`，所以主类内部的 `wait()`、`is_moving()` 才能依据状态工作。

## 3.2 子模块控制

新版 SDK 已经把各部件能力拆成明确的子控制器，调用入口都挂在 `robot` 实例上。

### `robot.left_arm`

- 类型是 `Arm`。
- 负责左臂 7 自由度关节控制、回零、等待、IK。

### `robot.right_arm`

- 类型是 `Arm`。
- 与 `left_arm` 对称，用于右臂 7 自由度控制。

### `robot.left_gripper`

- 类型是 `Gripper`。
- 负责左夹爪开合和位置控制。

### `robot.right_gripper`

- 类型是 `Gripper`。
- 负责右夹爪开合和位置控制。

### `robot.head`

- 类型是 `Head`。
- 负责头部 pitch/yaw 控制，例如 `look_down()`、`center()`。

### `robot.waist`

- 类型是 `Waist`。
- 负责腰部升降，单位是米。

### `robot.chassis`

- 类型是 `Chassis`。
- 负责前后、横移、转向等底盘运动。
- 当前 `coffee.py` 没有用到底盘，但这部分已经是模块化 SDK 的正式能力。

## 3.3 手臂控制方式

### 单关节控制：`set_shoulder_pitch()` 等

- `Arm` 在文件尾部动态生成了 `set_shoulder_pitch()`、`set_shoulder_yaw()`、`set_shoulder_roll()`、`set_elbow_pitch()`、`set_wrist_roll()`、`set_wrist_pitch()`、`set_wrist_yaw()`。
- 这些方法底层都会走 `set_joint()`，适合做示教式关节脚本。
- `coffee.py` 几乎全部使用的就是这一层。

### 多关节控制：`set_joints([...])`

- `set_joints()` 定义在 [arm.py](/home/lanchong/BlueWorm_ws/Mantis-SDK-github/mantis/arm.py:182)。
- 一次性发送一整条手臂的 7 个关节目标，比多次 `set_*()` 更适合做“姿态点”级的组织。
- 它还会自动执行限位约束，并同步 IK 内部目标状态。

### IK 控制：`ik(x, y, z, roll, pitch, yaw, block=True, abs=True)`

- `ik()` 定义在 [arm.py](/home/lanchong/BlueWorm_ws/Mantis-SDK-github/mantis/arm.py:272)。
- 它支持两种模式：
  - `abs=True`：按绝对末端位姿解 IK。
  - `abs=False`：按相对增量位姿解 IK。
- 这意味着新版 SDK 已经具备把“杯口上方某个位姿”表达成笛卡尔空间目标的能力，后续拉花和更稳定的接杯动作可以逐步迁移到这里。

### `block=True` 和 `block=False` 的区别

- 所有子模块基本都遵循同一语义：
  - `block=True`：发出命令后等待该部件动作完成。
  - `block=False`：发出命令后立刻返回，允许与其他关节或其他部件并行。
- 例如 `Arm._execute_motion()`、`Gripper._execute_motion()`、`Head._execute_motion()` 都是通过是否调用各自的 `wait()` 来实现的。
- 在实际工程中：
  - `block=True` 更容易保证时序，适合关键动作点。
  - `block=False` 更适合构造并行动作，但需要配合 `robot.wait()` 或显式阶段同步。

### `robot.wait()` 的作用

- `robot.wait()` 是整机级同步屏障。
- 它允许我们先发送多个 `block=False` 命令，再在一个统一时机等待所有相关部件到位。
- 这比在业务层散落大量 `time.sleep()` 更稳健，也更接近可维护工程的写法。

## 3.4 SN / IP 连接方式

### IP 是网络地址

- IP 反映的是当前网络上的接入地址。
- 优点是直接、易理解。
- 缺点是换网络、换路由、换设备部署后容易变化。

### SN 是机器人编号

- SN 更像设备身份标识，而不是临时网络地址。
- 对多机环境或长期维护来说，SN 通常比固定 IP 更稳定。

### 新版 SDK 支持通过 `/sn` 发现机器人

- `RobotDiscovery` 定义在 [discovery.py](/home/lanchong/BlueWorm_ws/Mantis-SDK-github/mantis/discovery.py:28)。
- `start()` 会订阅 `/sn` 话题并维护在线机器人列表。
- `list_robots()` 可返回当前 `[{sn, ip}, ...]`。
- README 也明确给出了 `RobotDiscovery.start()`、`RobotDiscovery.list_robots()`、`RobotDiscovery.stop()` 的用法。
- `Mantis.connect()` 本身也会通过身份解析逻辑利用这一机制找到目标机器人。

### 后续工程建议：优先支持 SN 配置，IP 作为备用配置

- 建议把业务配置分成两层：
  - 首选 `sn`
  - 备用 `ip`
- 推荐流程：
  1. 若配置中给出 `sn`，优先按 `sn` 连接。
  2. 若 `sn` 发现失败，再根据实验环境允许情况回退到 `ip`。
  3. 把实际连接到的 `robot_sn` 和 `robot_ip` 记录到运行日志。

## 3.5 旧 coffee.py 与模块化工程的对应关系

下表给出从旧脚本到后续可维护工程的推荐映射：

| 旧 `coffee.py` 内容 | 后续模块化归属 | 说明 |
| --- | --- | --- |
| 顶层 `Mantis(...)` + `connect()` + `disconnect()` | `robot_client.py` | 统一连接、断开、状态订阅、异常收尾 |
| `head.look_down()`、`left/right_arm.home()` 这类准备动作 | `prepare_actions.py` | 独立成可复用的准备阶段 |
| 右手拿咖啡杯动作段 | `coffee_actions.py` | 抽象成 `grasp_coffee_cup()` |
| 左手按按钮动作段 | `coffee_actions.py` | 抽象成 `press_coffee_button()` |
| 左手拿奶杯动作段 | `milk_actions.py` | 抽象成 `grasp_milk_cup()` |
| 当前倒奶动作 | `latte_pattern.py` + `trajectory.py` | 未来替换为真正的拉花轨迹执行 |
| 所有硬编码角度、夹爪值、等待时间 | `config/*.yaml` | 场景参数、设备参数、动作参数解耦 |
| 当前顺序脚本 | `task_state_machine.py` | 支持单步、重试、失败分支、人工确认 |
| 大量 `time.sleep()` | `safety_waits.py` 或统一同步层 | 优先改为 `wait()`、状态确认和超时控制 |
| 固定循环 `for i in range(200)` | `runner.py` | 由任务运行器决定是否循环、循环次数和人工确认策略 |

### 推荐的工程拆分方向

可以把后续工程化改造成如下结构：

```text
project/
├── config/
│   ├── robot.yaml
│   ├── coffee_scene.yaml
│   └── latte_pattern.yaml
├── robot_client.py
├── coffee_actions.py
├── milk_actions.py
├── trajectory.py
├── latte_pattern.py
├── task_state_machine.py
└── run_coffee_safe_replay.py
```

### 为什么这比旧脚本更可维护

- 连接逻辑和动作逻辑分离后，换设备不会改动作文件。
- 场景参数进 YAML 后，调参可以版本化、可追溯。
- 动作按阶段封装后，可以单步执行并插入人工确认。
- 拉花轨迹与抓杯/接杯动作分离后，后续可以只迭代“图案执行层”，不用反复改整套脚本。

## 对 `coffee.py` 的模块化结论

`coffee.py` 代表的是“基于新版 SDK 接口写出来的旧式硬编码流程脚本”。它已经使用了新版 SDK 的模块入口，如 `robot.right_arm`、`robot.left_gripper`、`robot.head`，但还没有真正利用新版 SDK 的工程化能力，包括：

- 统一状态同步：`wait()`、`is_any_moving`、`subscribe_status()`
- 设备发现：`sn` 与 `/sn` 发现机制
- 更高级的位姿表达：`Arm.ik(...)`
- 更清晰的模块边界：左臂、右臂、夹爪、头部、腰部、底盘

因此，下一阶段不是重写 SDK，而是把这些现成能力从“零散调用”提升到“带配置、带状态、带安全停点的任务工程”。
