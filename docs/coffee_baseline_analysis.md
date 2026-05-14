# coffee.py baseline 静态审计

## 2.1 脚本总体结论

`coffee.py` 本质上是一个固定场景下的关节空间动作复现脚本，而不是一个自主任务系统。

- 它直接在脚本里硬编码了大量关节角和夹爪开合位置，靠顺序执行复现既定流程，而不是通过环境感知实时决策。
- 它主要调用的是单关节级别 API，如 `set_shoulder_pitch()`、`set_elbow_pitch()`、`set_wrist_roll()`、`set_position()`、`home()`，没有使用视觉、力控、碰撞检测、轨迹优化或任务规划能力。
- 它虽然依赖新版 SDK 的阻塞/非阻塞接口，但动作编排仍然是“命令 + `time.sleep()` + 下一条命令”的脚本式控制。
- 它没有使用 `Arm.ik(...)`，因此不是基于笛卡尔空间目标位姿的轨迹执行。
- 它没有状态机、没有人工确认点、没有抓取成功判断，也没有对咖啡杯、奶杯、咖啡机进行显式标定。

结论上，这份脚本更接近“示教后整理成代码的固定动作回放”，适合做 baseline 复现理解，不适合直接作为长期维护的工程化任务执行入口。

## 2.2 动作阶段拆解

### 阶段总览

| 阶段 | 行号 | 主要部件 | 主要 API | 动作意图 |
| --- | --- | --- | --- | --- |
| CONNECT | 29-32 | 整机 | `Mantis(...)`, `connect()` | 建立到指定机器人 SN 的连接 |
| HEAD_PREPARE | 34 | 头部 | `head.look_down()` | 头部下视，准备执行台面任务 |
| LOOP_START | 36-37 | 整机流程 | `for`, `print` | 进入重复执行循环 |
| RIGHT_HAND_GRASP_COFFEE_CUP | 38-50 | 右臂、双夹爪 | `close()`, `open()`, 多个 `set_*()`, `set_position()` | 右手靠近并抓取咖啡杯 |
| RIGHT_HAND_MOVE_TO_COFFEE_MACHINE | 52-66 | 右臂 | 多个 `set_*()` | 将右手咖啡杯移动到咖啡机接取位置 |
| LEFT_HAND_PRESS_BUTTON | 69-78 | 左臂 | 多个 `set_*()`, `home()` | 左手点击咖啡机屏幕/按钮 |
| RIGHT_HAND_RETREAT_AFTER_COFFEE | 80-83 | 右臂 | `home()`, `set_*()` | 咖啡完成后右臂后撤 |
| LEFT_HAND_GRASP_MILK_CUP | 85-101 | 左臂、左夹爪 | `open()`, 多个 `set_*()`, `set_position()` | 左手靠近并抓取奶杯 |
| LEFT_HAND_LIFT_MILK_CUP | 103-110 | 左臂 | `set_*()`, `home(block=False)` | 将奶杯抬起并调整倾倒准备姿态 |
| RIGHT_HAND_MOVE_TO_RECEIVE_MILK | 112-117 | 右臂 | 多个 `set_*()` | 右手端咖啡杯到接奶位置 |
| LEFT_HAND_POUR_MILK | 119-130 | 左臂 | 多个 `set_*()` | 左手执行倒奶动作 |
| RIGHT_HAND_RELEASE_COFFEE_CUP | 132-145 | 右臂、右夹爪 | `home()`, 多个 `set_*()`, `open()` | 右手回到放杯位并释放咖啡杯 |
| LEFT_HAND_RELEASE_MILK_CUP | 147-168 | 左臂、左夹爪 | `home()`, 多个 `set_*()`, `open()` | 左手回到放奶位并释放奶杯 |
| DISCONNECT | 171 | 整机 | `disconnect()` | 断开连接 |

### 1. CONNECT

- 行号：29-32
- 控制部件：整机连接层
- 涉及 API：
  - `Mantis(sn="BW_3N5CRT22")`
  - `robot.connect(timeout=8, verify=True)`
- 动作意图：按固定 SN 连接真实机器人，并在连接失败时直接退出。
- `block=False`：无
- `time.sleep()`：无
- 安全风险：
  - 连接目标硬编码，误连到错误设备时没有二次确认。
  - 顶层脚本导入即执行，没有入口保护。
  - 成功连接后立刻进入动作流程，没有人工确认。

### 2. HEAD_PREPARE

- 行号：34
- 控制部件：头部
- 涉及 API：
  - `robot.head.look_down(angle=0.5, block=True)`
- 动作意图：让头部朝向台面区域，形成固定的任务起始观察姿态。
- `block=False`：未使用
- `time.sleep()`：无
- 安全风险：
  - 仅靠固定角度下视，没有检查当前头部姿态和周围障碍。
  - 即便头部朝向变化异常，后续任务仍会继续。

### 3. LOOP_START

- 行号：36-37
- 控制部件：整机流程
- 涉及 API：
  - `for i in range(200)`
  - `print(f"第{i}次测试")`
- 动作意图：重复执行整套咖啡流程 200 次。
- `block=False`：无
- `time.sleep()`：无
- 安全风险：
  - 高重复次数意味着一次定位误差、抓取偏差或杯具放置偏差会被不断放大。
  - 任何一次执行中途状态异常，下一轮仍会继续开始。

### 4. RIGHT_HAND_GRASP_COFFEE_CUP

- 行号：38-50
- 控制部件：右臂、右夹爪，同时重置左夹爪
- 涉及 API：
  - `robot.right_gripper.close()`
  - `robot.left_gripper.close()`
  - `robot.right_gripper.open()`
  - `robot.right_arm.set_shoulder_pitch(..., block=False)`
  - `robot.right_arm.set_shoulder_roll(..., block=False)`
  - `robot.right_arm.set_wrist_roll(..., block=True)`
  - `robot.right_arm.set_elbow_pitch(..., block=False)`
  - `robot.right_arm.set_wrist_pitch(...)`
  - `robot.right_gripper.set_position(0.6)`
- 动作意图：右手张开夹爪、靠近杯子、调整末端姿态并闭合到半握位置抓取咖啡杯。
- `block=False`：有，出现在右肩俯仰、右肩翻滚、右肘俯仰上
- `time.sleep()`：48、50 行
- 安全风险：
  - 多个关节并行运动但没有空间约束检查。
  - `set_position(0.6)` 只是发送夹爪目标，没有抓取成功反馈。
  - `sleep(1)` 假定杯子和运动耗时稳定，遇到不同摩擦或机械延迟容易失配。

### 5. RIGHT_HAND_MOVE_TO_COFFEE_MACHINE

- 行号：52-66
- 控制部件：右臂
- 涉及 API：
  - 多次 `robot.right_arm.set_elbow_pitch(...)`
  - 多次 `robot.right_arm.set_shoulder_pitch(...)`
  - 多次 `robot.right_arm.set_shoulder_roll(...)`
  - 多次 `robot.right_arm.set_wrist_roll(...)`
  - `robot.right_arm.set_wrist_pitch(0)`
- 动作意图：右手持杯从抓取位移动到咖啡机出液/接咖啡位置。
- `block=False`：有，出现在肩俯仰、肘俯仰、腕翻滚
- `time.sleep()`：无
- 安全风险：
  - 完全依赖预设关节角，没有杯子相对咖啡机位置校验。
  - 若初始抓杯姿态偏差，持杯进入咖啡机区域时可能碰撞。
  - 无杯口朝向精确控制，只是假设当前关节组合足够接咖啡。

### 6. LEFT_HAND_PRESS_BUTTON

- 行号：69-78
- 控制部件：左臂
- 涉及 API：
  - `robot.left_arm.set_shoulder_pitch(..., block=False)`
  - `robot.left_arm.set_elbow_pitch(..., block=True)`
  - `robot.left_arm.home()`
- 动作意图：左手抬起，向前轻触按钮或屏幕，再回到默认位。
- `block=False`：有，出现在左肩俯仰
- `time.sleep()`：78 行
- 安全风险：
  - “点击”仅靠肘关节角度变化，不知道末端是否真正接触到按钮。
  - 没有力控或接触检测，可能按不到，也可能过冲。
  - `home()` 直接回零，没有绕障路径规划。

### 7. RIGHT_HAND_RETREAT_AFTER_COFFEE

- 行号：80-83
- 控制部件：右臂
- 涉及 API：
  - `robot.right_arm.home()`
  - `robot.right_arm.set_shoulder_roll(0.6, block=False)`
  - `robot.right_arm.set_wrist_pitch(-0.3)`
- 动作意图：咖啡接取后右手从咖啡机前方撤离，准备转入加奶阶段。
- `block=False`：有，出现在右肩翻滚
- `time.sleep()`：无
- 安全风险：
  - `home()` 会先回零，再补一个后撤姿态，路径不一定安全。
  - 如果杯中已经有液体，快速姿态变化可能导致洒漏。

### 8. LEFT_HAND_GRASP_MILK_CUP

- 行号：85-101
- 控制部件：左臂、左夹爪
- 涉及 API：
  - `robot.left_gripper.open()`
  - `robot.left_arm.set_shoulder_yaw(..., block=False)`
  - `robot.left_arm.set_wrist_roll(..., block=False)`
  - `robot.left_arm.set_shoulder_roll(..., block=True)`
  - `robot.left_arm.set_shoulder_pitch(..., block=False)`
  - `robot.left_arm.set_elbow_pitch(..., block=False)`
  - `robot.left_arm.set_wrist_roll(..., block=True)`
  - `robot.left_gripper.set_position(0.6)`
- 动作意图：左手移动到奶杯位置并抓取奶杯。
- `block=False`：有，出现在左肩偏航、左腕翻滚、左肩俯仰、左肘俯仰
- `time.sleep()`：99、101 行
- 安全风险：
  - 没有奶杯坐标标定，位置稍偏就可能抓空或碰倒杯子。
  - `set_position(0.6)` 同样没有抓紧确认。
  - 抓取前没有确认左手是否已完全避开周边障碍。

### 9. LEFT_HAND_LIFT_MILK_CUP

- 行号：103-110
- 控制部件：左臂
- 涉及 API：
  - `robot.left_arm.set_elbow_pitch(0.9, block=True)`
  - `robot.left_arm.set_shoulder_pitch(0.2, block=False)`
  - `robot.left_arm.set_elbow_pitch(0.4, block=True)`
  - `robot.left_arm.home(block=False)`
  - `robot.left_arm.set_wrist_pitch(-0.45, block=True)`
- 动作意图：提起奶杯并把手腕调整到后续倒奶预备姿态。
- `block=False`：有，出现在左肩俯仰和 `left_arm.home(block=False)`
- `time.sleep()`：无
- 安全风险：
  - 抬杯路径仍是关节空间组合，不保证杯体始终竖直。
  - `home(block=False)` 与后续手腕动作叠加，时序依赖很强。
  - 如果奶杯没抓稳，提起阶段就是掉杯高风险点。

### 10. RIGHT_HAND_MOVE_TO_RECEIVE_MILK

- 行号：112-117
- 控制部件：右臂
- 涉及 API：
  - `robot.right_arm.set_wrist_yaw(..., block=False)`
  - `robot.right_arm.set_wrist_pitch(..., block=False)`
  - `robot.right_arm.set_wrist_roll(..., block=False)`
  - `robot.right_arm.set_shoulder_roll(..., block=False)`
- 动作意图：右手调整杯口位置，让咖啡杯移动到接奶位置。
- `block=False`：全段均使用
- `time.sleep()`：117 行
- 安全风险：
  - 全段非阻塞，只靠最后 `sleep(1)` 等待，依赖经验时间而非反馈。
  - 没有确认咖啡杯是否真正到达接奶位，也没有杯口姿态检测。

### 11. LEFT_HAND_POUR_MILK

- 行号：119-130
- 控制部件：左臂
- 涉及 API：
  - `robot.left_arm.set_shoulder_pitch(..., block=False)`
  - `robot.left_arm.set_elbow_pitch(..., block=True/False)`
  - `robot.left_arm.set_shoulder_roll(..., block=False)`
  - `robot.left_arm.set_wrist_roll(..., block=False)`
- 动作意图：左手执行固定的倒奶动作，把奶倒入右手所持咖啡杯。
- `block=False`：有，出现在左肩俯仰、左肩翻滚、左腕翻滚、最后一次左肘俯仰
- `time.sleep()`：无
- 安全风险：
  - 这不是拉花轨迹，只是固定角度组合的倾倒动作。
  - 没有流量、液面、杯口相对位姿反馈，奶流是否落入杯中无法确认。
  - `set_wrist_roll(1.7)` 靠近大幅翻转，若实际姿态有偏差容易洒漏。

### 12. RIGHT_HAND_RELEASE_COFFEE_CUP

- 行号：132-145
- 控制部件：右臂、右夹爪
- 涉及 API：
  - `robot.right_arm.home()`
  - 多个 `robot.right_arm.set_*()`
  - `robot.right_gripper.open()`
- 动作意图：右手把咖啡杯送回放置位并张开夹爪释放。
- `block=False`：有，出现在右肩俯仰、右肩翻滚、右肘俯仰
- `time.sleep()`：140、142、145 行
- 安全风险：
  - 仍然假设放杯位置完全固定。
  - 没有“已放稳再松爪”的接触判断，存在悬空松爪风险。
  - `home()` 与释放动作之间没有显式安全检查。

### 13. LEFT_HAND_RELEASE_MILK_CUP

- 行号：147-168
- 控制部件：左臂、左夹爪
- 涉及 API：
  - `robot.left_arm.home()`
  - 多个 `robot.left_arm.set_*()`
  - `robot.left_gripper.open()`
  - `robot.left_arm.home(block=False)`
- 动作意图：左手回到奶杯放回位，张开夹爪释放奶杯，再回零。
- `block=False`：有，出现在左肩偏航、左腕翻滚、左肩俯仰、左肘俯仰、`left_arm.home(block=False)`
- `time.sleep()`：160、168 行
- 安全风险：
  - 放杯流程与抓杯流程一样依赖固定场景，无位置确认。
  - 释放后立刻再做 `home(block=False)`，若奶杯未稳定会带倒容器。

### 14. DISCONNECT

- 行号：171
- 控制部件：整机连接层
- 涉及 API：
  - `robot.disconnect()`
- 动作意图：动作全部完成后断开机器人连接。
- `block=False`：无
- `time.sleep()`：无
- 安全风险：
  - 只有正常走到文件末尾才会断开；中途异常不会保证执行到这里。

## 2.3 风险点分析

### `for i in range(200)` 连续重复执行风险

- 36 行的 `for i in range(200)` 会把同一套硬编码动作执行 200 次。
- 这类脚本没有“场景重置成功”检测，也没有上一轮是否收尾正确的判断。
- 一旦某一轮杯子摆位、夹爪闭合、按钮点击或奶杯放回发生偏差，下一轮仍会在错误初始状态上继续执行。
- 对实机而言，这种长循环会显著提高碰撞、掉杯、洒液和机构热负荷风险。

### IP 硬编码风险

- 当前文件实际硬编码的是 SN：`Mantis(sn="BW_3N5CRT22")`（29 行），并非 IP。
- 但从工程风险角度看，IP 硬编码和 SN 硬编码属于同一类问题：部署目标写死在脚本中，缺少环境配置层。
- 一旦设备更换、网络变更、实验台切换，脚本就需要改源码，不利于维护和审批。

### 没有 `if __name__ == "__main__"` 入口保护

- `coffee.py` 顶层直接连接、直接执行动作。
- 任何导入、静态调试、误运行都可能触发真实动作。
- 这也是实机脚本最需要优先修复的结构性风险之一。

### 没有 try/finally 保护

- 脚本没有 `try/finally` 包裹主流程。
- 如果中途某条命令抛异常，`robot.disconnect()`（171 行）可能不会执行。
- 更重要的是，没有统一的异常收尾动作，例如停止、归位、松爪、记录错误状态。

### 没有真正的异常急停机制

- 脚本层面没有急停按钮监听、人工中断处理或异常停机策略。
- SDK 里虽然有 `stop()`，但这里并没有在异常路径或危险条件触发时调用。
- 且 `Mantis.stop()` 当前只明确停止底盘，不等价于双臂即时急停。

### 没有人工单步确认

- 全流程从连接到倒奶结束都是自动连续执行。
- 没有“抓杯成功后确认”“咖啡机前对位确认”“接奶位确认”“准备倒奶确认”等关键停点。
- 对带液体的双臂协作任务来说，这会明显放大风险。

### 没有参数配置文件

- 关节角、夹爪位置、等待时间、循环次数、连接目标都写死在脚本里。
- 这导致代码与场景参数强耦合，不利于版本管理、实验记录和快速回退。

### 没有状态机

- 当前只是线性顺序脚本，没有 `IDLE -> GRASP_CUP -> PRESS_BUTTON -> ...` 这种显式状态流转。
- 因此没有失败分支、重试分支、人工介入分支，也没有“当前走到哪一步”的可观测结构。

### 没有杯子、奶杯、咖啡机坐标标定

- 脚本只使用关节角，不维护任何工作空间坐标。
- 杯子、奶杯、咖啡机一旦移动，整套脚本就可能失效。
- 这也是它无法扩展到拉花轨迹的根本原因之一，因为拉花本质上要求对杯口相对位姿有更精确表达。

### 没有夹爪抓取成功判断

- `set_position(0.6)` 只是设定目标开度，不代表真的抓住了物体。
- 没有夹爪电流、力反馈、位移异常或二次视觉确认。
- 因此抓空、夹偏、夹滑在代码层面都不可见。

### 没有视觉或力控反馈

- 没有相机识别杯口、按钮、奶杯，也没有接触力或液面反馈。
- 整个流程属于 open-loop 执行，只能在场景不变、误差很小的前提下工作。

### 没有真实拉花轨迹，只是倒奶动作

- 119-130 行是固定姿态倾倒，不是控制奶缸相对杯口连续运动的拉花轨迹。
- 它没有定义杯口坐标系、轨迹采样、速度约束或图案参数，因此当前阶段不能称为“拉花”。

## 2.4 拉花扩展点

后续咖啡拉花应接在现有加奶链路中，而不是另起一套完全无关的流程。

- 左手拿到牛奶杯之后：对应 `LEFT_HAND_LIFT_MILK_CUP` 完成后，也就是 103-110 行之后，左手已经具备执行奶缸轨迹的物理前提。
- 右手把咖啡杯移动到接奶位置之后：对应 `RIGHT_HAND_MOVE_TO_RECEIVE_MILK` 完成后，也就是 112-117 行之后，右手已把咖啡杯移到接奶位。
- 原 `LEFT_HAND_POUR_MILK` 阶段应被替换或扩展为拉花轨迹执行：当前 119-130 行应在后续被抽象成“进入拉花起始位 -> 执行轨迹 -> 收尾离杯”。

推荐的后续插入点如下：

1. 保留 `LEFT_HAND_LIFT_MILK_CUP` 作为奶缸准备段。
2. 保留 `RIGHT_HAND_MOVE_TO_RECEIVE_MILK` 作为咖啡杯就位段。
3. 将 `LEFT_HAND_POUR_MILK` 替换为：
   - `LEFT_HAND_APPROACH_POUR_START`
   - `LATTE_PATTERN_EXECUTION`
   - `LEFT_HAND_RETREAT_AFTER_POUR`

## 额外静态观察

- 文件存在重复导入：`time`、`sys`、`threading`、`Mantis` 多次重复导入（10、19-26 行）。
- 注释与脚本用途存在轻微错位：打印语句写的是“全关节方向测试”，但实际内容是完整咖啡流程复现。
- 这些问题不会直接改变动作结果，但说明脚本更像临时实验脚本，而非已整理的工程入口。
