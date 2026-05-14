import argparse
import time
from typing import Optional

from mantis import Mantis


DEFAULT_SN = "BW_3N5CRT22"


# 左手调试起始姿态。该值来自 coffee.py 中“左手拿起奶杯后、进入倒奶前”
# 的局部关节动作（约第 109-125 行），这里只保留倒奶段调试所需的保守关节值，
# 不等价于完整拿奶杯流程。
LEFT_START_SHOULDER_PITCH = -0.50
LEFT_START_SHOULDER_ROLL = 0.58
LEFT_START_ELBOW_PITCH = -0.58
LEFT_START_WRIST_PITCH = -0.45
LEFT_START_WRIST_ROLL = 1.10

# 左手简化拉花 demo 参数。这里是关节空间示教参数，不是真正基于杯口坐标的拉花算法。
LEFT_POUR_SHOULDER_PITCH = -0.60
LEFT_POUR_WRIST_ROLL_PREP = 1.35
LEFT_POUR_WRIST_ROLL_MAX = 1.55
# 当前拉花摆动仍然是关节空间 demo。
# LEFT_SWING_CYCLES 表示完整左右摆动次数；6 个 cycle 对应 12 个半程目标点。
# 如果 RViz/实机上摆动仍不明显，优先增大 LEFT_SWING_DELAY 或扩大 shoulder_roll 左右差值。
# 如果实机上有碰撞风险，优先减小 shoulder_roll 幅度。
LEFT_SWING_CYCLES = 6
LEFT_SWING_LEFT = (0.55, -0.55)
LEFT_SWING_RIGHT = (0.70, -0.45)
LEFT_SWING_DELAY = 0.45

# 右手静态接奶测试位姿。该值复用 coffee.py 中“右手后撤后等待左手倒奶”
# 的局部动作（约第 80-83、112-116 行），仅用于双臂相对位置联调，
# 不是完整咖啡流程。
RIGHT_RECEIVE_SHOULDER_PITCH = 0.0
RIGHT_RECEIVE_SHOULDER_YAW = 0.0
RIGHT_RECEIVE_SHOULDER_ROLL = 0.70
RIGHT_RECEIVE_ELBOW_PITCH = 0.0
RIGHT_RECEIVE_WRIST_ROLL = 0.30
RIGHT_RECEIVE_WRIST_PITCH = -0.50
RIGHT_RECEIVE_WRIST_YAW = -0.70


class UserAbort(Exception):
    """Raised when the operator cancels before or during the tune flow."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mantis 倒奶/拉花段分模式调试工具（仅关节空间 demo，不运行完整 coffee 流程）"
    )
    parser.add_argument(
        "--mode",
        default="left-only",
        choices=("left-only", "with-right-cup"),
        help="调试模式：left-only 只调左手；with-right-cup 加入右手接奶位姿联调",
    )
    parser.add_argument(
        "--sn",
        default=DEFAULT_SN,
        help="目标机器人 SN，默认沿用 coffee.py 当前配置",
    )
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="跳过人工确认，仅保留打印提示",
    )
    return parser.parse_args()


def print_global_safety_banner() -> None:
    print("=" * 72)
    print("危险提示：当前脚本会控制真实机器人。")
    print("第一次建议空壶、空杯。")
    print("第二次建议少量水。")
    print("最后再用牛奶/奶泡。")
    print("请清空机器人周围障碍物。")
    print("请确认杯子、奶壶、咖啡机不会被碰撞。")
    print("请准备物理急停。")
    print("当前脚本不是稳定产品，只是宣传 demo 前的动作调试工具。")
    print("本脚本不包含抓杯/拿壶流程，请先人工确认夹持状态。")
    print("=" * 72)


def print_mode_safety(mode: str) -> None:
    if mode == "left-only":
        print("[模式] left-only")
        print("- 这是实机左手倒奶/拉花动作包络调试。")
        print("- 建议第一次空壶测试。")
        print("- 不验证右手杯口相对位置。")
        print("- 请确认周围安全。")
        print("- 请准备物理急停。")
        return

    if mode == "with-right-cup":
        print("[模式] with-right-cup")
        print("- 这是左右手相对位置联调。")
        print("- 右手会移动到接奶位置。")
        print("- 左手会执行倒奶/拉花动作。")
        print("- 不执行完整 coffee 流程。")
        print("- 建议第一次空杯、空壶测试。")
        print("- 第二次少量水测试。")
        print("- 最后再用牛奶/奶泡。")
        print("- 请确认右手夹持杯子安全。")
        print("- 请准备物理急停。")
        return

    raise SystemExit(f"不支持的模式: {mode}")


def confirm_or_exit(message: str, skip_confirm: bool) -> None:
    if skip_confirm:
        print(f"[跳过确认] {message}")
        return

    print(message)
    user_input = input("按 Enter 继续，输入 q 退出: ").strip().lower()
    if user_input == "q":
        raise UserAbort("用户取消执行")


def go_to_latte_start_pose(robot: Mantis) -> None:
    print("左手进入拉花前调试起始姿态...")
    robot.left_arm.set_wrist_pitch(LEFT_START_WRIST_PITCH, block=False)
    robot.left_arm.set_shoulder_pitch(LEFT_START_SHOULDER_PITCH, block=False)
    robot.left_arm.set_elbow_pitch(-0.60, block=True)
    robot.left_arm.set_shoulder_roll(LEFT_START_SHOULDER_ROLL, block=False)
    robot.left_arm.set_wrist_roll(LEFT_START_WRIST_ROLL, block=True)
    robot.left_arm.set_elbow_pitch(LEFT_START_ELBOW_PITCH, block=True)
    time.sleep(0.3)


def move_right_arm_to_receive_milk_pose(robot: Mantis) -> None:
    print("右手进入接奶测试位姿...")
    robot.right_arm.set_shoulder_pitch(RIGHT_RECEIVE_SHOULDER_PITCH, block=False)
    robot.right_arm.set_shoulder_yaw(RIGHT_RECEIVE_SHOULDER_YAW, block=False)
    robot.right_arm.set_elbow_pitch(RIGHT_RECEIVE_ELBOW_PITCH, block=False)
    robot.right_arm.set_shoulder_roll(0.60, block=False)
    robot.right_arm.set_wrist_pitch(-0.30, block=True)
    robot.right_arm.set_wrist_yaw(RIGHT_RECEIVE_WRIST_YAW, block=False)
    robot.right_arm.set_wrist_roll(RIGHT_RECEIVE_WRIST_ROLL, block=False)
    robot.right_arm.set_wrist_pitch(RIGHT_RECEIVE_WRIST_PITCH, block=False)
    robot.right_arm.set_shoulder_roll(RIGHT_RECEIVE_SHOULDER_ROLL, block=True)
    time.sleep(0.3)
    print("右手已到接奶测试位姿，请观察杯口高度、夹持稳定性和周围间隙。")


def latte_art_pour_demo(robot: Mantis) -> None:
    print("开始执行简化倒奶/拉花 demo...")

    # 进入接近原始倒奶姿态，再逐步增大 wrist_roll 进入倒奶角。
    robot.left_arm.set_shoulder_pitch(LEFT_POUR_SHOULDER_PITCH, block=False)
    robot.left_arm.set_elbow_pitch(-0.52, block=False)
    robot.left_arm.set_shoulder_roll(0.62, block=False)
    robot.left_arm.set_wrist_roll(LEFT_POUR_WRIST_ROLL_PREP, block=True)
    time.sleep(0.3)

    robot.left_arm.set_wrist_roll(LEFT_POUR_WRIST_ROLL_MAX, block=True)
    time.sleep(0.3)

    for cycle_idx in range(LEFT_SWING_CYCLES):
        print(f"拉花摆动 {cycle_idx + 1}/{LEFT_SWING_CYCLES}: left")
        robot.left_arm.set_shoulder_roll(LEFT_SWING_LEFT[0], block=False)
        robot.left_arm.set_elbow_pitch(LEFT_SWING_LEFT[1], block=True)
        time.sleep(LEFT_SWING_DELAY)

        print(f"拉花摆动 {cycle_idx + 1}/{LEFT_SWING_CYCLES}: right")
        robot.left_arm.set_shoulder_roll(LEFT_SWING_RIGHT[0], block=False)
        robot.left_arm.set_elbow_pitch(LEFT_SWING_RIGHT[1], block=True)
        time.sleep(LEFT_SWING_DELAY)

    # 逐步停止倒奶，避免直接从最大倒奶角回摆。
    robot.left_arm.set_shoulder_roll(0.55, block=False)
    robot.left_arm.set_elbow_pitch(-0.52, block=False)
    robot.left_arm.set_wrist_roll(1.10, block=True)
    time.sleep(0.3)

    robot.left_arm.set_wrist_roll(0.25, block=True)
    robot.left_arm.set_shoulder_roll(0.25, block=True)
    time.sleep(0.3)


def recover_left_arm_after_pour(robot: Mantis) -> None:
    print("左手执行倒奶段后的局部恢复...")
    robot.left_arm.set_elbow_pitch(-0.35, block=False)
    robot.left_arm.set_wrist_roll(0.0, block=False)
    robot.left_arm.set_shoulder_roll(0.15, block=True)
    robot.left_arm.set_shoulder_pitch(-0.30, block=True)
    time.sleep(0.3)


def main() -> None:
    args = parse_args()
    robot: Optional[Mantis] = None

    try:
        print_global_safety_banner()
        print_mode_safety(args.mode)

        confirm_or_exit("连接前确认：确认环境安全、夹持稳定，并准备好物理急停。", args.no_confirm)

        robot = Mantis(sn=args.sn)
        ok = robot.connect(verify=True)
        if not ok:
            raise SystemExit("连接失败，停止调试")

        if args.mode == "with-right-cup":
            move_right_arm_to_receive_milk_pose(robot)
            confirm_or_exit(
                "右手已到接奶测试位姿。请确认杯子安全、杯口无遮挡，再继续左手动作。",
                args.no_confirm,
            )

        go_to_latte_start_pose(robot)
        confirm_or_exit("左手即将执行倒奶/拉花动作，请确认奶壶轨迹范围内没有风险。", args.no_confirm)
        latte_art_pour_demo(robot)
        recover_left_arm_after_pour(robot)
        print("调试流程执行结束。")
    except UserAbort:
        print("用户取消，脚本退出。")
    except KeyboardInterrupt:
        print("\n检测到 Ctrl-C，脚本中止。")
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception as exc:
                print(f"断开连接时忽略异常: {exc}")


if __name__ == "__main__":
    main()
