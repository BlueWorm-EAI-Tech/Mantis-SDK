"""
Mantis 3.0 腰部控制示例
演示滑台上下和上半身前后弯腰的位置控制。

使用前请确保:
1. 实机侧已重新编译并启动:
   - bw_sdk_bridge
   - bw_status_machine
   - mantis_comm_node
2. SDK 与机器人在同一 Zenoh 网络

注意:
- 仅适用于 robot_version="3.0"
- `set_height()` 直接控制滑台目标位置，单位 m
- `set_bend()` 直接控制上半身前后弯腰目标角度，单位 rad
- `set_speed()` 设置滑台最大速度，单位 m/s
- `set_bend_speed()` 设置弯腰最大角速度，单位 rad/s
- 弯腰角度单位是 rad
- 负值 = 前倾, 正值 = 后仰
- 示例里的 sleep 只是给实机动作留出观察时间，不参与控制量计算
"""

import time

from mantis import Mantis


ROBOT_IP = "192.168.1.123"


def main():
    print("=== Mantis 3.0 腰部控制示例 ===\n")
    robot = Mantis(ip=ROBOT_IP, robot_version="3.0")
    robot.connect()

    try:
        robot.waist.set_speed(0.08)
        robot.waist.set_bend_speed(0.2)

        print("1. 回到默认姿态: 滑台 0.00m, 身体直立 0.00rad")
        robot.waist.home(block=True)

        # print("2. 滑台目标位置: 上升 5cm")
        # robot.waist.set_height(0.05, block=True)

        print("3. 滑台目标位置: 下降 10cm")
        robot.waist.set_height(-0.10, block=True)

        print("4. 上半身目标角度: 前倾弯腰 0.35rad")
        robot.waist.set_bend(-0.35, block=True)

        # print("5. 上半身目标角度: 回到直立 0.00rad")
        # robot.waist.set_bend(0.0, block=True)

        # print("6. 上半身目标角度: 后仰 0.05rad")
        # robot.waist.set_bend(0.05, block=True)

        # print("7. 滑台和上半身回到默认姿态")
        # robot.waist.home(block=True)

        print("\n演示完成。")
    finally:
        # 异常退出时也先发回正命令，避免身体保持前倾/后仰。
        robot.waist.set_bend(0.0, block=False)
        robot.waist.set_height(0.0, block=False)


if __name__ == "__main__":
    main()
