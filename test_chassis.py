"""
底盘运动测试
============

测试 Mantis 机器人底盘的各种运动方式。

新版 API（基于距离/角度，更安全）：
- forward(distance) - 前进指定米数
- backward(distance) - 后退指定米数  
- strafe_left(distance) - 左移指定米数
- strafe_right(distance) - 右移指定米数
- turn_left(degrees) - 左转指定角度
- turn_right(degrees) - 右转指定角度
- move(x, y, angle) - 组合运动

使用前请确保:
1. 启动 Gazebo 仿真: ros2 launch bw_sim2real sdk_gazebo.launch.py
2. 或启动 RViz 仿真: ros2 launch bw_sim2real sdk_sim.launch.py
3. 启动 zenoh 桥接: ~/zenoh_ros2/zenoh-bridge-ros2dds -d 99
"""

from mantis import Mantis
import time


def test_basic_movement(robot):
    """测试基本运动：前进、后退、左右移动、旋转"""
    print("\n=== 测试基本运动 ===\n")
    robot.chassis.set_friction(linear=3.0, angular=2.0)

    # 前进
    print("1. 前进 0.3 米")
    robot.chassis.forward(1.0)
    time.sleep(0.5)
    
    # # 后退
    # print("2. 后退 0.3 米")
    # robot.chassis.backward(1.0)
    # time.sleep(0.5)
    
    # # 左移
    # print("3. 左移 0.2 米")
    # robot.chassis.strafe_left(1.0)
    # time.sleep(0.5)
    
    # # 右移
    # print("4. 右移 0.2 米")
    # robot.chassis.strafe_right(1.0)
    # time.sleep(0.5)
    
    # 左转
    # print("5. 左转 90 度")
    # robot.chassis.turn_left(90)
    # time.sleep(0.5)
    
    # # 右转
    # print("6. 右转 90 度")
    # robot.chassis.turn_right(90)
    # time.sleep(0.5)
    
    # print("基本运动测试完成！")


def test_custom_speed(robot):
    """测试自定义速度"""
    print("\n=== 测试自定义速度 ===\n")
    
    print("1. 慢速前进 0.5 米 (0.05 m/s)")
    robot.chassis.forward(0.5, speed=0.05)
    time.sleep(0.5)
    
    print("2. 快速前进 0.5 米 (0.2 m/s)")
    robot.chassis.forward(0.5, speed=0.2)
    time.sleep(0.5)
    
    print("3. 快速左转 180 度 (1.0 rad/s)")
    robot.chassis.turn_left(180, speed=1.0)
    time.sleep(0.5)
    
    print("速度测试完成！")


def test_square_path(robot):
    """测试走正方形路径"""
    print("\n=== 测试正方形路径 ===\n")
    
    for i in range(4):
        print(f"第 {i+1} 边：前进 0.3 米")
        robot.chassis.forward(0.3)
        time.sleep(0.3)
        
        print(f"第 {i+1} 个角：右转 90 度")
        robot.chassis.turn_right(90)
        time.sleep(0.3)
    
    print("正方形路径完成！")


def test_combined_movement(robot):
    """测试组合运动"""
    print("\n=== 测试组合运动 ===\n")
    
    print("1. 组合运动：前进0.3m + 左移0.2m + 左转45度")
    robot.chassis.move(x=0.3, y=0.2, angle=45)
    time.sleep(0.5)
    
    print("2. 斜向移动：前进0.3m + 右移0.3m")
    robot.chassis.move(x=0.3, y=-0.3)
    time.sleep(0.5)
    
    print("组合运动测试完成！")


def test_non_blocking(robot):
    """测试非阻塞模式"""
    print("\n=== 测试非阻塞模式 ===\n")
    
    print("1. 非阻塞前进 1 米（同时做其他事情）")
    robot.chassis.forward(1.0, block=False)
    
    for i in range(5):
        print(f"   主线程工作中... {i+1}/5")
        time.sleep(0.5)
    
    print("2. 等待运动完成...")
    while robot.chassis.is_moving:
        time.sleep(0.1)
    
    print("非阻塞测试完成！")


def interactive_control(robot):
    """交互式控制模式"""
    print("\n=== 交互式控制模式 ===")
    print("输入命令控制机器人:")
    print("  w: 前进 0.2m")
    print("  s: 后退 0.2m")
    print("  a: 左移 0.2m")
    print("  d: 右移 0.2m")
    print("  q: 左转 30 度")
    print("  e: 右转 30 度")
    print("  x: 紧急停止")
    print("  0: 退出")
    print()
    
    while True:
        try:
            cmd = input("命令> ").strip().lower()
            if cmd == '0':
                print("退出交互控制")
                break
            elif cmd == 'w':
                robot.chassis.forward(0.2)
            elif cmd == 's':
                robot.chassis.backward(0.2)
            elif cmd == 'a':
                robot.chassis.strafe_left(0.2)
            elif cmd == 'd':
                robot.chassis.strafe_right(0.2)
            elif cmd == 'q':
                robot.chassis.turn_left(30)
            elif cmd == 'e':
                robot.chassis.turn_right(30)
            elif cmd == 'x':
                robot.chassis.stop()
                print("已停止")
            else:
                print("未知命令，请重试")
        except KeyboardInterrupt:
            robot.chassis.stop()
            print("\n中断，已停止")
            break


def main():
    print("=" * 50)
    print("  Mantis 底盘运动测试（安全版 API）")
    print("=" * 50)
    
    print("\n选择测试模式:")
    print("  1. 基本运动测试（前进、后退、左右、旋转）")
    print("  2. 自定义速度测试")
    print("  3. 正方形路径测试")
    print("  4. 组合运动测试")
    print("  5. 非阻塞模式测试")
    print("  6. 交互式控制")
    print("  7. 运行所有测试")
    print()
    
    choice = input("请选择 (1-7): ").strip()
    robot = Mantis(ip="192.168.1.111")
    robot.connect(verify=False)

    print("\n机器人已连接，开始测试...\n")
    time.sleep(1)  # 等待连接稳定
    
    if choice == '1':
        test_basic_movement(robot)
    elif choice == '2':
        test_custom_speed(robot)
    elif choice == '3':
        test_square_path(robot)
    elif choice == '4':
        test_combined_movement(robot)
    elif choice == '5':
        test_non_blocking(robot)
    elif choice == '6':
        interactive_control(robot)
    elif choice == '7':
        test_basic_movement(robot)
        test_custom_speed(robot)
        test_square_path(robot)
        test_combined_movement(robot)
    else:
        print("无效选择，运行基本测试...")
        test_basic_movement(robot)
    
    print("\n测试完成！")


if __name__ == "__main__":
    main()
