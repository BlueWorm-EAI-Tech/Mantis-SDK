#!/usr/bin/env python3
"""
测试所有部件的阻塞/非阻塞运动模式
=================================

这个脚本演示如何使用阻塞/非阻塞模式来实现并行运动。

运行方式：
    1. 启动仿真环境（二选一）：
       
       # RViz 模式（快速预览，默认）
       cd ~/bw_motion_ws && source install/setup.bash
       ros2 launch bw_sim2real sdk_sim.launch.py
       
       # Gazebo 模式（物理仿真，底盘可移动）
       cd ~/bw_motion_ws && source install/setup.bash
       ros2 launch bw_sim2real sdk_sim.launch.py use_gazebo:=true
       
    2. 在另一终端运行测试：
       cd ~/mantis
       python test_parallel_motion.py
"""

from mantis import Mantis
import time


def test_sequential_motion(robot):
    """测试顺序阻塞运动。
    
    每个动作等待完成后再执行下一个（慢，但安全）。
    """
    print("\n" + "=" * 50)
    print("测试 1: 顺序运动（阻塞模式，默认）")
    print("=" * 50)
    
    start_time = time.time()
    
    print("  左臂肩部上抬...")
    robot.left_arm.set_shoulder_pitch(-0.5)  # block=True by default
    
    print("  右臂肩部上抬...")
    robot.right_arm.set_shoulder_pitch(-0.5)
    
    print("  头部左转...")
    robot.head.look_left(0.5)
    
    elapsed = time.time() - start_time
    print(f"  总耗时: {elapsed:.2f} 秒（顺序执行）")
    
    # 回原位
    robot.home()


def test_parallel_motion(robot):
    """测试并行非阻塞运动。
    
    多个动作同时启动，然后等待全部完成（快）。
    """
    print("\n" + "=" * 50)
    print("测试 2: 并行运动（非阻塞模式）")
    print("=" * 50)
    
    start_time = time.time()
    
    print("  同时启动所有运动...")
    robot.left_arm.set_shoulder_pitch(-0.5, block=False)
    robot.right_arm.set_shoulder_pitch(-0.5, block=False)
    robot.head.look_left(0.5, block=False)
    
    print("  等待所有运动完成...")
    robot.wait()
    
    elapsed = time.time() - start_time
    print(f"  总耗时: {elapsed:.2f} 秒（并行执行）")
    
    # 回原位
    robot.home()


def test_mixed_motion(robot):
    """测试混合模式运动。
    
    一些动作并行，另一些顺序执行。
    """
    print("\n" + "=" * 50)
    print("测试 3: 混合模式")
    print("=" * 50)
    
    print("  第一组：双臂并行运动")
    robot.left_arm.set_shoulder_pitch(-0.3, block=False)
    robot.right_arm.set_shoulder_pitch(-0.3, block=False)
    robot.wait()
    
    print("  第二组：头部 + 腰部并行")
    robot.head.look_down(0.2, block=False)
    robot.waist.up(0.05, block=False)
    robot.wait()
    
    print("  第三组：双手夹爪并行张开")
    robot.left_gripper.open(block=False)
    robot.right_gripper.open(block=False)
    robot.wait()
    
    print("  完成！")
    
    # 回原位
    robot.home()


def test_chassis_motion(robot):
    """测试底盘运动。
    
    底盘运动使用距离/角度控制，确保安全。
    """
    print("\n" + "=" * 50)
    print("测试 4: 底盘运动")
    print("=" * 50)
    
    print("  前进 0.3 米...")
    robot.chassis.forward(0.3)
    
    print("  左转 45 度...")
    robot.chassis.turn_left(45)
    
    print("  后退 0.3 米...")
    robot.chassis.backward(0.3)
    
    print("  右转 45 度...")
    robot.chassis.turn_right(45)
    
    print("  完成！")


def test_all_parallel(robot):
    """测试全身并行运动。
    
    所有部件同时运动。
    """
    print("\n" + "=" * 50)
    print("测试 5: 全身并行运动")
    print("=" * 50)
    
    start_time = time.time()
    
    print("  同时启动所有部件...")
    robot.left_arm.set_shoulder_pitch(-0.4, block=False)
    robot.right_arm.set_shoulder_pitch(-0.4, block=False)
    robot.head.look_left(0.3, block=False)
    robot.waist.up(0.05, block=False)
    robot.left_gripper.half_open(block=False)
    robot.right_gripper.half_open(block=False)
    robot.chassis.forward(0.2, block=False)
    
    print("  等待全部完成...")
    robot.wait()
    
    elapsed = time.time() - start_time
    print(f"  总耗时: {elapsed:.2f} 秒")
    
    # 回原位
    robot.home()


def main():
    print("=" * 60)
    print("  Mantis 阻塞/非阻塞运动模式测试")
    print("=" * 60)
    
    with Mantis(sim=True) as robot:
        print(f"\n已连接到机器人（仿真模式）")
        
        # 先回原位
        print("\n初始化：回到原位...")
        robot.home()
        time.sleep(1)
        
        # 运行各项测试
        test_sequential_motion(robot)
        time.sleep(1)
        
        test_parallel_motion(robot)
        time.sleep(1)
        
        test_mixed_motion(robot)
        time.sleep(1)
        
        test_chassis_motion(robot)
        time.sleep(1)
        
        test_all_parallel(robot)
        
        print("\n" + "=" * 60)
        print("  所有测试完成！")
        print("=" * 60)


if __name__ == "__main__":
    main()
