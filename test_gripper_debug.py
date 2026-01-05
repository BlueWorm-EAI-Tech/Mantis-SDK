"""
夹爪调试测试
"""

from mantis import Mantis
import time

def main():
    print("=== 夹爪调试测试 ===\n")
    
    robot = Mantis(ip="192.168.1.111")
    robot.connect(verify=False)
    
    print(f"初始状态:")
    print(f"  left_gripper._position = {robot.left_gripper._position}")
    print(f"  _real_gripper_positions = {robot._real_gripper_positions}")
    
    time.sleep(1)
    
    print(f"\n执行 close()...")
    robot.left_gripper.close()
    time.sleep(2)
    
    print(f"close 后状态:")
    print(f"  left_gripper._position = {robot.left_gripper._position}")
    print(f"  _real_gripper_positions = {robot._real_gripper_positions}")
    print(f"  发送的 URDF 值 = {robot._real_gripper_positions[0] * 0.04}")
    
    print(f"\n执行 open()...")
    robot.left_gripper.open()
    time.sleep(2)
    
    print(f"open 后状态:")
    print(f"  left_gripper._position = {robot.left_gripper._position}")
    print(f"  _real_gripper_positions = {robot._real_gripper_positions}")
    print(f"  发送的 URDF 值 = {robot._real_gripper_positions[0] * 0.04}")
    
    print(f"\n执行 half_open()...")
    robot.left_gripper.half_open()
    time.sleep(2)
    
    print(f"half_open 后状态:")
    print(f"  left_gripper._position = {robot.left_gripper._position}")
    print(f"  _real_gripper_positions = {robot._real_gripper_positions}")
    print(f"  发送的 URDF 值 = {robot._real_gripper_positions[0] * 0.04}")
    
    robot.disconnect()
    print("\n测试完成!")

if __name__ == "__main__":
    main()
