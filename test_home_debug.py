"""
调试零位问题
"""

from mantis import Mantis
import time

def main():
    print("=== 调试零位问题 ===\n")
    
    robot = Mantis(ip="192.168.1.111")
    robot.connect(verify=False)
    
    # 先归零
    print("初始归零...")
    robot.left_arm.home()
    time.sleep(3)  # 等待足够长时间
    
    # 检查内部状态
    print(f"\n--- 归零后状态 ---")
    print(f"left_arm._positions: {robot.left_arm._positions}")
    print(f"SDK._real_arm_positions[0:7]: {robot._real_arm_positions[0:7]}")
    
    # 设置一个角度
    angle = 0.3
    print(f"\n设置 shoulder_pitch = {angle}...")
    robot.left_arm.set_shoulder_pitch(angle)
    time.sleep(2)
    
    print(f"--- 设置后状态 ---")
    print(f"left_arm._positions: {robot.left_arm._positions}")
    print(f"SDK._real_arm_positions[0:7]: {robot._real_arm_positions[0:7]}")
    
    # 再次归零
    print("\n再次归零...")
    robot.left_arm.home()
    
    # 逐秒检查收敛情况
    for i in range(5):
        time.sleep(1)
        print(f"t={i+1}s: _real_arm_positions[0] = {robot._real_arm_positions[0]:.6f}")
    
    print(f"\n--- 最终状态 ---")
    print(f"left_arm._positions: {robot.left_arm._positions}")
    print(f"SDK._real_arm_positions[0:7]: {robot._real_arm_positions[0:7]}")
    
    robot.disconnect()
    print("\n测试完成!")

if __name__ == "__main__":
    main()
