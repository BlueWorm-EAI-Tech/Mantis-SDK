"""
底盘调试测试
"""

from mantis import Mantis
import time

def main():
    print("=== 底盘调试测试 ===\n")
    
    robot = Mantis(ip="192.168.1.111")
    robot.connect(verify=False)
    
    print(f"初始状态:")
    print(f"  vx = {robot.chassis._vx}")
    print(f"  vy = {robot.chassis._vy}")
    print(f"  omega = {robot.chassis._omega}")
    
    time.sleep(1)
    
    print(f"\n执行 forward(0.3)...")
    print(f"发送前: vx={robot.chassis._vx}, vy={robot.chassis._vy}")
    
    # 手动设置速度并发布，看看是否有效
    robot.chassis._vx = 0.1
    robot.chassis._vy = 0.0
    robot.chassis._omega = 0.0
    robot._publish_chassis()
    
    print(f"发送后: vx={robot.chassis._vx}, vy={robot.chassis._vy}")
    print("等待 3 秒观察底盘是否移动...")
    time.sleep(3)
    
    # 停止
    robot.chassis.stop()
    print("已停止")
    
    time.sleep(1)
    
    print(f"\n测试左转...")
    robot.chassis._vx = 0.0
    robot.chassis._vy = 0.0
    robot.chassis._omega = 0.3
    robot._publish_chassis()
    print(f"omega = {robot.chassis._omega}")
    print("等待 3 秒观察底盘是否旋转...")
    time.sleep(3)
    
    robot.chassis.stop()
    print("已停止")
    
    robot.disconnect()
    print("\n测试完成!")

if __name__ == "__main__":
    main()
