"""
腰部位移测试
在 RViz 中预览腰部上下移动

使用前请确保:
1. 启动纯仿真环境: ros2 launch bw_sim2real sdk_sim.launch.py
2. 启动 zenoh 桥接: ~/zenoh_ros2/zenoh-bridge-ros2dds -d 99

注意: Waist_Joint 是 prismatic 类型，范围 -0.62m 到 0.24m
"""

from mantis import Mantis
import time

def main():
    print("=== Mantis 腰部位移测试 ===\n")
    print("Waist_Joint: prismatic 关节, 范围 -0.62m 到 0.24m")
    print("负值 = 下降, 正值 = 上升\n")
    robot = Mantis(ip="192.168.1.111")
    robot.connect(verify=False)
    print("开始演示腰部动作...\n")
    
    # 1. 回到零位
    print("1. 腰部零位 (0.0m)")
    robot.waist.home()
    time.sleep(1)
    
    # 2. 逐步上升
    print("2. 逐步上升 (每次 5cm)")
    for i in range(5):
        robot.waist.up()  # 每次上升 5cm
        print(f"   当前高度: {robot.waist.height:.2f}m")
        time.sleep(0.5)
    
    # 3. 逐步下降
    print("3. 逐步下降 (每次 10cm)")
    for i in range(5):
        robot.waist.down(0.1)  # 每次下降 10cm
        print(f"   当前高度: {robot.waist.height:.2f}m")
        time.sleep(0.5)
    
    # # 4. 渐变回零位
    # print("4. 渐变回零位")
    # for i in range(10):
    #     pos = -0.62 + (0.62 * (i + 1) / 10)
    #     robot.waist.set_height(pos)
    #     print(f"   位置: {pos:.2f}m")
    #     time.sleep(0.2)
    
    # # 5. 使用相对移动
    # print("5. 相对移动测试")
    # robot.waist.home()
    # time.sleep(0.5)
    # robot.waist.move(0.1)  # 上升 10cm
    # print("   上升 10cm")
    # time.sleep(0.5)
    # robot.waist.move(-0.2)  # 下降 20cm
    # print("   下降 20cm")
    # time.sleep(0.5)
    
    # 6. 回零
    print("6. 保持零位")
    robot.waist.home()
    time.sleep(1)
    
    print("\n演示完成！")


if __name__ == "__main__":
    main()
