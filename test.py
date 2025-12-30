from mantis_sdk import Mantis
import time

with Mantis() as robot:
    # 控制左臂肩关节
    robot.left_arm.set_shoulder_pitch(0.5)
    
    # 控制夹爪（使用 left_gripper 或 right_gripper）
    robot.left_gripper.open()
    
    # 控制头部
    robot.head.look_up()
    
    # 保持连接 3 秒，观察效果
    time.sleep(3)