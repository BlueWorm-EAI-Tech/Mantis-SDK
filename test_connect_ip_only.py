from mantis import Mantis

ROBOT_IP = "192.168.1.151"

robot = None

try:
    print(f"准备按 IP 连接机器人: {ROBOT_IP}")
    robot = Mantis()
    ok = robot.connect(ip=ROBOT_IP)
    print("connect result:", ok)
finally:
    if robot is not None:
        print("准备断开连接")
        robot.disconnect()
        print("disconnect done")
