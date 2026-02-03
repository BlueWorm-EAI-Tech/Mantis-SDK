from mantis import Mantis
import time

def main():
    print("正在连接机器人...")
    # 使用自动发现模式
    with Mantis() as robot:
        if robot.is_connected:
            print(f"连接成功！")
            print(f"机器人 IP: {robot.robot_ip}")
            
            # 等待几秒看看 IP 是否稳定
            time.sleep(2)
            print(f"再次确认 IP: {robot.robot_ip}")
        else:
            print("连接失败")

if __name__ == "__main__":
    main()
