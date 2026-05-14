import argparse
import time

from connection_selector import add_connection_args, connect_robot_with_selector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="连接机器人并读取 IP")
    add_connection_args(parser, default_profile="interactive")
    return parser.parse_args()


def main():
    args = parse_args()
    print("正在连接机器人...")

    robot = None
    try:
        robot = connect_robot_with_selector(args, script_name=__file__)
        if robot is None:
            return

        if robot.is_connected:
            print(f"连接成功！")
            print(f"机器人 IP: {robot.robot_ip}")

            # 等待几秒看看 IP 是否稳定
            time.sleep(2)
            print(f"再次确认 IP: {robot.robot_ip}")
        else:
            print("连接失败")
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass

if __name__ == "__main__":
    main()
