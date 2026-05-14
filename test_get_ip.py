import argparse
import time

from mantis import Mantis


DEFAULT_ROBOT_IP = "192.168.1.151"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="连接机器人并读取 IP")
    parser.add_argument(
        "--ip",
        default=DEFAULT_ROBOT_IP,
        help="目标机器人 IP，默认 192.168.1.151",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    print("正在连接机器人...")
    print(f"当前按 IP 连接: {args.ip}")

    robot = None
    try:
        robot = Mantis()
        ok = robot.connect(ip=args.ip)
        if not ok:
            raise SystemExit("连接失败，停止测试")

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
