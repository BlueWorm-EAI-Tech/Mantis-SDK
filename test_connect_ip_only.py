import argparse

from connection_selector import add_connection_args, connect_robot_with_selector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="仅通过连接选择器连接机器人并立即断开")
    add_connection_args(parser, default_profile="interactive")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    robot = None
    try:
        robot = connect_robot_with_selector(args, script_name=__file__)
        if robot is None:
            return
        print("connect result: True")
    finally:
        if robot is not None:
            print("准备断开连接")
            robot.disconnect()
            print("disconnect done")


if __name__ == "__main__":
    main()
