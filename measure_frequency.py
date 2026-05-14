import argparse
import time
import threading

from connection_selector import add_connection_args, connect_robot_with_selector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="测量机器人状态更新频率")
    add_connection_args(parser, default_profile="interactive")
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="测量时长（秒）",
    )
    return parser.parse_args()


class FrequencyMeasurer:
    def __init__(self):
        self.count = 0
        self.start_time = 0
        self.lock = threading.Lock()
        
    def callback(self, data):
        with self.lock:
            self.count += 1
            
    def measure(self, robot, duration=5.0):
        print(f"开始测量状态更新频率 (持续 {duration} 秒)...")
        robot.subscribe_status(self.callback)

        self.start_time = time.time()
        self.count = 0

        time.sleep(duration)

        elapsed = time.time() - self.start_time
        avg_freq = self.count / elapsed

        print(f"\n测量结果:")
        print(f"收到消息数: {self.count}")
        print(f"耗时: {elapsed:.2f} 秒")
        print(f"平均频率: {avg_freq:.2f} Hz")
        print(f"平均间隔: {1000/avg_freq:.2f} ms" if avg_freq > 0 else "平均间隔: N/A")


def main() -> None:
    args = parse_args()
    measurer = FrequencyMeasurer()
    robot = None
    try:
        robot = connect_robot_with_selector(args, script_name=__file__)
        if robot is None:
            return
        measurer.measure(robot, duration=args.duration)
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    main()
