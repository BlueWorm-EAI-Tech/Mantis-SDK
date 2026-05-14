import argparse
import time
import threading

from mantis import Mantis


DEFAULT_ROBOT_IP = "192.168.1.151"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="测量机器人状态更新频率")
    parser.add_argument(
        "--ip",
        default=DEFAULT_ROBOT_IP,
        help="目标机器人 IP，默认 192.168.1.151",
    )
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
            
    def measure(self, duration=5.0, ip: str = DEFAULT_ROBOT_IP):
        print(f"开始测量状态更新频率 (持续 {duration} 秒)...")

        robot = None
        try:
            robot = Mantis()
            ok = robot.connect(ip=ip)
            if not ok:
                raise SystemExit("连接失败，停止测试")

            # 注册回调
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
        finally:
            if robot is not None:
                try:
                    robot.disconnect()
                except Exception:
                    pass

if __name__ == "__main__":
    args = parse_args()
    measurer = FrequencyMeasurer()
    measurer.measure(duration=args.duration, ip=args.ip)
