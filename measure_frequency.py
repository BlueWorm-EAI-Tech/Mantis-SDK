
import time
import threading
from mantis import Mantis
import sys

class FrequencyMeasurer:
    def __init__(self):
        self.count = 0
        self.start_time = 0
        self.lock = threading.Lock()
        
    def callback(self, data):
        with self.lock:
            self.count += 1
            
    def measure(self, duration=5.0):
        print(f"开始测量状态更新频率 (持续 {duration} 秒)...")
        
        robot = Mantis(ip="192.168.1.112")
        # 手动连接以避免 home() 等操作干扰
        if not robot.connect(verify=True):
            print("连接失败")
            return

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
        
        robot.disconnect()

if __name__ == "__main__":
    measurer = FrequencyMeasurer()
    measurer.measure()
