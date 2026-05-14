from mantis import RobotDiscovery
import time

# 无需实例化，直接类方法调用
RobotDiscovery.start(ttl_sec=3.0)

while 1:
    print(RobotDiscovery.list_robots())  # [{'sn': 'BW_XXXXXXX', 'ip': '192.168.1.111'}, ...]
    time.sleep(1)
RobotDiscovery.stop()