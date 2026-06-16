from __future__ import annotations

import argparse
import time

from mantis import RobotDiscovery, list_discovered_robots, start_robot_discovery, stop_robot_discovery


def main() -> int:
    parser = argparse.ArgumentParser(description="Discover robots publishing the Zenoh sn topic.")
    parser.add_argument("--router-ip", help="Optional Zenoh router IP.")
    parser.add_argument("--router-port", type=int, default=7447)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--ttl-sec", type=float, default=3.0)
    args = parser.parse_args()

    def on_change(robots):
        print("robots changed:", robots)

    RobotDiscovery.register_callback(on_change)
    start_robot_discovery(
        router_ip=args.router_ip,
        router_port=args.router_port,
        ttl_sec=args.ttl_sec,
    )
    try:
        end = time.time() + max(args.duration, 0.1)
        while time.time() < end:
            print("robots:", list_discovered_robots())
            time.sleep(1.0)
    finally:
        RobotDiscovery.unregister_callback(on_change)
        stop_robot_discovery()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
