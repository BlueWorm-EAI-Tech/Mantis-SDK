from __future__ import annotations

import argparse
import threading
import time

from examples.common import add_robot_arguments, connected_robot


def main() -> int:
    parser = argparse.ArgumentParser(description="Subscribe to robot status and measure update frequency.")
    add_robot_arguments(parser)
    parser.add_argument("--duration", type=float, default=5.0, help="Measurement duration in seconds.")
    args = parser.parse_args()

    count = 0
    lock = threading.Lock()

    def on_status(_data):
        nonlocal count
        with lock:
            count += 1

    with connected_robot(args) as robot:
        robot.subscribe_status(on_status)
        start = time.time()
        time.sleep(max(args.duration, 0.1))
        elapsed = time.time() - start

    with lock:
        total = count

    hz = total / elapsed if elapsed > 0 else 0.0
    print(f"messages={total}")
    print(f"elapsed={elapsed:.2f}s")
    print(f"frequency={hz:.2f}Hz")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
