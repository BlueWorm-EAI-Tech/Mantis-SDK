from __future__ import annotations

import argparse
import json

from examples.common import add_robot_arguments, connected_robot, print_robot_identity


def main() -> int:
    parser = argparse.ArgumentParser(description="Connect to a Mantis robot by IP or SN.")
    add_robot_arguments(parser)
    args = parser.parse_args()

    with connected_robot(args) as robot:
        print_robot_identity(robot)
        print("supports_ik=", robot.supports_ik)
        print("system_status=")
        print(json.dumps(robot.system_status, ensure_ascii=False, indent=2))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
