#!/usr/bin/env python3
"""
诊断脚本：检查 Zenoh 侧是否能读到机器人身份话题 sn。

用法：
  /bin/python test_sn_read.py
  /bin/python test_sn_read.py --topic /sn --duration 30
  /bin/python test_sn_read.py --router-ip 192.168.1.111 --router-port 7447
"""

from __future__ import annotations

import argparse
import json
import time

import zenoh


def normalize_topic(topic: str) -> str:
    normalized = (topic or "").strip().strip("/")
    if not normalized:
        raise ValueError("topic 不能为空，且不能只包含 '/'")
    return normalized


def main() -> int:
    parser = argparse.ArgumentParser(description="读取并诊断 Zenoh sn 话题数据")
    parser.add_argument("--topic", default="sn", help="话题名，支持 sn 或 /sn")
    parser.add_argument("--router-ip", default=None, help="指定 Zenoh router IP（可选）")
    parser.add_argument("--router-port", type=int, default=7447, help="Zenoh router 端口")
    parser.add_argument("--duration", type=float, default=15.0, help="监听时长（秒）")
    args = parser.parse_args()

    topic = normalize_topic(args.topic)
    print(f"[INFO] subscribe topic: {topic}")

    config = zenoh.Config()
    if args.router_ip:
        endpoint = f"tcp/{args.router_ip}:{args.router_port}"
        config.insert_json5("connect/endpoints", f'["{endpoint}"]')
        print(f"[INFO] connect endpoint: {endpoint}")
    else:
        print("[INFO] connect mode: auto-discovery")

    session = zenoh.open(config)
    msg_count = 0

    def on_msg(sample) -> None:
        nonlocal msg_count
        msg_count += 1
        payload_bytes = sample.payload.to_bytes()
        ts = time.strftime("%H:%M:%S")
        print(f"\n[{ts}] message #{msg_count}, bytes={len(payload_bytes)}")

        try:
            text = payload_bytes.decode("utf-8")
            print(f"  utf8: {text}")
        except Exception as e:
            print(f"  utf8 decode failed: {e}")
            print(f"  raw: {payload_bytes[:64]!r}")
            return

        try:
            obj = json.loads(text)
            print(f"  json: {obj}")
            if isinstance(obj, dict):
                print(f"  sn={obj.get('sn')}, ip={obj.get('ip')}")
        except Exception as e:
            print(f"  json parse failed: {e}")

    subscriber = session.declare_subscriber(topic, on_msg)
    print(f"[INFO] listening for {args.duration:.1f}s ...")

    try:
        end_time = time.time() + max(args.duration, 0.1)
        while time.time() < end_time:
            time.sleep(0.1)
    finally:
        try:
            subscriber.undeclare()
        except Exception:
            pass
        try:
            session.close()
        except Exception:
            pass

    if msg_count == 0:
        print("[WARN] 未收到任何消息")
        print("[HINT] 请检查机器人端是否在发布 /sn，以及 ROS->Zenoh 是否有桥接到 key `sn`")
    else:
        print(f"[INFO] 收到消息总数: {msg_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
