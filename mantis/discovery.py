"""
机器人发现模块
==============

通过 Zenoh 订阅 `/sn` 话题，维护局域网内在线机器人列表。
外部无需实例化，可直接调用类方法：

    RobotDiscovery.start()
    robots = RobotDiscovery.list_robots()
    RobotDiscovery.stop()
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional
import json
import threading
import time

try:
    import zenoh
except ImportError:
    raise ImportError("请安装 zenoh: pip install eclipse-zenoh")

from .constants import Topics


class RobotDiscovery:
    """局域网机器人发现器（静态类，无需实例化）。"""

    _DEFAULT_PORT = 7447

    _lock = threading.Lock()
    _session: Optional[zenoh.Session] = None
    _subscriber = None
    _cleanup_thread: Optional[threading.Thread] = None
    _stop_event = threading.Event()

    _robots: Dict[str, Dict[str, object]] = {}
    _callbacks: List[Callable[[List[dict]], None]] = []

    _running = False
    _topic = Topics.ROBOT_IDENTITY
    _router_ip: Optional[str] = None
    _router_port: int = _DEFAULT_PORT
    _ttl_sec = 3.0
    _cleanup_interval_sec = 0.5

    @classmethod
    def start(
        cls,
        topic: str = Topics.ROBOT_IDENTITY,
        router_ip: Optional[str] = None,
        router_port: int = _DEFAULT_PORT,
        ttl_sec: float = 3.0,
        cleanup_interval_sec: float = 0.5,
        callback: Optional[Callable[[List[dict]], None]] = None,
    ) -> None:
        """启动发现服务。

        Args:
            topic: Zenoh 发现话题，默认 `/sn`。
            router_ip: 指定 Zenoh router IP；None 为自动发现。
            router_port: Zenoh 端口，默认 7447。
            ttl_sec: 离线判定时间（秒），默认 3.0。
            cleanup_interval_sec: 清理周期（秒），默认 0.5。
            callback: 列表变化时回调，签名 `callback(robots: list[dict])`。
        """
        if ttl_sec <= 0:
            raise ValueError("ttl_sec 必须大于 0")
        if cleanup_interval_sec <= 0:
            raise ValueError("cleanup_interval_sec 必须大于 0")

        with cls._lock:
            if cls._running:
                cls._ttl_sec = ttl_sec
                cls._cleanup_interval_sec = cleanup_interval_sec
                if callback:
                    cls._callbacks.append(callback)
                    snapshot = cls._snapshot_locked()
                else:
                    snapshot = None
            else:
                cls._topic = cls._normalize_zenoh_topic(topic)
                cls._router_ip = router_ip
                cls._router_port = router_port
                cls._ttl_sec = ttl_sec
                cls._cleanup_interval_sec = cleanup_interval_sec
                if callback:
                    cls._callbacks.append(callback)
                snapshot = cls._snapshot_locked()
                cls._start_locked()

        if callback:
            cls._safe_callback(callback, snapshot or [])

    @classmethod
    def stop(cls) -> None:
        """停止发现服务并释放订阅资源。"""
        with cls._lock:
            if not cls._running:
                return
            cls._running = False
            cls._stop_event.set()

            subscriber = cls._subscriber
            session = cls._session
            cleanup_thread = cls._cleanup_thread

            cls._subscriber = None
            cls._session = None
            cls._cleanup_thread = None
            cls._callbacks.clear()

        if subscriber:
            try:
                subscriber.undeclare()
            except Exception:
                pass
        if session:
            try:
                session.close()
            except Exception:
                pass
        if cleanup_thread and cleanup_thread.is_alive():
            cleanup_thread.join(timeout=1.0)

    @classmethod
    def list_robots(cls) -> List[dict]:
        """返回当前在线机器人列表。"""
        with cls._lock:
            return cls._snapshot_locked()

    @classmethod
    def clear(cls) -> None:
        """清空已缓存机器人。"""
        with cls._lock:
            cls._robots.clear()
        cls._notify_change()

    @classmethod
    def register_callback(cls, callback: Callable[[List[dict]], None]) -> None:
        """注册机器人列表变化回调。"""
        with cls._lock:
            cls._callbacks.append(callback)
            snapshot = cls._snapshot_locked()
        cls._safe_callback(callback, snapshot)

    @classmethod
    def unregister_callback(cls, callback: Callable[[List[dict]], None]) -> None:
        """注销机器人列表变化回调。"""
        with cls._lock:
            cls._callbacks = [cb for cb in cls._callbacks if cb != callback]

    @classmethod
    def _start_locked(cls) -> None:
        config = zenoh.Config()
        if cls._router_ip:
            endpoint = f"tcp/{cls._router_ip}:{cls._router_port}"
            config.insert_json5("connect/endpoints", f'["{endpoint}"]')

        cls._session = zenoh.open(config)
        cls._stop_event.clear()
        cls._subscriber = cls._session.declare_subscriber(cls._topic, cls._on_sn)
        cls._cleanup_thread = threading.Thread(target=cls._cleanup_loop, daemon=True)
        cls._cleanup_thread.start()
        cls._running = True

    @staticmethod
    def _normalize_zenoh_topic(topic: str) -> str:
        """将 ROS 风格话题名转换为 Zenoh 合法 key expression。"""
        normalized = (topic or "").strip().strip("/")
        if not normalized:
            raise ValueError("topic 不能为空，且不能只包含 '/'")
        return normalized

    @classmethod
    def _on_sn(cls, sample) -> None:
        """`/sn` 订阅回调，读取方式与状态订阅一致（payload JSON 解码）。"""
        try:
            data = json.loads(sample.payload.to_bytes().decode("utf-8"))
        except Exception:
            return

        sn = data.get("sn")
        ip = data.get("ip")
        if not sn or not ip:
            return

        now = time.monotonic()
        changed = False

        with cls._lock:
            old = cls._robots.get(sn)
            if old is None or old.get("ip") != ip:
                changed = True

            cls._robots[sn] = {
                "sn": sn,
                "ip": ip,
                "last_seen_monotonic": now,
            }

        if changed:
            cls._notify_change()

    @classmethod
    def _cleanup_loop(cls) -> None:
        while not cls._stop_event.wait(cls._cleanup_interval_sec):
            now = time.monotonic()
            removed = False
            with cls._lock:
                expired_sns = [
                    sn
                    for sn, info in cls._robots.items()
                    if now - float(info["last_seen_monotonic"]) > cls._ttl_sec
                ]
                if expired_sns:
                    for sn in expired_sns:
                        cls._robots.pop(sn, None)
                    removed = True
            if removed:
                cls._notify_change()

    @classmethod
    def _notify_change(cls) -> None:
        with cls._lock:
            snapshot = cls._snapshot_locked()
            callbacks = list(cls._callbacks)
        for callback in callbacks:
            cls._safe_callback(callback, snapshot)

    @classmethod
    def _snapshot_locked(cls) -> List[dict]:
        robots = [
            {"sn": str(item["sn"]), "ip": str(item["ip"])}
            for item in cls._robots.values()
        ]
        robots.sort(key=lambda x: x["sn"])
        return robots

    @staticmethod
    def _safe_callback(callback: Callable[[List[dict]], None], robots: List[dict]) -> None:
        try:
            callback(robots)
        except Exception as e:
            print(f"⚠️ 发现回调执行失败: {e}")


def start_robot_discovery(
    topic: str = Topics.ROBOT_IDENTITY,
    router_ip: Optional[str] = None,
    router_port: int = 7447,
    ttl_sec: float = 3.0,
    cleanup_interval_sec: float = 0.5,
    callback: Optional[Callable[[List[dict]], None]] = None,
) -> None:
    """无类调用入口：启动机器人发现。"""
    RobotDiscovery.start(
        topic=topic,
        router_ip=router_ip,
        router_port=router_port,
        ttl_sec=ttl_sec,
        cleanup_interval_sec=cleanup_interval_sec,
        callback=callback,
    )


def stop_robot_discovery() -> None:
    """无类调用入口：停止机器人发现。"""
    RobotDiscovery.stop()


def list_discovered_robots() -> List[dict]:
    """无类调用入口：读取当前在线机器人列表。"""
    return RobotDiscovery.list_robots()
