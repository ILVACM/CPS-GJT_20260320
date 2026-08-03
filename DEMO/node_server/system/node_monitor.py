"""检测节点 状态监测器（服务节点 侧）。

对齐 AGENTS.md §5.7（节点状态监测与反馈机制）与 Design-server.md §3.8.2。

监测策略：被动接收检测节点 心跳 + watchdog 超时判定（A 推 B 收单向模式）。
- 检测节点 每 5 秒主动推送心跳，每次心跳刷新"存活计时器"
- 连续 3 次心跳未收到（约 15 秒）→ 判定离线 → 触发降级
- 收到 A 的 Shutdown RPC → 立即标记离线（不等超时，§5.7.6 兜底策略）
- 状态变更事件写入 logs/node_detect_status.jsonl（JSONL 持久化）
- 线程安全：threading.Lock 保护状态字段；文件 I/O 与回调在锁外执行避免死锁

状态码：直接使用 proto NodeStatus 枚举值（0=UNKNOWN, 1=IDLE, 2=BUSY, 3=SHUTTING_DOWN）。
超时/未收到心跳时使用内部哨兵 NODE_STATUS_OFFLINE=-1（proto 枚举无 OFFLINE 值）。
"""
import json
import logging
import os
import threading
import time
from typing import Callable, Optional

from proto import rebar_inference_pb2

# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 system）
_logger = logging.getLogger("node_server.system")

# 内部离线状态哨兵（proto NodeStatus 枚举无 OFFLINE 值，用 -1 表示超时/未收到心跳）
NODE_STATUS_OFFLINE: int = -1

# 在线状态集合（IDLE / BUSY 视为在线）
_ONLINE_STATES = (
    rebar_inference_pb2.NODE_STATUS_IDLE,
    rebar_inference_pb2.NODE_STATUS_BUSY,
)
# 离线类状态集合（OFFLINE 哨兵 + SHUTTING_DOWN）
_OFFLINE_STATES = (
    NODE_STATUS_OFFLINE,
    rebar_inference_pb2.NODE_STATUS_SHUTTING_DOWN,
)


def _status_name(status_code: int) -> str:
    """将状态码映射为可读名称（供日志与 JSONL 记录）。"""
    names = {
        rebar_inference_pb2.NODE_STATUS_UNKNOWN: "UNKNOWN",
        rebar_inference_pb2.NODE_STATUS_IDLE: "IDLE",
        rebar_inference_pb2.NODE_STATUS_BUSY: "BUSY",
        rebar_inference_pb2.NODE_STATUS_SHUTTING_DOWN: "SHUTTING_DOWN",
        NODE_STATUS_OFFLINE: "OFFLINE",
    }
    return names.get(int(status_code), f"UNKNOWN({status_code})")


class NodeMonitor:
    """检测节点 状态监测器（被动接收心跳 + watchdog 超时判定）。

    生命周期：
        monitor = NodeMonitor(heartbeat_interval_s=5.0, timeout_count=3,
                              on_lost=cb1, on_recovered=cb2)
        monitor.start()              # 启动 watchdog 线程
        monitor.record_heartbeat(1)  # 由 gRPC Heartbeat handler 调用
        ...
        monitor.on_shutdown()        # 收到 A 的 Shutdown RPC 时调用
        monitor.stop()               # 停止 watchdog 线程
    """

    def __init__(
        self,
        heartbeat_interval_s: float = 5.0,
        timeout_count: int = 3,
        on_lost: Optional[Callable[[], None]] = None,
        on_recovered: Optional[Callable[[], None]] = None,
    ) -> None:
        """初始化节点状态监测器。

        :param heartbeat_interval_s: 心跳间隔（秒，默认 5）
        :param timeout_count: 超时判定阈值（连续丢失心跳次数，默认 3）
        :param on_lost: 在线→离线 回调（watchdog 超时或收到 Shutdown 时触发）
        :param on_recovered: 离线→在线 回调（收到心跳恢复在线时触发）
        """
        self._heartbeat_interval_s: float = float(heartbeat_interval_s)
        self._timeout_count: int = int(timeout_count)
        # 超时阈值 = 间隔 × 次数（默认 5×3=15 秒）
        self._timeout_threshold: float = self._heartbeat_interval_s * self._timeout_count
        self._on_lost: Optional[Callable[[], None]] = on_lost
        self._on_recovered: Optional[Callable[[], None]] = on_recovered

        # 状态字段（受 _lock 保护）
        self._lock: threading.Lock = threading.Lock()
        self._current_status: int = rebar_inference_pb2.NODE_STATUS_UNKNOWN
        self._last_heartbeat_time: float = 0.0
        self._heartbeat_count: int = 0

        # watchdog 线程
        self._running: bool = False
        self._watchdog_thread: Optional[threading.Thread] = None

        # JSONL 持久化路径（对齐 Design-server.md §3.8.2）
        self._status_log_path: str = "logs/node_detect_status.jsonl"
        # 文件 I/O 串行化锁（与状态锁分离，避免持锁做 I/O）
        self._io_lock: threading.Lock = threading.Lock()

    def start(self) -> None:
        """启动 watchdog 线程（幂等，重复调用安全）。"""
        with self._lock:
            if self._running:
                _logger.warning("NodeMonitor 已启动，忽略重复 start()")
                return
            self._running = True
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="node-monitor-watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()
        _logger.info(
            "NodeMonitor 已启动（心跳间隔=%.1fs, 超时阈值=%.1fs）",
            self._heartbeat_interval_s, self._timeout_threshold,
        )

    def stop(self) -> None:
        """停止 watchdog 线程（幂等，等待最多 2 秒）。"""
        with self._lock:
            self._running = False
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=2.0)
        self._watchdog_thread = None
        _logger.info("NodeMonitor 已停止")

    def record_heartbeat(self, status_code: int, infer_count: int = 0) -> None:
        """接收心跳（供 gRPC Heartbeat handler 调用）。

        更新最近心跳时间戳与当前状态；离线→在线翻转时触发 on_recovered 回调
        并写 JSONL 持久化。开销极小：仅更新时间戳 + 推断状态。

        :param status_code: proto NodeStatus 枚举整数值（来自 HeartbeatRequest.status）
        :param infer_count: 检测节点 累计推理次数（监控用，可选）
        """
        now: float = time.time()
        event_to_write: Optional[dict] = None
        fire_recovered: bool = False

        with self._lock:
            old_status: int = self._current_status
            self._last_heartbeat_time = now
            self._heartbeat_count += 1
            self._current_status = int(status_code)

            # 离线 → 在线 翻转判定（OFFLINE/SHUTTING_DOWN → IDLE/BUSY）
            was_offline = old_status in _OFFLINE_STATES
            is_online_now = self._current_status in _ONLINE_STATES
            if was_offline and is_online_now:
                fire_recovered = True
                event_to_write = {
                    "ts": round(now, 3),
                    "from": _status_name(old_status),
                    "to": _status_name(self._current_status),
                    "reason": "heartbeat_received",
                    "heartbeat_count": self._heartbeat_count,
                }

        # 锁外执行 I/O 与回调（避免持锁做 I/O / 死锁）
        if event_to_write is not None:
            self._write_status_event(event_to_write)
        if fire_recovered and self._on_recovered is not None:
            try:
                self._on_recovered()
            except Exception as e:
                _logger.error("on_recovered 回调异常: %s", e)

        _logger.debug(
            "心跳收到: status=%s, infer_count=%d, 累计=%d",
            _status_name(int(status_code)), infer_count, self._heartbeat_count,
        )

    def on_shutdown(self) -> None:
        """接收 A 的 Shutdown RPC，立即标记离线（不等超时，§5.7.6 兜底策略）。

        状态置为 NODE_STATUS_SHUTTING_DOWN（视为离线）；若此前在线则触发 on_lost 回调。
        重置心跳计时器，避免 watchdog 立即再触发重复 on_lost。
        """
        now: float = time.time()
        event_to_write: Optional[dict] = None
        fire_lost: bool = False

        with self._lock:
            old_status: int = self._current_status
            self._current_status = rebar_inference_pb2.NODE_STATUS_SHUTTING_DOWN
            self._last_heartbeat_time = now  # 重置计时，避免 watchdog 立即再触发
            # 在线 → 离线 翻转判定
            if old_status not in _OFFLINE_STATES:
                fire_lost = True
                event_to_write = {
                    "ts": round(now, 3),
                    "from": _status_name(old_status),
                    "to": _status_name(self._current_status),
                    "reason": "shutdown_received",
                    "heartbeat_count": self._heartbeat_count,
                }

        if event_to_write is not None:
            self._write_status_event(event_to_write)
        if fire_lost and self._on_lost is not None:
            try:
                self._on_lost()
            except Exception as e:
                _logger.error("on_lost 回调异常: %s", e)

        _logger.warning("收到检测节点 Shutdown 通知，已立即标记离线")

    def is_online(self) -> bool:
        """当前是否在线（IDLE 或 BUSY）。"""
        with self._lock:
            return self._current_status in _ONLINE_STATES

    def get_current_status(self) -> int:
        """返回当前状态码（proto NodeStatus 枚举值或 OFFLINE 哨兵 -1）。"""
        with self._lock:
            return self._current_status

    # ---- 内部：watchdog 超时判定 ----

    def _watchdog_loop(self) -> None:
        """watchdog 线程主循环：1 秒周期检查心跳是否超时。"""
        while True:
            with self._lock:
                running = self._running
            if not running:
                break
            time.sleep(1.0)
            self._check_timeout()

    def _check_timeout(self) -> None:
        """检查心跳是否超时，超时则翻转状态为 OFFLINE 并触发 on_lost。"""
        now: float = time.time()
        event_to_write: Optional[dict] = None
        fire_lost: bool = False

        with self._lock:
            # 尚未收到过心跳：不触发超时（等启动阶段 probe 兜底）
            if self._heartbeat_count == 0:
                return
            # 已离线（OFFLINE/SHUTTING_DOWN）：不重复触发
            if self._current_status in _OFFLINE_STATES:
                return
            elapsed: float = now - self._last_heartbeat_time
            if elapsed > self._timeout_threshold:
                old_status: int = self._current_status
                self._current_status = NODE_STATUS_OFFLINE
                fire_lost = True
                event_to_write = {
                    "ts": round(now, 3),
                    "from": _status_name(old_status),
                    "to": _status_name(NODE_STATUS_OFFLINE),
                    "reason": "heartbeat_timeout",
                    "heartbeat_count": self._heartbeat_count,
                }

        if event_to_write is not None:
            self._write_status_event(event_to_write)
        if fire_lost:
            if self._on_lost is not None:
                try:
                    self._on_lost()
                except Exception as e:
                    _logger.error("on_lost 回调异常: %s", e)
            _logger.warning(
                "心跳超时（%.1fs 无心跳），检测节点 标记离线", self._timeout_threshold,
            )

    # ---- 内部：JSONL 持久化 ----

    def _write_status_event(self, event: dict) -> None:
        """将状态变更事件写入 JSONL 持久化日志（锁外调用，_io_lock 串行化）。

        每行一个 JSON 对象，便于后续 ELK / Pandas 解析：
        {"ts": 1711234567.890, "level": "INFO", "node": "A",
         "event": "status_change", "from": "IDLE", "to": "OFFLINE",
         "reason": "heartbeat_timeout", "heartbeat_count": 42}
        """
        record = {
            "ts": event.get("ts", round(time.time(), 3)),
            "level": "INFO",
            "node": "A",
            "module": "node_monitor",
            "event": "status_change",
            "from": event.get("from", "UNKNOWN"),
            "to": event.get("to", "UNKNOWN"),
            "reason": event.get("reason", "unknown"),
            "heartbeat_count": event.get("heartbeat_count", 0),
        }
        with self._io_lock:
            try:
                log_dir = os.path.dirname(self._status_log_path)
                if log_dir:
                    os.makedirs(log_dir, exist_ok=True)
                with open(self._status_log_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(record, ensure_ascii=False) + "\n")
            except OSError as e:
                _logger.warning("状态记录写入 JSONL 失败: %s", e)
