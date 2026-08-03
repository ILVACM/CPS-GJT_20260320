"""
HeartbeatWorker — 周期心跳推送（daemon 线程）。

职责：
  - 在独立 daemon 线程中，按 heartbeat_interval_seconds 周期调用
    NodeServerClient.send_heartbeat()
  - shutdown_event 触发时优雅退出
  - 记录连续失败次数（failing_count），供上层监控调用

设计要点（对齐 Design-AI_detect.md §3.10）：
  - 心跳循环不使用 time.sleep(interval)，而是 shutdown_event.wait(timeout=interval)
    使线程能在事件被 set 时立即响应退出信号
  - 连续失败超阈值时记 WARNING（服务节点 可能宕机），但不停止心跳

线程安全：
  - shutdown_event 使用 threading.Event，跨线程安全
  - 指标（consecutive_failures / total_sent）加锁保护
"""

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from grpc_client.node_server_client import NodeServerClient

logger = logging.getLogger("node_detect.grpc_client.heartbeat_worker")

# 连续失败告警阈值
_CONSECUTIVE_FAILURE_WARN_THRESHOLD: int = 3


@dataclass
class HeartbeatStats:
    """心跳统计（线程安全读）"""
    total_sent: int = 0            # 累计发送次数
    total_failed: int = 0          # 累计失败次数
    consecutive_failures: int = 0   # 当前连续失败次数
    last_sent_timestamp_ms: Optional[int] = None  # 最近一次发送时间戳
    last_latency_ms: Optional[int] = None         # 最近一次往返延迟（估算）


class HeartbeatWorker:
    """
    心跳推送 worker（daemon 线程）。

    用法：
        worker = HeartbeatWorker(
            node_server_client=client,
            shutdown_event=shutdown_event,
            interval_seconds=5,
            status_provider=lambda: state_machine.current_proto_status(),
            infer_count_provider=lambda: servicer.infer_count,
        )
        worker.start()          # 启动 daemon 线程
        # ... 业务逻辑 ...
        shutdown_event.set()    # 触发退出
        worker.join(timeout=10) # 等待心跳线程退出（gRPC deadline 已设 2s）

    Attributes:
        _node_server_client:           NodeServerClient 实例
        _shutdown_event:          threading.Event — 退出信号
        _interval_seconds:        心跳间隔（秒）
        _status_provider:         Callable — 状态询问闭包（返回 proto NodeStatus int）
        _infer_count_provider:    Callable — 推理次数闭包
        _infer_duration_provider: Callable — 最近推理耗时闭包
        _thread:                  threading.Thread
        _lock:                    threading.Lock — 保护 _stats
        _stats:                   HeartbeatStats
    """

    def __init__(
        self,
        node_server_client: NodeServerClient,
        shutdown_event: threading.Event,
        interval_seconds: int = 5,
        status_provider: Callable[[], int] = lambda: 1,
        infer_count_provider: Callable[[], int] = lambda: 0,
        infer_duration_provider: Callable[[], int] = lambda: 0,
    ) -> None:
        """
        初始化 HeartbeatWorker。

        Args:
            node_server_client:          服务节点 gRPC 客户端（已连接）
            shutdown_event:         threading.Event — 退出信号（由 main.py 控制）
            interval_seconds:       心跳间隔（秒，默认 5）
            status_provider:        闭包 — 返回当前 proto NodeStatus int
            infer_count_provider:   闭包 — 返回累计推理次数
            infer_duration_provider: 闭包 — 返回最近推理耗时（毫秒）
        """
        self._node_server_client = node_server_client
        self._shutdown_event = shutdown_event
        self._interval_seconds = interval_seconds
        self._status_provider = status_provider
        self._infer_count_provider = infer_count_provider
        self._infer_duration_provider = infer_duration_provider

        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._stats = HeartbeatStats()

        logger.info(
            f"[HeartbeatWorker] 初始化: interval={interval_seconds}s, "
            f"node_server={node_server_client.node_server_address}"
        )

    def start(self) -> None:
        """启动心跳推送线程（daemon 模式）"""
        if self._thread is not None:
            raise RuntimeError("HeartbeatWorker 已启动，不可重复 start()")

        self._thread = threading.Thread(
            target=self._run,
            name="heartbeat-worker",
            daemon=True,
        )
        self._thread.start()
        logger.info("[HeartbeatWorker] 启动成功（daemon 线程）")

    def _run(self) -> None:
        """
        心跳推送主循环（daemon 线程入口）。

        使用 shutdown_event.wait(timeout=interval) 实现：
          - 有超时则执行心跳
          - 被 set 则立即退出
        """
        logger.info(
            f"[HeartbeatWorker] 循环开始（间隔 {self._interval_seconds}s）"
        )

        while not self._shutdown_event.is_set():
            # 使用 wait() 而非 sleep()：事件被 set 时立即响应
            if self._shutdown_event.wait(timeout=self._interval_seconds):
                # 事件被 set → 收到退出信号
                break

            # 采集状态 + 推送
            self._send_one_heartbeat()

        logger.info("[HeartbeatWorker] 循环结束（shutdown_event 触发）")

    def _send_one_heartbeat(self) -> None:
        """执行单次心跳推送并更新统计"""
        try:
            status = self._status_provider()
            infer_count = self._infer_count_provider()
            infer_duration = self._infer_duration_provider()

            t_start = time.time()
            success = self._node_server_client.send_heartbeat(
                status=status,
                infer_count=infer_count,
                last_infer_duration_ms=infer_duration,
            )
            latency_ms = int((time.time() - t_start) * 1000)

            with self._lock:
                self._stats.total_sent += 1
                self._stats.last_sent_timestamp_ms = int(time.time() * 1000)
                self._stats.last_latency_ms = latency_ms
                if success:
                    self._stats.consecutive_failures = 0
                else:
                    self._stats.total_failed += 1
                    self._stats.consecutive_failures += 1
                    # 连续失败告警
                    if self._stats.consecutive_failures >= _CONSECUTIVE_FAILURE_WARN_THRESHOLD:
                        logger.warning(
                            f"[HeartbeatWorker] 连续失败 "
                            f"{self._stats.consecutive_failures} 次，"
                            f"服务节点 可能不可达"
                        )
        except Exception as e:
            logger.error(f"[HeartbeatWorker] 单次心跳异常: {e}", exc_info=True)
            with self._lock:
                self._stats.total_failed += 1
                self._stats.consecutive_failures += 1

    def join(self, timeout: float = 10.0) -> None:
        """
        等待心跳线程退出。

        Args:
            timeout: 最大等待时间（秒）。建议 >= heartbeat_interval + gRPC deadline。
        """
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                logger.warning(
                    f"[HeartbeatWorker] 线程在 {timeout}s 后仍未退出"
                )
            else:
                logger.info("[HeartbeatWorker] 线程已退出")
        self._thread = None

    @property
    def stats(self) -> HeartbeatStats:
        """获取心跳统计快照（线程安全深拷贝）"""
        with self._lock:
            return HeartbeatStats(
                total_sent=self._stats.total_sent,
                total_failed=self._stats.total_failed,
                consecutive_failures=self._stats.consecutive_failures,
                last_sent_timestamp_ms=self._stats.last_sent_timestamp_ms,
                last_latency_ms=self._stats.last_latency_ms,
            )

    @property
    def is_alive(self) -> bool:
        """心跳线程是否正在运行"""
        return self._thread is not None and self._thread.is_alive()
