"""网络动态监测器（服务节点 侧）。

对齐 AGENTS.md §4.3（启动环境自检）与 Design-server.md §3.8（运行时健康检查）。
本模块解决"网络属功能受限型依赖，不应阻塞程序启动"的设计需求（§4.3 软故障/降级原则）。

设计定位：
- 程序启动**不阻塞于网络检测**（bootstrap 不再因网络项 sys.exit(1)）
- 运行时由本 watchdog 线程**周期探测**网络状态，动态翻转"在线/降级"
- 网络可用 → 相关功能（gRPC 推理）立即上线
- 运行中网络异常/断开 → 自动降级，程序继续运行
- 网络恢复 → 自动重新上线
- 状态变化不影响主进程稳定性

探测内容（复用 self_check 的探测函数）：
- 对端可达性（TCP 探测检测节点 IP:端口）—— 核心判定项
- 本机 IP 校验 / 端口可用性 —— 辅助诊断，作为降级原因记录

防抖策略：连续 N 次（默认 3）探测结果一致才翻转状态，避免瞬时抖动导致 UI 反复切换。

状态机：
  NETWORK_ONLINE   —— 网络/对端可达，推理功能可用
  NETWORK_DEGRADED —— 网络/对端不可达，推理功能禁用

线程安全：threading.Lock 保护状态字段；JSONL I/O 与回调在锁外执行避免死锁（对齐 NodeMonitor 模式）。
"""
import json
import logging
import os
import threading
import time
from typing import Callable, Optional

from utils.common.config_loader import NetworkConfig
from utils.system.self_check import (
    _check_local_ip,
    _check_peer_reachable,
    _check_port_available,
)

# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 system）
_logger = logging.getLogger("node_server.system")

# 网络状态常量
NETWORK_ONLINE: str = "ONLINE"
NETWORK_DEGRADED: str = "DEGRADED"

# 状态翻转防抖阈值（连续探测结果一致才翻转）
_DEFAULT_DEBOUNCE_COUNT: int = 3


class NetworkMonitor:
    """网络动态监测器（watchdog 线程周期探测）。

    生命周期：
        monitor = NetworkMonitor(net_cfg, on_online=cb1, on_degraded=cb2)
        monitor.start()              # 启动 watchdog 线程（立即执行一次探测）
        ...
        online = monitor.is_online()  # 实时查询网络状态
        monitor.stop()               # 停止 watchdog 线程

    :param net_cfg: 网络配置（含 local_ip / remote_ip / grpc_port / 监测周期）
    :param probe_interval_s: 探测周期（秒）；默认取 net_cfg.network_monitor_interval_seconds
    :param debounce_count: 状态翻转防抖次数（连续探测结果一致才翻转，默认 3）
    :param on_online: DEGRADED → ONLINE 翻转回调（网络恢复）
    :param on_degraded: ONLINE → DEGRADED 翻转回调（网络断开）
    """

    def __init__(
        self,
        net_cfg: NetworkConfig,
        probe_interval_s: Optional[float] = None,
        debounce_count: int = _DEFAULT_DEBOUNCE_COUNT,
        on_online: Optional[Callable[[], None]] = None,
        on_degraded: Optional[Callable[[], None]] = None,
    ) -> None:
        self._net_cfg: NetworkConfig = net_cfg
        self._probe_interval_s: float = (
            probe_interval_s
            if probe_interval_s is not None
            else float(net_cfg.network_monitor_interval_seconds)
        )
        self._debounce_count: int = max(1, int(debounce_count))
        self._on_online: Optional[Callable[[], None]] = on_online
        self._on_degraded: Optional[Callable[[], None]] = on_degraded

        # 状态字段（受 _lock 保护）
        self._lock: threading.Lock = threading.Lock()
        self._state: str = NETWORK_DEGRADED  # 初始降级，首次探测后翻转
        self._probe_count: int = 0  # 累计探测次数
        self._last_probe_result: Optional[bool] = None  # 最近一次可达性结果
        self._pending_state: Optional[str] = None  # 防抖中待定的状态
        self._pending_count: int = 0  # 防抖连续命中计数
        self._last_probe_time: float = 0.0

        # watchdog 线程
        self._running: bool = False
        self._watchdog_thread: Optional[threading.Thread] = None

        # JSONL 持久化路径
        self._status_log_path: str = "logs/network_status.jsonl"
        self._io_lock: threading.Lock = threading.Lock()

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(self) -> None:
        """启动 watchdog 线程（幂等，立即执行一次探测）。"""
        with self._lock:
            if self._running:
                _logger.warning("NetworkMonitor 已启动，忽略重复 start()")
                return
            self._running = True
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="network-monitor-watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()
        _logger.info(
            "NetworkMonitor 已启动（探测周期=%.1fs, 防抖=%d 次）",
            self._probe_interval_s, self._debounce_count,
        )

    def stop(self) -> None:
        """停止 watchdog 线程（幂等，等待最多 2 秒）。"""
        with self._lock:
            self._running = False
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=2.0)
        self._watchdog_thread = None
        _logger.info("NetworkMonitor 已停止")

    # ------------------------------------------------------------------
    # 查询接口
    # ------------------------------------------------------------------

    def is_online(self) -> bool:
        """当前网络是否在线（对端可达 + 本地配置就绪）。"""
        with self._lock:
            return self._state == NETWORK_ONLINE

    def get_state(self) -> str:
        """返回当前网络状态（NETWORK_ONLINE / NETWORK_DEGRADED）。"""
        with self._lock:
            return self._state

    def get_last_probe_info(self) -> dict:
        """返回最近一次探测的诊断信息（供 UI / CLI 展示）。"""
        with self._lock:
            return {
                "state": self._state,
                "probe_count": self._probe_count,
                "last_probe_time": round(self._last_probe_time, 3),
                "last_reachable": self._last_probe_result,
                "probe_interval_s": self._probe_interval_s,
            }

    # ------------------------------------------------------------------
    # watchdog 主循环
    # ------------------------------------------------------------------

    def _watchdog_loop(self) -> None:
        """watchdog 线程主循环：周期探测网络状态，防抖翻转。"""
        # 启动后立即执行一次探测，尽快获得初始状态
        self._probe_once()
        while True:
            with self._lock:
                running = self._running
            if not running:
                break
            time.sleep(self._probe_interval_s)
            self._probe_once()

    def _probe_once(self) -> None:
        """执行一次网络探测：可达性判定 + 防抖状态翻转。"""
        now: float = time.time()
        diagnostics: dict = {}
        reachable: bool = False

        # 对端可达性（核心判定项）
        try:
            peer_ok, peer_msg = _check_peer_reachable(self._net_cfg)
            diagnostics["peer"] = {"ok": peer_ok, "msg": peer_msg}
        except Exception as e:
            peer_ok, peer_msg = False, f"对端探测异常: {e}"
            diagnostics["peer"] = {"ok": False, "msg": peer_msg}

        # 辅助诊断：本机 IP / 端口可用性（不直接决定状态，仅记录降级原因）
        try:
            ip_ok, ip_msg = _check_local_ip(self._net_cfg)
            diagnostics["ip"] = {"ok": ip_ok, "msg": ip_msg}
        except Exception as e:
            ip_ok, ip_msg = False, f"IP 校验异常: {e}"
            diagnostics["ip"] = {"ok": False, "msg": ip_msg}
        try:
            port_ok, port_msg = _check_port_available(self._net_cfg)
            diagnostics["port"] = {"ok": port_ok, "msg": port_msg}
        except Exception as e:
            port_ok, port_msg = False, f"端口检查异常: {e}"
            diagnostics["port"] = {"ok": False, "msg": port_msg}

        # 综合可达性判定：对端可达即为在线（IP/端口仅作诊断辅助）
        # 注：即使本机 IP 与配置不符，只要对端能连上，推理功能即可用
        reachable = peer_ok
        self._apply_probe_result(now, reachable, peer_ok, diagnostics)

    def _apply_probe_result(
        self,
        now: float,
        reachable: bool,
        peer_ok: bool,
        diagnostics: dict,
    ) -> None:
        """应用探测结果，执行防抖状态翻转。"""
        event_to_write: Optional[dict] = None
        fire_online: bool = False
        fire_degraded: bool = False

        with self._lock:
            self._probe_count += 1
            self._last_probe_time = now
            self._last_probe_result = reachable

            # 防抖：结果与上次一致则累加计数，否则重置
            if self._pending_state is None or self._pending_state != ("ONLINE" if reachable else "DEGRADED"):
                self._pending_state = "ONLINE" if reachable else "DEGRADED"
                self._pending_count = 1
            else:
                self._pending_count += 1

            # 达到防抖阈值，且目标状态与当前不同 → 翻转
            if (
                self._pending_count >= self._debounce_count
                and self._pending_state != self._state
            ):
                old_state: str = self._state
                self._state = self._pending_state
                self._pending_count = 0
                event_to_write = {
                    "ts": round(now, 3),
                    "from": old_state,
                    "to": self._state,
                    "reason": "probe_ok" if reachable else "probe_fail",
                    "probe_count": self._probe_count,
                    "peer_ok": peer_ok,
                    "diagnostics": diagnostics,
                }
                if self._state == NETWORK_ONLINE:
                    fire_online = True
                else:
                    fire_degraded = True

        # 锁外执行 I/O 与回调（避免持锁做 I/O / 死锁）
        if event_to_write is not None:
            self._write_status_event(event_to_write)
        if fire_online and self._on_online is not None:
            try:
                self._on_online()
            except Exception as e:
                _logger.error("on_online 回调异常: %s", e)
        if fire_degraded and self._on_degraded is not None:
            try:
                self._on_degraded()
            except Exception as e:
                _logger.error("on_degraded 回调异常: %s", e)

        _logger.debug(
            "网络探测: reachable=%s, state=%s, pending=%s(%d/%d)",
            reachable, self._state, self._pending_state, self._pending_count,
            self._debounce_count,
        )

    # ------------------------------------------------------------------
    # JSONL 持久化
    # ------------------------------------------------------------------

    def _write_status_event(self, event: dict) -> None:
        """将网络状态变更事件写入 JSONL（锁外调用，_io_lock 串行化）。

        {"ts": ..., "level": "INFO", "node": "S", "module": "network_monitor",
         "event": "status_change", "from": "ONLINE", "to": "DEGRADED",
         "reason": "probe_fail", "probe_count": 42, "peer_ok": false}
        """
        record = {
            "ts": event.get("ts", round(time.time(), 3)),
            "level": "INFO",
            "node": "S",
            "module": "network_monitor",
            "event": "status_change",
            "from": event.get("from", "UNKNOWN"),
            "to": event.get("to", "UNKNOWN"),
            "reason": event.get("reason", "unknown"),
            "probe_count": event.get("probe_count", 0),
            "peer_ok": event.get("peer_ok", None),
        }
        with self._io_lock:
            try:
                log_dir = os.path.dirname(self._status_log_path)
                if log_dir:
                    os.makedirs(log_dir, exist_ok=True)
                with open(self._status_log_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(record, ensure_ascii=False) + "\n")
            except OSError as e:
                _logger.warning("网络状态记录写入 JSONL 失败: %s", e)