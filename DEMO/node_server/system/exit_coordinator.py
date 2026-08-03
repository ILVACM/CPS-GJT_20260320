"""统一退出协调器（供 GUI 与 CLI 共用）。

对齐 Design-server.md §10.2（统一退出行为）、§10.3（B→A 退出通知 RPC）、
§10.5（退出告警落盘）与任务 §7.4（资源释放顺序）。

设计要点：
- ``notify_peer_shutdown()``：封装"检查 A 在线 → 发 B→A 退出通知 → 等反馈 → 告警落盘"
  的统一逻辑，GUI 与 CLI 共用；调用方仅需传入 deadline
- ``release_resources()``：按 §7.4 严格顺序释放本地资源
  （停止标志 → join 线程 → 相机 → 串口 → gRPC channel → GrpcServerB → NodeMonitor → logging.shutdown）
- B→A ``RequestPeerShutdown`` 当前为占位实现（待 AGENTS.md §5.5 proto 扩展），
  本协调器完善调用框架与日志记录，实际 RPC 调用由 ``InferenceGrpcClient.request_peer_shutdown`` 承担
- 线程安全：``_notified`` 标志防止重复发送退出通知
"""
import logging
import threading
from typing import Any, Callable, List, Optional

# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 system）
_logger = logging.getLogger("node_server.system")


class ExitCoordinator:
    """统一退出协调器。

    生命周期：
        coord = ExitCoordinator(node_monitor, inference_client)
        coord.notify_peer_shutdown(deadline_s=3.0)   # 通知检测节点（best-effort）
        coord.release_resources(stop_callback, threads, camera, serial_mgr, grpc_server)
    """

    def __init__(self, node_monitor: Any, inference_client: Any) -> None:
        """初始化退出协调器。

        :param node_monitor: NodeMonitor 实例（提供 ``is_online()`` 与 ``stop()``）
        :param inference_client: InferenceGrpcClient 实例（提供 ``request_peer_shutdown()``
                                 与 ``close()``）
        """
        self._node_monitor: Any = node_monitor
        self._inference_client: Any = inference_client
        self._lock: threading.Lock = threading.Lock()
        self._notified: bool = False

    # ------------------------------------------------------------------
    # 统一退出行为：B→A 退出通知（§10.2 ①②③④⑤）
    # ------------------------------------------------------------------

    def notify_peer_shutdown(self, deadline_s: float = 3.0) -> Optional[bool]:
        """检查检测节点 在线状态，若在线则发 B→A 退出通知并等反馈（§10.2 ①~⑤）。

        统一逻辑（GUI 与 CLI 共用）：
        1. 检查 ``node_monitor.is_online()``
        2. 在线 → ``inference_client.request_peer_shutdown(deadline_s)``
        3. 收到反馈 True → INFO 日志（正常退出）
        4. 反馈 False / None（超时/失败）→ WARNING 日志 + 终端告警（§10.5 落盘）
        5. 离线 → INFO 日志，直接退出（无需通知）

        :param deadline_s: 等待反馈超时（秒），默认 3.0
        :return:
            True  — A 已确认退出 / A 离线（可正常退出）
            False — A 反馈未接受 / 超时 / 调用失败（告警退出）
        """
        # 防止重复通知（GUI 与 CLI 并发触发时仅执行一次）
        with self._lock:
            if self._notified:
                _logger.info("B→A 退出通知已发送过，跳过重复通知")
                return True
            self._notified = True

        # ① 检查检测节点 在线状态
        try:
            online: bool = self._node_monitor.is_online()
        except Exception as e:
            _logger.warning("检查检测节点 在线状态异常: %s，按离线处理", e)
            online = False

        # ⑤ 检测节点 离线：直接退出（无需通知）
        if not online:
            _logger.info("检测节点 离线，直接退出（无需发送退出通知）")
            return True

        # ② 检测节点 在线：发送 B→A 退出通知，deadline 内等反馈
        _logger.info(
            "检测节点 在线，发送 B→A 退出通知 RequestPeerShutdown（deadline=%.1fs）",
            deadline_s,
        )
        try:
            result: Optional[bool] = self._inference_client.request_peer_shutdown(
                deadline_s=deadline_s,
            )
        except Exception as e:
            _logger.warning("B→A 退出通知调用异常: %s", e)
            result = None

        # ③④ 反馈处理 + 告警落盘（§10.5）
        if result is True:
            # ③ 时限内收到反馈 → 正常退出
            _logger.info("检测节点 已确认退出（accepted=True），正常退出流程")
            return True
        elif result is False:
            # ④ A 反馈未接受 → 告警退出
            _logger.warning(
                "检测节点 反馈未接受退出请求（accepted=False, deadline=%.1fs），"
                "已 best-effort 退出", deadline_s,
            )
            print(
                "[WARNING] 检测节点 反馈未接受退出请求，已 best-effort 退出。"
                "检测节点 状态异常，请人工核查。"
            )
            return False
        else:
            # ④ 超时/调用失败（None）→ 告警退出
            _logger.warning(
                "检测节点 退出反馈超时（deadline=%.1fs），已 best-effort 退出。"
                "检测节点 可能异常离线，请人工核查。", deadline_s,
            )
            print(
                "[WARNING] 检测节点 退出反馈超时，已 best-effort 退出。"
                "检测节点 可能异常离线，请人工核查。"
            )
            return False

    # ------------------------------------------------------------------
    # 资源释放（§7.4 严格顺序）
    # ------------------------------------------------------------------

    def release_resources(
        self,
        stop_callback: Callable[[], None],
        threads: List[Optional[threading.Thread]],
        camera: Any,
        serial_manager: Any,
        grpc_server: Any,
    ) -> None:
        """按 §7.4 严格顺序释放本地资源。

        顺序：
        1. 设置停止标志（``stop_callback()`` —— 由调用方提供，设置 ``_is_running=False``
           与 ``_stop_event.set()``）
        2. join 所有工作线程（跳过 None 与当前线程，避免自 join 死锁）
        3. 关闭相机（``camera.close()``）
        4. 关闭串口（``serial_manager.close()``）
        5. 关闭 gRPC channel（``inference_client.close()``）
        6. 停止 GrpcServerB（``grpc_server.stop()``）
        7. 停止 NodeMonitor（``node_monitor.stop()``）
        8. ``logging.shutdown()``

        每步均 try/except，单步失败不阻断后续释放。

        :param stop_callback: 设置停止标志的回调（设置 ``_is_running=False`` 等）
        :param threads: 待 join 的工作线程列表（None 与当前线程自动跳过）
        :param camera: 相机实例（可为 None）
        :param serial_manager: 串口管理器实例
        :param grpc_server: GrpcServerB 实例
        """
        current_thread: threading.Thread = threading.current_thread()

        # 1. 设置停止标志
        _logger.info("退出清理 1/8: 设置停止标志")
        try:
            stop_callback()
        except Exception as e:
            _logger.warning("设置停止标志异常: %s", e)

        # 2. join 所有工作线程
        _logger.info("退出清理 2/8: join 工作线程")
        for t in threads:
            if t is None or t is current_thread:
                continue
            try:
                if t.is_alive():
                    # daemon 线程给 1 秒，非 daemon 给 2 秒
                    timeout: float = 1.0 if t.daemon else 2.0
                    t.join(timeout=timeout)
                    if t.is_alive():
                        _logger.warning("线程 %s join 超时（%ss），继续后续清理",
                                        t.name, timeout)
            except Exception as e:
                _logger.warning("join 线程异常: %s", e)

        # 3. 关闭相机
        _logger.info("退出清理 3/8: 关闭相机")
        if camera is not None:
            try:
                camera.close()
            except Exception as e:
                _logger.warning("相机关闭异常: %s", e)

        # 4. 关闭串口
        _logger.info("退出清理 4/8: 关闭串口")
        if serial_manager is not None:
            try:
                serial_manager.close()
            except Exception as e:
                _logger.warning("串口关闭异常: %s", e)

        # 5. 关闭 gRPC channel（推理客户端）
        _logger.info("退出清理 5/8: 关闭推理客户端 gRPC channel")
        try:
            self._inference_client.close()
        except Exception as e:
            _logger.warning("推理客户端关闭异常: %s", e)

        # 6. 停止 GrpcServerB
        _logger.info("退出清理 6/8: 停止 GrpcServerB")
        if grpc_server is not None:
            try:
                grpc_server.stop()
            except Exception as e:
                _logger.warning("GrpcServerB 停止异常: %s", e)

        # 7. 停止 NodeMonitor
        _logger.info("退出清理 7/8: 停止 NodeMonitor")
        try:
            self._node_monitor.stop()
        except Exception as e:
            _logger.warning("NodeMonitor 停止异常: %s", e)

        # 8. logging.shutdown()
        _logger.info("退出清理 8/8: flush 日志（logging.shutdown）")
        try:
            logging.shutdown()
        except Exception:
            pass
