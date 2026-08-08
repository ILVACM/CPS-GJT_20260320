"""服务节点 gRPC 服务端（A→B 方向：Heartbeat + Shutdown）。

对齐：
- AGENTS.md §3.1（grpc_server 模块职责）、§5.5（双节点各监听 50051，独立 channel）、
  §5.7（节点状态监测与反馈机制）、§5.7.3（职责分工：B 侧接收反馈 → 判定状态 → 记录事件）
- Design-server.md §3.8（NodeMonitor 衔接）、§3.11.1（状态记录维护）

设计要点：
- 服务节点 在 local_ip:grpc_port 监听，注册 Heartbeat + Shutdown RPC handler
- Infer RPC 返回 UNIMPLEMENTED（服务节点 不提供推理服务，推理由检测节点 提供）
- 接收 Heartbeat → 调用 NodeMonitor.record_heartbeat(status_code, infer_count)
- 接收 Shutdown → 调用 NodeMonitor.on_shutdown()（立即标记 A 离线，无需等心跳超时）
- NodeMonitor 在 Step 4 实现，本 Step 通过 set_node_monitor() 延迟注入
- node_monitor 为 None 时，handler 仅记日志（不阻塞 RPC 响应）
- 服务端启停线程安全，stop() 幂等
"""
import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Optional

import grpc

from utils.common.config_loader import NetworkConfig
from utils.proto import rebar_inference_pb2
from utils.proto import rebar_inference_pb2_grpc


# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 grpc_server）
_logger = logging.getLogger("node_server.grpc_server")


class _RebarInferenceServicerB(rebar_inference_pb2_grpc.RebarInferenceServicer):
    """服务节点 侧 RebarInference 服务实现。

    仅实现 Heartbeat + Shutdown（A→B 方向）；Infer 返回 UNIMPLEMENTED
    （推理服务由检测节点 提供，服务节点 不承接该 RPC）。
    """

    def __init__(self, owner: "GrpcServerB") -> None:
        self._owner = owner

    def Infer(self, request, context):  # type: ignore[no-untyped-def]
        """服务节点 不提供推理服务（Infer RPC 由检测节点 承接）。"""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("服务节点 不提供 Infer RPC，请发送至检测节点")
        _logger.warning("收到非预期 Infer 请求 frame_id=%s，已拒绝", request.frame_id)
        return rebar_inference_pb2.InferResponse()

    def Heartbeat(self, request, context):  # type: ignore[no-untyped-def]
        """处理检测节点 心跳（A→B 方向）。

        - 解析 status / infer_count
        - 转交 NodeMonitor.record_heartbeat() 更新状态缓存与超时计时器
        - 返回 HeartbeatResponse（accepted=True, server_timestamp_ms=当前毫秒）
        """
        server_ts_ms: int = int(time.time() * 1000)
        _logger.info(
            "收到心跳: node_id=%s, status=%d, infer_count=%d, "
            "last_infer_duration_ms=%d, ts=%d",
            request.node_id, request.status, request.infer_count,
            request.last_infer_duration_ms, request.timestamp_ms,
        )

        # 转交 NodeMonitor（Step 4 注入；为 None 时仅记日志）
        monitor = self._owner._node_monitor
        if monitor is not None:
            try:
                # 预期接口契约：record_heartbeat(status_code, infer_count)
                # duck typing，不强制类型约束
                monitor.record_heartbeat(
                    status_code=int(request.status),
                    infer_count=int(request.infer_count),
                )
            except Exception as e:
                _logger.error("NodeMonitor.record_heartbeat 异常: %s", e)
        else:
            _logger.debug("NodeMonitor 未注入，心跳仅记日志")

        return rebar_inference_pb2.HeartbeatResponse(
            accepted=True,
            server_timestamp_ms=server_ts_ms,
        )

    def Shutdown(self, request, context):  # type: ignore[no-untyped-def]
        """处理检测节点 退出通知（A→B 方向，best-effort）。

        - 立即转交 NodeMonitor.on_shutdown() 标记 A 离线并触发降级
        - 无需等心跳超时（§5.7.6 兜底策略）
        - 返回 ShutdownResponse（acknowledged=True, server_timestamp_ms=当前毫秒）
        """
        server_ts_ms: int = int(time.time() * 1000)
        _logger.warning(
            "收到检测节点 退出通知: node_id=%s, status=%d, reason=%s, ts=%d",
            request.node_id, request.status, request.reason, request.timestamp_ms,
        )

        # 立即转交 NodeMonitor 触发降级（无需等心跳超时）
        monitor = self._owner._node_monitor
        if monitor is not None:
            try:
                # 预期接口契约：on_shutdown()
                monitor.on_shutdown()
            except Exception as e:
                _logger.error("NodeMonitor.on_shutdown 异常: %s", e)
        else:
            _logger.debug("NodeMonitor 未注入，退出通知仅记日志")

        return rebar_inference_pb2.ShutdownResponse(
            acknowledged=True,
            server_timestamp_ms=server_ts_ms,
        )


class GrpcServerB:
    """服务节点 gRPC 服务端。

    在 local_ip:grpc_port 监听，提供 Heartbeat + Shutdown RPC（A→B 方向）。
    NodeMonitor 在 Step 4 实现，可通过构造函数或 set_node_monitor() 注入。

    生命周期：
        server = GrpcServerB(network_cfg)
        server.set_node_monitor(monitor)  # Step 4 注入（可选）
        server.start()                     # 非阻塞，后台线程处理请求
        ...
        server.stop()                      # 等待优雅退出
    """

    def __init__(
        self,
        network_cfg: NetworkConfig,
        node_monitor: Optional[Any] = None,
    ) -> None:
        """初始化服务节点 gRPC 服务端。

        :param network_cfg: 网络配置（含 local_ip / grpc_port）
        :param node_monitor: NodeMonitor 实例（Step 4 注入；为 None 时 handler 仅记日志）
        """
        self._target: str = f"{network_cfg.local_ip}:{network_cfg.grpc_port}"
        self._node_monitor: Optional[Any] = node_monitor
        self._server: Optional[grpc.Server] = None
        self._servicer: Optional[_RebarInferenceServicerB] = None
        self._lock: threading.Lock = threading.Lock()
        self._started: bool = False

    def set_node_monitor(self, node_monitor: Any) -> None:
        """注入 NodeMonitor 实例（Step 4 注入，支持运行时替换）。

        :param node_monitor: NodeMonitor 实例，需提供 ``record_heartbeat(status_code, infer_count)``
                             与 ``on_shutdown()`` 方法（duck typing）
        """
        with self._lock:
            self._node_monitor = node_monitor
        _logger.info("NodeMonitor 已注入 GrpcServerB")

    def start(self) -> None:
        """启动 gRPC 服务端（非阻塞，后台线程处理请求）。

        绑定 local_ip:grpc_port，注册 RebarInference servicer。
        若端口已被占用或绑定失败，抛出 RuntimeError（启动自检硬前置条件，对齐 §4.3）。
        """
        with self._lock:
            if self._started:
                _logger.warning("gRPC 服务端已启动，忽略重复 start()")
                return

            server = grpc.server(
                # 线程池执行器，原型阶段固定 4 工作线程（心跳 + 退出通知并发量极低）
                ThreadPoolExecutor(max_workers=4),
            )
            self._servicer = _RebarInferenceServicerB(self)
            rebar_inference_pb2_grpc.add_RebarInferenceServicer_to_server(
                self._servicer, server,
            )

            # 绑定 local_ip:grpc_port（对齐 §4.1 网络拓扑）
            bind_target = self._target
            try:
                port = server.add_insecure_port(bind_target)
            except Exception as e:
                _logger.error("gRPC 端口绑定异常: %s, 错误: %s", bind_target, e)
                raise RuntimeError(f"gRPC 端口绑定失败: {bind_target}") from e

            if port == 0:
                # add_insecure_port 返回 0 表示绑定失败（端口被占用或 IP 不可用）
                _logger.error(
                    "gRPC 端口绑定失败: %s（端口被占用或 IP 未配置），返回 0",
                    bind_target,
                )
                raise RuntimeError(
                    f"gRPC 端口绑定失败: {bind_target}（端口被占用或 IP 未配置）"
                )

            server.start()
            self._server = server
            self._started = True
            _logger.info("gRPC 服务端已启动，监听: %s", bind_target)

    def stop(self) -> None:
        """停止 gRPC 服务端（阻塞等待优雅退出，超时 5 秒）。

        幂等：多次调用安全。
        """
        with self._lock:
            if not self._started or self._server is None:
                return
            # grace=5.0 给在途请求 5 秒处理时间，超时强制终止
            self._server.stop(grace=5.0)
            self._server = None
            self._servicer = None
            self._started = False
            _logger.info("gRPC 服务端已停止")

    def is_running(self) -> bool:
        """返回服务端是否在运行。"""
        with self._lock:
            return self._started

    @property
    def target(self) -> str:
        """返回监听目标（host:port）。"""
        return self._target
