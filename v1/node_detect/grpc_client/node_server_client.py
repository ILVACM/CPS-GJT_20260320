"""
NodeServerClient — 检测节点 的 gRPC 客户端，向服务节点 发送 Heartbeat 与 Shutdown。

协议对齐：AGENTS.md §5.5 / §5.7。
  - Heartbeat: 检测节点 周期推送 HeartbeatRequest（A→B）
  - Shutdown:  检测节点 退出前一次性推送 ShutdownRequest（A→B，best-effort）

连接管理：
  - 独立 gRPC channel（与 B→A 的 Infer channel 完全隔离，符合 AGENTS.md §4.1）
  - channel 在构造时建立， close() 在退出时调用
  - 连接失败仅记 WARNING，不阻塞推理主循环

deadline 策略：
  - Heartbeat: 2s（超时记 WARNING，不重试，下个周期再发）
  - Shutdown:  2s（best-effort，超时/失败仅记 WARNING，不阻塞退出）
"""

import logging
import threading
import time
from typing import Optional

import grpc

from proto import rebar_inference_pb2 as pb2
from proto import rebar_inference_pb2_grpc as pb2_grpc

logger = logging.getLogger("node_detect.grpc_client.node_server_client")

# deadline 常量（秒）
_HEARTBEAT_DEADLINE_S: float = 2.0
_SHUTDOWN_DEADLINE_S: float = 2.0

# 节点 ID（对齐 Design-AI_detect.md §3.10）
_NODE_ID: str = "node-detect"


class NodeServerClient:
    """
    服务节点 的 gRPC 客户端（主动推送端）。

    用途：
      1. Heartbeat 周期推送（存活探测）
      2. Shutdown 一次性推送（退出通知，best-effort）

    线程安全：stub 方法调用本身线程安全（gRPC Python stub 是 thread-safe 的），
             但 heartbeat 与 shutdown 建议由不同线程调用，避免竞争。
    """

    def __init__(
        self,
        node_server_address: str,
        node_id: str = _NODE_ID,
        heartbeat_deadline_s: float = _HEARTBEAT_DEADLINE_S,
        shutdown_deadline_s: float = _SHUTDOWN_DEADLINE_S,
    ) -> None:
        """
        初始化服务节点 的 gRPC 客户端。

        Args:
            node_server_address: 服务节点 的 gRPC 地址（如 "192.168.10.1:50051"）
            node_id:       节点标识（默认 "node-detect"）
            heartbeat_deadline_s: Heartbeat RPC deadline（秒）
            shutdown_deadline_s:  Shutdown RPC deadline（秒）
        """
        self._node_server_address = node_server_address
        self._node_id = node_id
        self._heartbeat_deadline_s = heartbeat_deadline_s
        self._shutdown_deadline_s = shutdown_deadline_s

        # 建立 insecure channel（原型阶段不启用 TLS）
        # 与服务节点 的 gRPC server 建立单一 channel，所有 RPC 复用
        self._channel = grpc.insecure_channel(
            node_server_address,
            options=[
                ('grpc.keepalive_time_ms', 5000),
                ('grpc.keepalive_timeout_ms', 2000),
                ('grpc.keepalive_permit_without_calls', 1),
            ],
        )
        self._stub = pb2_grpc.RebarInferenceStub(self._channel)

        logger.info(
            f"[NodeServerClient] 已连接对端: {node_server_address} "
            f"(heartbeat_deadline={heartbeat_deadline_s}s, shutdown_deadline={shutdown_deadline_s}s)"
        )

    def send_heartbeat(
        self,
        status: int,
        infer_count: int = 0,
        last_infer_duration_ms: int = 0,
    ) -> bool:
        """
        向服务节点 发送一次心跳包（A→B）。

        Args:
            status:                 当前 proto NodeStatus 值（0-3）
            infer_count:            累计推理成功次数
            last_infer_duration_ms: 最近一次推理耗时（毫秒）

        Returns:
            True — 发送成功（对端 accepted）；False — 失败（超时/网络错误）
        """
        request = pb2.HeartbeatRequest(
            node_id=self._node_id,
            timestamp_ms=int(time.time() * 1000),
            status=status,
            infer_count=infer_count,
            last_infer_duration_ms=last_infer_duration_ms,
        )
        try:
            response = self._stub.Heartbeat(
                request,
                timeout=self._heartbeat_deadline_s,
            )
            if response.accepted:
                logger.debug(
                    f"[Heartbeat] 推送成功: status={status}, "
                    f"server_ts={response.server_timestamp_ms}"
                )
                return True
            else:
                # B 端不应返回 accepted=False（正常场景），仅记日志
                logger.warning(f"[Heartbeat] 对端 accepted=False")
                return False
        except grpc.RpcError as e:
            logger.warning(
                f"[Heartbeat] gRPC 错误: code={e.code()}, "
                f"details={e.details()}"
            )
            return False
        except Exception as e:
            logger.error(f"[Heartbeat] 未知异常: {e}", exc_info=True)
            return False

    def send_shutdown(self, reason: str = "manual") -> bool:
        """
        向服务节点 发送退出通知（A→B，best-effort）。

        调用场景：检测节点 退出前（SIGTERM / 异常捕获）。
        deadline 设短（2s），失败不阻塞退出流程。

        Args:
            reason: 退出原因字符串（如 "SIGTERM" / "exception" / "manual"）

        Returns:
            True — 发送成功；False — 失败（best-effort，调用方不依赖此返回值）
        """
        request = pb2.ShutdownRequest(
            node_id=self._node_id,
            timestamp_ms=int(time.time() * 1000),
            status=pb2.NODE_STATUS_SHUTTING_DOWN,
            reason=reason,
        )
        try:
            self._stub.Shutdown(
                request,
                timeout=self._shutdown_deadline_s,
            )
            logger.info(f"[Shutdown] 退出通知已发送: reason={reason}")
            return True
        except grpc.RpcError as e:
            logger.warning(
                f"[Shutdown] 发送失败（best-effort，不阻塞退出）: "
                f"code={e.code()}, details={e.details()}"
            )
            return False
        except Exception as e:
            logger.error(f"[Shutdown] 未知异常: {e}", exc_info=True)
            return False

    def close(self) -> None:
        """
        关闭 gRPC channel，释放连接资源。

        应在退出流程中调用（cleanup_resources）。
        """
        try:
            if self._channel is not None:
                self._channel.close()
                logger.info("[NodeServerClient] gRPC channel 已关闭")
        except Exception as e:
            logger.warning(f"[NodeServerClient] channel close 异常: {e}")
        finally:
            self._channel = None
            self._stub = None

    @property
    def node_id(self) -> str:
        """节点标识"""
        return self._node_id

    @property
    def node_server_address(self) -> str:
        """对端地址"""
        return self._node_server_address
