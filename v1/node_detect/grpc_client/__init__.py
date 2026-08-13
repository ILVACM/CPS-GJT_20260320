"""
grpc_client 包 — 检测节点 的 gRPC 客户端侧（A→B 方向）。

检测节点 作为 gRPC 客户端，向服务节点 周期推送 Heartbeat 并在退出时发 Shutdown。

子模块：
  - node_server_client: NodeServerClient — 封装 Heartbeat 与 Shutdown 调用
  - heartbeat_worker: HeartbeatWorker — daemon 线程周期推送心跳
"""

from grpc_client.node_server_client import NodeServerClient
from grpc_client.heartbeat_worker import HeartbeatWorker

__all__ = ["NodeServerClient", "HeartbeatWorker"]
