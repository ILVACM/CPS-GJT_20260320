"""
grpc_server 模块 — 检测节点 gRPC 服务端。

提供 Infer RPC（B→A 方向，接收 JPEG 帧，返回 PNG 掩码）。

子模块：
  - servicer: RPC 实现（Infer / Heartbeat / Shutdown 回调），注入 predictor + state_machine
  - server_factory: gRPC Server 生命周期封装
"""

__all__ = ["RebarInferenceServicer", "GRPCServer"]
