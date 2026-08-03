"""
gRPC Server 生命周期封装。

职责：
  1. 封装 grpc.server 的构造、Servicer 注册、端口绑定、启停控制
  2. 支持优雅停止（grace period 等待未完成请求）
  3. 提供回调钩子供 main.py 编排启动序列

用法：
    predictor = RebarPredictor(...)
    state_machine = StateMachine(NodeState.IDLE)
    grpc_server = GRPCServer(predictor, "0.0.0.0:50051", state_machine)
    grpc_server.start()
    # ...
    grpc_server.stop(grace=2.0)
"""

import logging
import threading
import time
from concurrent import futures
from typing import Optional

import grpc

from system.lifecycle import StateMachine
from inference.predictor import RebarPredictor
from grpc_server.servicer import RebarInferenceServicer
from proto import rebar_inference_pb2_grpc as pb2_grpc

logger = logging.getLogger("node_detect.grpc_server.server_factory")


class GRPCServer:
    """
    封装 gRPC 服务端生命周期。

    Attributes:
        _predictor: 推理器实例
        _state_machine: 状态机（由 servicer 注入）
        _address: 监听地址（如 "0.0.0.0:50051"）
        _max_workers: 线程池大小（默认 4，对齐 Design-AI_detect.md §3.5）
        _server: grpc.server 实例（start 后赋值）
    """

    def __init__(
        self,
        predictor: RebarPredictor,
        listen_address: str,
        state_machine: StateMachine,
        max_workers: int = 4,
    ) -> None:
        """
        初始化 gRPC 服务器。

        Args:
            predictor: 已加载权重的推理器实例
            listen_address: 监听地址（如 "0.0.0.0:50051"）
            state_machine: 状态机实例
            max_workers: 线程池大小（原型阶段 4 工作线程）
        """
        self._predictor = predictor
        self._state_machine = state_machine
        self._address = listen_address
        self._max_workers = max_workers
        self._server: Optional[grpc.Server] = None
        self._lock = threading.Lock()

        logger.info(
            f"[GRPCServer] 创建: address={listen_address}, "
            f"max_workers={max_workers}"
        )

    def start(self) -> None:
        """
        启动 gRPC 服务端（非阻塞，服务在后台线程运行）。

        步骤：
          1. 构造 grpc.server（ThreadPoolExecutor）
          2. 注册 RebarInferenceServicer（注入 predictor + state_machine）
          3. 绑定端口（insecure_port，原型阶段不启用 TLS）
          4. 启动 server

        Raises:
            RuntimeError: 重复调用 start() 或端口绑定失败
        """
        with self._lock:
            if self._server is not None:
                raise RuntimeError("gRPC server 已启动，不可重复调用 start()")

            # 构造 server
            self._server = grpc.server(
                futures.ThreadPoolExecutor(max_workers=self._max_workers),
                options=[
                    ('grpc.max_send_message_length', 50 * 1024 * 1024),   # 50MB
                    ('grpc.max_receive_message_length', 50 * 1024 * 1024),  # 50MB
                ],
            )

            # 注册 Servicer
            servicer = RebarInferenceServicer(
                predictor=self._predictor,
                state_machine=self._state_machine,
            )
            pb2_grpc.add_RebarInferenceServicer_to_server(servicer, self._server)

            # 绑定端口
            port = self._server.add_insecure_port(self._address)
            if port == 0:
                self._server = None
                raise RuntimeError(
                    f"无法绑定地址: {self._address}（端口可能被占用）"
                )

            # 启动
            self._server.start()
            logger.info(
                f"[GRPCServer] ✓ 启动成功: {self._address} "
                f"(线程池={self._max_workers})"
            )

    def stop(self, grace: float = 2.0) -> None:
        """
        停止 gRPC 服务端。

        Args:
            grace: 优雅停止宽限期（秒）。
                   grace > 0 时等待未完成请求；grace=0 立即停止。
                   默认 2 秒，与 Design-AI_detect.md §7.4 退出机制对齐。

        Raises:
            RuntimeError: 重复调用 stop()
        """
        with self._lock:
            if self._server is None:
                logger.warning("[GRPCServer] stop() 调用时 server 未运行")
                return

            logger.info(f"[GRPCServer] 正在停止（grace={grace}s）...")
            self._server.stop(grace=grace)

            # 等待终止
            deadline = time.time() + max(grace + 1, 5.0)
            while time.time() < deadline:
                time.sleep(0.1)

            self._server = None
            logger.info("[GRPCServer] ✓ 已停止")

    @property
    def is_running(self) -> bool:
        """检查 gRPC 服务是否正在运行"""
        with self._lock:
            return self._server is not None

    @property
    def address(self) -> str:
        """监听地址"""
        return self._address

    @property
    def state_machine(self) -> StateMachine:
        """关联的状态机实例（供 main.py 访问）"""
        return self._state_machine
