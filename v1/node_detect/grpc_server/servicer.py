"""
gRPC Service 实现 — RebarInferenceServicer。

职责：
  1. 接收 InferRequest（JPEG 帧）→ predictor.infer() → InferResponse（PNG 掩码）
  2. 管理状态机：Infer 触发 IDLE → INFERENCING → RETURNING → IDLE
  3. InferResponse 中捎带当前 NodeStatus（status 字段）
  4. 异常场景：predictor 返回 None → InferResponse(width=0) + status=BUSY

依赖注入：
  - predictor: RebarPredictor 实例（已加载权重）
  - state_machine: StateMachine 实例（管理状态转换）

线程安全：
  - 使用 state_machine.try_transition() 非阻塞获取推理锁
  - ThreadPoolExecutor(max_workers=4) 由 server_factory 提供
  - 单 GPU 串行通过 try_transition 保证（仅 1 个请求可进入 INFERENCING）
"""

import logging
import time
from typing import Optional

import cv2
import numpy as np

from system.lifecycle import NodeState, StateMachine
from inference.predictor import RebarPredictor
from proto import rebar_inference_pb2 as pb2
from proto import rebar_inference_pb2_grpc as pb2_grpc

logger = logging.getLogger("node_detect.grpc_server.servicer")


class RebarInferenceServicer(pb2_grpc.RebarInferenceServicer):
    """
    gRPC 服务实现类（检测节点 端）。

    提供 3 个 RPC：
      - Infer: B→A 推理请求（核心业务）
      - Heartbeat: A→B 保活推送的接收端（实际由服务节点 的 server 受理，
                   检测节点 仅作为客户端调用对端；此方法在 A 的 server 上保留
                   以支持未来对等心跳或健康检查，但原型阶段服务节点 不会调用）
      - Shutdown: A→B 退出通知的接收端（同理）
    """

    def __init__(self, predictor: Optional[RebarPredictor], state_machine: StateMachine) -> None:
        """
        初始化 Servicer。

        Args:
            predictor: 已加载权重的推理器实例（可为 None — 降级模式，推理 RPC 返回空结果）
            state_machine: 状态机实例（管理 IDLE/INFERENCING/RETURNING/SHUTDOWN）
        """
        super().__init__()
        self._predictor = predictor
        self._state_machine = state_machine
        self._infer_count: int = 0          # 累计推理成功次数
        self._last_infer_duration_ms: int = 0  # 最近一次推理耗时（毫秒）
        logger.info("[Servicer] 初始化完成")

    # ---- 业务 RPC：Infer ----

    def Infer(self, request: pb2.InferRequest, context) -> pb2.InferResponse:
        """
        推理 RPC（B→A）。

        流程：
          1. 非阻塞 try_transition 获取推理锁（IDLE → INFERENCING）
          2. 记录时间戳
          3. predictor.infer(jpeg_bytes) 执行推理
          4. 状态转换 INFERENCING → RETURNING（后处理 + PNG 编码）
          5. PNG 编码掩码，组装 InferResponse
          6. 状态转换 RETURNING → IDLE

        异常处理：
          - 推理被拒绝（状态机不在 IDLE）→ 返回 width=0 + status=BUSY
          - predictor 返回 None → width=0 + status=RETURNING→IDLE
          - PNG 编码异常 → width=0

        Args:
            request: InferRequest（JPEG 帧 + frame_id + timestamp_ms + camera_distance_mm）
            context: gRPC 上下文

        Returns:
            InferResponse（PNG 掩码 + frame_id 回传 + status 捎带）
        """
        frame_id = request.frame_id
        timestamp_ms = request.timestamp_ms
        camera_distance_mm = request.camera_distance_mm

        logger.info(
            f"[Infer] frame_id={frame_id}, distance={camera_distance_mm}mm"
        )

        # ---- 1. 尝试进入 INFERENCING 态 ----
        if not self._state_machine.try_transition(NodeState.INFERENCING):
            logger.warning(
                f"[Infer] 推理被拒绝（当前 {self._state_machine.current.name}），"
                f"frame_id={frame_id}"
            )
            return pb2.InferResponse(
                label_mask=b"",
                frame_id=frame_id,
                timestamp_ms=timestamp_ms,
                width=0,
                height=0,
                status=self._state_machine.current_proto_status(),
            )

        # ---- 2. 执行推理 ----
        t_start = time.time()
        try:
            mask, width, height = None, 0, 0

            # 降级模式检查：predictor 为 None 或未就绪（模型加载失败/降级）时，拒绝推理但服务不崩溃
            if self._predictor is None:
                logger.error(
                    f"[Infer] 节点处于降级模式（predictor 为 None，模型未加载），拒绝 frame_id={frame_id}"
                )
                mask, width, height = None, 0, 0
            elif not self._predictor.is_ready:
                logger.error(
                    f"[Infer] predictor 未就绪（is_ready=False，可能 state_dict 未完全加载），"
                    f"拒绝 frame_id={frame_id}"
                )
                mask, width, height = None, 0, 0
            else:
                # 状态转换到 RETURNING（后处理态）— 在 predictor.infer 内部完成推理
                result = self._predictor.infer(request.image)

                if result is not None:
                    mask, width, height = result
                    self._infer_count += 1
                    logger.debug(
                        f"[Infer] 推理成功: mask={width}x{height}, "
                        f"累计推理={self._infer_count}"
                    )
                else:
                    logger.error(f"[Infer] predictor 返回 None（frame_id={frame_id}）")

        except Exception as e:
            logger.error(f"[Infer] 推理异常: {e}", exc_info=True)
            mask = None
            width, height = 0, 0

        elapsed_ms = int((time.time() - t_start) * 1000)
        self._last_infer_duration_ms = elapsed_ms

        # ---- 3. 进入 RETURNING 态 ----
        self._state_machine.transition(NodeState.RETURNING)

        # ---- 4. PNG 编码 + 组装响应 ----
        png_bytes = b""
        try:
            if mask is not None and width > 0 and height > 0:
                success, encoded = cv2.imencode('.png', mask)
                if success:
                    png_bytes = encoded.tobytes()
                    logger.debug(
                        f"[Infer] PNG 编码: {len(png_bytes)} bytes"
                    )
                else:
                    logger.error(f"[Infer] PNG 编码失败（frame_id={frame_id}）")
                    width, height = 0, 0
        except Exception as e:
            logger.error(f"[Infer] PNG 编码异常: {e}", exc_info=True)
            width, height = 0, 0

        # ---- 5. 释放锁，回 IDLE ----
        self._state_machine.transition(NodeState.IDLE)

        response = pb2.InferResponse(
            label_mask=png_bytes,
            frame_id=frame_id,
            timestamp_ms=timestamp_ms,
            width=width,
            height=height,
            status=self._state_machine.current_proto_status(),
        )

        logger.info(
            f"[Infer] 完成 frame_id={frame_id}, size={width}x{height}, "
            f"耗时={elapsed_ms}ms, status={response.status}"
        )
        return response

    # ---- 辅助 RPC：Heartbeat + Shutdown（检测节点 端接收，proto 兼容预留） ----

    def Heartbeat(self, request: pb2.HeartbeatRequest, context) -> pb2.HeartbeatResponse:
        """
        心跳 RPC（保留方法）。

        原型阶段此方法不会被调用（心跳方向为 A→B，不在 A 端保留心跳接收）。
        但实现以支持未来节点对等心跳或其他健康检查场景。
        """
        logger.debug(
            f"[Heartbeat] node_id={request.node_id}, "
            f"status={request.status}"
        )
        import time
        return pb2.HeartbeatResponse(
            accepted=True,
            server_timestamp_ms=int(time.time() * 1000),
        )

    def Shutdown(self, request: pb2.ShutdownRequest, context) -> pb2.ShutdownResponse:
        """
        退出通知 RPC（保留方法）。

        原型阶段保留（Shutdown 方向为 A→B，不在 A 端接收）。
        """
        logger.info(
            f"[Shutdown] node_id={request.node_id}, reason={request.reason}"
        )
        return pb2.ShutdownResponse(
            acknowledged=True,
            server_timestamp_ms=int(time.time() * 1000),
        )

    # ---- 监控属性（Heartbeat 推送时使用） ----

    @property
    def infer_count(self) -> int:
        """累计推理成功次数"""
        return self._infer_count

    @property
    def last_infer_duration_ms(self) -> int:
        """最近一次推理耗时（毫秒）"""
        return self._last_infer_duration_ms
