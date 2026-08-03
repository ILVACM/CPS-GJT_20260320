"""服务节点 → 检测节点 推理服务 gRPC 客户端。

封装 JPEG 编码 → gRPC 请求 → PNG 解码的全链路，对齐：
- AGENTS.md §5.2（推理段数据流）、§5.5（proto 冻结字段）、§5.6（帧同步）、§8.1（性能）
- Design-server.md §3.5（接口契约）、§10.3（B→A 退出通知占位）

设计要点：
- 超时/异常一律返回 None，不抛异常（不阻塞采集主循环，对齐 §8.1 / §5.6）
- gRPC channel 线程安全，但连接状态字段以 Lock 保护
- deadline 从 NetworkConfig.grpc_deadline_ms 读取，禁止硬编码
- 帧同步：response.frame_id 与请求 frame_id 不一致直接丢弃（§5.6 强制要求）
"""
import logging
import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import grpc
import numpy as np

from common.config_loader import NetworkConfig
from proto import rebar_inference_pb2
from proto import rebar_inference_pb2_grpc


# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 grpc_client）
_logger = logging.getLogger("node_server.grpc_client")

# JPEG 编码质量（兼顾带宽与质量，原型阶段固定值；不属于值类参数，无需配置化）
_JPEG_QUALITY: int = 90


@dataclass
class InferenceResult:
    """推理响应解析结果（对齐任务契约 3.3）。

    :ivar mask: 单通道 0/1/2 类别掩码（uint8，与原图同尺寸）
    :ivar frame_id: 帧序号（与请求一致，帧同步主键）
    :ivar timestamp_ms: 时间戳（与请求一致，帧同步辅助校验）
    :ivar width: 掩码宽
    :ivar height: 掩码高
    :ivar status_code: 检测节点 状态（NodeStatus 枚举值，状态捎带 §5.7.2）
    """

    mask: np.ndarray
    frame_id: int
    timestamp_ms: int
    width: int
    height: int
    status_code: int


class InferenceGrpcClient:
    """检测节点 推理服务 gRPC 客户端。

    封装 JPEG 编码 → gRPC 请求 → PNG 解码的全链路。
    所有公共方法在失败/超时时返回 None 或 False，不抛异常，确保不阻塞主循环。
    """

    def __init__(self, network_cfg: NetworkConfig) -> None:
        """初始化客户端。

        :param network_cfg: 网络配置（含 remote_ip / grpc_port / grpc_deadline_ms）
        """
        self._target: str = f"{network_cfg.remote_ip}:{network_cfg.grpc_port}"
        # gRPC 单次调用超时（秒），从配置读取，禁止硬编码
        self._deadline_s: float = network_cfg.grpc_deadline_ms / 1000.0
        self._channel: Optional[grpc.Channel] = None
        self._stub: Optional[rebar_inference_pb2_grpc.RebarInferenceStub] = None
        # 连接状态以 Lock 保护（channel 本身线程安全，但 _connected 状态需保护）
        self._lock: threading.Lock = threading.Lock()
        self._connected: bool = False

    def connect(self) -> bool:
        """建立到检测节点（remote_ip:grpc_port）的 gRPC channel。

        :return: True 表示信道建立成功（不代表对端服务在线，仅代表 channel 创建成功）
        """
        with self._lock:
            if self._connected and self._channel is not None:
                return True
            try:
                self._channel = grpc.insecure_channel(self._target)
                self._stub = rebar_inference_pb2_grpc.RebarInferenceStub(self._channel)
                self._connected = True
                _logger.info("gRPC 信道建立成功: %s", self._target)
                return True
            except Exception as e:
                _logger.error("gRPC 信道建立失败: %s, 错误: %s", self._target, e)
                self._channel = None
                self._stub = None
                self._connected = False
                return False

    def is_connected(self) -> bool:
        """返回当前信道是否已建立。"""
        with self._lock:
            return self._connected

    def infer(
        self,
        rgb: np.ndarray,
        frame_id: int,
        camera_distance_mm: float,
    ) -> Optional[InferenceResult]:
        """发送单帧推理请求并解析响应。

        全链路：
        1. RGB 帧 → JPEG 编码（cv2.imencode）
        2. 构造 InferRequest（image, frame_id, timestamp_ms=当前毫秒, camera_distance_mm）
        3. 调用 stub.Infer(request, timeout=deadline_s)
        4. 响应 PNG 掩码 → cv2.imdecode 为单通道 ndarray
        5. 帧同步校验：response.frame_id == request.frame_id，不符返回 None
        6. 超时/异常返回 None，不抛异常

        :param rgb: BGR numpy 数组（OpenCV 默认色彩格式）
        :param frame_id: 帧序号（单调递增，帧同步主键）
        :param camera_distance_mm: 拍摄距离标量（激光测距值，mm）
        :return: InferenceResult 或 None（超时/失败/帧同步不符）
        """
        # 前置检查：信道未建立
        with self._lock:
            stub = self._stub
            connected = self._connected
        if not connected or stub is None:
            _logger.warning("推理请求 frame_id=%s 失败: gRPC 信道未建立", frame_id)
            return None

        # 1. JPEG 编码（失败返回 None，不抛异常）
        try:
            ok, jpeg_buf = cv2.imencode(".jpg", rgb, [int(cv2.IMWRITE_JPEG_QUALITY), _JPEG_QUALITY])
            if not ok:
                _logger.error("JPEG 编码失败 frame_id=%s", frame_id)
                return None
            jpeg_bytes: bytes = jpeg_buf.tobytes()
        except Exception as e:
            _logger.error("JPEG 编码异常 frame_id=%s: %s", frame_id, e)
            return None

        # 2. 构造请求（timestamp_ms = 当前毫秒时间戳）
        timestamp_ms: int = int(time.time() * 1000)
        request = rebar_inference_pb2.InferRequest(
            image=jpeg_bytes,
            frame_id=frame_id,
            timestamp_ms=timestamp_ms,
            camera_distance_mm=camera_distance_mm,
        )

        _logger.info(
            "推理请求: frame_id=%s, distance=%.1fmm, jpeg_size=%dB",
            frame_id, camera_distance_mm, len(jpeg_bytes),
        )

        # 3. 调用 gRPC（超时/异常返回 None）
        try:
            response = stub.Infer(request, timeout=self._deadline_s)
        except grpc.RpcError as e:
            # DEADLINE_EXCEEDED / UNAVAILABLE 等均视为软故障，返回 None
            code = e.code() if hasattr(e, "code") else "UNKNOWN"
            _logger.warning(
                "gRPC 推理调用失败 frame_id=%s, code=%s, deadline=%.2fs",
                frame_id, code, self._deadline_s,
            )
            return None
        except Exception as e:
            _logger.error("gRPC 推理异常 frame_id=%s: %s", frame_id, e)
            return None

        # 5. 帧同步校验（§5.6 强制要求）：response.frame_id 必须与请求一致
        if response.frame_id != frame_id:
            _logger.warning(
                "帧同步不一致: 请求 frame_id=%s, 响应 frame_id=%s, 丢弃该响应",
                frame_id, response.frame_id,
            )
            return None

        # 4. PNG 掩码 → 单通道 ndarray
        try:
            mask_buf = np.frombuffer(response.label_mask, dtype=np.uint8)
            mask = cv2.imdecode(mask_buf, cv2.IMREAD_GRAYSCALE)
            if mask is None:
                _logger.error("PNG 掩码解码失败 frame_id=%s", frame_id)
                return None
        except Exception as e:
            _logger.error("PNG 掩码解码异常 frame_id=%s: %s", frame_id, e)
            return None

        _logger.info(
            "推理响应: frame_id=%s, mask_size=%dx%d, 耗时=%dms",
            frame_id, response.width, response.height,
            int(time.time() * 1000) - timestamp_ms,
        )

        return InferenceResult(
            mask=mask,
            frame_id=response.frame_id,
            timestamp_ms=response.timestamp_ms,
            width=int(response.width),
            height=int(response.height),
            status_code=int(response.status),
        )

    def probe(self, timeout_ms: int = 2000) -> bool:
        """TCP 探测检测节点 端口可达性（用于启动自检与健康检查）。

        使用 socket.connect_ex 探测，不依赖 gRPC channel 状态。
        与 grpc.channel_ready_future 互补：TCP 探测更快、更轻量，适合启动自检。

        :param timeout_ms: 探测超时（毫秒），默认 2000
        :return: True 可达 / False 不可达
        """
        host: str
        port: int
        try:
            host, port_str = self._target.rsplit(":", 1)
            port = int(port_str)
        except Exception as e:
            _logger.error("probe 目标解析失败: %s, 错误: %s", self._target, e)
            return False

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout_ms / 1000.0)
        try:
            # connect_ex 返回 0 表示成功，非 0 为错误码
            result = sock.connect_ex((host, port))
            if result == 0:
                _logger.info("probe 成功: %s:%d 可达", host, port)
                return True
            _logger.warning("probe 失败: %s:%d 不可达, 错误码=%d", host, port, result)
            return False
        except Exception as e:
            _logger.warning("probe 异常: %s:%d, 错误: %s", host, port, e)
            return False
        finally:
            sock.close()

    def request_peer_shutdown(self, deadline_s: float = 3.0) -> Optional[bool]:
        """B→A 退出通知（关键决策 D1，见 §10.3）。

        服务节点 退出时主动请求检测节点 终止服务并反馈。

        - 方向：B → A（与现有 A→B Shutdown 语义不同，二者并存）
        - 暂记 RPC 名：RequestPeerShutdown
        - deadline：默认 3 秒，超时即放弃等待
        - 失败处理：超时/失败仅记 WARNING 日志，不重试、不阻塞 B 退出

        :param deadline_s: 超时（秒），默认 3.0
        :return:
            True  — A 反馈 accepted（A 已正常终止）
            False — A 反馈未接受
            None  — 超时/调用失败（调用方按 best-effort 退出）

        .. warning::
            **待 AGENTS.md §5.5 proto 扩展确认**：本 RPC 为本设计新增的 B→A
            退出通知，AGENTS.md §5.5 现仅定义 A→B ``Shutdown``（A 自报下线）。
            落地此设计需后续 spec 修订 AGENTS.md §5.5 proto 增加 B→A RPC
            ``RequestPeerShutdown``（含请求/响应消息体定义），并修订
            Design-AI_detect.md 消除 §7.4.1 / §9.3 / §3.10 三处方向矛盾。
            引用 spec 问题清单 #1 / #2。

            当前为**占位实现**：仅记 WARNING 日志，返回 None，不发起真实 RPC。
        """
        # 待 AGENTS.md proto 扩展确认 —— 当前为占位实现
        # 真实实现应在 proto 扩展后补充：
        #     request = RequestPeerShutdownRequest(
        #         node_id="node-server",
        #         timestamp_ms=int(time.time() * 1000),
        #         reason="user_exit",
        #     )
        #     response = self._stub.RequestPeerShutdown(
        #         request, timeout=deadline_s
        #     )
        #     return bool(response.accepted)
        _logger.warning(
            "S→D RequestPeerShutdown 待 AGENTS.md proto 扩展，当前为占位实现"
            "（deadline=%.1fs），返回 None", deadline_s,
        )
        return None

    def close(self) -> None:
        """关闭 gRPC 信道。"""
        with self._lock:
            if self._channel is not None:
                try:
                    self._channel.close()
                except Exception as e:
                    _logger.warning("gRPC channel 关闭异常: %s", e)
                finally:
                    self._channel = None
                    self._stub = None
                    self._connected = False
                    _logger.info("gRPC 信道已关闭")
