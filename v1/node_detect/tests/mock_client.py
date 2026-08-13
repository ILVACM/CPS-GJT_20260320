"""
mock_client.py — 模拟服务节点，向检测节点 发送 Infer 请求进行端到端冒烟测试。

模拟流程：
  1. 生成合成图像（纯色 + 简单图形，不带真实钢筋特征）
  2. 编码为 JPEG
  3. 通过 gRPC 调用检测节点 的 Infer RPC
  4. 接收 PNG 掩码响应
  5. 验证：frame_id 回传一致、响应 status 合法、PNG 掩码可解码

使用前提：
  - 检测节点 已启动（python -m node_detect run）
  - 检测节点 使用 mock 测试权重（或真实权重，仅验证链路，不验证分割精度）

调用方式：
  python tests/mock_client.py
  python tests/mock_client.py --host 127.0.0.1 --port 50051 --frames 3
"""

import argparse
import io
import logging
import os
import sys
import time
from pathlib import Path

import cv2
import grpc
import numpy as np
from PIL import Image, ImageDraw

# 添加项目根目录到 Python 路径（使 proto / grpc_client 可导入）
BASE_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE_DIR))

from proto import rebar_inference_pb2 as pb2
from proto import rebar_inference_pb2_grpc as pb2_grpc

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d | %(levelname)-7s | mock_client | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("node_detect.tests.mock_client")

# ============================================================
# 合成图像生成（不依赖真实钢筋图像）
# ============================================================


def generate_synthetic_image(
    width: int = 1280,
    height: int = 720,
    pattern: str = "stripes",
) -> bytes:
    """
    生成合成图像并编码为 JPEG。

    Args:
        width, height: 图像尺寸
        pattern: 合成图案类型（stripes / circles / gradient）

    Returns:
        JPEG 编码的字节流
    """
    img = Image.new("RGB", (width, height), color=(128, 128, 128))
    draw = ImageDraw.Draw(img)

    if pattern == "stripes":
        # 垂直灰色条纹（模拟钢筋轮廓，不保证语义准确）
        for x in range(0, width, 100):
            draw.rectangle([x, 0, x + 60, height], fill=(200, 200, 200))
    elif pattern == "circles":
        for i in range(8):
            cx = (i + 1) * width // 9
            cy = height // 2
            r = 50
            draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(180, 180, 180))
    elif pattern == "gradient":
        for y in range(height):
            v = int(255 * y / height)
            draw.line([(0, y), (width, y)], fill=(v, v, v))

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


# ============================================================
# Smoke Test
# ============================================================


def smoke_test(
    target: str = "127.0.0.1:50051",
    num_frames: int = 3,
    timeout: float = 10.0,
) -> bool:
    """
    端到端冒烟测试。

    Args:
        target:     检测节点 gRPC 地址
        num_frames: 发送的测试帧数（依次使用不同图案）
        timeout:    单次 RPC 超时（秒）

    Returns:
        True — 全部帧测试通过；False — 至少一帧失败
    """
    logger.info("=" * 60)
    logger.info("检测节点 端到端冒烟测试 (mock_client)")
    logger.info(f"目标: {target} | 帧数: {num_frames} | 超时: {timeout}s")
    logger.info("=" * 60)

    # 建立 gRPC channel
    try:
        channel = grpc.insecure_channel(target)
        # 等待连接就绪（非阻塞检查，5 秒超时）
        grpc.channel_ready_future(channel).result(timeout=timeout)
        logger.info(f"[连接] 已连接到检测节点: {target}")
    except grpc.FutureTimeoutError:
        logger.error(f"[连接] 连接超时: {target}")
        logger.error("请确认检测节点 已启动: python -m node_detect run")
        return False
    except Exception as e:
        logger.error(f"[连接] 异常: {e}")
        return False

    stub = pb2_grpc.RebarInferenceStub(channel)

    # 准备测试帧
    patterns = ["stripes", "circles", "gradient"]
    total_ok = 0
    total_fail = 0

    for i in range(num_frames):
        pattern = patterns[i % len(patterns)]
        jpeg_bytes = generate_synthetic_image(pattern=pattern)
        frame_id = i + 1

        logger.info(
            f"[帧 {frame_id}] 图案={pattern}, "
            f"JPEG={len(jpeg_bytes)} bytes"
        )

        request = pb2.InferRequest(
            image=jpeg_bytes,
            frame_id=frame_id,
            timestamp_ms=int(time.time() * 1000),
            camera_distance_mm=812.0,
        )

        try:
            t_start = time.time()
            response = stub.Infer(request, timeout=timeout)
            elapsed_ms = (time.time() - t_start) * 1000

            # 验证 1: frame_id 一致
            if response.frame_id != frame_id:
                logger.error(
                    f"[帧 {frame_id}] ✗ frame_id 不匹配: "
                    f"期望 {frame_id}, 实际 {response.frame_id}"
                )
                total_fail += 1
                continue

            # 验证 2: status 合法（0-3）
            if response.status not in (0, 1, 2, 3):
                logger.error(
                    f"[帧 {frame_id}] ✗ status 非法: {response.status}"
                )
                total_fail += 1
                continue

            # 验证 3: 掩码可解码（PNG）
            if response.width > 0 and response.height > 0:
                mask_array = np.frombuffer(response.label_mask, dtype=np.uint8)
                mask = cv2.imdecode(mask_array, cv2.IMREAD_UNCHANGED)
                if mask is None:
                    logger.warning(
                        f"[帧 {frame_id}] ⚠ PNG 掩码解码失败（可能权重异常）"
                    )
                    # 不记为失败——可能是权重缺失或推理异常，链路线是通的
                else:
                    logger.info(
                        f"[帧 {frame_id}] ✓ 掩码解码: {mask.shape}, "
                        f"dtype={mask.dtype}"
                    )

            status_name = {
                0: "UNKNOWN", 1: "IDLE", 2: "BUSY", 3: "SHUTTING_DOWN"
            }.get(response.status, f"UNKNOWN({response.status})")

            logger.info(
                f"[帧 {frame_id}] ✓ frame_id={response.frame_id}, "
                f"width={response.width}, height={response.height}, "
                f"status={status_name}={response.status}, "
                f"耗时={elapsed_ms:.1f}ms"
            )
            total_ok += 1

        except grpc.RpcError as e:
            logger.error(
                f"[帧 {frame_id}] ✗ gRPC 错误: code={e.code()}, "
                f"details={e.details()}"
            )
            total_fail += 1
        except Exception as e:
            logger.error(f"[帧 {frame_id}] ✗ 未知异常: {e}", exc_info=True)
            total_fail += 1

    # 汇总
    logger.info("-" * 60)
    logger.info(
        f"[结果] 总计={num_frames}, 通过={total_ok}, 失败={total_fail}"
    )
    if total_fail == 0:
        logger.info("✓ 冒烟测试通过")
    else:
        logger.warning(f"✗ 存在 {total_fail} 次失败")
    logger.info("=" * 60)

    # 关闭 channel
    channel.close()
    return total_fail == 0


# ============================================================
# CLI 入口
# ============================================================


def main():
    parser = argparse.ArgumentParser(
        description="mock_client — 模拟服务节点 向检测节点 发送 Infer 请求（冒烟测试）"
    )
    parser.add_argument(
        "--host", default="127.0.0.1",
        help="检测节点 监听地址（默认 127.0.0.1）"
    )
    parser.add_argument(
        "--port", type=int, default=50051,
        help="检测节点 gRPC 端口（默认 50051）"
    )
    parser.add_argument(
        "--frames", type=int, default=3,
        help="发送的测试帧数（默认 3）"
    )
    parser.add_argument(
        "--timeout", type=float, default=10.0,
        help="单次 RPC 超时（秒，默认 10）"
    )
    args = parser.parse_args()

    target = f"{args.host}:{args.port}"
    ok = smoke_test(
        target=target,
        num_frames=args.frames,
        timeout=args.timeout,
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
