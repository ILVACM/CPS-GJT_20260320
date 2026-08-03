"""
test_predictor_offline.py - 离线验证 predictor 推理流水线（mock 权重）。

验证内容：
  1. 构造随机初始化的 UNet 权重
  2. RebarPredictor.infer() 全流程（JPEG → 推理 → PNG mask）
  3. 验证输出 mask 维度正确、dtype=uint8、值域 [0, 2]
  4. 异常路径：损坏 JPEG → 返回 None

调用方式：
  python tests/test_predictor_offline.py
"""

import io
import logging
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageDraw

# 添加项目根目录到路径
BASE_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE_DIR))

from inference.predictor import RebarPredictor
from inference.model import UNetModel

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d | %(levelname)-7s | offl_test | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("node_detect.tests.predictor_offline")


def test_predictor_full_pipeline():
    """端到端 predictor 测试"""
    logger.info("=" * 60)
    logger.info("predictor 离线验证（mock 权重）")
    logger.info("=" * 60)

    # 创建 mock 权重
    with tempfile.TemporaryDirectory() as tmpdir:
        model_path = Path(tmpdir) / "mock_unet.pth"
        model = UNetModel(num_classes=3)
        torch.save(model.state_dict(), str(model_path))

        # 初始化 predictor
        predictor = RebarPredictor(
            model_path=str(model_path),
            use_cuda=False,  # 开发环境用 CPU
        )
        assert predictor.is_ready

        # 构造测试 JPEG 图像
        test_img = Image.new("RGB", (1920, 1080), color=(100, 150, 200))
        draw = ImageDraw.Draw(test_img)
        for x in range(0, 1920, 100):
            draw.rectangle([x, 0, x + 50, 1080], fill=(200, 200, 200))

        buf = io.BytesIO()
        test_img.save(buf, format="JPEG", quality=85)
        jpeg_bytes = buf.getvalue()
        logger.info(f"测试图像: 1920x1080 JPEG={len(jpeg_bytes)} bytes")

        # 执行推理
        t_start = time.time()
        result = predictor.infer(jpeg_bytes)
        elapsed_ms = (time.time() - t_start) * 1000
        logger.info("推理耗时: %.1fms", elapsed_ms)

        assert result is not None, "predictor 返回 None"
        mask, width, height = result

        # 验证输出
        assert isinstance(mask, np.ndarray), f"mask 类型错误: {type(mask)}"
        assert mask.dtype == np.uint8, f"mask dtype 错误: {mask.dtype}"
        assert mask.shape == (1080, 1920), f"mask shape 错误: {mask.shape}"
        assert width == 1920 and height == 1080, f"尺寸错误: {width}x{height}"
        unique_vals = set(np.unique(mask).tolist())
        assert unique_vals.issubset({0, 1, 2}), f"mask 值域错误: {unique_vals}"

        logger.info(
            "mask 验证: shape=%s, dtype=%s, 唯一值=%s",
            mask.shape, mask.dtype, unique_vals,
        )

    # 测试异常路径（predictor 即使出了 with 范围仍可调用）
    bad_result = predictor.infer(b"not a jpeg image")
    assert bad_result is None, "损坏 JPEG 应返回 None"
    logger.info("异常路径: 损坏 JPEG → None")

    logger.info("=" * 60)
    logger.info("[DONE] predictor 离线验证通过")
    logger.info("=" * 60)
    return True


if __name__ == "__main__":
    try:
        ok = test_predictor_full_pipeline()
        sys.exit(0 if ok else 1)
    except Exception as e:
        logger.error("测试失败: %s", e, exc_info=True)
        sys.exit(1)
