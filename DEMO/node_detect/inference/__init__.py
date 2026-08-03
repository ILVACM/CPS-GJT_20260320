"""
inference 模块 — 检测节点 推理核心。

子模块：
  - constants: 推理常量（类别/尺寸/ImageNet 参数）
  - model: UNet（ResNet50 backbone）网络定义
  - predictor: 完整推理流水线封装
"""

from inference.constants import (
    NUM_CLASSES,
    INPUT_SIZE,
    IMAGENET_MEAN,
    IMAGENET_STD,
    CLASS_NAMES,
    CLASS_BACKGROUND,
    CLASS_REBAR_VERTICAL,
    CLASS_REBAR_HORIZONTAL,
)
from inference.predictor import RebarPredictor

__all__ = [
    "NUM_CLASSES",
    "INPUT_SIZE",
    "IMAGENET_MEAN",
    "IMAGENET_STD",
    "CLASS_NAMES",
    "CLASS_BACKGROUND",
    "CLASS_REBAR_VERTICAL",
    "CLASS_REBAR_HORIZONTAL",
    "RebarPredictor",
]
