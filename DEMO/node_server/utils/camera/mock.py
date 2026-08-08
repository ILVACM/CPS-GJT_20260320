"""Mock 相机（调试用）。

对齐 Design-server.md §3.3.3。
服务节点 独立联调场景：无真实相机时以纯黑帧模拟采集，
使上层逻辑（测量、显示、gRPC 转发）可在无硬件环境下跑通。
"""
from typing import Dict, Optional

import numpy as np

from .base import BaseCameraInput
from utils.common.constants import DEFAULT_FX, DEFAULT_FY


class MockCameraInput(BaseCameraInput):
    """模拟相机，返回纯黑测试帧。

    所有抽象方法均有真实实现，``open()`` 直接返回成功。
    内参 cx/cy 由帧宽高的一半推算（与 intrinsics.json 默认值一致）。
    """

    def __init__(self, width: int = 1920, height: int = 1080):
        """初始化 Mock 相机。

        :param width: 帧宽度（像素），默认 1920
        :param height: 帧高度（像素），默认 1080
        """
        self._width: int = width
        self._height: int = height
        self._is_opened: bool = False

    def open(self) -> bool:
        """打开 Mock 设备，直接返回成功。"""
        self._is_opened = True
        return True

    def close(self) -> None:
        """关闭 Mock 设备。"""
        self._is_opened = False

    def is_opened(self) -> bool:
        return self._is_opened

    def get_rgb_frame(self) -> Optional[np.ndarray]:
        """返回一帧纯黑图像 (H, W, 3) uint8。

        每次调用都新建数组，避免外部修改影响后续取帧。
        """
        if not self._is_opened:
            return None
        return np.zeros((self._height, self._width, 3), dtype=np.uint8)

    def get_depth_frame(self) -> Optional[np.ndarray]:
        """原型阶段不实现，返回 None。"""
        return None

    def get_intrinsics(self) -> Dict[str, float]:
        """返回内参：fx/fy 取默认值，cx/cy 由帧宽高推算。"""
        return {
            "fx": DEFAULT_FX,
            "fy": DEFAULT_FY,
            "cx": float(self._width / 2),
            "cy": float(self._height / 2),
        }
