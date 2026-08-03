"""相机输入抽象基类。

对齐 Design-server.md §3.3.1 与 AGENTS.md §3.2（输入模块抽象）。
所有具体实现（Orbbec 336L、网络摄像头、Mock 等）必须继承此类，
上层逻辑仅依赖此抽象接口，不感知具体接入方式差异。
"""
from abc import ABC, abstractmethod
from typing import Dict, Optional

import numpy as np


class BaseCameraInput(ABC):
    """相机输入抽象基类。

    定义统一的相机设备接入契约：打开/关闭/状态查询/取帧/内参获取。
    具体实现需保证线程安全（后台读帧线程与主线程并发访问）。
    """

    @abstractmethod
    def open(self) -> bool:
        """打开设备，返回是否成功。

        :return: 成功返回 True，失败返回 False
        """
        ...

    @abstractmethod
    def close(self) -> None:
        """关闭设备，释放底层资源（VideoCapture / Socket 等）。"""
        ...

    @abstractmethod
    def is_opened(self) -> bool:
        """设备是否已就绪可供取帧。

        :return: 已打开返回 True，否则 False
        """
        ...

    @abstractmethod
    def get_rgb_frame(self) -> Optional[np.ndarray]:
        """获取一帧 RGB 图像（最新帧策略）。

        :return: BGR 格式 numpy 数组 (H, W, 3) uint8；
                 设备未就绪或取帧失败返回 None。
                 返回值应为内部缓存的拷贝，避免外部修改污染内部状态。
        """
        ...

    @abstractmethod
    def get_depth_frame(self) -> Optional[np.ndarray]:
        """获取一帧 Depth 深度图。

        原型阶段可不实现，直接返回 None；属后续精度增强项（AGENTS.md §5.3）。

        :return: 深度图 numpy 数组或 None
        """
        ...

    @abstractmethod
    def get_intrinsics(self) -> Dict[str, float]:
        """获取相机内参。

        :return: ``{"fx": float, "fy": float, "cx": float, "cy": float}``
        """
        ...
