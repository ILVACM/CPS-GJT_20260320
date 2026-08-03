"""Orbbec Gemini 336L 相机实现（USB 直连）。

对齐 Design-server.md §3.3.2 与 AGENTS.md §3.2。
原型阶段不依赖 Orbbec SDK，统一用 OpenCV VideoCapture 接入：
- Windows 用 DirectShow 后端（cv2.CAP_DSHOW）
- Linux 用默认 V4L2 后端
后台线程持续读帧、取最新帧策略；断线自动重连。
"""
import logging
import platform
import threading
import time
from typing import Dict, Optional

import cv2
import numpy as np

from .base import BaseCameraInput
from common.config_loader import CameraConfig, IntrinsicsConfig

logger = logging.getLogger("node_server.camera")


class Orbbec336LInput(BaseCameraInput):
    """Orbbec Gemini 336L 相机实现。

    后台线程持续读帧，取最新帧策略；断线自动重连（最多 5 次，间隔 3 秒）。
    所有可变状态（_latest_frame）由 ``_frame_lock`` 保护，线程安全。
    """

    def __init__(self, camera_cfg: CameraConfig, intrinsics_cfg: IntrinsicsConfig):
        """初始化相机。

        :param camera_cfg: 相机参数（分辨率/帧率/设备索引等）
        :param intrinsics_cfg: 相机内参
        """
        self._camera_cfg: CameraConfig = camera_cfg
        self._intrinsics_cfg: IntrinsicsConfig = intrinsics_cfg
        self._cap: Optional[cv2.VideoCapture] = None
        self._is_opened: bool = False
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock: threading.Lock = threading.Lock()
        self._read_thread: Optional[threading.Thread] = None
        self._is_running: bool = False
        # 断线重连参数
        self._reconnect_interval: float = 3.0
        self._max_reconnect_attempts: int = 5
        self._reconnect_attempts: int = 0

    def open(self) -> bool:
        """打开设备并启动后台读帧线程。

        根据 CameraConfig 设置分辨率、帧率与 MJPG 格式。
        :return: 打开成功返回 True
        """
        try:
            backend = self._select_backend()
            if backend is not None:
                self._cap = cv2.VideoCapture(self._camera_cfg.device_index, backend)
            else:
                self._cap = cv2.VideoCapture(self._camera_cfg.device_index)

            # 设置采集参数
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._camera_cfg.rgb_width))
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._camera_cfg.rgb_height))
            self._cap.set(cv2.CAP_PROP_FPS, float(self._camera_cfg.rgb_fps))
            # MJPG 格式（FourCC）
            if self._camera_cfg.rgb_format.upper() == "MJPG":
                self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            # 缓冲区设为 1，降低延迟（取最新帧）
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if not self._cap.isOpened():
                logger.error("336L 设备打开失败: device_index=%d", self._camera_cfg.device_index)
                self._cap = None
                return False

            self._is_opened = True
            self._is_running = True
            self._reconnect_attempts = 0
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            logger.info(
                "336L 设备打开成功: %dx%d@%dfps",
                self._camera_cfg.rgb_width,
                self._camera_cfg.rgb_height,
                self._camera_cfg.rgb_fps,
            )
            return True
        except Exception as e:
            logger.error("336L 设备打开异常: %s", e, exc_info=True)
            self._cap = None
            return False

    def close(self) -> None:
        """关闭设备并停止后台读帧线程。"""
        self._is_running = False
        if self._read_thread is not None and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
            self._read_thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._is_opened = False
        with self._frame_lock:
            self._latest_frame = None
        logger.info("336L 设备已关闭")

    def is_opened(self) -> bool:
        return self._is_opened

    def get_rgb_frame(self) -> Optional[np.ndarray]:
        """获取最新帧的拷贝（线程安全）。

        :return: BGR numpy 数组拷贝；无可用帧返回 None
        """
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()

    def get_depth_frame(self) -> Optional[np.ndarray]:
        """原型阶段不实现，返回 None（AGENTS.md §5.3）。"""
        return None

    def get_intrinsics(self) -> Dict[str, float]:
        """从 IntrinsicsConfig 返回内参。"""
        return {
            "fx": self._intrinsics_cfg.fx,
            "fy": self._intrinsics_cfg.fy,
            "cx": self._intrinsics_cfg.cx,
            "cy": self._intrinsics_cfg.cy,
        }

    def try_reconnect(self) -> bool:
        """断线重连：间隔 3 秒，最多 5 次。

        :return: 重连成功返回 True；达到上限返回 False
        """
        if self._reconnect_attempts >= self._max_reconnect_attempts:
            logger.error("336L 重连次数已达上限 %d 次，放弃重连", self._max_reconnect_attempts)
            return False
        self._reconnect_attempts += 1
        logger.warning(
            "336L 尝试第 %d/%d 次重连（间隔 %.1fs）",
            self._reconnect_attempts,
            self._max_reconnect_attempts,
            self._reconnect_interval,
        )
        time.sleep(self._reconnect_interval)
        return self.open()

    @staticmethod
    def _select_backend() -> Optional[int]:
        """根据操作系统选择 VideoCapture 后端。

        :return: Windows 返回 cv2.CAP_DSHOW；Linux 返回 None（用默认 V4L2）
        """
        if platform.system() == "Windows":
            return cv2.CAP_DSHOW
        return None

    def _read_loop(self) -> None:
        """后台线程：持续读帧，更新 _latest_frame。

        读帧失败或设备掉线时触发重连流程。
        """
        while self._is_running:
            if self._cap is not None and self._cap.isOpened():
                ret, frame = self._cap.read()
                if ret and frame is not None:
                    with self._frame_lock:
                        self._latest_frame = frame
                    self._reconnect_attempts = 0
                # 读帧失败不立即退出，短暂让出 CPU 后继续尝试
                time.sleep(0.001)
            else:
                # 设备掉线
                logger.warning("336L 设备掉线，触发重连")
                self._is_opened = False
                if self._cap is not None:
                    self._cap.release()
                    self._cap = None
                if not self.try_reconnect():
                    self._is_running = False
                    break
