"""Orbbec Gemini 336L 相机实现（USB 直连，pyorbbecsdk Pipeline）。

对齐 Design-server.md §3.3.2 与 AGENTS.md §3.2。

P1 改造（Q1 决策）：从 OpenCV VideoCapture 切换到 pyorbbecsdk Pipeline。
- F3：Pipeline 默认 profile 获取（get_default_video_stream_profile）
- F4：帧聚合模式 FULL_FRAME_REQUIRE（保证帧完整性）
- F6：frame_to_bgr_image 帧转换（VideoFrame → BGR numpy）

保留现有线程模型：后台线程持续读帧、Lock 保护 _latest_frame、断线自动重连。
pyorbbecsdk 懒导入（A6/D3）：Windows 开发环境无 wheel 时 open() 返回 False 不崩溃。
"""
import logging
import threading
import time
from typing import Any, Dict, Optional

import numpy as np

from .base import BaseCameraInput
from utils.common.config_loader import CameraConfig, IntrinsicsConfig

logger = logging.getLogger("node_server.camera")

# A6/D3：pyorbbecsdk 懒导入 — Windows 开发环境无 wheel 时不崩溃
_PYORBBECSDK_AVAILABLE = False
_Pipeline = None
_Config = None
_OBSensorType = None
_OBFrameAggregateOutputMode = None
_OBError = None
_frame_to_bgr_image = None

try:
    from pyorbbecsdk import (
        Pipeline,
        Config,
        OBSensorType,
        OBFrameAggregateOutputMode,
        OBError,
    )
    from .orbbec_utils import frame_to_bgr_image

    _PYORBBECSDK_AVAILABLE = True
    _Pipeline = Pipeline
    _Config = Config
    _OBSensorType = OBSensorType
    _OBFrameAggregateOutputMode = OBFrameAggregateOutputMode
    _OBError = OBError
    _frame_to_bgr_image = frame_to_bgr_image
except ImportError:
    logger.warning(
        "pyorbbecsdk 未安装，Orbbec336LInput 将不可用（D3：Windows 开发环境正常现象）"
    )


class Orbbec336LInput(BaseCameraInput):
    """Orbbec Gemini 336L 相机实现（pyorbbecsdk Pipeline）。

    后台线程持续读帧，取最新帧策略；断线自动重连（最多 5 次，间隔 3 秒）。
    所有可变状态（_latest_frame）由 ``_frame_lock`` 保护，线程安全。

    A1：构造可接收 ``device`` 参数（SDK Device 对象，由 scanner 枚举获得），
    Pipeline 使用该 Device 句柄启动；若不传则 Pipeline 自动选择默认设备。
    """

    def __init__(
        self,
        camera_cfg: CameraConfig,
        intrinsics_cfg: IntrinsicsConfig,
        device: Any = None,
    ):
        """初始化相机。

        :param camera_cfg: 相机参数（分辨率/帧率等，Pipeline 实际使用默认 profile）
        :param intrinsics_cfg: 相机内参
        :param device: pyorbbecsdk Device 对象（A1：由 scanner 通过 Context.query_devices() 枚举获得）；
                       None 时 Pipeline 自动选择默认设备
        """
        self._camera_cfg: CameraConfig = camera_cfg
        self._intrinsics_cfg: IntrinsicsConfig = intrinsics_cfg
        self._device: Any = device  # A1：SDK Device 句柄
        self._pipeline: Any = None  # pyorbbecsdk Pipeline 实例
        self._is_opened: bool = False
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock: threading.Lock = threading.Lock()
        self._read_thread: Optional[threading.Thread] = None
        self._is_running: bool = False
        # 断线重连参数
        self._reconnect_interval: float = 3.0
        self._max_reconnect_attempts: int = 5
        self._reconnect_attempts: int = 0
        # 实际 RGB 分辨率/帧率（F3：从默认 profile 读取）
        self._rgb_width: int = 0
        self._rgb_height: int = 0
        self._rgb_fps: int = 0

    def open(self) -> bool:
        """打开设备并启动后台读帧线程。

        P1 改造（Q1 决策）：使用 pyorbbecsdk Pipeline 替代 cv2.VideoCapture。
        F3：获取 ColorSensor 默认 profile；F4：设置 FULL_FRAME_REQUIRE 帧聚合模式。

        :return: 打开成功返回 True；pyorbbecsdk 不可用或 Pipeline 启动失败返回 False
        """
        # A6/D3：pyorbbecsdk 不可用时直接返回 False
        if not _PYORBBECSDK_AVAILABLE:
            logger.error("pyorbbecsdk 未安装，无法打开 336L 设备（D3：Windows 开发环境）")
            return False

        try:
            # A2/F3：创建 Pipeline + 获取默认 Color profile
            if self._device is not None:
                try:
                    pipeline = _Pipeline(self._device)
                except Exception:
                    # Pipeline(device) 构造不支持时回退到默认 Pipeline()
                    pipeline = _Pipeline()
            else:
                pipeline = _Pipeline()
            self._pipeline = pipeline

            cfg = _Config()

            # F3：获取 ColorSensor 默认 profile
            try:
                profiles = pipeline.get_stream_profile_list(_OBSensorType.COLOR_SENSOR)
                color_profile = profiles.get_default_video_stream_profile()
            except _OBError as e:
                logger.error("336L 获取 Color profile 列表失败 (OBError): %s", e)
                self._pipeline = None
                return False
            except Exception as e:
                logger.error("336L 获取 Color profile 列表异常: %s", e)
                self._pipeline = None
                return False

            self._rgb_width = color_profile.get_width()
            self._rgb_height = color_profile.get_height()
            self._rgb_fps = color_profile.get_fps()

            # F3：启用 Color 流
            try:
                cfg.enable_stream(color_profile)
            except _OBError as e:
                logger.error("336L enable_stream 失败 (OBError): %s", e)
                self._pipeline = None
                return False
            except Exception as e:
                logger.error("336L enable_stream 异常: %s", e)
                self._pipeline = None
                return False

            # F4：设置帧聚合模式（保证帧完整性）
            cfg.set_frame_aggregate_output_mode(
                _OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE
            )

            # 启动 Pipeline
            try:
                pipeline.start(cfg)
            except _OBError as e:
                logger.error("336L Pipeline 启动失败 (OBError): %s", e)
                self._pipeline = None
                return False
            except Exception as e:
                logger.error("336L Pipeline 启动异常: %s", e)
                self._pipeline = None
                return False

            self._is_opened = True
            self._is_running = True
            self._reconnect_attempts = 0
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            logger.info(
                "336L 设备打开成功: %dx%d@%dfps (pyorbbecsdk Pipeline)",
                self._rgb_width, self._rgb_height, self._rgb_fps,
            )
            return True
        except Exception as e:
            logger.error("336L 设备打开异常: %s", e, exc_info=True)
            self._pipeline = None
            return False

    def close(self) -> None:
        """关闭设备并停止后台读帧线程。

        A5/K10：先停止 Pipeline 再 join 线程，避免 SDK 报错残留。
        """
        self._is_running = False
        # A5/K10：先 stop Pipeline，再 join 线程
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception as e:
                logger.warning("336L Pipeline.stop() 异常: %s", e)
            self._pipeline = None
        if self._read_thread is not None and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
            self._read_thread = None
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

    def _read_loop(self) -> None:
        """后台线程：持续读帧，更新 _latest_frame。

        A4/F6：使用 pipeline.wait_for_frames + get_color_frame + frame_to_bgr_image
        替代原来的 cap.read()。
        读帧失败或设备掉线时触发重连流程。
        """
        while self._is_running:
            if self._pipeline is not None:
                try:
                    # A4：wait_for_frames 超时 1000ms
                    frames = self._pipeline.wait_for_frames(1000)
                except _OBError as e:
                    logger.warning("336L wait_for_frames OBError: %s", e)
                    continue
                except Exception as e:
                    logger.warning("336L wait_for_frames 异常: %s", e)
                    continue

                if frames is None:
                    # 超时，继续下一轮
                    continue

                # A4：获取 Color 帧
                color_frame = frames.get_color_frame()
                if color_frame is None:
                    continue

                try:
                    # F6：VideoFrame → BGR numpy
                    bgr = _frame_to_bgr_image(color_frame)
                    if bgr is None:
                        continue
                    with self._frame_lock:
                        self._latest_frame = bgr
                    self._reconnect_attempts = 0
                except Exception as e:
                    logger.warning("336L 帧处理异常（跳过）: %s", e)
                    continue
            else:
                # 设备掉线
                logger.warning("336L 设备掉线，触发重连")
                self._is_opened = False
                if not self.try_reconnect():
                    self._is_running = False
                    break
