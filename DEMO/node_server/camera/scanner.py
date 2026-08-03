"""本地摄像头自动扫描与兼容白名单。

对齐 Design-server.md §3.3.5。
本地 USB 摄像头（如 Orbbec 336L）在程序启动与摄像头选择阶段需自动扫描枚举，
并按配置白名单进行兼容性标注。白名单外型号告警但不阻塞连接。
"""
import logging
import os
import sys
from dataclasses import dataclass
from typing import List, Optional

import cv2

logger = logging.getLogger("node_server.camera")


@dataclass
class CameraDeviceInfo:
    """扫描到的摄像头设备信息。"""
    index: int          # 设备索引（/dev/video* 序号 或 DirectShow 索引）
    name: str           # 设备名称
    is_compatible: bool # 是否在兼容白名单内
    backend: str        # 使用的后端名称（如 "DirectShow" / "V4L2"）


class LocalCameraScanner:
    """本地摄像头自动扫描器。

    Linux 枚举 /dev/video0..N；Windows 枚举 DirectShow 索引 0..max_index。
    对每个可打开设备读取一帧验证可用性，并按白名单标注兼容性。
    """

    def __init__(self, compatible_models: Optional[List[str]] = None):
        """初始化扫描器。

        :param compatible_models: 兼容型号白名单（来自 config/camera.json）；
                                  None 时使用默认白名单
        """
        if compatible_models is None:
            self._compatible_models: List[str] = ["Orbbec Gemini 336L", "Orbbec Astra"]
        else:
            self._compatible_models = list(compatible_models)

    def scan(self, max_index: int = 10) -> List[CameraDeviceInfo]:
        """扫描本地摄像头，返回可用设备列表。

        :param max_index: 最大扫描索引（默认 10）
        :return: 可用设备信息列表（每个设备已通过读帧验证）
        """
        results: List[CameraDeviceInfo] = []
        if sys.platform.startswith("linux"):
            results = self._scan_linux(max_index)
        else:
            # Windows 及其他平台用 DirectShow
            results = self._scan_windows(max_index)
        logger.info("本地摄像头扫描完成，发现 %d 个可用设备", len(results))
        return results

    def _scan_linux(self, max_index: int) -> List[CameraDeviceInfo]:
        """Linux 平台扫描：枚举 /dev/video0..N。"""
        results: List[CameraDeviceInfo] = []
        for i in range(max_index + 1):
            dev_path: str = f"/dev/video{i}"
            if not os.path.exists(dev_path):
                continue
            name: str = self._read_linux_device_name(i)
            if self._probe_device(i, None, name):
                is_compat: bool = self._check_compatible(name)
                info: CameraDeviceInfo = CameraDeviceInfo(
                    index=i,
                    name=name,
                    is_compatible=is_compat,
                    backend="V4L2",
                )
                results.append(info)
                self._log_scan_result(info)
        return results

    def _scan_windows(self, max_index: int) -> List[CameraDeviceInfo]:
        """Windows 平台扫描：用 DirectShow 后端尝试打开索引 0..max_index。"""
        results: List[CameraDeviceInfo] = []
        for i in range(max_index + 1):
            # 先尝试用 DirectShow 打开
            cap: Optional[cv2.VideoCapture] = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if not cap.isOpened():
                cap.release()
                continue
            # 读一帧验证可用性
            ret, _ = cap.read()
            cap.release()
            if not ret:
                continue
            # Windows 下 OpenCV 难以获取设备友好名，用索引占位
            name: str = f"DirectShow Camera {i}"
            is_compat: bool = self._check_compatible(name)
            info: CameraDeviceInfo = CameraDeviceInfo(
                index=i,
                name=name,
                is_compatible=is_compat,
                backend="DirectShow",
            )
            results.append(info)
            self._log_scan_result(info)
        return results

    @staticmethod
    def _read_linux_device_name(index: int) -> str:
        """读取 Linux 设备名：/sys/class/video4linux/videoN/name。"""
        name_path: str = f"/sys/class/video4linux/video{index}/name"
        try:
            with open(name_path, "r", encoding="utf-8") as f:
                return f.read().strip()
        except (OSError, IOError):
            return f"/dev/video{index}"

    @staticmethod
    def _probe_device(index: int, backend: Optional[int], name: str) -> bool:
        """打开设备并读一帧验证可用性。

        :return: 可打开且能读帧返回 True
        """
        try:
            if backend is not None:
                cap: cv2.VideoCapture = cv2.VideoCapture(index, backend)
            else:
                cap = cv2.VideoCapture(index)
            if not cap.isOpened():
                cap.release()
                return False
            ret, _ = cap.read()
            cap.release()
            return ret
        except Exception:
            return False

    def _check_compatible(self, name: str) -> bool:
        """检查设备名是否在兼容白名单内（模糊匹配）。"""
        for model in self._compatible_models:
            if model.lower() in name.lower():
                return True
        return False

    def _log_scan_result(self, info: CameraDeviceInfo) -> None:
        """记录扫描结果日志：白名单外型号 WARNING 但不阻塞。"""
        if info.is_compatible:
            logger.info("发现兼容摄像头: index=%d name='%s' backend=%s",
                        info.index, info.name, info.backend)
        else:
            logger.warning("发现非白名单摄像头: index=%d name='%s' backend=%s（标注未验证，不阻塞）",
                           info.index, info.name, info.backend)
