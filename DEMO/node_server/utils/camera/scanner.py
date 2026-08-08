"""本地摄像头自动扫描与兼容白名单。

对齐 Design-server.md §3.3.5。

P1/P2 改造：
- D4：双枚举策略（pyorbbecsdk SDK 优先 + OpenCV /dev/video 兜底显示非 Orbbec 设备）
- N2/N3：SDK 设备通过 identify_orbbec_device() 读取富信息 + match_compatible() 结构化匹配
- A7：CameraDeviceInfo 扩展 pid/serial/connection_type/device 字段
- A10：__init__ 接收 compatible_devices: List[CompatibleDevice] 替代 compatible_models: List[str]
"""
import logging
import os
import platform
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import cv2

from .device_identifier import (
    CompatibleDevice,
    identify_orbbec_device,
    match_compatible,
)

logger = logging.getLogger("node_server.camera")

# A6/D3：pyorbbecsdk 懒导入
_PYORBBECSDK_AVAILABLE = False
_Context = None
_OBError = None
try:
    from pyorbbecsdk import Context, OBError

    _PYORBBECSDK_AVAILABLE = True
    _Context = Context
    _OBError = OBError
except ImportError:
    logger.warning(
        "pyorbbecsdk 未安装，scanner 将退化为 OpenCV 枚举（D3：Windows 开发环境）"
    )


@dataclass
class CameraDeviceInfo:
    """扫描到的摄像头设备信息。

    A7：扩展 pid/serial/connection_type/device 字段（Optional），
    供 N3 匹配与 N4 显示使用；device 字段供 Orbbec336LInput 打开。
    """
    index: int                          # 设备索引（/dev/video* 序号 或 DirectShow 索引）
    name: str                           # 设备名称
    is_compatible: bool                 # 是否在兼容白名单内
    backend: str                        # 使用的后端名称（如 "pyorbbecsdk" / "V4L2" / "DirectShow"）
    # A7：富信息字段（SDK 枚举时填充，OpenCV 枚举时为 None）
    pid: Optional[int] = None           # USB Product ID
    serial: Optional[str] = None        # 序列号
    connection_type: Optional[str] = None  # 连接类型（如 "USB"）
    device: Any = None                  # SDK Device 对象引用（供 Orbbec336LInput 打开）


class LocalCameraScanner:
    """本地摄像头自动扫描器。

    D4：双枚举策略——
    1. 优先 _scan_orbbec_sdk()：用 pyorbbecsdk Context.query_devices() 枚举 Orbbec 设备
    2. 再跑 OpenCV /dev/video 枚举：按 name 去重，非 Orbbec 设备标"未知"不可选

    Linux 枚举 /dev/video0..N；Windows 枚举 DirectShow 索引 0..max_index。
    """

    def __init__(self, compatible_devices: Optional[List[CompatibleDevice]] = None):
        """初始化扫描器。

        A10：接收 compatible_devices: List[CompatibleDevice] 替代旧 compatible_models: List[str]。

        :param compatible_devices: 兼容设备结构化列表（来自 config/camera.json）；
                                   None 或空列表时所有设备标为不兼容
        """
        self._compatible_devices: List[CompatibleDevice] = (
            list(compatible_devices) if compatible_devices else []
        )

    def scan(self, max_index: int = 10) -> List[CameraDeviceInfo]:
        """扫描本地摄像头，返回可用设备列表。

        D4：优先 SDK 路径，OpenCV 路径作兜底显示非 Orbbec 设备。

        :param max_index: 最大扫描索引（默认 10，仅 OpenCV 兜底路径使用）
        :return: 可用设备信息列表
        """
        results: List[CameraDeviceInfo] = []

        # A8/D4：优先 SDK 枚举
        sdk_names: List[str] = []
        if _PYORBBECSDK_AVAILABLE:
            sdk_devices = self._scan_orbbec_sdk()
            results.extend(sdk_devices)
            sdk_names = [d.name.lower() for d in sdk_devices]
            logger.info("pyorbbecsdk 枚举到 %d 个 Orbbec 设备", len(sdk_devices))

        # D4：OpenCV 兜底枚举（显示非 Orbbec 设备）
        if sys.platform.startswith("linux"):
            ocv_devices = self._scan_linux(max_index)
        else:
            ocv_devices = self._scan_windows(max_index)

        # A9：按 name 去重，跳过已由 SDK 识别的设备
        for info in ocv_devices:
            name_lower: str = info.name.lower()
            is_duplicate: bool = False
            for sdk_name in sdk_names:
                if sdk_name in name_lower or name_lower in sdk_name:
                    is_duplicate = True
                    break
            if not is_duplicate:
                results.append(info)

        logger.info("本地摄像头扫描完成，发现 %d 个可用设备", len(results))
        return results

    def _scan_orbbec_sdk(self) -> List[CameraDeviceInfo]:
        """A8/F5/N2/N3：使用 pyorbbecsdk 枚举 Orbbec 设备。

        基于 demo3 viewer.py::scan_device() 的 Context.query_devices() 逻辑。
        每台设备调 identify_orbbec_device() 读取富信息 + match_compatible() 结构化匹配。

        :return: Orbbec 设备信息列表
        """
        results: List[CameraDeviceInfo] = []
        if not _PYORBBECSDK_AVAILABLE:
            return results

        try:
            ctx = _Context()
            dev_list = ctx.query_devices()
            count: int = dev_list.get_count()
            if count == 0:
                logger.info("pyorbbecsdk 未检测到 Orbbec 设备")
                return results

            for i in range(count):
                dev = dev_list[i]
                # N2：识别设备富信息
                device_info: Optional[Dict[str, Any]] = identify_orbbec_device(dev)
                if device_info is None:
                    continue

                # N3：结构化匹配
                is_compat: bool = match_compatible(device_info, self._compatible_devices)

                name: str = device_info.get("name", "未知")
                pid: Optional[int] = device_info.get("pid")
                serial: Optional[str] = device_info.get("serial")
                conn: Optional[str] = device_info.get("connection_type")

                info: CameraDeviceInfo = CameraDeviceInfo(
                    index=i,
                    name=name,
                    is_compatible=is_compat,
                    backend="pyorbbecsdk",
                    pid=pid,
                    serial=serial,
                    connection_type=conn,
                    device=dev,
                )
                results.append(info)
                self._log_scan_result(info)

        except _OBError as e:
            logger.error("pyorbbecsdk 枚举设备 OBError: %s", e)
        except Exception as e:
            logger.error("pyorbbecsdk 枚举设备异常: %s", e, exc_info=True)

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
                # OpenCV 枚举的设备无富信息，标为不兼容（N4：不可选）
                is_compat: bool = False
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
            cap: Optional[cv2.VideoCapture] = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if not cap.isOpened():
                cap.release()
                continue
            ret, _ = cap.read()
            cap.release()
            if not ret:
                continue
            name: str = f"DirectShow Camera {i}"
            is_compat: bool = False
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

    def _log_scan_result(self, info: CameraDeviceInfo) -> None:
        """记录扫描结果日志。"""
        if info.is_compatible:
            logger.info(
                "发现兼容摄像头: index=%d name='%s' backend=%s pid=%s serial=%s",
                info.index, info.name, info.backend,
                f"0x{info.pid:04X}" if info.pid else "N/A",
                info.serial or "N/A",
            )
        else:
            logger.warning(
                "发现非兼容摄像头: index=%d name='%s' backend=%s（标注不可用，不可选）",
                info.index, info.name, info.backend,
            )
