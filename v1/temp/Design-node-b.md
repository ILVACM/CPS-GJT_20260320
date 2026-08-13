# 节点 B 详细设计方案 — 系统主控节点（OrangePi KunPeng）

> **文档版本**：v1.0
> **创建日期**：2026-08-28
> **关联文档**：`AGENTS.md`（项目总览与设计约定）、`Design-node-a.md`（节点 A 设计方案，待配套产出）
> **适用范围**：节点 B 原型阶段的工程实现，覆盖模块划分、类接口、并发模型、自检流程、日志与异常策略

---

## 目录

- [1. 节点职责与运行环境](#1-节点职责与运行环境)
- [2. 工程目录结构](#2-工程目录结构)
- [3. 核心模块与类设计](#3-核心模块与类设计)
- [4. 并发与线程模型](#4-并发与线程模型)
- [5. 配置加载与启动自检流程](#5-配置加载与启动自检流程)
- [6. 日志与异常处理策略](#6-日志与异常处理策略)

---

## 1. 节点职责与运行环境

### 1.1 节点角色定位

节点 B 是系统的**主控与交互枢纽**，承担以下四项核心职责：

| 编号 | 职责 | 输入 | 输出 |
|------|------|------|------|
| R1 | 深度相机 RGB 帧采集 | Orbbec 336L USB 视频流 | JPEG 编码帧（供推理） |
| R2 | 激光测距数据采集 | S21C 主控串口 11 字节帧 | 4 路 uint16 距离值（mm） |
| R3 | 推理任务调度与帧同步 | RGB 帧 + 距离标量 + 用户触发 | gRPC 请求；InferResponse 解析 |
| R4 | 测量换算 + 画面渲染 + 用户交互 | 掩码 + 距离 + 内参 | 标注画面、直径/间距数据、交互反馈 |

### 1.2 运行环境约束

| 维度 | 说明 |
|------|------|
| 硬件 | HUAWEI OrangePi KunPeng（ARM Cortex-A 系列，无 GPU） |
| 操作系统 | openEuler 22 LTS |
| Python | 3.9+（PEP 8，Type Hints） |
| 关键依赖 | `opencv-python`、`pyserial`、`Pillow`、`grpcio`、`protobuf`、`numpy` |
| 显示 | 本地 GUI（tkinter）；需 `DISPLAY` 环境变量或 XWayland |
| 权限 | 摄像头需 `video` 组；串口需 `dialout` 组 |
| 网络 | 静态 IP（如 `192.168.10.1`），经网线直连节点 A |

### 1.3 平台兼容声明

- **生产环境**：openEuler 22 LTS，所有功能完整启用
- **开发调试环境**：Windows 10/11，仅负责代码开发与调试
  - 摄像头后端使用 DirectShow（`cv2.CAP_DSHOW`）
  - 串口使用 CH9102 驱动虚拟串口
  - 环境自检中 IP 校验 bypass 或仅warning
  - 节点 A 不可达时进入降级模式而非退出

### 1.4 触发模式约束

推理触发为**手动按需触发**：用户在 UI 点击"测量"后执行单帧推理；相邻两次推理请求间隔 **不少于 10 秒**。该约束在 `ui` 模块中通过时间戳门控。

---

## 2. 工程目录结构

```
node_b/
├── main.py                          # 程序入口
├── config/
│   ├── camera.json                  # 相机参数配置
│   ├── intrinsics.json              # 相机内参配置
│   └── network.json                 # 网络参数配置
├── proto/
│   ├── rebar_inference.proto        # gRPC Proto 定义
│   ├── rebar_inference_pb2.py       # protoc 生成
│   └── rebar_inference_pb2_grpc.py  # protoc 生成
├── common/
│   ├── __init__.py
│   ├── constants.py                 # 共享常量集中定义
│   └── config_loader.py             # 配置加载器
├── camera/
│   ├── __init__.py
│   ├── base.py                      # BaseCameraInput 抽象基类
│   ├── orbbec_336l.py               # Orbbec336LInput 实现
│   └── mock.py                      # MockCameraInput（调试用）
├── serial/
│   ├── __init__.py
│   ├── serial_buffer.py             # SerialBuffer
│   ├── laser_parser.py              # LaserDataParser + 有效性检查
│   ├── laser_fusion.py              # 4路融合策略
│   └── serial_manager.py            # SerialManager
├── grpc_client/
│   ├── __init__.py
│   └── inference_client.py          # InferenceGrpcClient
├── measure/
│   ├── __init__.py
│   ├── rebar_measure.py             # RebarMeasure 类（源自 RebarMeasureV53）
│   └── intrinsics_manager.py        # 内参管理
├── ui/
│   ├── __init__.py
│   ├── app.py                       # RebarMeasureApp（主应用）
│   ├── cli_fusion_menu.py           # 激光融合策略 CLI 菜单
│   └── render.py                    # 画面渲染与叠加
├── system/
│   ├── __init__.py
│   ├── self_check.py                # 启动环境自检
│   ├── health_check.py              # 运行时健康检查
│   └── bootstrap.py                 # 启动引导
├── logs/
│   └── node_b.log                   # 节点 B 运行日志
├── requirements.txt                 # 节点 B 专属依赖
└── test_node_b.py                   # 诊断脚本
```

**设计理由**：
- 按 §3.1 模块划分对齐目录
- `proto/` 存放 protobuf 定义与生成代码，**双节点禁止各自硬编码副本**（节点 A 同样引用此目录或独立构建，但 proto 源文件唯一）
- `common/` 存放双节点共享的常量与配置加载逻辑，可被节点 A 引用（如 proto 定义、分割类别常量）
- `requirements.txt` 按节点拆分，避免在 OrangePi 上安装 torch 等推理依赖

---

## 3. 核心模块与类设计

### 3.1 共享常量（`common/constants.py`）

```python
"""
共享常量集中定义，双节点禁止各自硬编码副本。
"""

# ---- 串口帧协议 ----
SERIAL_FRAME_HEADER: int = 0x7B
SERIAL_FRAME_TAIL: int = 0x7D
SERIAL_FRAME_LENGTH: int = 11
SERIAL_BAUDRATE: int = 115200
SERIAL_CHANNELS: int = 4

# ---- 激光测距有效性范围（STP-23L Datasheet §4.1） ----
LASER_MIN_MM: int = 70
LASER_MAX_MM: int = 7500

# ---- 分割类别定义 ----
class SegClass:
    BACKGROUND: int = 0
    REBAR_VERTICAL: int = 1   # 纵向钢筋
    REBAR_HORIZONTAL: int = 2 # 横向钢筋

# ---- 国标规格表（mm） ----
STANDARD_DIAMETERS = [6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40]

# ---- 推理触发间隔 ----
MIN_INFER_INTERVAL_SEC: float = 10.0

# ---- 内参默认值 ----
DEFAULT_FX: float = 950.0
DEFAULT_FY: float = 950.0
```

### 3.2 配置加载器（`common/config_loader.py`）

```python
"""
从 JSON 配置文件读取所有值类参数，严禁硬编码。
"""

import json
import os
from dataclasses import dataclass, field
from typing import Optional

@dataclass
class CameraConfig:
    rgb_width: int = 1920
    rgb_height: int = 1080
    rgb_fps: int = 30
    rgb_format: str = "MJPG"
    depth_width: int = 640
    depth_height: int = 480
    depth_fps: int = 30
    device_index: int = 0  # /dev/video* 或 Windows DirectShow 索引

@dataclass
class IntrinsicsConfig:
    fx: float = 950.0
    fy: float = 950.0
    cx: float = 960.0
    cy: float = 540.0

@dataclass
class NetworkConfig:
    local_ip: str = "192.168.10.1"
    remote_ip: str = "192.168.10.2"
    grpc_port: int = 50051
    grpc_deadline_ms: int = 5000  # gRPC 单次调用 deadline
    health_check_interval_s: float = 5.0

class ConfigLoader:
    """读取并解析 JSON 配置文件。"""

    @staticmethod
    def load_camera_config(path: str) -> CameraConfig:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return CameraConfig(**data)

    @staticmethod
    def load_intrinsics_config(path: str) -> IntrinsicsConfig:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return IntrinsicsConfig(**data)

    @staticmethod
    def load_network_config(path: str) -> NetworkConfig:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return NetworkConfig(**data)
```

**配置文件示例**：

`config/camera.json`:
```json
{
  "rgb_width": 1920,
  "rgb_height": 1080,
  "rgb_fps": 30,
  "rgb_format": "MJPG",
  "depth_width": 640,
  "depth_height": 480,
  "depth_fps": 30,
  "device_index": 0
}
```

`config/intrinsics.json`:
```json
{
  "fx": 950.0,
  "fy": 950.0,
  "cx": 960.0,
  "cy": 540.0
}
```

`config/network.json`:
```json
{
  "local_ip": "192.168.10.1",
  "remote_ip": "192.168.10.2",
  "grpc_port": 50051,
  "grpc_deadline_ms": 5000,
  "health_check_interval_s": 5.0
}
```

### 3.3 输入模块（`camera/`）

#### 3.3.1 抽象基类

```python
"""camera/base.py"""

from abc import ABC, abstractmethod
from typing import Optional, Dict
import numpy as np


class BaseCameraInput(ABC):
    """
    相机输入抽象基类。
    所有具体实现（Orbbec 336L、Mock 等）必须继承此类。
    """

    @abstractmethod
    def open(self) -> bool:
        """打开设备，返回是否成功。"""
        ...

    @abstractmethod
    def close(self) -> None:
        """关闭设备，释放资源。"""
        ...

    @abstractmethod
    def is_opened(self) -> bool:
        """设备是否已就绪。"""
        ...

    @abstractmethod
    def get_rgb_frame(self) -> Optional[np.ndarray]:
        """
        获取一帧 RGB 图像。
        返回: BGR 格式 numpy 数组 (H, W, 3)，失败返回 None。
        """
        ...

    @abstractmethod
    def get_depth_frame(self) -> Optional[np.ndarray]:
        """
        获取一帧 Depth 深度图。
        原型阶段可不实现，返回 None。
        """
        ...

    @abstractmethod
    def get_intrinsics(self) -> Dict[str, float]:
        """
        获取相机内参。
        返回: {"fx": float, "fy": float, "cx": float, "cy": float}
        """
        ...
```

#### 3.3.2 Orbbec 336L 实现

```python
"""camera/orbbec_336l.py"""

import threading
import time
from typing import Optional, Dict

import cv2
import numpy as np

from .base import BaseCameraInput
from common.config_loader import CameraConfig, IntrinsicsConfig


class Orbbec336LInput(BaseCameraInput):
    """
    Orbbec Gemini 336L 相机实现。
    后台线程持续读帧，取最新帧策略；断线自动重连。
    """

    def __init__(self, camera_cfg: CameraConfig, intrinsics_cfg: IntrinsicsConfig):
        self._camera_cfg = camera_cfg
        self._intrinsics_cfg = intrinsics_cfg
        self._cap: Optional[cv2.VideoCapture] = None
        self._is_opened: bool = False
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        self._read_thread: Optional[threading.Thread] = None
        self._is_running: bool = False
        self._reconnect_interval: float = 3.0
        self._max_reconnect_attempts: int = 5
        self._reconnect_attempts: int = 0

    def open(self) -> bool:
        """打开设备并启动后台读帧线程。"""
        # 伪代码：
        # 根据 platform 选择后端: Windows -> CAP_DSHOW; Linux -> CAP_V4L2
        # self._cap = cv2.VideoCapture(self._camera_cfg.device_index, backend)
        # self._cap.set(CAP_PROP_FRAME_WIDTH, cfg.rgb_width)
        # self._cap.set(CAP_PROP_FRAME_HEIGHT, cfg.rgb_height)
        # self._cap.set(CAP_PROP_FPS, cfg.rgb_fps)
        # self._cap.set(CAP_PROP_BUFFERSIZE, 1)
        # if not self._cap.isOpened(): return False
        # self._is_opened = True; self._is_running = True
        # self._read_thread = Thread(target=self._read_loop, daemon=True)
        # self._read_thread.start()
        return True

    def close(self) -> None:
        """关闭设备并停止线程。"""
        # self._is_running = False
        # if self._read_thread: self._read_thread.join(timeout=2.0)
        # if self._cap: self._cap.release()
        # self._is_opened = False
        ...

    def is_opened(self) -> bool:
        return self._is_opened

    def get_rgb_frame(self) -> Optional[np.ndarray]:
        """获取最新帧的拷贝（线程安全）。"""
        # with self._frame_lock:
        #     return self._latest_frame.copy() if self._latest_frame is not None else None
        return None

    def get_depth_frame(self) -> Optional[np.ndarray]:
        """原型阶段不实现，返回 None。"""
        return None

    def get_intrinsics(self) -> Dict[str, float]:
        """从配置返回内参。"""
        # return {"fx": self._intrinsics_cfg.fx, "fy": self._intrinsics_cfg.fy,
        #         "cx": self._intrinsics_cfg.cx, "cy": self._intrinsics_cfg.cy}
        return {}

    def try_reconnect(self) -> bool:
        """断线重连。"""
        # if self._reconnect_attempts >= self._max_reconnect_attempts: return False
        # self._reconnect_attempts += 1
        # time.sleep(self._reconnect_interval)
        # return self.open()
        return False

    def _read_loop(self):
        """后台线程：持续读取帧，更新 _latest_frame。"""
        # while self._is_running:
        #     if self._cap and self._cap.isOpened():
        #         ret, frame = self._cap.read()
        #         if ret:
        #             with self._frame_lock:
        #                 self._latest_frame = frame
        #             self._reconnect_attempts = 0
        #         time.sleep(0.001)
        #     else:
        #         self._is_opened = False
        #         break
        ...
```

#### 3.3.3 Mock 相机（调试用）

```python
"""camera/mock.py"""

import numpy as np
from typing import Optional, Dict
from .base import BaseCameraInput


class MockCameraInput(BaseCameraInput):
    """模拟相机，返回纯黑测试帧，节点 B 独立联调用。"""

    def __init__(self, width: int = 1920, height: int = 1080):
        self._width = width
        self._height = height
        self._is_opened = False

    def open(self) -> bool:
        self._is_opened = True
        return True

    def close(self) -> None:
        self._is_opened = False

    def is_opened(self) -> bool:
        return self._is_opened

    def get_rgb_frame(self) -> Optional[np.ndarray]:
        return np.zeros((self._height, self._width, 3), dtype=np.uint8)

    def get_depth_frame(self) -> Optional[np.ndarray]:
        return None

    def get_intrinsics(self) -> Dict[str, float]:
        return {"fx": 950.0, "fy": 950.0, "cx": self._width / 2, "cy": self._height / 2}
```

### 3.4 串口模块（`serial/`）

#### 3.4.1 串口缓冲区（源自现有 `SerialBuffer`）

```python
"""serial/serial_buffer.py"""

import threading
from typing import Optional

from common.constants import (
    SERIAL_FRAME_HEADER, SERIAL_FRAME_TAIL, SERIAL_FRAME_LENGTH
)


class SerialBuffer:
    """
    串口数据缓冲区，处理粘包和断包问题。
    与现有 demo.py 实现完全一致。
    """

    def __init__(self):
        self._buffer = bytearray()
        self._lock = threading.Lock()

    def append(self, data: bytes) -> None:
        """追加原始字节到缓冲区。"""
        with self._lock:
            self._buffer.extend(data)

    def find_frame(self) -> Optional[bytes]:
        """
        从缓冲区中提取一个完整且校验通过的帧（11 字节）。
        返回: 完整帧 bytes，或 None（数据不足 / 无帧）。
        """
        with self._lock:
            while len(self._buffer) >= SERIAL_FRAME_LENGTH:
                # 寻找帧头
                header_index = -1
                for i in range(len(self._buffer) - SERIAL_FRAME_LENGTH + 1):
                    if self._buffer[i] == SERIAL_FRAME_HEADER:
                        header_index = i
                        break

                if header_index == -1:
                    self._buffer.clear()
                    return None

                if header_index > 0:
                    del self._buffer[:header_index]

                if len(self._buffer) < SERIAL_FRAME_LENGTH:
                    return None

                candidate = bytes(self._buffer[:SERIAL_FRAME_LENGTH])

                if candidate[-1] != SERIAL_FRAME_TAIL:
                    del self._buffer[0]
                    continue

                if self._verify_checksum(candidate):
                    del self._buffer[:SERIAL_FRAME_LENGTH]
                    return candidate
                else:
                    del self._buffer[0]

            return None

    @staticmethod
    def _verify_checksum(frame: bytes) -> bool:
        """XOR 校验：前 9 字节异或和等于第 10 字节。"""
        xor_sum = 0
        for i in range(9):
            xor_sum ^= frame[i]
        return xor_sum == frame[9]
```

#### 3.4.2 激光数据解析器 + 有效性检查（源自现有 `LaserDataParser`）

```python
"""serial/laser_parser.py"""

import struct
from dataclasses import dataclass
from typing import List, Tuple

from common.constants import (
    LASER_MIN_MM, LASER_MAX_MM
)


@dataclass
class LaserParseResult:
    """单帧激光数据解析结果。"""
    raw_values: Tuple[int, int, int, int]   # 4 路原始值
    valid_values: List[int]                  # 通过有效性检查的值
    valid_count: int                         # 有效路数


class LaserDataParser:
    """激光测距数据解析器，含有效性检查。"""

    @staticmethod
    def parse(frame: bytes) -> LaserParseResult:
        """
        解析 11 字节帧：4 路大端序 uint16。
        返回 LaserParseResult（包含原始值与有效值列表）。
        """
        # 伪代码：
        # d1 = struct.unpack('>H', frame[1:3])[0]
        # d2 = struct.unpack('>H', frame[3:5])[0]
        # d3 = struct.unpack('>H', frame[5:7])[0]
        # d4 = struct.unpack('>H', frame[7:9])[0]
        # raw = (d1, d2, d3, d4)
        # valid = [v for v in raw if _is_valid(v)]
        # return LaserParseResult(raw, valid, len(valid))
        return LaserParseResult((0, 0, 0, 0), [], 0)

    @staticmethod
    def _is_valid(value: int) -> bool:
        """单值有效性检查。"""
        # if value == 0: return False          # 零值剔除
        # if value < LASER_MIN_MM: return False
        # if value > LASER_MAX_MM: return False
        # return True
        return False
```

#### 3.4.3 4 路融合策略

```python
"""serial/laser_fusion.py"""

import statistics
from enum import Enum
from typing import Optional

from serial.laser_parser import LaserParseResult
from common.constants import SERIAL_CHANNELS


class FusionStrategy(Enum):
    """激光融合策略枚举。"""
    MEAN = 1       # 取均值（默认）
    MEDIAN = 2     # 取中位数
    SPECIFIC = 3   # 取特定一路
    MANUAL = 4     # 手动输入


class LaserFusion:
    """
    4 路激光测距值融合为 1 个标量距离。
    支持运行时策略切换。
    """

    def __init__(self):
        self._strategy: FusionStrategy = FusionStrategy.MEAN
        self._specific_index: int = 0     # 3 路选择时：0~3
        self._manual_value: Optional[float] = None

    def set_strategy(self, strategy: FusionStrategy,
                     specific_index: int = 0,
                     manual_value: Optional[float] = None) -> None:
        """运行时切换策略。"""
        self._strategy = strategy
        self._specific_index = specific_index
        self._manual_value = manual_value

    def get_strategy(self) -> FusionStrategy:
        return self._strategy

    def fuse(self, parse_result: LaserParseResult) -> Optional[float]:
        """
        根据当前策略融合 4 路距离值为 1 个标量。
        返回: 融合结果（mm），全无效返回 None。
        """
        # 伪代码：
        # if self._strategy == FusionStrategy.MANUAL:
        #     return self._manual_value
        #
        # valid = parse_result.valid_values
        # if len(valid) == 0:
        #     return None
        #
        # if self._strategy == FusionStrategy.MEAN:
        #     return sum(valid) / len(valid)
        # elif self._strategy == FusionStrategy.MEDIAN:
        #     return statistics.median(valid)
        # elif self._strategy == FusionStrategy.SPECIFIC:
        #     idx = self._specific_index
        #     raw = parse_result.raw_values
        #     if 0 <= idx < SERIAL_CHANNELS and LaserDataParser._is_valid(raw[idx]):
        #         return float(raw[idx])
        #     return None  # 所选路无效
        return None
```

#### 3.4.4 串口管理器（重构现有 `SerialManager`）

```python
"""serial/serial_manager.py"""

import time
import threading
from typing import Optional, Tuple

import serial
import serial.tools.list_ports

from serial.serial_buffer import SerialBuffer
from serial.laser_parser import LaserDataParser, LaserParseResult
from common.constants import SERIAL_BAUDRATE


class SerialManager:
    """
    串口设备管理类。
    封装端口打开、后台读取、帧解析、数据获取与重连。
    """

    def __init__(self, baudrate: int = SERIAL_BAUDRATE, timeout: float = 0.1):
        self._baudrate = baudrate
        self._timeout = timeout
        self._serial_conn: Optional[serial.Serial] = None
        self._port_name: str = ""
        self._is_running: bool = False
        self._buffer = SerialBuffer()
        self._data_lock = threading.Lock()
        self._latest_result: Optional[LaserParseResult] = None
        self._data_timestamp: float = 0.0
        self._read_thread: Optional[threading.Thread] = None
        self._reconnect_attempts: int = 0
        self._max_reconnect_attempts: int = 5
        self._reconnect_interval: float = 3.0

    def open_port(self, port_name: str) -> Tuple[bool, str]:
        """打开指定串口并启动后台读取线程。"""
        # 伪代码：
        # self._serial_conn = serial.Serial(port=port_name, baudrate=self._baudrate, timeout=self._timeout)
        # self._port_name = port_name
        # self._is_running = True
        # self._buffer = SerialBuffer()
        # self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        # self._read_thread.start()
        # return True, f"成功打开串口 {port_name}"
        return False, "not implemented"

    def close(self) -> None:
        """关闭串口并停止线程。"""
        # self._is_running = False
        # if self._read_thread: self._read_thread.join(timeout=1.0)
        # if self._serial_conn and self._serial_conn.is_open:
        #     self._serial_conn.close()
        ...

    def is_running(self) -> bool:
        return self._is_running

    def get_port_name(self) -> str:
        return self._port_name

    def get_latest(self) -> Tuple[Optional[LaserParseResult], float]:
        """获取最新解析结果与时间戳（线程安全拷贝）。"""
        # with self._data_lock:
        #     return self._latest_result, self._data_timestamp
        return None, 0.0

    def try_reconnect(self) -> bool:
        """断线重连。"""
        # if self._reconnect_attempts >= self._max_reconnect_attempts: return False
        # self._reconnect_attempts += 1
        # time.sleep(self._reconnect_interval)
        # ok, _ = self.open_port(self._port_name)
        # return ok
        return False

    def _read_loop(self):
        """后台线程：读取串口 → 缓冲区 → 帧解析 → 更新结果。"""
        # while self._is_running and self._serial_conn:
        #     try:
        #         if self._serial_conn.in_waiting > 0:
        #             data = self._serial_conn.read(self._serial_conn.in_waiting)
        #             self._buffer.append(data)
        #             while True:
        #                 frame = self._buffer.find_frame()
        #                 if frame is None: break
        #                 result = LaserDataParser.parse(frame)
        #                 with self._data_lock:
        #                     self._latest_result = result
        #                     self._data_timestamp = time.time()
        #     except (serial.SerialException, OSError):
        #         self._is_running = False
        #         break
        #     time.sleep(0.001)
        ...
```

### 3.5 gRPC 客户端模块（`grpc_client/inference_client.py`）

```python
"""grpc_client/inference_client.py"""

import time
from typing import Optional

import grpc
import numpy as np

# protoc 生成的桩代码
from proto import rebar_inference_pb2
from proto import rebar_inference_pb2_grpc
from common.config_loader import NetworkConfig


class InferenceResult:
    """推理响应解析结果。"""
    def __init__(self, mask: np.ndarray, frame_id: int, timestamp_ms: int):
        self.mask = mask                  # (H, W) uint8 类别掩码
        self.frame_id: int = frame_id
        self.timestamp_ms: int = timestamp_ms


class InferenceGrpcClient:
    """
    节点 A 推理服务 gRPC 客户端。
    封装 JPEG 编码 → gRPC 请求 → PNG 解码的全链路。
    """

    def __init__(self, net_cfg: NetworkConfig):
        self._target = f"{net_cfg.remote_ip}:{net_cfg.grpc_port}"
        self._deadline_ms = net_cfg.grpc_deadline_ms
        self._channel: Optional[grpc.Channel] = None
        self._stub: Optional[rebar_inference_pb2_grpc.RebarInferenceStub] = None
        self._connected: bool = False

    def connect(self) -> bool:
        """建立 gRPC 信道。"""
        # 伪代码：
        # self._channel = grpc.insecure_channel(self._target)
        # self._stub = rebar_inference_pb2_grpc.RebarInferenceStub(self._channel)
        # self._connected = True
        # return True
        return False

    def is_connected(self) -> bool:
        return self._connected

    def infer(
        self,
        rgb_frame: np.ndarray,
        frame_id: int,
        camera_distance_mm: float,
    ) -> Optional[InferenceResult]:
        """
        发送单帧推理请求。
        参数: rgb_frame=BGR numpy 数组; frame_id=帧序号; distance=激光距离(mm)
        返回: InferenceResult 或 None（超时/失败）

        伪代码：
        1. jpeg_bytes = cv2.imencode('.jpg', rgb_frame).tobytes()
        2. ts_ms = int(time.time() * 1000)
        3. request = InferRequest(image=jpeg_bytes, frame_id=frame_id,
                                   timestamp_ms=ts_ms,
                                   camera_distance_mm=camera_distance_mm)
        4. try:
               response = self._stub.Infer(request, deadline=self._deadline_ms)
           except grpc.RpcError:
               return None
        5. mask = cv2.imdecode(np.frombuffer(response.label_mask, np.uint8), cv2.IMREAD_GRAYSCALE)
        6. return InferenceResult(mask, response.frame_id, response.timestamp_ms)
        """
        return None

    def probe(self, timeout_ms: int = 2000) -> bool:
        """
        探测节点 A 存活性（用于启动自检与健康检查）。
        返回: True 可达 / False 不可达
        """
        # 伪代码：
        # try:
        #     channel = grpc.insecure_channel(self._target)
        #     grpc.channel_ready_future(channel).result(timeout=timeout_ms / 1000)
        #     return True
        # except grpc.FutureTimeoutError:
        #     return False
        return False

    def close(self) -> None:
        """关闭信道。"""
        # if self._channel: self._channel.close()
        ...
```

### 3.6 测量模块（`measure/`）

#### 3.6.1 内参管理器

```python
"""measure/intrinsics_manager.py"""

from typing import Dict
from common.config_loader import IntrinsicsConfig


class IntrinsicsManager:
    """
    管理相机内参，提供 mm/px 换算所需参数。
    所有内参值从配置文件读取，禁止硬编码。
    """

    def __init__(self, cfg: IntrinsicsConfig):
        self._fx = cfg.fx
        self._fy = cfg.fy
        self._cx = cfg.cx
        self._cy = cfg.cy

    def get_fx(self) -> float:
        return self._fx

    def get_fy(self) -> float:
        return self._fy

    def get_params(self) -> Dict[str, float]:
        return {"fx": self._fx, "fy": self._fy, "cx": self._cx, "cy": self._cy}

    def calc_mm_per_pixel(self, distance_mm: float) -> float:
        """
        根据工作距离与焦距计算 mm/px。
        公式: mm_per_pixel = distance_mm / fx
        """
        # return distance_mm / self._fx
        return 0.0
```

#### 3.6.2 钢筋测量类（源自 `RebarMeasureV53`）

```python
"""measure/rebar_measure.py"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from common.constants import (
    STANDARD_DIAMETERS, SegClass,
    DEFAULT_FX, DEFAULT_FY,
)

# 从 new-predict.py 迁移时的函数别名
_safe_to_bgr = None


@dataclass
class RebarInfo:
    """单根钢筋测量结果。"""
    center_x: float
    center_y: float
    diameter_mm: float
    standard_diameter_mm: float   # 就近对齐国标后的直径
    width_px: float


@dataclass
class MeasureResult:
    """单次测量完整结果。"""
    rebars: List[RebarInfo]               # 钢筋列表
    spacings_mm: List[float]              # 相邻钢筋间距
    mm_per_pixel: float                   # 像素换算系数
    distance_mm: float                    # 所用工作距离
    annotated_frame: Optional[np.ndarray] # 可视化结果帧


class RebarMeasure:
    """
    钢筋直径与间距测量类。
    源自 RebarMeasureV53，适配新架构的接口形态。
    """

    def __init__(
        self,
        fx: float = DEFAULT_FX,
        fy: float = DEFAULT_FY,
        cx: float = 960.0,
        cy: float = 540.0,
        min_component_area: int = 120,
        min_height_ratio: float = 0.12,
        merge_x_ratio: float = 0.012,
        min_diameter_mm: float = 5.0,
        max_diameter_mm: float = 45.0,
        scanline_ratio: float = 0.50,
    ):
        self._fx = fx
        self._fy = fy
        self._cx = cx
        self._cy = cy
        self._min_component_area = min_component_area
        self._min_height_ratio = min_height_ratio
        self._merge_x_ratio = merge_x_ratio
        self._min_diameter_mm = min_diameter_mm
        self._max_diameter_mm = max_diameter_mm
        self._scanline_ratio = scanline_ratio
        self._standard_diameters = np.array(STANDARD_DIAMETERS, dtype=np.float32)

    def measure(
        self,
        mask: np.ndarray,
        distance_mm: float,
        rgb_frame: Optional[np.ndarray] = None,
    ) -> Optional[MeasureResult]:
        """
        执行一次测量。
        参数:
            mask: 单通道 uint8 类别掩码 (0/1/2)
            distance_mm: 工作距离（mm）
            rgb_frame: 可选，用于生成可视化结果
        返回: MeasureResult 或 None（无有效钢筋）

        伪代码：
        1. mm_per_pixel = distance_mm / self._fx
        2. 从 mask 提取类别 1（纵向钢筋）的子掩码
        3. _find_raw_rebars: 连通域 + 几何筛选
        4. _merge_collinear_segments: 合并分段
        5. 逐根 _scan_edges_for_one_rebar: 扫描边缘估算宽度(px)
        6. 宽度(px) × mm_per_pixel → 直径(mm)
        7. _normalize_rebar_diameter: 就近对齐国标规格
        8. 按 center_x 排序，计算相邻间距
        9. 若提供 rgb_frame → _draw_results: 叠加标注
        10. 返回 MeasureResult
        """
        return None

    def _find_raw_rebars(
        self, mask: np.ndarray, mm_per_pixel: float
    ) -> List[dict]:
        """连通域提取与几何筛选。"""
        # 伪代码：轮廓查找 → 面积筛选 → 高宽比筛选 → minAreaRect
        return []

    def _merge_collinear_segments(
        self, raw_rebars: List[dict], img_w: int
    ) -> List[dict]:
        """合并共线分段。"""
        # 按 cx 排序 → x_tol 容差合并
        return []

    def _scan_edges_for_one_rebar(
        self, mask: np.ndarray, bbox: Tuple[int, int, int, int]
    ) -> Optional[dict]:
        """单根钢筋多行扫描估算稳定边缘与宽度。"""
        # 水平条带扫描 → 中线宽度 → 过滤异常行 → 取中位数
        return None

    def _normalize_rebar_diameter(self, d_mm: float) -> float:
        """就近对齐国标规格。"""
        # idx = np.argmin(np.abs(self._standard_diameters - d_mm))
        # return float(self._standard_diameters[idx])
        return 0.0

    def _draw_results(
        self, frame: np.ndarray, result: MeasureResult
    ) -> np.ndarray:
        """在帧上叠加测量标注（掩罩 + 直径 + 间距）。"""
        # 伪代码：
        # overlay = 纵向钢筋区域半透明叠加（红色）
        # 每根钢筋：绿色边框 + 标注直径文本（白色）
        # 相邻钢筋：灰色虚线连接 + 标注间距文本（青色）
        return frame
```

### 3.7 UI 模块（`ui/`）

#### 3.7.1 CLI 融合策略菜单

```python
"""ui/cli_fusion_menu.py"""

import sys
from typing import Callable, Optional

from serial.laser_fusion import LaserFusion, FusionStrategy


class CliFusionMenu:
    """
    命令行终端交互式融合策略菜单。
    使用标准 input() / print()，零额外依赖。
    """

    def __init__(
        self,
        fusion: LaserFusion,
        on_change: Optional[Callable[[FusionStrategy], None]] = None,
    ):
        self._fusion = fusion
        self._on_change = on_change

    def show(self) -> None:
        """
        显示菜单并获取用户选择。
        在 CLI 线程中循环运行。
        """
        # 伪代码：
        # while True:
        #     print("\n=== 激光融合策略 ===")
        #     print("1. 取均值（默认）")
        #     print("2. 取中位数")
        #     print("3. 取特定一路（一~四）")
        #     print("4. 手动输入指定值（mm）")
        #     choice = input("请选择 [1-4]: ").strip()
        #     if choice == "1": self._fusion.set_strategy(FusionStrategy.MEAN)
        #     elif choice == "2": self._fusion.set_strategy(FusionStrategy.MEDIAN)
        #     elif choice == "3":
        #         idx = int(input("请输入接口编号（1-4）: ")) - 1
        #         self._fusion.set_strategy(FusionStrategy.SPECIFIC, specific_index=idx)
        #     elif choice == "4":
        #         val = float(input("请输入距离值（mm）: "))
        #         self._fusion.set_strategy(FusionStrategy.MANUAL, manual_value=val)
        #     if self._on_change: self._on_change(self._fusion.get_strategy())
        ...
```

#### 3.7.2 画面渲染器

```python
"""ui/render.py"""

import cv2
import numpy as np
from PIL import Image, ImageTk

from measure.rebar_measure import MeasureResult
from common.constants import SegClass


class UIRenderer:
    """
    叠加渲染器：将掩码、测量结果、激光数据叠加到 RGB 帧。
    返回 ImageTk.PhotoImage 供 tkinter Label 显示。
    """

    # 类别颜色映射（BGR）
    COLOR_BACKGROUND = (0, 0, 0)
    COLOR_VERTICAL = (0, 0, 255)    # 红色 — 纵向钢筋
    COLOR_HORIZONTAL = (0, 255, 0)  # 绿色 — 横向钢筋

    def __init__(self, alpha: float = 0.45):
        self._alpha = alpha

    def compose(
        self,
        rgb_frame: np.ndarray,
        mask: np.ndarray,
        measure_result: Optional[MeasureResult],
        laser_values: tuple,
        distance_mm: float,
        display_size: tuple,
        degraded: bool = False,
    ) -> ImageTk.PhotoImage:
        """
        合成最终显示画面。

        伪代码：
        1. overlay = rgb_frame.copy()
        2. 叠加掩码：类别1区域半透明红色、类别2区域半透明绿色
        3. 若有 measure_result: 绘制绿色边框 + 直径标注（白字）
           相邻钢筋：灰色虚线 + 间距标注（青字）
        4. 绘制激光数据面板：4 路原始值 + 融合值
        5. 左上显示系统状态：fps / 工作距离 / 策略 / 帧同步信息
        6. 若 degraded=True: 顶部显示黄色横幅"推理服务离线 — 降级模式"
        7. resize 到 display_size
        8. BGR → RGB → PIL.Image → ImageTk.PhotoImage
        """
        # h, w = rgb_frame.shape[:2]
        # pil_img = Image.fromarray(cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB))
        # return ImageTk.PhotoImage(pil_img)
        return None  # type: ignore
```

#### 3.7.3 主应用类

```python
"""ui/app.py"""

import sys
import time
import threading
from typing import Optional

import tkinter as tk
from tkinter import ttk, messagebox

from camera.base import BaseCameraInput
from camera.orbbec_336l import Orbbec336LInput
from camera.mock import MockCameraInput
from serial.serial_manager import SerialManager
from serial.laser_fusion import LaserFusion, FusionStrategy
from grpc_client.inference_client import InferenceGrpcClient, InferenceResult
from measure.rebar_measure import RebarMeasure, MeasureResult
from measure.intrinsics_manager import IntrinsicsManager
from ui.cli_fusion_menu import CliFusionMenu
from ui.render import UIRenderer
from common.config_loader import (
    ConfigLoader, CameraConfig, IntrinsicsConfig, NetworkConfig
)
from common.constants import MIN_INFER_INTERVAL_SEC


class RebarMeasureApp:
    """
    节点 B 主应用类（tkinter GUI 主线程）。
    整合所有子模块，驱动采集 → 推理 → 测量 → 显示全链路。
    """

    def __init__(self, root: tk.Tk):
        self._root = root
        root.title("钢筋直径测量系统 — 节点 B 主控")
        root.geometry("1280x800")

        # ---- 加载配置 ----
        self._camera_cfg = ConfigLoader.load_camera_config(
            "config/camera.json"
        )
        self._intrinsics_cfg = ConfigLoader.load_intrinsics_config(
            "config/intrinsics.json"
        )
        self._network_cfg = ConfigLoader.load_network_config(
            "config/network.json"
        )

        # ---- 初始化模块 ----
        self._camera: Optional[BaseCameraInput] = None
        self._serial_mgr = SerialManager(self._network_cfg)
        self._fusion = LaserFusion()
        self._grpc_client = InferenceGrpcClient(self._network_cfg)
        self._intrinsics_mgr = IntrinsicsManager(self._intrinsics_cfg)
        self._rebar_measure = RebarMeasure(
            fx=self._intrinsics_cfg.fx,
            fy=self._intrinsics_cfg.fy,
        )
        self._renderer = UIRenderer()

        # ---- 运行状态 ----
        self._is_running: bool = False
        self._degraded_mode: bool = False       # 推理服务离线降级标志
        self._frame_id: int = 0
        self._last_infer_time: float = 0.0
        self._latest_mask: Optional[np.ndarray] = None
        self._latest_measure: Optional[MeasureResult] = None

        # 工作线程
        self._main_loop_thread: Optional[threading.Thread] = None
        self._cli_thread: Optional[threading.Thread] = None
        self._health_thread: Optional[threading.Thread] = None

        self._setup_ui()

    def _setup_ui(self) -> None:
        """初始化 Tkinter 界面布局。"""
        # 布局结构（自上而下）：
        # row 0: 视频显示区 (LabelFrame "测量画面")
        #   - self._video_label: ttk.Label (anchor=center)
        # row 1: 激光数据面板 (LabelFrame "激光测距数据")
        #   - 4x 传感器值标签 + 融合策略与结果
        # row 2: 钢筋测量结果面板 (LabelFrame "钢筋直径/间距")
        #   - self._rebar_treeview: ttk.Treeview (编号, 直径, 国标直径, 间距)
        # row 3: 操作面板 (LabelFrame "操作")
        #   - self._measure_btn: "测量"按钮
        #   - self._status_label: 系统状态文本
        pass

    def start(self) -> None:
        """启动所有工作线程。"""
        # 伪代码：
        # self._is_running = True
        #
        # 1. 打开相机（异步扫描 → 选择 → 打开）
        # 2. 连接 gRPC（probe）→ 不可达则 self._degraded_mode = True
        # 3. 打开串口（异步扫描选择）
        # 4. 启动主循环线程: self._main_loop_thread
        # 5. 启动 CLI 线程: self._cli_thread
        # 6. 启动健康检查线程: self._health_thread
        pass

    def _request_measurement(self) -> None:
        """用户触发测量：推理 + 测量换算（在工作线程执行）。"""
        # if time.time() - self._last_infer_time < MIN_INFER_INTERVAL_SEC:
        #     root.after(0, lambda: messagebox.showwarning("提示", "两次测量间隔至少10秒"))
        #     return
        # if self._degraded_mode:
        #     root.after(0, lambda: messagebox.showwarning("降级", "推理服务离线，无法测量"))
        #     return
        # threading.Thread(target=self._do_measurement, daemon=True).start()

    def _do_measurement(self) -> None:
        """工作线程中执行推理 + 测量。"""
        # 伪代码：
        # 1. rgb_frame = self._camera.get_rgb_frame()
        # 2. laser_result, _ = self._serial_mgr.get_latest()
        # 3. distance = self._fusion.fuse(laser_result)
        # 4. if distance is None: 提示激光无效 → return
        # 5. self._frame_id += 1
        # 6. infer_result = self._grpc_client.infer(rgb_frame, self._frame_id, distance)
        # 7. if infer_result is None: 提示推理超时 → return
        # 8. 帧同步校验: infer_result.frame_id == self._frame_id
        # 9. measure_result = self._rebar_measure.measure(
        #        infer_result.mask, distance, rgb_frame)
        # 10. self._last_infer_time = time.time()
        # 11. self._latest_measure = measure_result
        # 12. self._root.after(0, self._refresh_display)
        pass

    def _main_loop(self) -> None:
        """主循环线程：仅负责视频预览刷新，不执行推理与测量。"""
        # while self._is_running:
        #     frame = self._camera.get_rgb_frame() if self._camera else None
        #     if frame is None: time.sleep(0.033); continue
        #     laser_result, _ = self._serial_mgr.get_latest()
        #     distance = self._fusion.fuse(laser_result) if laser_result else 0.0
        #     raw_values = laser_result.raw_values if laser_result else (0,0,0,0)
        #     img = self._renderer.compose(frame, self._latest_mask,
        #         self._latest_measure, raw_values, distance,
        #         (display_w, display_h), self._degraded_mode)
        #     self._root.after(0, lambda: self._update_video(img))
        #     time.sleep(0.033)
        pass

    def _refresh_display(self) -> None:
        """测量完成后，用最新结果刷新显示（主线程）。"""
        # if self._latest_measure:
        #     更新 Treeview 钢筋列表与间距
        #     _refresh_video_display()
        pass

    def _refresh_video_display(self, img: ImageTk.PhotoImage) -> None:
        """更新视频标签图像（主线程安全）。"""
        # self._video_label.imgtk = img
        # self._video_label.config(image=img)
        pass

    def _on_closing(self) -> None:
        """窗口关闭时的清理。"""
        # self._is_running = False
        # self._camera.close() if self._camera else None
        # self._serial_mgr.close()
        # self._grpc_client.close()
        # self._root.destroy()
        pass
```

### 3.8 系统模块（`system/`）

#### 3.8.1 启动环境自检

```python
"""system/self_check.py"""

import socket
import sys
from typing import Tuple

from common.config_loader import NetworkConfig


class SelfCheckResult:
    """自检结果数据类。"""
    def __init__(self):
        self.local_ip_ok: bool = False
        self.port_ok: bool = False
        self.peer_reachable: bool = False
        self.errors: list = []
        self.warnings: list = []


def check_local_ip(cfg: NetworkConfig) -> Tuple[bool, str]:
    """
    检查本机 IP 是否与 network.json 中 local_ip 一致。
    Windows 开发环境下可 bypass。
    返回: (是否通过, 消息)
    """
    # 伪代码：
    # if platform == "win32":
    #     return True, "Windows 开发环境：IP 校验 bypass"
    #
    # 获取本机网卡 IP:
    #   s = socket.socket(AF_INET, SOCK_DGRAM)
    #   s.connect(("8.8.8.8", 80))
    #   actual_ip = s.getsockname()[0]
    #
    # if actual_ip != cfg.local_ip:
    #     return False, f"本机 IP {actual_ip} 与配置 {cfg.local_ip} 不一致"
    # return True, "本机 IP 校验通过"
    return False, "not implemented"


def check_port_available(cfg: NetworkConfig) -> Tuple[bool, str]:
    """检查 gRPC 端口是否未被占用。"""
    # 伪代码：
    # s = socket.socket(AF_INET, SOCK_STREAM)
    # result = s.connect_ex(("0.0.0.0", cfg.grpc_port))
    # s.close()
    # if result == 0: return False, f"端口 {cfg.grpc_port} 已被占用"
    # return True, "端口可用"
    return False, "not implemented"


def check_peer_reachable(cfg: NetworkConfig) -> Tuple[bool, str]:
    """TCP 探测对端节点 A 的 IP:端口是否可达。"""
    # 伪代码：
    # try:
    #     s = socket.socket(AF_INET, SOCK_STREAM)
    #     s.settimeout(2.0)
    #     s.connect((cfg.remote_ip, cfg.grpc_port))
    #     s.close()
    #     return True, "对端节点可达"
    # except (socket.timeout, ConnectionRefusedError, OSError):
    #     return False, f"对端 {cfg.remote_ip}:{cfg.grpc_port} 不可达"
    return False, "not implemented"


def run_self_check(cfg: NetworkConfig) -> SelfCheckResult:
    """
    执行完整的启动环境自检。
    返回 SelfCheckResult。
    硬故障（IP/端口）需退出；软故障（对端不可达）启用降级模式。
    """
    result = SelfCheckResult()

    ok, msg = check_local_ip(cfg)
    result.local_ip_ok = ok
    if not ok:
        result.errors.append(msg)
    else:
        result.warnings.append(msg)

    ok, msg = check_port_available(cfg)
    result.port_ok = ok
    if not ok:
        result.errors.append(msg)

    ok, msg = check_peer_reachable(cfg)
    result.peer_reachable = ok
    if not ok:
        result.warnings.append(msg)

    return result
```

#### 3.8.2 运行时健康检查

```python
"""system/health_check.py"""

import threading
import time
from typing import Callable

from common.config_loader import NetworkConfig
from system.self_check import check_peer_reachable


class HealthChecker:
    """
    运行时健康检查器。
    周期性探测对端节点 A 存活性，失败时回调设置降级模式。
    """

    def __init__(
        self,
        net_cfg: NetworkConfig,
        on_lost: Callable[[], None],
        on_recovered: Callable[[], None],
    ):
        self._net_cfg = net_cfg
        self._on_lost = on_lost
        self._on_recovered = on_recovered
        self._is_running: bool = False
        self._was_reachable: bool = True
        self._check_interval = net_cfg.health_check_interval_s
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """启动健康检查线程。"""
        # self._is_running = True
        # self._thread = threading.Thread(target=self._check_loop, daemon=True)
        # self._thread.start()
        pass

    def stop(self) -> None:
        # self._is_running = False
        pass

    def _check_loop(self) -> None:
        """后台周期探测。"""
        # while self._is_running:
        #     ok, _ = check_peer_reachable(self._net_cfg)
        #     if ok and not self._was_reachable:
        #         self._on_recovered()  # 恢复在线
        #     elif not ok and self._was_reachable:
        #         self._on_lost()       # 进入降级
        #     self._was_reachable = ok
        #     time.sleep(self._check_interval)
        pass
```

#### 3.8.3 启动引导

```python
"""system/bootstrap.py"""

import sys
import logging

import tkinter as tk

from common.config_loader import ConfigLoader, NetworkConfig
from system.self_check import run_self_check
from ui.app import RebarMeasureApp


def bootstrap() -> None:
    """
    启动引导：环境自检 → Tkinter 初始化 → 应用启动。
    任何硬故障直接 sys.exit(1)。
    """
    # 伪代码：
    # 1. 初始化日志系统（logging.basicConfig → logs/node_b.log）
    #
    # net_cfg = ConfigLoader.load_network_config("config/network.json")
    #
    # result = run_self_check(net_cfg)
    # if result.errors:
    #     for e in result.errors: logging.error(f"自检失败: {e}")
    #     sys.exit(1)
    #
    # if not result.peer_reachable:
    #     for w in result.warnings: logging.warning(f"自检警告: {w}")
    #     degraded = True  # 降级模式
    # else:
    #     degraded = False
    #
    # root = tk.Tk()
    # app = RebarMeasureApp(root)
    # if degraded: app.enable_degraded_mode()
    # app.start()
    # root.mainloop()
    pass
```

### 3.9 程序入口（`main.py`）

```python
"""节点 B 程序入口。"""

from system.bootstrap import bootstrap


if __name__ == "__main__":
    bootstrap()
```

---

## 4. 并发与线程模型

### 4.1 线程总览

节点 B 共使用 **5 类线程**（不含 Tkinter 主线程）：

| 线程 | 数量 | 驻留周期 | 职责 | 守护线程 |
|------|------|----------|------|----------|
| 主线程 (Main) | 1 | 全程 | Tkinter 事件循环、UI 渲染、测量绘制 | 否 |
| 相机读取线程 | 1 | 从 `open()` 到 `close()` | 后台读帧 + 断线重连 | 是 |
| 串口读取线程 | 1 | 从 `open_port()` 到 `close()` | 后台读串口 + 帧解析 | 是 |
| 主循环线程 | 1 | 从 `start()` 到 `_on_closing()` | 视频预览刷新 30fps | 是 |
| CLI 菜单线程 | 1 | 全程 | 终端交互式融合策略选择 | 是 |
| 健康检查线程 | 1 | 全程 | 周期性探测节点 A 存活性 | 是 |

注：用户点击"测量"按钮触发的推理 + 测量任务为**临时工作线程**（daemon），完成后销毁。

```
Main Thread (tkinter)
  ├── UI 事件处理
  ├── root.after(0, ...) 调度回调  ←── 子线程投递
  ├── 画面渲染与绘制
  └── 钢筋结果列表刷新

Camera Read Thread                     Serial Read Thread
  ├── 持续 read() 帧                     ├── 持续 read() 串口
  ├── 更新 _latest_frame (lock)          ├── SerialBuffer.append()
  └── 断线自动 try_reconnect()           ├── LaserDataParser.parse()
                                          └── 更新 _latest_result (lock)

Measure Worker Thread (temporary)       Main Loop Thread
  ├── gRPC Infer()                       ├── 取最新帧 (lock)
  ├── 帧同步校验                         ├── 取最新激光值 (lock)
  ├── RebarMeasure.measure()             ├── 融合距离
  └── root.after(0, refresh)             ├── UIRenderer.compose()
                                         └── root.after(0, update_label)

Health Check Thread                     CLI Menu Thread
  ├── 周期 probe()                       ├── input() / print()
  ├── 状态变化回调                       └── LaserFusion.set_strategy()
  └── app.set_degraded(on/off)
```

### 4.2 共享状态与锁策略

| 共享变量 | 类型 | 保护锁 | 读方 | 写方 |
|----------|------|--------|------|------|
| `_latest_frame` | `np.ndarray` | `_frame_lock` | 主循环线程、测量线程 | 相机读取线程 |
| `_latest_result` | `LaserParseResult` | `_data_lock` | 主循环线程、测量线程 | 串口读取线程 |
| `_degraded_mode` | `bool` | GIL（Python bool 原子性） | 主循环线程、测量线程 | 健康检查线程回调（via after） |
| `_latest_mask` | `np.ndarray` | GIL | 主循环线程 | 测量回调线程 |
| `_latest_measure` | `MeasureResult` | GIL | 主循环线程 | 测量回调线程 |

**锁使用原则**：
- 仅相机帧与激光数据因持续高频更新需独立 Lock
- 测量结果与掩码由单次触发更新、`root.after(0, ...)` 调度读取，利用 GIL 保证 Python 字节码级安全
- Lock 持有时间严格最小化，禁止在 Lock 内执行 IO 或耗时计算

### 4.3 线程安全规则

1. **tkinter 主线程唯一性**：所有 widget 操作（`Label.config()`、`Treeview.insert()`、`messagebox.show*()`）必须在主线程执行。子线程通过 `root.after(0, callback)` 调度
2. **禁止跨模块直接状态读写**：`FusionDisplayApp` 过去直接访问 `camera_manager.latest_frame`、`serial_manager.latest_data` — 重构后必须通过公共接口（`get_frame()`、`get_latest()`）且内部加锁返回拷贝
3. **daemon 线程**：除主线程外均为 daemon，确保主线程退出时所有子线程自动终止
4. **资源释放顺序**：停止标志 → join 工作线程 → 释放设备资源。关闭窗口时先设 `_is_running = False`，再 join，最后 `root.destroy()`

### 4.4 帧同步实现

```
用户触发测量
  ↓
生成 frame_id = ++self._frame_id
获取 rgb_frame = camera.get_rgb_frame()
获取 laser_result = serial_mgr.get_latest()
融合 distance = fusion.fuse(laser_result)    ← 使用 laser_result 的时间戳与 rgb_frame 的时间戳对齐
                                   （原型阶段：认为同一 cycle 内采到的帧与激光值"近似同步"）
  ↓
gRPC Infer(frame_id, distance)
  ↓
收到 InferResponse:
  if response.frame_id != self._frame_id: 丢弃（迟到响应）
  if response.timestamp_ms != self._request_timestamp_ms: 丢弃
  else: 执行测量
```

**超时控制**：gRPC 客户端设置 `deadline`（默认 5000ms）。超时后跳过当前帧，不阻塞采集主循环。

---

## 5. 配置加载与启动自检流程

### 5.1 启动流程总览

```
main.py
  ↓
bootstrap.bootstrap()
  ↓
初始化日志系统 (→ logs/node_b.log)
  ↓
加载 network.json
  ↓
run_self_check(net_cfg)
  ├── check_local_ip()       ──硬故障─┐
  ├── check_port_available() ──硬故障─┼→ sys.exit(1)
  └── check_peer_reachable() ─软故障─→ degraded = True
  ↓
初始化 Tkinter root
  ↓
RebarMeasureApp(root)
  ├── 加载 camera.json / intrinsics.json
  ├── 初始化全部子模块
  └── _setup_ui()
  ↓
app.start()
  ├── 异步扫描 → 打开相机
  ├── gRPC probe → 连接（如可达）
  ├── 异步扫描 → 打开串口
  ├── 启动主循环线程
  ├── 启动 CLI 线程
  ├── 启动健康检查线程
  └── 若有 degraded → UI 显示黄色横幅
  ↓
root.mainloop()
```

### 5.2 自检检查项与失败处理

| 检查项 | 检查逻辑 | 硬/软故障 | 失败处理 |
|--------|----------|-----------|----------|
| 本机 IP 校验 | 获取网卡实际 IP 与 `network.json.local_ip` 对比（Windows bypass） | **硬** | 记录 ERROR 日志 + `sys.exit(1)` |
| 端口可用性 | 尝试 bind `0.0.0.0:grpc_port` | **硬** | 记录 ERROR 日志 + `sys.exit(1)` |
| 对端可达性 | TCP 探测 `remote_ip:grpc_port`（2 秒超时） | **软** | 记录 WARNING 日志 + 进入降级模式 + UI 黄色横幅提示 |

### 5.3 降级模式行为

当节点 A 不可达（启动自检失败，或运行中健康检查检测到丢失）时，进入降级模式：

| 能力 | 正常模式 | 降级模式 |
|------|----------|----------|
| 视频预览 | 有（30fps） | 有（30fps） |
| 激光数据显示 | 有 | 有 |
| AI 推理 | 有（gRPC 到节点 A） | 无 |
| 钢筋直径测量 | 推理 + 测量 | 无（提示"推理服务离线"） |
| 融合策略选择 | CLI 可用 | CLI 可用 |
| 恢复机制 | — | 健康检查恢复后自动退出降级模式 |

### 5.4 平台兼容处理

| 平台 | IP 校验 | 端口校验 | 相机后端 | 串口 |
|------|---------|----------|----------|------|
| openEuler 生产 | 严格执行 | 严格执行 | CAP_V4L2 | `/dev/ttyUSB*` |
| Windows 开发 | bypass（仅 warning） | 严格执行 | CAP_DSHOW | `COM*` |

---

## 6. 日志与异常处理策略

### 6.1 日志规范

严格遵循 AGENTS.md §7.5 "一机一日志"规范：

```python
"""system/logging_setup.py"""

import logging
import os
from datetime import datetime


def setup_logging(log_path: str = "logs/node_b.log") -> logging.Logger:
    """
    初始化节点 B 全局日志系统。
    输出格式: 时间戳(毫秒) | 级别 | 节点 | 模块 | 消息
    """
    # 伪代码：
    # os.makedirs(os.path.dirname(log_path), exist_ok=True)
    #
    # formatter = logging.Formatter(
    #     fmt="%(asctime)s.%(msecs)03d | %(levelname)-8s | %(node)s | %(module)s | %(message)s",
    #     datefmt="%Y-%m-%d %H:%M:%S"
    # )
    #
    # handler = logging.FileHandler(log_path, encoding="utf-8")
    # handler.setFormatter(formatter)
    #
    # console_handler = logging.StreamHandler()
    # console_handler.setFormatter(formatter)
    #
    # logger = logging.getLogger("node_b")
    # logger.setLevel(logging.INFO)  # 原型阶段默认 INFO
    # logger.addHandler(handler)
    # logger.addHandler(console_handler)
    #
    # return logger
    return logging.getLogger("node_b")
```

### 6.2 日志模块命名空间

按照 §7.5，双节点统一模块标识符：

| 模块标识 | 使用位置 | 示例日志 |
|----------|----------|----------|
| `camera` | `camera/` | "336L 设备打开成功: 1920x1080@30fps" |
| `serial` | `serial/` | "S21C 帧解析成功: 4路距离=[812,815,810,813]mm" |
| `grpc_client` | `grpc_client/` | "推理请求 frame_id=42, deadine=5000ms" |
| `measure` | `measure/` | "测量完成: 3 根钢筋, 直径 = [12,12,12]mm" |
| `ui` | `ui/` | "用户触发测量 / 融合策略切换为 MEAN" |
| `system` | `system/` | "启动自检: 本机 IP 校验通过" |
| `laser_fusion` | `serial/laser_fusion.py` | "4 路有效=[812,815,810,813], 均值=812.5mm" |

### 6.3 各级别日志触发条件

| 级别 | 触发场景 | 示例 |
|------|----------|------|
| DEBUG | 开发调试（帧序列号、时间戳细节） | "frame_id=42 采集时刻=1690700000123ms" |
| INFO | 正常流程节点 | "相机打开成功"、"推理完成: 3根钢筋" |
| WARNING | 可恢复异常 / 数据质量下降 | "激光通道 2 零值剔除"、"对端可达性检查失败: 进入降级模式" |
| ERROR | 不可恢复异常 / 功能无法执行 | "4 路激光全无效"、"gRPC 调用超时" |
| CRITICAL | 程序即将退出 | "本机 IP 不匹配: 配置=192.168.10.1 actual=192.168.0.50" |

### 6.4 异常分类与处理

#### 6.4.1 异常分类

| 异常类别 | 来源 | 是否退出 | 处理策略 |
|----------|------|----------|----------|
| 环境配置异常 | 本机 IP 不匹配、端口被占用 | **是** | `self_check` → `sys.exit(1)` |
| 推理服务离线 | gRPC 超时、对端不可达 | 否（降级） | 日志 WARNING + 降级模式 + UI 提示 |
| 激光数据全无效 | 4 路全无效 | 否 | 日志 ERROR + 界面"激光测距无效" |
| 相机断线 | VideoCapture.read() 持续失败 | 否 | 后台自动重连；UI 提示"摄像头断线" |
| 串口断线 | SerialException | 否 | 后台自动重连；UI"重连串口"按钮启用 |
| 测量无结果 | 无有效连通域 | 否 | 日志 INFO + UI"未检测到钢筋" |
| 帧同步失败 | frame_id 不匹配 | 否 | 日志 WARNING + 跳过当前帧 |

#### 6.4.2 异常处理模板

```python
# 在 camera/orbbec_336l.py _read_loop 中:
#   except (cv2.error, OSError) as e:
#       logger.error("camera", f"读帧异常: {e}, 尝试重连")
#       self._is_opened = False
#       self.try_reconnect()

# 在 grpc_client/inference_client.py infer 中:
#   except grpc.RpcError as e:
#       logger.error("grpc_client", f"gRPC 调用失败: {e.code()}")
#       return None

# 在 serial/serial_manager.py _read_loop 中:
#   except (serial.SerialException, OSError) as e:
#       logger.error("serial", f"串口异常: {e}")
#       self._is_running = False
#       break  # 由主循环线程检测后触发重连

# 在 ui/app.py _do_measurement 中:
#   except Exception as e:
#       logger.error("ui", f"测量执行异常: {e}")
#       self._root.after(0, lambda: messagebox.showerror("错误", str(e)))
```

### 6.5 关键数据日志格式（双节点对齐示例）

为确保跨节点比对调试时间线可对齐，以下为关键日志的标准格式：

```text
2026-07-30 14:23:01.125 | INFO  | B | system     | 启动自检: 本机 IP 校验通过 [192.168.10.1]
2026-07-30 14:23:01.456 | INFO  | B | grpc_client| gRPC 连接成功: 192.168.10.2:50051
2026-07-30 14:23:02.001 | INFO  | B | camera     | 336L 设备打开成功: 1920x1080@30fps
2026-07-30 14:23:02.345 | INFO  | B | serial     | S21C 帧解析成功: 4路距离=[812,815,810,813]mm
2026-07-30 14:23:02.346 | INFO  | B | laser_fusion| 4路有效=[812,815,810,813], 融合策略=MEAN, 结果=812.5mm
2026-07-30 14:23:05.789 | INFO  | B | ui         | 用户触发测量: frame_id=1
2026-07-30 14:23:05.790 | INFO  | B | grpc_client| 推理请求: frame_id=1, distance=812.5ms, jpeg_size=245678B
2026-07-30 14:23:07.123 | INFO  | B | grpc_client| 推理响应: frame_id=1, mask_size=480x640, 耗时=1333ms
2026-07-30 14:23:07.456 | INFO  | B | measure    | 测量完成: 找到 3 根钢筋, 直径(mm)=[12.0,12.0,12.0], 间距(mm)=[150.2,149.8]
2026-07-30 14:23:08.001 | WARNING| B | laser_fusion| 通道 2 零值剔除, 有效路数=3/4
2026-07-30 14:23:15.002 | ERROR | B | laser_fusion| 4 路激光全无效: raw=[0,0,0,0], 当前帧测量不可执行
2026-07-30 14:23:15.003 | WARNING| B | system     | 健康检查: 对端 192.168.10.2:50051 不可达, 进入降级模式
```

### 6.6 日志文件管理

- `logs/node_b.log` 为单次运行全量日志
- 原型阶段不引入日志轮转与大小限制（C 类要素）
- 关闭程序时 flush 并关闭 handler
- 异常退出时 logging.shutdown() 确保落盘

### 6.7 异常退出码约定

| 退出码 | 含义 |
|--------|------|
| 0 | 正常退出 |
| 1 | 环境自检硬故障（IP 不匹配 / 端口冲突） |
| 2 | Tkinter 初始化失败（无 DISPLAY / tkinter 未安装） |
| 3 | 配置文件加载失败（JSON 解析错误 / 缺少必需字段） |

---

## 附录 A：现有代码到新架构的映射清单

| 现有类 / 函数 | 源文件 | 对应新模块 | 对应新类 |
|----------------|--------|------------|----------|
| `SerialBuffer` | `demo.py:25-84` | `serial/serial_buffer.py` | `SerialBuffer` |
| `LaserDataParser` | `demo.py:87-104` | `serial/laser_parser.py` | `LaserDataParser` |
| `CameraManager` | `demo.py:107-1215` | `camera/orbbec_336l.py` | `Orbbec336LInput` |
| `SerialManager` | `demo.py:1217-1439` | `serial/serial_manager.py` | `SerialManager` |
| `FusionDisplayApp` | `demo.py:1442-1796` | `ui/app.py` | `RebarMeasureApp` |
| `check_gui_environment` | `demo.py:1799-1843` | `system/self_check.py` | `_check_gui_environment()` |
| `RebarMeasureV53` | `new-predict.py:504+` | `measure/rebar_measure.py` | `RebarMeasure` |
| `Unet` | `new-predict.py:357+` | （移至节点 A） | `grpc_server/inference_service.py` |
| `LaserDataParser.parse()` 有效性检查 | — | `serial/laser_parser.py` | `LaserParseResult` + `_is_valid()` |
| 4 路融合逻辑 | — | `serial/laser_fusion.py` | `LaserFusion` |
| CLI 融合菜单 | — | `ui/cli_fusion_menu.py` | `CliFusionMenu` |
| 画面渲染 + 掩码叠加 | — | `ui/render.py` | `UIRenderer` |
| gRPC 客户端封装 | — | `grpc_client/inference_client.py` | `InferenceGrpcClient` |
| 启动环境自检 | — | `system/self_check.py` | `run_self_check()` |
| 运行时健康检查 | — | `system/health_check.py` | `HealthChecker` |
| 配置加载器 | — | `common/config_loader.py` | `ConfigLoader` + dataclasses |

## 附录 B：接口依赖矩阵

```
[camera]  ←→ [ measure ] : 提供 RGB 帧用于测量结果可视化
[camera]  ←→ [ ui ]      : 提供预览帧
[camera]  ←→ [grpc_client]: 提供 JPEG 编码帧用于推理请求

[serial]  ←→ [ measure ] : 提供圆整距离 mm
[serial]  ←→ [ ui ]      : 提供激光原始值用于面板显示
[laser_fusion] ←→ [grpc_client]: 提供融合距离用于请求携带

[grpc_client] ←→ [ measure ] : 提供类别掩码用于测量
[grpc_client] ←→ [ ui ]  : 提供推理状态（在线/离线）

[measure] ←→ [ ui ]      : 提供 MeasureResult 用于渲染与列表
[intrinsics] ←→ [ measure ] : 提供 fx/fy 用于 mm/px 换算
[system.self_check] ←→ [ system.bootstrap ] : 提供自检结果
[system.health_check] ←→ [ ui ] : 提供降级状态切换回调
```

---

> **文档结束**
> 
> 本文件是节点 B 原型阶段的工程蓝本。所有接口签名与关键逻辑以本方案为准；具体实现可据实测微调，但模块边界与职责划分、锁策略、帧同步机制、自检与降级逻辑为强制约定，未经评审不得破坏。
