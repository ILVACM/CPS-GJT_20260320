"""从 JSON 配置文件读取所有值类参数，严禁硬编码。

对齐 Design-server.md §3.2 与 AGENTS.md §3.3 / §7.1（统一标准 JSON、json 标准库解析）。
"""
import json
from dataclasses import dataclass, field
from typing import List


@dataclass
class CameraConfig:
    """相机参数配置（config/camera.json）。"""

    rgb_width: int = 1920
    rgb_height: int = 1080
    rgb_fps: int = 30
    rgb_format: str = "MJPG"
    depth_width: int = 640
    depth_height: int = 480
    depth_fps: int = 30
    device_index: int = 0  # /dev/video* 或 Windows DirectShow 索引
    compatible_models: List[str] = field(
        default_factory=lambda: ["Orbbec Gemini 336L", "Orbbec Astra"]
    )


@dataclass
class IntrinsicsConfig:
    """相机内参配置（config/intrinsics.json）。"""

    fx: float = 950.0
    fy: float = 950.0
    cx: float = 960.0
    cy: float = 540.0


@dataclass
class NetworkConfig:
    """网络参数配置（config/network.json）。"""

    local_ip: str = "192.168.10.1"
    remote_ip: str = "192.168.10.2"
    grpc_port: int = 50051
    grpc_deadline_ms: int = 5000  # gRPC 单次调用 deadline（毫秒）
    health_check_interval_s: float = 5.0  # 健康检查探测周期（秒）
    heartbeat_interval_seconds: int = 5  # 心跳间隔（秒，对齐 AGENTS.md §5.7.4）
    heartbeat_timeout_count: int = 3  # 心跳超时判定阈值（连续丢失次数）


@dataclass
class InferenceConfig:
    """推理节奏控制参数（config/inference.json）。

    inference_interval_seconds 控制服务节点 侧相邻两次推理请求的最小时间间隔。
    """

    inference_interval_seconds: float = 3.0


@dataclass
class LoggingConfig:
    """日志轮转参数（config/logging.json，对齐 Design-server.md §6.7）。"""

    max_bytes: int = 10 * 1024 * 1024  # 单文件大小上限，默认 10MB
    backup_count: int = 3  # 保留归档份数
    level: str = "INFO"  # 默认日志级别


class ConfigLoader:
    """读取并解析 JSON 配置文件。

    所有方法均为静态方法，统一使用 Python 标准库 ``json.load`` 解析，
    禁止引入 pyyaml / toml / configparser 等替代解析依赖（AGENTS.md §7.1）。
    """

    @staticmethod
    def load_camera_config(path: str) -> CameraConfig:
        """加载相机配置。compatible_models 字段可选，缺失时使用默认白名单。"""
        with open(path, "r", encoding="utf-8") as f:
            data: dict = json.load(f)
        # compatible_models 字段可选：缺失时使用 dataclass 默认值
        if "compatible_models" not in data:
            # 不传入该字段，让 dataclass 默认值生效
            return CameraConfig(
                rgb_width=data["rgb_width"],
                rgb_height=data["rgb_height"],
                rgb_fps=data["rgb_fps"],
                rgb_format=data["rgb_format"],
                depth_width=data["depth_width"],
                depth_height=data["depth_height"],
                depth_fps=data["depth_fps"],
                device_index=data["device_index"],
            )
        return CameraConfig(**data)

    @staticmethod
    def load_intrinsics_config(path: str) -> IntrinsicsConfig:
        """加载相机内参配置。"""
        with open(path, "r", encoding="utf-8") as f:
            data: dict = json.load(f)
        return IntrinsicsConfig(**data)

    @staticmethod
    def load_network_config(path: str) -> NetworkConfig:
        """加载网络参数配置。"""
        with open(path, "r", encoding="utf-8") as f:
            data: dict = json.load(f)
        return NetworkConfig(**data)

    @staticmethod
    def load_inference_config(path: str) -> InferenceConfig:
        """加载推理节奏配置（config/inference.json）。"""
        with open(path, "r", encoding="utf-8") as f:
            data: dict = json.load(f)
        return InferenceConfig(**data)

    @staticmethod
    def load_logging_config(path: str) -> LoggingConfig:
        """加载日志轮转配置（config/logging.json）。"""
        with open(path, "r", encoding="utf-8") as f:
            data: dict = json.load(f)
        return LoggingConfig(**data)
