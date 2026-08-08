"""从 JSON 配置文件读取所有值类参数，严禁硬编码。

对齐 Design-server.md §3.2 与 AGENTS.md §3.3 / §7.1（统一标准 JSON、json 标准库解析）。
"""
import json
import logging
from dataclasses import dataclass, field
from typing import List

from utils.camera.device_identifier import CompatibleDevice

logger = logging.getLogger(__name__)


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
    # N1/Q3：结构化兼容设备列表（替代旧 compatible_models: List[str]）
    compatible_devices: List[CompatibleDevice] = field(default_factory=list)


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
    # 网络动态监测周期（秒，默认 5）：NetworkMonitor watchdog 探测网络状态间隔
    network_monitor_interval_seconds: float = 5.0
    # 以下两字段供 setup-network 子命令使用（对齐 AGENTS.md §4.2，详见 QUICKSTART.md §5.4）
    network_interface: str = ""  # 指定网卡名；留空时 setup-network 自动检测物理网卡
    netmask: str = "255.255.255.252"  # 子网掩码；/30 双节点直连够用，/24 更通用


@dataclass
class InferenceConfig:
    """推理节奏控制参数（config/inference.json）。

    inference_interval_seconds 控制服务节点 侧相邻两次推理请求的最小时间间隔。
    """

    inference_interval_seconds: float = 3.0


@dataclass
class SerialConfig:
    """串口配置（config/serial.json）。

    对齐 AGENTS.md §5.1（S21C 串口协议）与 §3.3（配置文件管理）。
    支持自动扫描 CH9102 设备（VID=0x1A86, PID∈{0x55D4, 0x55D5}），
    兼容 USB 直连与扩展坞接入（节点类型可为 /dev/ttyUSB* 或 /dev/ttyACM*）。
    """
    device_path: str = ""  # 非空时直接使用，跳过自动扫描
    auto_scan: bool = True  # 是否启用自动扫描
    vid_whitelist: List[int] = field(default_factory=lambda: [6790])  # WCH 厂商 ID（0x1A86=6790）
    pid_whitelist: List[int] = field(
        default_factory=lambda: [21972, 21973]  # CH9102 系列（0x55D4=21972, 0x55D5=21973）
    )
    description_keywords: List[str] = field(
        default_factory=lambda: ["Serial", "CH910", "CH340"]  # 描述关键字兜底匹配
    )
    path_prefix_whitelist: List[str] = field(
        default_factory=lambda: ["/dev/ttyUSB", "/dev/ttyACM"]  # 路径前缀白名单
    )
    path_prefix_blacklist: List[str] = field(
        default_factory=lambda: ["/dev/ttyAMA", "/dev/ttyS", "/dev/ttyconsole"]  # 路径前缀黑名单
    )
    baudrate: int = 115200  # CH9102 串口波特率
    select_first_when_multiple: bool = True  # 多候选时取字典序首个
    reconnect_interval_seconds: float = 3.0  # 重连间隔（秒）
    max_reconnect_attempts: int = 5  # [已弃用] 最大重连次数；热插拔扫描改为无限重连，该字段保留仅向后兼容
    max_age_seconds: float = 2.0  # 数据新鲜度阈值（秒）
    usb_unstable_threshold: int = 5  # 5 分钟内断线次数阈值，触发供电告警
    scan_interval_seconds: float = 2.0  # S21C 热插拔周期扫描间隔（秒）


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
        """加载相机配置。

        N1/Q3：解析 compatible_devices 结构化数组（pid/vid hex 字符串→int）。
        兼容旧 compatible_models 字段：存在时记录 WARNING 并忽略（R8 回退兼容）。
        compatible_devices 缺失时使用空列表默认值。
        """
        with open(path, "r", encoding="utf-8") as f:
            data: dict = json.load(f)

        # 旧字段兼容（R8）：compatible_models 已废弃，存在时 WARNING 并忽略
        if "compatible_models" in data:
            logger.warning(
                "camera.json: compatible_models 字段已废弃，请迁移为 compatible_devices 结构化数组。"
                "当前 compatible_models 将被忽略。"
            )
            data.pop("compatible_models", None)

        # 解析 compatible_devices（N1/Q3：hex 字符串→int，D5 决策）
        compatible_devices: List[CompatibleDevice] = []
        raw_list: list = data.pop("compatible_devices", [])
        for item in raw_list:
            try:
                pid_str: str = item.get("pid", "0")
                vid_str: str = item.get("vid", "0")
                # hex 字符串 → int（如 "0x0807" → 2055）
                pid_int: int = int(pid_str, 16) if isinstance(pid_str, str) else int(pid_str)
                vid_int: int = int(vid_str, 16) if isinstance(vid_str, str) else int(vid_str)
                dev: CompatibleDevice = CompatibleDevice(
                    brand=item.get("brand", ""),
                    model=item.get("model", ""),
                    pid=pid_int,
                    vid=vid_int,
                    connection_type=item.get("connection_type", "USB"),
                )
                compatible_devices.append(dev)
            except (ValueError, TypeError) as e:
                logger.warning("camera.json: compatible_devices 项解析失败 (%s): %s", item, e)

        return CameraConfig(
            rgb_width=data.get("rgb_width", 1920),
            rgb_height=data.get("rgb_height", 1080),
            rgb_fps=data.get("rgb_fps", 30),
            rgb_format=data.get("rgb_format", "MJPG"),
            depth_width=data.get("depth_width", 640),
            depth_height=data.get("depth_height", 480),
            depth_fps=data.get("depth_fps", 30),
            device_index=data.get("device_index", 0),
            compatible_devices=compatible_devices,
        )

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

    @staticmethod
    def load_serial_config(path: str) -> SerialConfig:
        """加载串口配置（config/serial.json）。

        缺失字段使用 SerialConfig dataclass 默认值，便于逐步引入新字段。
        启动时校验 VID 白名单的十进制值是否与常见 USB 转串口芯片的预期值一致，
        防止因十六进制→十进制转换算术错误导致自动扫描永远无法命中设备。
        """
        with open(path, "r", encoding="utf-8") as f:
            data: dict = json.load(f)
        # 仅取 SerialConfig 已定义字段，忽略多余键，避免 KeyError
        valid_keys: set = {f.name for f in SerialConfig.__dataclass_fields__.values()}
        filtered: dict = {k: v for k, v in data.items() if k in valid_keys}
        config = SerialConfig(**filtered)

        # ─── 启动一致性校验：VID 十进制值 vs 常见芯片十六进制预期 ──────────────
        # 常见 USB 转串口芯片的 VID（十六进制 → 十进制正确换算）
        #   0x1A86 = 6790  (WCH / QinHeng, CH9102 / CH340 / CH341)
        #   0x0403 = 1026  (FTDI FT232)
        #   0x067B = 1659  (Prolific PL2303)
        #   0x10C4 = 4292  (Silicon Labs CP210x)
        #   0x0483 = 1155  (ST Micro STM32 Virtual COM)
        _COMMON_USB_VID_DECIMALS: set = {6790, 1026, 1659, 4292, 1155}
        _config_vid_set: set = set(config.vid_whitelist)

        # 如果配置中存在"看起来像十六进制直接当十进制写"的值（4290 是 0x10C4 的错误心算结果；
        # 或 0x1A86 被误算为 4290 而非正确的 6790），给出明确警告。
        _HEX_AS_DECIMAL_TRAP: dict = {
            4290: "疑似 0x1A86 (WCH) 的正确十进制应为 6790，而非 4290",
        }
        for _trap_val, _hint in _HEX_AS_DECIMAL_TRAP.items():
            if _trap_val in _config_vid_set:
                logger.warning(
                    "serial.json: vid_whitelist 包含可疑值 %d —— %s。请核对后修正。",
                    _trap_val,
                    _hint,
                )

        # 如果配置中的所有 VID 均不在常见集合中，给出提醒（不阻断，避免误伤新型号芯片）
        if _config_vid_set and not (_config_vid_set & _COMMON_USB_VID_DECIMALS):
            logger.warning(
                "serial.json: vid_whitelist = %s 不在常见 USB 转串口芯片列表 %s 中。"
                "若这是新型号芯片可忽略；否则请确认十进制值是通过正确的十六进制换算得到的。",
                config.vid_whitelist,
                sorted(_COMMON_USB_VID_DECIMALS),
            )
        # ─── 校验结束 ──────────────────────────────────────────────────────────

        return config
