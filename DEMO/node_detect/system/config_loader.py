"""
配置加载器：读取 JSON 配置文件并校验。

配置文件规范遵循 AGENTS.md §3.3 / §4.1 / §4.2：
  - network.json: local_ip, remote_ip, grpc_port, heartbeat_interval_seconds, heartbeat_timeout_count
  - inference.json: model_path, input_width, input_height, use_cuda

加载入口：load_config(config_dir: Path) -> AppConfig
"""

import ipaddress
import logging
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict

import json

logger = logging.getLogger("node_detect.system.config_loader")


class ConfigValidationError(ValueError):
    """配置校验失败异常"""
    pass


@dataclass
class AppConfig:
    """
    应用配置 — 数据类。

    所有字段从 JSON 配置文件加载，禁止硬编码默认值（原型阶段外置配置原则）。
    """

    # ---- network.json 段 ----
    local_ip: str = ""              # 本机静态 IP（如 "192.168.10.2"）
    remote_ip: str = ""             # 对端 IP（服务节点）
    grpc_port: int = 50051          # gRPC 监听端口
    heartbeat_interval_seconds: int = 5     # 心跳间隔（秒）
    heartbeat_timeout_count: int = 3        # 心跳超时次数
    # 以下两字段供 setup-network 子命令使用（对齐 AGENTS.md §4.2，详见 QUICKSTART.md §5.3）
    network_interface: str = ""     # 指定网卡名；留空时 setup-network 自动检测物理网卡
    netmask: str = "255.255.255.252"  # 子网掩码；/30 双节点直连够用，/24 更通用

    # ---- inference.json 段 ----
    model_path: str = ""            # 权重文件绝对路径
    input_width: int = 640          # 模型输入宽度
    input_height: int = 640         # 模型输入高度
    use_cuda: bool = True           # 是否使用 CUDA（Jetson 上必须 True）

    # ---- 日志段（自动装配） ----
    log_dir: str = "logs"           # 日志目录
    log_level: str = "INFO"         # 日志级别
    log_file: str = "node_detect.log"    # 日志文件名

    # ---- 元信息 （不写入配置文件） ----
    config_dir: str = field(default="", repr=False)  # 配置文件所在目录

    def to_safe_dict(self) -> Dict[str, Any]:
        """返回安全的配置字典（不含路径等敏感信息，可写入日志）"""
        return {
            "local_ip": self.local_ip,
            "remote_ip": self.remote_ip,
            "grpc_port": self.grpc_port,
            "heartbeat_interval_seconds": self.heartbeat_interval_seconds,
            "heartbeat_timeout_count": self.heartbeat_timeout_count,
            "network_interface": self.network_interface,
            "netmask": self.netmask,
            "model_path": self.model_path,
            "input_width": self.input_width,
            "input_height": self.input_height,
            "use_cuda": self.use_cuda,
            "log_level": self.log_level,
        }


def _read_json_file(path: Path) -> Dict[str, Any]:
    """
    读取 JSON 文件。

    Raises:
        ConfigValidationError: 文件不存在或格式错误
    """
    if not path.exists():
        raise ConfigValidationError(f"配置文件不存在: {path}")
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        raise ConfigValidationError(f"JSON 格式错误: {path} — {e}")
    if not isinstance(data, dict):
        raise ConfigValidationError(f"配置根节点必须为 JSON 对象: {path}")
    return data


def _validate_ip(ip: str, field_name: str) -> None:
    """校验 IPv4 地址格式"""
    if not ip:
        raise ConfigValidationError(f"{field_name} 不能为空")
    try:
        ipaddress.IPv4Address(ip)
    except ipaddress.AddressValueError:
        raise ConfigValidationError(f"{field_name} 格式无效: {ip}")


def _validate_port(port: int, field_name: str = "grpc_port") -> None:
    """校验端口号范围（1024-65535 公认端口，排除 0-1023 系统端口）"""
    if not isinstance(port, int) or not (1024 <= port <= 65535):
        raise ConfigValidationError(f"{field_name} 不在有效范围 [1024, 65535]: {port}")


def load_config(config_dir: str) -> AppConfig:
    """
    加载配置入口函数。

    读取 config_dir 下的 network.json 与 inference.json，合并为 AppConfig。

    Args:
        config_dir: 配置文件目录路径

    Returns:
        AppConfig — 已校验的配置对象

    Raises:
        ConfigValidationError: 配置文件缺失或格式/值无效
    """
    config_path = Path(config_dir)

    # 读取两个 JSON 文件
    network_data = _read_json_file(config_path / "network.json")
    inference_data = _read_json_file(config_path / "inference.json")

    # 合并为 AppConfig
    merged = {**network_data, **inference_data}
    # 移除不属于 AppConfig 的字段
    valid_fields = {f.name for f in AppConfig.__dataclass_fields__.values()}
    filtered = {k: v for k, v in merged.items() if k in valid_fields}

    config = AppConfig(**filtered)
    config.config_dir = str(config_path)

    # 校验
    _validate_ip(config.local_ip, "local_ip")
    _validate_ip(config.remote_ip, "remote_ip")
    _validate_port(config.grpc_port)
    _validate_port_range(config.heartbeat_interval_seconds, "heartbeat_interval_seconds", 1, 300)
    _validate_port_range(config.heartbeat_timeout_count, "heartbeat_timeout_count", 1, 10)
    _validate_port_range(config.input_width, "input_width", 320, 4096)
    _validate_port_range(config.input_height, "input_height", 320, 4096)

    # 模型路径：允许相对路径（相对于工作目录），也允许绝对路径
    model_path = Path(config.model_path)
    if not model_path.exists():
        # 仅警告，不抛异常：权重可能在部署时再放置
        logger.warning(f"模型权重文件未找到（部署时需存在）: {config.model_path}")
    else:
        config.model_path = str(model_path.resolve())

    return config


def _validate_port_range(value: int, name: str, min_val: int, max_val: int) -> None:
    """通用整数范围校验"""
    if not isinstance(value, int) or not (min_val <= value <= max_val):
        raise ConfigValidationError(f"{name} 不在有效范围 [{min_val}, {max_val}]: {value}")
