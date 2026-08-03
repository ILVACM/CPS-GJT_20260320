"""服务节点 全局日志初始化。

对齐 Design-server.md §6.1 + §6.7 与 AGENTS.md §7.5（一机一日志）。
- 输出格式：``时间戳(毫秒) | 级别 | 节点 | 模块 | 消息``
- 同时输出到文件（RotatingFileHandler）与控制台（StreamHandler）
- 轮转参数从 LoggingConfig 读取，默认 10MB / 3 份
"""
import logging
import os
from logging.handlers import RotatingFileHandler
from typing import Optional

from common.config_loader import LoggingConfig


# 统一日志格式（对齐 AGENTS.md §7.5）
# 示例：2026-07-30 14:23:01.125 | INFO     | S | camera    | 336L 设备打开成功
_LOG_FORMAT: str = "%(asctime)s.%(msecs)03d | %(levelname)-8s | S | %(name)s | %(message)s"
_LOG_DATEFMT: str = "%Y-%m-%d %H:%M:%S"

# 默认轮转参数（logging.json 缺失或未传入时兜底）
_DEFAULT_MAX_BYTES: int = 10 * 1024 * 1024  # 10MB
_DEFAULT_BACKUP_COUNT: int = 3
_DEFAULT_LEVEL: str = "INFO"

# 日志级别名称到 logging 常量的映射
_LEVEL_MAP = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
    "CRITICAL": logging.CRITICAL,
}


def setup_logging(
    log_path: str = "logs/node_server.log",
    logging_cfg: Optional[LoggingConfig] = None,
) -> logging.Logger:
    """初始化服务节点 全局日志系统。

    :param log_path: 日志文件路径，默认 ``logs/node_server.log``
    :param logging_cfg: 日志轮转参数；为 None 时使用默认值（10MB / 3 份 / INFO）
    :return: 名为 ``node_server`` 的全局 logger
    """
    # 解析轮转参数
    if logging_cfg is not None:
        max_bytes: int = logging_cfg.max_bytes
        backup_count: int = logging_cfg.backup_count
        level_name: str = logging_cfg.level.upper()
    else:
        max_bytes = _DEFAULT_MAX_BYTES
        backup_count = _DEFAULT_BACKUP_COUNT
        level_name = _DEFAULT_LEVEL
    level = _LEVEL_MAP.get(level_name, logging.INFO)

    # 自适应创建日志目录
    log_dir = os.path.dirname(log_path)
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)

    formatter = logging.Formatter(fmt=_LOG_FORMAT, datefmt=_LOG_DATEFMT)

    # 文件 handler：RotatingFileHandler，零额外依赖（标准库）
    file_handler = RotatingFileHandler(
        filename=log_path,
        maxBytes=max_bytes,
        backupCount=backup_count,
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    file_handler.setLevel(level)

    # 控制台 handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    console_handler.setLevel(level)

    # 获取名为 node_server 的全局 logger，避免重复挂载 handler
    logger = logging.getLogger("node_server")
    logger.setLevel(level)
    # 清理已存在的 handler，防止重复初始化导致日志重复输出
    for existing in list(logger.handlers):
        logger.removeHandler(existing)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    # 不向上游 root logger 传播，避免被 root 的 handler 二次输出
    logger.propagate = False

    return logger
