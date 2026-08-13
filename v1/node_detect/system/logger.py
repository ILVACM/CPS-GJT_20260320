"""
日志初始化 — 一机一日志 + 轮转。

日志规范遵循 AGENTS.md §7.5：
  - 检测节点 全局维护一个日志文件: logs/node_detect.log
  - 统一格式: `时间戳(毫秒) | 级别 | 节点 | 模块 | 消息`
  - 默认 INFO 级，10MB × 5 轮转

使用方式：
  init_logger(log_dir, log_level, log_file) — 在 main.py 入口处调用一次
  logger = logging.getLogger("node_detect.<module>.<name>") — 各模块获取日志器

各模块禁止自行创建 root logger，必须通过 logging.getLogger 获取命名日志器。
"""

import logging
import logging.handlers
import os
import sys
from pathlib import Path
from typing import Optional


# 日志格式常量 — 与 AGENTS.md §7.5 规范一致
# 格式: 时间戳(毫秒) | 级别 | 节点 | 模块 | 消息
LOG_FORMAT = "%(asctime)s.%(msecs)03d | %(levelname)-7s | %(node_name)s | %(module_short)s | %(message)s"
LOG_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"
LOG_MAX_BYTES = 10 * 1024 * 1024  # 10 MB
LOG_BACKUP_COUNT = 5
LOG_NODE_NAME = "D"  # 节点标识（D=检测节点）


class NodeNameFormatter(logging.Formatter):
    """
    自定义格式器 — 在每条记录上注入 node_name 和 module_short。

    使用 Formatter 而非 Filter 注入属性（Python 3.12 兼容性更好），
    确保 %(node_name)s / %(module_short)s 在格式化时可访问。
    """

    def __init__(self, fmt: str = LOG_FORMAT, datefmt: str = LOG_DATE_FORMAT,
                 node_name: str = LOG_NODE_NAME) -> None:
        super().__init__(fmt=fmt, datefmt=datefmt)
        self._node_name = node_name

    def format(self, record: logging.LogRecord) -> str:
        # 在格式化前注入动态属性
        record.node_name = self._node_name
        # 提取简短模块名（路径最后一层，如 node_detect.inference.predictor → predictor）
        record.module_short = record.name.rsplit(".", 1)[-1] if "." in record.name else record.name
        return super().format(record)


def _get_level_from_str(level_str: str) -> int:
    """
    将字符串级别转换为 logging 级别常量。

    Args:
        level_str: DEBUG / INFO / WARNING / ERROR / CRITICAL（不区分大小写）

    Returns:
        logging 级别常量（无效输入默认 INFO）
    """
    mapping = {
        "DEBUG": logging.DEBUG,
        "INFO": logging.INFO,
        "WARNING": logging.WARNING,
        "ERROR": logging.ERROR,
        "CRITICAL": logging.CRITICAL,
    }
    level = mapping.get(level_str.upper())
    if level is None:
        print(f"[日志] 无效级别: {level_str}，回退到 INFO，请检查 inference.json", file=sys.stderr)
        return logging.INFO
    return level


def init_logger(
    log_dir: str = "logs",
    log_level: str = "INFO",
    log_file: str = "node_detect.log",
    overwrite: bool = False,
) -> logging.Logger:
    """
    初始化检测节点 全局日志。

    启动时调用一次。重复调用不会重复添加 handler（unless overwrite=True）。

    Args:
        log_dir: 日志目录路径
        log_level: 日志级别字符串（DEBUG/INFO/WARNING/ERROR/CRITICAL）
        log_file: 日志文件名
        overwrite: 是否覆盖已有 handler（调试用，生产环境保持 False）

    Returns:
        根日志器（检测节点 的 root logger）
    """
    # 创建日志目录
    log_path = Path(log_dir)
    log_path.mkdir(parents=True, exist_ok=True)

    # 根日志器
    root_logger = logging.getLogger("node_detect")
    level = _get_level_from_str(log_level)
    root_logger.setLevel(level)

    # 清除已有 handler（避免重复调用时输出两份）
    if overwrite:
        root_logger.handlers.clear()

    # 自定义 Formatter（注入 node_name / module_short 动态属性）
    formatter = NodeNameFormatter(
        fmt=LOG_FORMAT, datefmt=LOG_DATE_FORMAT, node_name=LOG_NODE_NAME,
    )

    # 文件 handler（带轮转）
    file_handler = logging.handlers.RotatingFileHandler(
        filename=log_path / log_file,
        maxBytes=LOG_MAX_BYTES,
        backupCount=LOG_BACKUP_COUNT,
        encoding="utf-8",
    )
    file_handler.setLevel(level)
    file_handler.setFormatter(formatter)
    root_logger.addHandler(file_handler)

    # 控制台 handler（仅 ERROR 及以上输出到 stderr，避免 Jetson 终端干扰）
    console_handler = logging.StreamHandler(sys.stderr)
    console_handler.setLevel(logging.ERROR)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    root_logger.info(
        f"日志初始化完成: {log_path.resolve() / log_file} "
        f"(级别={log_level.upper()}, 轮转={LOG_MAX_BYTES//(1024*1024)}MB × {LOG_BACKUP_COUNT})"
    )
    return root_logger


def get_logger(name: str) -> logging.Logger:
    """
    获取命名日志器（各模块使用此函数而非 logging.getLogger）。

    Args:
        name: 模块命名空间（如 "grpc_server.servicer"）

    Returns:
        子日志器（继承 node_detect 根的 handler 与 formatter）
    """
    return logging.getLogger(f"node_detect.{name}")
