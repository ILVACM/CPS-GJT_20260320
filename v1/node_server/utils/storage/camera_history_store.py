"""网络摄像头连接历史记录存储。

对齐 AGENTS.md §7.5 模块命名空间 storage，与 result_store.py 同层。
属运行时状态数据（非 config 配置文件），可读写；config/*.json 仍只读。

落盘文件：``./camera_history.json``（相对程序启动目录）
格式：标准 JSON（RFC 8259），Python ``json`` 标准库解析，禁止外部依赖。

去重规则：按 ``host:port`` 唯一；重复连接同一地址时更新时间戳并前置。
排序规则：最近成功连接时间倒序（最新在前）。
"""
import json
import logging
import os
from typing import List

# 服务节点 全局 logger
_logger: logging.Logger = logging.getLogger("node_server.storage")

# 历史记录文件路径（相对程序启动目录，与 result/ 同级）
_HISTORY_PATH: str = "camera_history.json"

# 历史记录条数上限（防止无限增长）
_MAX_RECORDS: int = 20


class CameraHistoryStore:
    """网络摄像头连接历史记录存储器。"""

    def load(self) -> List[dict]:
        """读取历史记录。

        :return: 历史记录列表，每条形如
                 ``{"host": str, "port": int, "last_connected": str}``；
                 文件不存在或解析失败时返回空列表
        """
        if not os.path.exists(_HISTORY_PATH):
            return []
        try:
            with open(_HISTORY_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, list):
                _logger.warning("camera_history.json 格式异常（非数组），已忽略")
                return []
            records: List[dict] = []
            for item in data:
                if (isinstance(item, dict)
                        and isinstance(item.get("host"), str)
                        and isinstance(item.get("port"), int)
                        and isinstance(item.get("last_connected"), str)):
                    records.append(item)
            return records
        except (OSError, json.JSONDecodeError) as e:
            _logger.warning("读取摄像头历史记录失败: %s", e)
            return []

    def add(self, host: str, port: int, last_connected: str) -> None:
        """新增/更新一条历史记录（按 host:port 去重，最近在前）。

        :param host: 主机 IP
        :param port: 端口
        :param last_connected: 最近成功连接时间（ISO 格式字符串）
        """
        records = self.load()
        # 去重：移除同 host:port 的旧记录
        records = [
            r for r in records
            if not (r["host"] == host and r["port"] == port)
        ]
        # 新记录前置（最近成功在前）
        records.insert(0, {
            "host": host,
            "port": port,
            "last_connected": last_connected,
        })
        # 截断到上限
        records = records[:_MAX_RECORDS]
        try:
            with open(_HISTORY_PATH, "w", encoding="utf-8") as f:
                json.dump(records, f, ensure_ascii=False, indent=2)
            _logger.info(
                "storage | 网络摄像头历史已更新: %s:%d（共 %d 条）",
                host, port, len(records),
            )
        except OSError as e:
            _logger.warning("写入摄像头历史记录失败: %s", e)
