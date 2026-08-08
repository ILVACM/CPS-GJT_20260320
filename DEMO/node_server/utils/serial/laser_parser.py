"""激光测距数据解析器 + 有效性检查。

对齐 Design-server.md §3.4.2 与 AGENTS.md §6.3（数据有效性检查）。
解析 11 字节帧中的 4 路大端序 uint16 距离值，并对每路执行有效性检查：
- 零值剔除（距离=0 表示无回波/未连接）
- 负值/超量程剔除（< LASER_MIN_MM 或 > LASER_MAX_MM）
"""
import logging
import struct
import time
from dataclasses import dataclass
from typing import List, Optional

from utils.common.constants import (
    LASER_MAX_MM,
    LASER_MIN_MM,
    SERIAL_FRAME_LENGTH,
)

logger = logging.getLogger("node_server.serial")


@dataclass
class LaserParseResult:
    """单帧激光数据解析结果。"""
    distances_mm: List[int]        # 4 路原始距离值（mm）
    valid_mask: List[bool]         # 4 路有效性标记（True=有效）
    timestamp: float               # 解析时刻时间戳（time.time()）
    valid_count: int = 0           # 有效路数（冗余字段，便于日志）


class LaserDataParser:
    """激光测距数据解析器，含有效性检查。

    所有方法均为静态方法，无状态，线程安全。
    """

    @staticmethod
    def parse(frame: bytes) -> Optional["LaserParseResult"]:
        """解析 11 字节帧：4 路大端序 uint16 距离值。

        帧结构（AGENTS.md §5.1）：
        - 字节 0：帧头 0x7B
        - 字节 1-8：4 路大端 uint16 距离值（mm）
        - 字节 9：XOR 校验
        - 字节 10：帧尾 0x7D

        :param frame: 11 字节完整帧（已通过 SerialBuffer 校验）
        :return: 解析结果；帧长度不正确返回 None
        """
        if len(frame) != SERIAL_FRAME_LENGTH:
            logger.warning("激光帧长度异常: %d 字节（期望 %d）", len(frame), SERIAL_FRAME_LENGTH)
            return None

        # 解包 4 路大端序 uint16
        d1: int
        d2: int
        d3: int
        d4: int
        d1, d2, d3, d4 = struct.unpack(">HHHH", frame[1:9])
        distances: List[int] = [d1, d2, d3, d4]

        # 逐路有效性检查
        valid_mask: List[bool] = []
        for i, val in enumerate(distances):
            ok: bool = LaserDataParser._is_valid(val)
            valid_mask.append(ok)
            if not ok:
                logger.warning("激光第 %d 路无效: value=%dmm（量程 %d~%dmm）",
                               i + 1, val, LASER_MIN_MM, LASER_MAX_MM)

        valid_count: int = sum(1 for v in valid_mask if v)
        timestamp: float = time.time()

        result: LaserParseResult = LaserParseResult(
            distances_mm=distances,
            valid_mask=valid_mask,
            timestamp=timestamp,
            valid_count=valid_count,
        )

        if valid_count == 0:
            logger.error("激光 4 路数据全部无效: distances=%s", distances)
        else:
            logger.info("S21C 帧解析成功: %d 路有效, distances=%s", valid_count, distances)

        return result

    @staticmethod
    def _is_valid(value: int) -> bool:
        """单值有效性检查（对齐 AGENTS.md §6.3）。

        - 零值剔除：距离=0 表示探头无回波或未连接
        - 负值/超量程剔除：< LASER_MIN_MM 或 > LASER_MAX_MM

        :param value: 距离值（mm）
        :return: 有效返回 True
        """
        if value == 0:
            return False
        if value < LASER_MIN_MM:
            return False
        if value > LASER_MAX_MM:
            return False
        return True
