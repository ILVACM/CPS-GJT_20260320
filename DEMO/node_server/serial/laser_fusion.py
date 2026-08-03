"""4 路激光测距值融合策略。

对齐 Design-server.md §3.4.3 与 AGENTS.md §6.2（交互式融合策略）。
将 4 路 STP23L 测距值融合为 1 个标量距离，支持运行时策略切换（线程安全）。

四种策略：
- MEAN（默认）：有效路算术平均值
- MEDIAN：有效路中位数
- SPECIFIC：取指定一路（若该路无效返回 None）
- MANUAL：用户手动输入值（忽略探头数据）
"""
import logging
import statistics
import threading
from enum import Enum
from typing import Optional

from .laser_parser import LaserParseResult
from common.constants import SERIAL_CHANNELS

logger = logging.getLogger("node_server.serial")


class FusionStrategy(Enum):
    """激光融合策略枚举（对齐 AGENTS.md §6.2 CLI 菜单选项）。"""
    MEAN = 1       # 取均值（默认）
    MEDIAN = 2     # 取中位数
    SPECIFIC = 3   # 取特定一路
    MANUAL = 4     # 手动输入指定值


# 策略字符串名 → 枚举映射（支持 CLI 传入字符串）
_STRATEGY_MAP = {
    "MEAN": FusionStrategy.MEAN,
    "MEDIAN": FusionStrategy.MEDIAN,
    "SPECIFIC": FusionStrategy.SPECIFIC,
    "MANUAL": FusionStrategy.MANUAL,
}


class LaserFusion:
    """4 路激光测距值融合器。

    支持运行时策略切换，线程安全（``_lock`` 保护策略状态）。
    全无效时 ``fuse()`` 返回 None。
    """

    def __init__(self):
        self._strategy: FusionStrategy = FusionStrategy.MEAN
        self._specific_index: int = 0   # SPECIFIC 策略：指定路编号 0~3
        self._manual_value: Optional[float] = None  # MANUAL 策略：用户输入值
        self._lock: threading.Lock = threading.Lock()

    def set_strategy(
        self,
        strategy: str,
        specific_index: int = 0,
        manual_value: Optional[float] = None,
    ) -> None:
        """运行时切换融合策略（线程安全）。

        :param strategy: 策略名称字符串，取值 "MEAN" / "MEDIAN" / "SPECIFIC" / "MANUAL"
        :param specific_index: SPECIFIC 策略下指定的路编号（0~3）
        :param manual_value: MANUAL 策略下用户手动输入的距离值（mm）
        """
        enum_strategy = _STRATEGY_MAP.get(strategy.upper())
        if enum_strategy is None:
            logger.error("未知融合策略: %s（支持: MEAN/MEDIAN/SPECIFIC/MANUAL）", strategy)
            return
        with self._lock:
            self._strategy = enum_strategy
            self._specific_index = specific_index
            self._manual_value = manual_value
        logger.info(
            "融合策略已切换: strategy=%s specific_index=%d manual_value=%s",
            strategy, specific_index, manual_value,
        )

    def get_strategy(self) -> FusionStrategy:
        """获取当前策略枚举。"""
        with self._lock:
            return self._strategy

    def fuse(self, result: LaserParseResult) -> Optional[float]:
        """根据当前策略融合 4 路距离值为 1 个标量。

        :param result: 激光解析结果（含 4 路距离值与有效性标记）
        :return: 融合结果（mm），全无效返回 None
        """
        with self._lock:
            strategy = self._strategy
            specific_index = self._specific_index
            manual_value = self._manual_value

        # MANUAL 策略：忽略探头数据，返回用户手动输入值
        if strategy == FusionStrategy.MANUAL:
            if manual_value is None:
                logger.warning("MANUAL 策略但未设置 manual_value，返回 None")
                return None
            return float(manual_value)

        # 提取有效路距离值
        valid_distances = [
            d for d, valid in zip(result.distances_mm, result.valid_mask) if valid
        ]
        if not valid_distances:
            logger.warning("融合失败: 4 路数据全部无效")
            return None

        if strategy == FusionStrategy.MEAN:
            return sum(valid_distances) / len(valid_distances)
        elif strategy == FusionStrategy.MEDIAN:
            return float(statistics.median(valid_distances))
        elif strategy == FusionStrategy.SPECIFIC:
            if 0 <= specific_index < SERIAL_CHANNELS:
                if result.valid_mask[specific_index]:
                    return float(result.distances_mm[specific_index])
                logger.warning("SPECIFIC 策略: 第 %d 路无效", specific_index + 1)
                return None
            logger.error("SPECIFIC 策略: 索引 %d 越界（有效范围 0~%d）",
                         specific_index, SERIAL_CHANNELS - 1)
            return None
        return None
