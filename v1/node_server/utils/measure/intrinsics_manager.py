"""相机内参管理与 mm/px 换算。

对齐 AGENTS.md §5.3（mm/px = 距离 ÷ fx）与 §8.3（内参从配置文件读取，不硬编码）。
内参由 ``config/intrinsics.json`` 加载（IntrinsicsConfig），默认 fx=fy=950（§8.3）。
"""
import logging

from utils.common.config_loader import IntrinsicsConfig

# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 measure）
_logger = logging.getLogger("node_server.measure")


class IntrinsicsManager:
    """相机内参管理器：根据工作距离计算 mm/px 换算系数。

    mm/px 换算直接依赖工作距离（激光测距值）与内参 fx（AGENTS.md §5.3 / §8.3）。
    测量数值的准确性由人工在后续阶段评估优化，原型阶段不作承诺。
    """

    def __init__(self, intrinsics_cfg: IntrinsicsConfig) -> None:
        """初始化内参管理器。

        :param intrinsics_cfg: 相机内参配置（fx/fy/cx/cy，从 config/intrinsics.json 加载）
        """
        self._fx: float = float(intrinsics_cfg.fx)
        self._fy: float = float(intrinsics_cfg.fy)
        self._cx: float = float(intrinsics_cfg.cx)
        self._cy: float = float(intrinsics_cfg.cy)
        _logger.info(
            "内参加载完成: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
            self._fx, self._fy, self._cx, self._cy,
        )

    @property
    def fx(self) -> float:
        """焦距 fx（像素）。"""
        return self._fx

    @property
    def fy(self) -> float:
        """焦距 fy（像素）。"""
        return self._fy

    @property
    def cx(self) -> float:
        """主点 cx（像素）。"""
        return self._cx

    @property
    def cy(self) -> float:
        """主点 cy（像素）。"""
        return self._cy

    def calc_mm_per_pixel(self, distance_mm: float) -> float:
        """计算 mm/px 换算系数（对齐 AGENTS.md §5.3：mm/px = distance_mm / fx）。

        :param distance_mm: 拍摄距离（激光测距值，mm）
        :return: mm/px 系数；距离非正或 fx 非正时返回 0.0 并记录告警
        """
        if distance_mm <= 0:
            _logger.warning("距离非正数: distance_mm=%.2f，返回 0.0", distance_mm)
            return 0.0
        if self._fx <= 0:
            _logger.error("fx 非正数: fx=%.2f，无法换算", self._fx)
            return 0.0
        return float(distance_mm) / self._fx
