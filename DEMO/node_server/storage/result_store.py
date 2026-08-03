"""识别结果落盘存储。

对齐 Design-server.md §8（识别结果存储规范）与 AGENTS.md §7.5 日志规范。

落盘路径：``./result/{YYYYMMDD-HHMMSS}/``
- result.jpg：恒强制保存（可视化结果图，优先用 measure_result.annotated_frame，否则用 rgb_frame）
- report.csv：按 save_options["csv"] 保存（UTF-8 with BOM）
- ClassMask.png：按 save_options["mask"] 保存（8-bit 单通道 PNG 无损）

注意：``measure_result`` 参数的实际类型为 ``measure.rebar_measure.MeasureResult``，
此处用 ``typing.Any`` 占位以避免循环导入（measure 模块尚未实现）。
"""
import csv
import logging
import os
from datetime import datetime
from typing import Any, Optional, TYPE_CHECKING

import cv2
import numpy as np

if TYPE_CHECKING:
    # 仅用于类型提示，运行期不导入，避免循环依赖
    from measure.rebar_measure import MeasureResult  # noqa: F401

# 服务节点 全局 logger（由 common.logging_setup.setup_logging 初始化）
# 对齐 AGENTS.md §7.5 模块命名空间：storage（结果存储）
_logger: logging.Logger = logging.getLogger("node_server.storage")

# 结果根目录（相对路径，由程序启动位置决定）
_RESULT_ROOT: str = "result"

# CSV 列名（对齐 Design-server.md §8.2）
_CSV_COLUMNS = ["图片名称", "钢筋编号", "检测直径(mm)", "相邻区间", "检测间距(mm)"]


class ResultStore:
    """识别结果存储器：负责将单次测量结果三件套落盘。"""

    def save(
        self,
        frame_id: int,
        rgb_frame: np.ndarray,
        mask: Optional[np.ndarray],
        measure_result: Optional[Any],
        save_options: dict,
    ) -> str:
        """保存一次测量结果。

        :param frame_id: 帧序号（仅用于日志追溯）
        :param rgb_frame: RGB 原图（非空）
        :param mask: 类别掩码（可为 None；save_options["mask"] 为 True 且非 None 时保存）
        :param measure_result: 测量结果对象，实际类型为 ``measure.rebar_measure.MeasureResult``；
                               为 None 时只保存 result.jpg（+ 可选 mask）
        :param save_options: 保存开关 dict，形如 ``{"csv": bool, "mask": bool}``；
                               ``result.jpg`` 恒强制保存，不受开关控制
        :return: 成功时返回结果目录路径；失败时返回空字符串
        """
        # 解析保存开关（缺字段视为 False）
        save_csv: bool = bool(save_options.get("csv", False))
        save_mask: bool = bool(save_options.get("mask", False))

        # 构造子目录：./result/{YYYYMMDD-HHMMSS}/，同秒覆盖（exist_ok 不报错）
        timestamp_dir: str = datetime.now().strftime("%Y%m%d-%H%M%S")
        result_dir: str = os.path.join(_RESULT_ROOT, timestamp_dir)

        try:
            os.makedirs(result_dir, exist_ok=True)
        except OSError as e:
            _logger.error(
                "storage | 创建结果目录失败: dir=%s err=%s frame_id=%s",
                result_dir,
                e,
                frame_id,
            )
            return ""

        # ---------- 1. result.jpg（恒强制保存） ----------
        # 优先使用 measure_result.annotated_frame（含标注），否则回退到 rgb_frame
        annotated = None
        if measure_result is not None:
            annotated = getattr(measure_result, "annotated_frame", None)
        jpg_data = annotated if annotated is not None else rgb_frame
        try:
            result_jpg_path = os.path.join(result_dir, "result.jpg")
            ok, encoded = cv2.imencode(".jpg", jpg_data)
            if ok:
                encoded.tofile(result_jpg_path)
            else:
                _logger.error(
                    "storage | result.jpg 编码失败 frame_id=%s", frame_id
                )
                # 编码失败不阻断后续 csv/mask 落盘
        except Exception as e:
            _logger.error(
                "storage | result.jpg 写入失败: path=%s err=%s frame_id=%s",
                result_jpg_path,
                e,
                frame_id,
            )

        # ---------- 2. report.csv（按开关保存） ----------
        if save_csv and measure_result is not None:
            try:
                self._write_csv(result_dir, measure_result)
            except Exception as e:
                _logger.error(
                    "storage | report.csv 写入失败: dir=%s err=%s frame_id=%s",
                    result_dir,
                    e,
                    frame_id,
                )

        # ---------- 3. ClassMask.png（按开关保存，8-bit 单通道无损） ----------
        if save_mask and mask is not None:
            try:
                self._write_mask_png(result_dir, mask)
            except Exception as e:
                _logger.error(
                    "storage | ClassMask.png 写入失败: dir=%s err=%s frame_id=%s",
                    result_dir,
                    e,
                    frame_id,
                )

        _logger.info(
            "storage | 结果已保存: dir=%s frame_id=%s csv=%s mask=%s",
            result_dir,
            frame_id,
            save_csv,
            save_mask,
        )
        return result_dir

    @staticmethod
    def _write_csv(result_dir: str, measure_result: Any) -> None:
        """写入 report.csv（UTF-8 with BOM）。

        :param result_dir: 结果目录
        :param measure_result: MeasureResult 对象，需含 ``rebars`` 与 ``spacings_mm`` 属性
        """
        csv_path = os.path.join(result_dir, "report.csv")
        rebars = getattr(measure_result, "rebars", []) or []
        spacings_mm = getattr(measure_result, "spacings_mm", []) or []

        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow(_CSV_COLUMNS)
            # 逐根钢筋写一行；首行无前驱时相邻区间/间距填 "-"
            for idx, rebar in enumerate(rebars):
                # 直径：优先取对齐国标后的值（int，取整），回退原始测量值（1 位小数）
                # 对齐 Design-server.md §8.2：就近对齐国标后的直径（取整）或原始测量值（1 位小数）
                std_d = getattr(rebar, "standard_diameter_mm", None)
                if std_d is not None:
                    diameter: Any = int(std_d)
                else:
                    raw_d = getattr(rebar, "diameter_mm", None)
                    diameter = f"{raw_d:.1f}" if raw_d is not None else "-"
                if idx == 0:
                    interval = "-"
                    spacing = "-"
                else:
                    interval = f"H{idx}~H{idx + 1}"
                    # spacings_mm 长度比 rebars 少 1，取 idx-1
                    spacing_val = (
                        spacings_mm[idx - 1] if (idx - 1) < len(spacings_mm) else None
                    )
                    spacing = f"{spacing_val:.1f}" if spacing_val is not None else "-"
                writer.writerow(["result.jpg", f"H{idx + 1}", diameter, interval, spacing])

    @staticmethod
    def _write_mask_png(result_dir: str, mask: np.ndarray) -> None:
        """写入 ClassMask.png（8-bit 单通道 PNG 无损）。

        :param result_dir: 结果目录
        :param mask: 类别掩码，像素值 0/1/2
        """
        mask_path = os.path.join(result_dir, "ClassMask.png")
        # 强制 8-bit 单通道
        if mask.dtype != np.uint8:
            mask = mask.astype(np.uint8)
        if mask.ndim == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        # PNG 无损压缩（compression=0）
        cv2.imwrite(mask_path, mask, [cv2.IMWRITE_PNG_COMPRESSION, 0])
