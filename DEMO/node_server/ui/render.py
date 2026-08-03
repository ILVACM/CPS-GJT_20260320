"""UI 画面渲染器：掩码叠加 + 测量标注 + OSD + 双显示模式 + 降级横幅。

对齐 Design-server.md §3.7.2 / §3.7.7 与 AGENTS.md §7.2（绘制在主线程执行）。

设计要点：
- ``render_overlay`` 返回 BGR ``np.ndarray``，不直接生成 ``ImageTk.PhotoImage``
  （tkinter 转换由调用方在主线程完成，保持渲染器与 GUI 框架解耦）
- 掩码以半透明颜色叠加：纵向钢筋红色、横向钢筋绿色
- OSD 左上角：``FPS:xx | 数据来源 | 来源协议``
- 叠加模式：画面中心上下左右四方向黄色数字
- 底栏模式：底部参数栏统一显示 4 路测距 + 融合距离 + 系统参数
- 降级横幅：顶部黄色横幅"推理服务离线 — 降级模式"
"""
import logging
from typing import Optional, Tuple

import cv2
import numpy as np

from common.constants import SegClass
from measure.rebar_measure import MeasureResult

# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 ui）
_logger = logging.getLogger("node_server.ui")

# 类别颜色映射（BGR）
_COLOR_BACKGROUND: Tuple[int, int, int] = (0, 0, 0)
_COLOR_VERTICAL: Tuple[int, int, int] = (0, 0, 255)       # 红色 — 纵向钢筋
_COLOR_HORIZONTAL: Tuple[int, int, int] = (0, 255, 0)     # 绿色 — 横向钢筋

# 显示模式枚举字符串
MODE_OVERLAY: str = "overlay"
MODE_PANEL: str = "panel"


class UIRenderer:
    """UI 画面渲染器。

    所有方法返回 BGR ``np.ndarray``，无 tkinter 依赖，可在任意线程调用。
    实际投递到 tkinter Label 显示由调用方在主线程完成（BGR→RGB→PIL→ImageTk）。
    """

    def __init__(self, mask_alpha: float = 0.45) -> None:
        """初始化渲染器。

        :param mask_alpha: 掩码叠加透明度（0~1，默认 0.45）
        """
        self._mask_alpha: float = float(mask_alpha)

    # ------------------------------------------------------------------
    # 公共入口
    # ------------------------------------------------------------------

    def render_overlay(
        self,
        frame: np.ndarray,
        measure_result: Optional[MeasureResult],
        osd_info: dict,
        display_mode: str = MODE_PANEL,
        mask: Optional[np.ndarray] = None,
        degraded: bool = False,
    ) -> np.ndarray:
        """合成最终显示画面（BGR ndarray）。

        :param frame: 原始 BGR 帧（不被修改，内部 copy）
        :param measure_result: 测量结果（可为 None）；不为 None 时优先使用其 annotated_frame
        :param osd_info: OSD 信息字典，含字段：
                         - fps: int/str
                         - source: str（"NetCamera" / "LocalCamera"）
                         - protocol: str（"TCP" / "USB" / "DirectShow"）
                         - laser_values: tuple/list[int]（4 路 mm）
                         - distance_mm: Optional[float]
                         - strategy: str（融合策略名）
        :param display_mode: 显示模式，``MODE_OVERLAY`` 或 ``MODE_PANEL``
        :param mask: 类别掩码（可选，用于实时叠加预览）；为 None 时不叠加掩码
        :param degraded: 是否降级模式
        :return: BGR ndarray（与输入同尺寸）
        """
        if frame is None:
            _logger.warning("render_overlay 收到空帧，返回纯黑占位")
            return np.zeros((480, 640, 3), dtype=np.uint8)

        # 优先使用测量结果的标注图（含掩码叠加 + 直径/间距标注）
        if measure_result is not None and measure_result.annotated_frame is not None:
            canvas: np.ndarray = measure_result.annotated_frame.copy()
        else:
            canvas = frame.copy()
            # 实时预览时叠加掩码（若有）
            if mask is not None:
                self._overlay_mask(canvas, mask)

        h, w = canvas.shape[:2]

        # OSD 左上角
        self._draw_osd(canvas, osd_info)

        # 显示模式分支：叠加模式 / 底栏模式
        if display_mode == MODE_OVERLAY:
            self._draw_overlay_mode(canvas, osd_info)
        else:
            self._draw_panel_mode(canvas, osd_info)

        # 降级横幅（顶部黄色横幅）
        if degraded:
            self._draw_degraded_banner(canvas)

        return canvas

    # ------------------------------------------------------------------
    # 内部：掩码叠加
    # ------------------------------------------------------------------

    def _overlay_mask(self, canvas: np.ndarray, mask: np.ndarray) -> None:
        """将类别掩码以半透明颜色叠加到 canvas（in-place）。

        :param canvas: BGR 图像（被修改）
        :param mask: 0/1/2 类别掩码（单通道）
        """
        try:
            mask_arr = np.asarray(mask)
            if mask_arr.ndim == 3:
                mask_arr = mask_arr[:, :, 0]
            mask_arr = mask_arr.astype(np.uint8)
            h, w = canvas.shape[:2]
            if mask_arr.shape[:2] != (h, w):
                mask_arr = cv2.resize(mask_arr, (w, h), interpolation=cv2.INTER_NEAREST)

            overlay = canvas.copy()
            # 纵向钢筋 — 红色
            v_mask = (mask_arr == SegClass.REBAR_VERTICAL)
            if v_mask.any():
                overlay[v_mask] = _COLOR_VERTICAL
            # 横向钢筋 — 绿色
            h_mask = (mask_arr == SegClass.REBAR_HORIZONTAL)
            if h_mask.any():
                overlay[h_mask] = _COLOR_HORIZONTAL

            cv2.addWeighted(overlay, self._mask_alpha, canvas, 1 - self._mask_alpha, 0, dst=canvas)
        except Exception as e:
            _logger.warning("掩码叠加失败: %s", e)

    # ------------------------------------------------------------------
    # 内部：OSD 左上角
    # ------------------------------------------------------------------

    @staticmethod
    def _draw_osd(canvas: np.ndarray, osd_info: dict) -> None:
        """绘制左上角 OSD：``FPS:xx | 数据来源 | 来源协议``。"""
        fps = osd_info.get("fps", "--")
        source = osd_info.get("source", "--")
        protocol = osd_info.get("protocol", "--")
        text = f"FPS:{fps} | {source} | {protocol}"

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.6
        thick = 1
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick)

        # 半透明黑色背景条
        pad = 6
        x0, y0 = 8, 8
        x1, y1 = x0 + tw + pad * 2, y0 + th + pad * 2
        overlay = canvas.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, canvas, 0.5, 0, dst=canvas)

        cv2.putText(
            canvas, text, (x0 + pad, y0 + th + pad),
            font, scale, (255, 255, 255), thick, lineType=cv2.LINE_AA,
        )

    # ------------------------------------------------------------------
    # 内部：叠加模式（中心上下左右四方向黄色数字）
    # ------------------------------------------------------------------

    @staticmethod
    def _draw_overlay_mode(canvas: np.ndarray, osd_info: dict) -> None:
        """叠加模式：画面中心上下左右四方向各一个测距点 + 黄色数字。"""
        laser_values = osd_info.get("laser_values") or (0, 0, 0, 0)
        # 4 路依序映射：上/下/左/右
        if len(laser_values) < 4:
            return
        up, down, left, right = laser_values[:4]

        h, w = canvas.shape[:2]
        cx, cy = w // 2, h // 2
        offset_x = max(80, w // 6)
        offset_y = max(60, h // 6)

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 1.0
        thick = 2

        def _draw_label(text: str, x: int, y: int) -> None:
            (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
            tx = x - tw // 2
            ty = y + th // 2
            # 黑色描边 + 黄色填充
            cv2.putText(canvas, text, (tx, ty), font, scale, (0, 0, 0), thick + 2, lineType=cv2.LINE_AA)
            cv2.putText(canvas, text, (tx, ty), font, scale, (0, 255, 255), thick, lineType=cv2.LINE_AA)

        _draw_label(f"{int(up)}", cx, cy - offset_y)
        _draw_label(f"{int(down)}", cx, cy + offset_y)
        _draw_label(f"{int(left)}", cx - offset_x, cy)
        _draw_label(f"{int(right)}", cx + offset_x, cy)

    # ------------------------------------------------------------------
    # 内部：底栏模式（底部参数栏）
    # ------------------------------------------------------------------

    @staticmethod
    def _draw_panel_mode(canvas: np.ndarray, osd_info: dict) -> None:
        """底栏模式：底部参数栏统一显示 4 路测距 + 融合距离 + 策略。"""
        h, w = canvas.shape[:2]
        laser_values = osd_info.get("laser_values") or (0, 0, 0, 0)
        distance_mm = osd_info.get("distance_mm")
        strategy = osd_info.get("strategy", "--")

        bar_h = 60
        y0 = h - bar_h
        # 半透明黑色底栏
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, y0), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, canvas, 0.4, 0, dst=canvas)

        # 4 路原始值
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thick = 1
        labels = []
        for i, v in enumerate(laser_values[:4]):
            labels.append(f"CH{i + 1}:{int(v)}")
        dist_text = f"DIST:{int(distance_mm) if distance_mm is not None else '--'}"
        strat_text = f"STRAT:{strategy}"

        # 等距分布
        all_labels = labels + [dist_text, strat_text]
        n = len(all_labels)
        slot_w = w / n
        for i, lbl in enumerate(all_labels):
            cx = int(slot_w * (i + 0.5))
            cy = y0 + bar_h // 2
            (tw, th), _ = cv2.getTextSize(lbl, font, scale, thick)
            cv2.putText(
                canvas, lbl, (cx - tw // 2, cy + th // 2),
                font, scale, (255, 255, 255), thick, lineType=cv2.LINE_AA,
            )

    # ------------------------------------------------------------------
    # 内部：降级横幅
    # ------------------------------------------------------------------

    @staticmethod
    def _draw_degraded_banner(canvas: np.ndarray) -> None:
        """顶部黄色横幅：推理服务离线 — 降级模式。"""
        h, w = canvas.shape[:2]
        banner_h = 36
        text = "推理服务离线 — 降级模式"
        font = cv2.FONT_HERSHEY_SIMPLEX

        # 黄色背景
        cv2.rectangle(canvas, (0, 0), (w, banner_h), (0, 200, 255), -1)
        scale = 0.7
        thick = 2
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
        cx = (w - tw) // 2
        cy = (banner_h + th) // 2
        cv2.putText(
            canvas, text, (cx, cy), font, scale, (0, 0, 0), thick, lineType=cv2.LINE_AA,
        )
