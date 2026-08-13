"""钢筋直径与间距测量（服务节点 测量模块）。

对齐 AGENTS.md §5.3（测量换算）、§8.4（分割类别定义）与 Design-server.md §3.6。
算法源：``temp/new-predict.py`` 的 ``RebarMeasureV53`` 类（第 504 行起），
本模块迁移其 ``measure_from_label`` 标签路径流水线，简化直径后处理与调参逻辑。

迁移要点：
- 输入是 0/1/2 类别掩码（来自检测节点 推理），不是彩色预测图
- 纵向钢筋（类别 1）为主测量对象；横向钢筋（类别 2）用于重叠剔除
- 距离来自激光测距（不是固定 camera_distance_mm）
- mm/px 通过 IntrinsicsManager.calc_mm_per_pixel(distance_mm) 计算
- 国标对齐用 common/constants.STANDARD_DIAMETERS
- 间距计算：相邻钢筋中心距 × mm_per_pixel（简化，不引入 V53 的 spacing_scale_factor）
- 标注图：在 rgb_frame 上叠加掩码 + 直径/间距标注；rgb_frame 为 None 时用掩码生成灰度背景

简化原则（原型阶段，对齐 AGENTS.md 文首"最快打通链路、可演示"原则）：
- 直径后处理去除 unify_diameter_per_image / small_rebar_nominal_mm 等调参 hack，
  仅保留 raw_diameter + 国标对齐
- 间距直接用 mm_per_pixel（不引入 spacing_scale_factor=1.50）
"""
import logging
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from utils.common.constants import STANDARD_DIAMETERS, SegClass
from utils.measure.intrinsics_manager import IntrinsicsManager

# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 measure）
_logger = logging.getLogger("node_server.measure")

# 国标规格表转 numpy 数组（便于就近对齐）
_STANDARD_DIAMETERS_ARR: np.ndarray = np.asarray(STANDARD_DIAMETERS, dtype=np.float32)


@dataclass
class RebarInfo:
    """单根钢筋测量信息。"""

    index: int  # 钢筋编号（从 1 开始）
    diameter_mm: float  # 测量直径（mm，原始像素宽 × mm/px）
    standard_diameter_mm: int  # 就近对齐国标规格（mm）
    center_x: int  # 中心 x 坐标（像素）
    bbox: Tuple[int, int, int, int]  # 边界框 (x, y, w, h)


@dataclass
class MeasureResult:
    """单帧测量结果。"""

    rebars: List[RebarInfo]  # 检测到的钢筋列表（按 center_x 升序）
    spacings_mm: List[float]  # 相邻钢筋间距（mm），len = len(rebars) - 1
    mm_per_pixel: float  # 像素换算系数
    distance_mm: float  # 拍摄距离（激光测距值）
    annotated_frame: np.ndarray  # 标注后的可视化结果图（BGR）


class RebarMeasure:
    """钢筋直径与间距测量器（V53 标签路径迁移）。

    生命周期：
        mgr = IntrinsicsManager(intrinsics_cfg)
        measurer = RebarMeasure(mgr)
        result = measurer.measure(label_mask, distance_mm=812.0, rgb_frame=frame)
    """

    def __init__(self, intrinsics_manager: IntrinsicsManager) -> None:
        """初始化测量器。

        :param intrinsics_manager: 相机内参管理器（提供 mm/px 换算）
        """
        self._intrinsics: IntrinsicsManager = intrinsics_manager

        # ---- V53 默认调参（原型阶段保留默认值，不做过度优化） ----
        self._min_component_area: int = 120  # 最小连通域面积
        self._min_height_ratio: float = 0.12  # 最小高度占图像比例
        self._merge_x_ratio: float = 0.012  # 共线合并 x 容差比例
        self._min_diameter_mm: float = 5.0  # 最小直径（mm）
        self._max_diameter_mm: float = 45.0  # 最大直径（mm）
        self._min_height_ratio_filter: float = 0.34  # 主钢筋最小长度比例（间距过滤用）

        # 像素直径上下界（依赖 mm_per_pixel，每次 measure 时更新）
        self._min_diameter_px: float = 5.0
        self._max_diameter_px: float = 45.0

        _logger.info("RebarMeasure 初始化完成（V53 标签路径）")

    # ------------------------------------------------------------------
    # 公共入口
    # ------------------------------------------------------------------

    def measure(
        self,
        mask: np.ndarray,
        distance_mm: float,
        rgb_frame: Optional[np.ndarray] = None,
    ) -> Optional[MeasureResult]:
        """从类别掩码测量钢筋直径与间距。

        :param mask: 0/1/2 类别掩码（单通道，与检测节点 推理输出一致）
        :param distance_mm: 拍摄距离（激光测距值，mm）
        :param rgb_frame: 原始 RGB 帧（BGR，用于标注图绘制）；为 None 时用掩码生成灰度背景
        :return: MeasureResult；输入无效或距离非正时返回 None
        """
        if mask is None:
            _logger.warning("measure 收到空掩码，返回 None")
            return None

        # 计算 mm/px 换算系数
        mm_per_pixel: float = self._intrinsics.calc_mm_per_pixel(distance_mm)
        if mm_per_pixel <= 0:
            _logger.warning("mm/px 非正数（distance_mm=%.2f），测量不可执行", distance_mm)
            return None
        self._update_scale_params(mm_per_pixel)

        # 规范化标签掩码为 2D uint8
        label_mask: np.ndarray = np.asarray(mask)
        if label_mask.ndim == 3:
            label_mask = label_mask[:, :, 0]
        label_mask = label_mask.astype(np.uint8)
        h, w = label_mask.shape[:2]

        # 若提供 rgb_frame，以 rgb_frame 尺寸为基准（与 V53 一致）
        if rgb_frame is not None:
            rgb_bgr = self._safe_to_bgr(rgb_frame)
            rh, rw = rgb_bgr.shape[:2]
            if (rh, rw) != (h, w):
                label_mask = cv2.resize(
                    label_mask, (rw, rh), interpolation=cv2.INTER_NEAREST,
                )
                h, w = label_mask.shape[:2]
        else:
            rgb_bgr = None

        # ① 提取纵向（red）/ 横向（green）掩码
        red_mask, green_mask = self._label_to_masks(label_mask)

        # ② 构建规则化钢筋
        final, _clean, _rejected = self._build_regular_rebars(red_mask, green_mask)
        final = sorted(final, key=lambda r: r["edge"]["center_x"])

        # ③ 间距计算用主钢筋过滤
        final, measure_y = self._filter_main_rebars_for_spacing(final, h)

        # ④ 按间距插补缺失钢筋
        final, _inserted = self._insert_missing_rebars_by_gap(final, h, measure_y)

        # ⑤ 计算每根钢筋直径 + 国标对齐
        for r in final:
            raw_d: float = float(r["edge"]["width_px"] * mm_per_pixel)
            r["raw_diameter_mm"] = raw_d
            r["diameter_mm"] = raw_d
            r["standard_mm"] = self._normalize_rebar_diameter(raw_d)

        # ⑥ 计算相邻间距（中心距 × mm/px）
        spacings_mm: List[float] = []
        for i in range(1, len(final)):
            prev_cx = float(final[i - 1]["edge"]["center_x"])
            curr_cx = float(final[i]["edge"]["center_x"])
            center_gap_px = curr_cx - prev_cx
            spacings_mm.append(float(center_gap_px * mm_per_pixel))

        # ⑦ 构建 RebarInfo 列表
        rebars: List[RebarInfo] = []
        for i, r in enumerate(final):
            edge = r["edge"]
            left_x = int(round(float(edge["left_x"])))
            y_top = int(float(edge["y_top"]))
            width_px = int(round(float(edge["width_px"])))
            height_px = int(float(edge["y_bottom"]) - float(edge["y_top"]))
            rebars.append(RebarInfo(
                index=i + 1,
                diameter_mm=float(r["diameter_mm"]),
                standard_diameter_mm=int(r["standard_mm"]),
                center_x=int(round(float(edge["center_x"]))),
                bbox=(left_x, y_top, width_px, height_px),
            ))

        # ⑧ 绘制标注图
        annotated = self._draw_annotated_frame(
            rgb_bgr, label_mask, final, spacings_mm, measure_y, h, w,
        )

        _logger.info(
            "测量完成: 钢筋数=%d, 间距数=%d, mm/px=%.4f, 距离=%.1fmm",
            len(rebars), len(spacings_mm), mm_per_pixel, distance_mm,
        )

        return MeasureResult(
            rebars=rebars,
            spacings_mm=spacings_mm,
            mm_per_pixel=mm_per_pixel,
            distance_mm=float(distance_mm),
            annotated_frame=annotated,
        )

    # ------------------------------------------------------------------
    # 内部：尺度参数更新
    # ------------------------------------------------------------------

    def _update_scale_params(self, mm_per_pixel: float) -> None:
        """根据当前 mm/px 更新像素直径上下界（对齐 V53 __init__ 逻辑）。"""
        safe_mmpp = max(mm_per_pixel, 1e-6)
        self._min_diameter_px = self._min_diameter_mm / safe_mmpp
        self._max_diameter_px = self._max_diameter_mm / safe_mmpp

    # ------------------------------------------------------------------
    # 内部：掩码提取与规则化（迁移自 V53 _label_to_masks / _regularize_rebar_mask）
    # ------------------------------------------------------------------

    @staticmethod
    def _safe_to_bgr(img: np.ndarray) -> np.ndarray:
        """确保图像为 BGR 三通道。"""
        if img is None:
            return None
        if img.ndim == 2:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if img.shape[2] == 4:
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img

    def _label_to_masks(
        self, label_mask: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """从 0/1/2 标签掩码提取纵向（类别 1）与横向（类别 2）二值掩码。

        对齐 V53 _label_to_masks：red_mask=纵向钢筋，green_mask=横向钢筋。
        """
        label = np.asarray(label_mask)
        if label.ndim == 3:
            label = label[:, :, 0]
        red_mask: np.ndarray = (
            label == SegClass.REBAR_VERTICAL
        ).astype(np.uint8) * 255
        green_mask: np.ndarray = (
            label == SegClass.REBAR_HORIZONTAL
        ).astype(np.uint8) * 255
        return red_mask, green_mask

    def _regularize_rebar_mask(self, red_mask: np.ndarray) -> np.ndarray:
        """纵向钢筋掩码形态学规则化（迁移自 V53 _regularize_rebar_mask）。

        开运算去噪 + 垂直/水平闭运算修补断裂 + 中值滤波。
        """
        mask = (red_mask > 0).astype(np.uint8) * 255
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        kernel_close_v = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 23))
        kernel_close_h = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close_v)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close_h)
        mask = cv2.medianBlur(mask, 3)
        return mask

    # ------------------------------------------------------------------
    # 内部：几何辅助（迁移自 V53 静态方法）
    # ------------------------------------------------------------------

    @staticmethod
    def _rect_long_angle(rect) -> float:
        """计算 minAreaRect 长边角度（迁移自 V53 _rect_long_angle）。"""
        box = cv2.boxPoints(rect).astype(np.float32)
        edges = []
        for i in range(4):
            p1 = box[i]
            p2 = box[(i + 1) % 4]
            vec = p2 - p1
            length = float(np.hypot(vec[0], vec[1]))
            angle = float(np.degrees(np.arctan2(vec[1], vec[0])))
            edges.append((length, angle))
        _, angle = max(edges, key=lambda x: x[0])
        while angle < -90:
            angle += 180
        while angle >= 90:
            angle -= 180
        return angle

    @staticmethod
    def _vertical_angle_error(angle: float) -> float:
        """纵向钢筋角度误差（与 90° 的偏差绝对值）。"""
        return abs(90.0 - abs(angle))

    @staticmethod
    def _box_from_center_angle(
        cx: float, cy: float, width: float, length: float, angle_deg: float,
    ) -> np.ndarray:
        """由中心、宽、长、角度构造旋转框 4 顶点（迁移自 V53 _box_from_center_angle）。"""
        theta = np.deg2rad(angle_deg)
        u = np.array([np.cos(theta), np.sin(theta)], dtype=np.float32)
        n = np.array([-np.sin(theta), np.cos(theta)], dtype=np.float32)
        c = np.array([cx, cy], dtype=np.float32)
        half_l = float(length) / 2.0
        half_w = float(width) / 2.0
        return np.array(
            [
                c - u * half_l - n * half_w,
                c + u * half_l - n * half_w,
                c + u * half_l + n * half_w,
                c - u * half_l + n * half_w,
            ],
            dtype=np.float32,
        )

    def _robust_component_width(
        self, component_mask: np.ndarray, bbox: Tuple[int, int, int, int],
    ) -> Optional[float]:
        """计算连通域鲁棒宽度（行扫描中位数，迁移自 V53 _robust_component_width）。"""
        x, y, bw, bh = bbox
        roi = component_mask[y:y + bh, x:x + bw]
        widths = []
        for row in roi:
            xs = np.where(row > 0)[0]
            if len(xs) >= 3:
                widths.append(float(xs[-1] - xs[0] + 1))
        if len(widths) < 8:
            return None
        widths_arr = np.asarray(widths, dtype=np.float32)
        med = float(np.median(widths_arr))
        valid = widths_arr[(widths_arr >= med * 0.55) & (widths_arr <= med * 1.35)]
        if len(valid) >= 6:
            widths_arr = valid
        return float(np.median(widths_arr))

    # ------------------------------------------------------------------
    # 内部：构建规则化钢筋（迁移自 V53 _build_regular_rebars）
    # ------------------------------------------------------------------

    def _build_regular_rebars(
        self, red_mask: np.ndarray, green_mask: np.ndarray,
    ) -> Tuple[List[dict], np.ndarray, List[Tuple[str, float]]]:
        """从纵向掩码构建规则化钢筋列表（迁移自 V53 _build_regular_rebars）。

        :return: (regular_rebars, clean_mask, rejected)
        """
        h, w = red_mask.shape[:2]
        clean = self._regularize_rebar_mask(red_mask)
        contours, _ = cv2.findContours(
            clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
        )

        candidates: List[dict] = []
        rejected: List[Tuple[str, float]] = []
        min_len = max(45.0, h * self._min_height_ratio)
        max_len = h * 1.20
        min_width = max(3.0, self._min_diameter_px * 0.45)
        max_width = min(w * 0.10, self._max_diameter_px * 1.35)

        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < self._min_component_area:
                rejected.append(("area", area))
                continue

            rect = cv2.minAreaRect(cnt)
            (cx, cy), (rw, rh), _ = rect
            short_side = float(min(rw, rh))
            long_side = float(max(rw, rh))
            if short_side <= 1 or long_side <= 1:
                rejected.append(("empty", area))
                continue

            angle = self._rect_long_angle(rect)
            angle_err = self._vertical_angle_error(angle)
            x, y, bw, bh = cv2.boundingRect(cnt)
            component_mask = np.zeros_like(clean)
            cv2.drawContours(component_mask, [cnt], -1, 255, -1)
            robust_width = self._robust_component_width(component_mask, (x, y, bw, bh))
            if robust_width is None:
                rejected.append(("unstable_width", area))
                continue
            aspect = long_side / max(short_side, 1e-6)
            fill_ratio = area / max(short_side * long_side, 1.0)
            green_overlap = (
                float(np.mean(green_mask[y:y + bh, x:x + bw] > 0))
                if bw > 0 and bh > 0 else 0.0
            )

            reason: Optional[str] = None
            if long_side < min_len:
                reason = "too_short"
            elif long_side > max_len:
                reason = "too_long"
            elif robust_width < min_width:
                reason = "too_narrow"
            elif robust_width > max_width:
                reason = "too_wide"
            elif long_side / max(robust_width, 1e-6) < 2.4:
                reason = "bad_aspect"
            elif angle_err > 16.0:
                reason = "bad_angle"
            elif fill_ratio < 0.22:
                reason = "weak_fill"
            elif green_overlap > 0.28:
                reason = "x_rebar_overlap"

            if reason is not None:
                rejected.append((reason, area))
                continue

            candidates.append({
                "rect": rect,
                "box": self._box_from_center_angle(
                    cx, cy, robust_width, long_side, angle,
                ),
                "bbox": (x, y, bw, bh),
                "area": area,
                "center_x": float(cx),
                "center_y": float(cy),
                "width_px": robust_width,
                "length_px": long_side,
                "angle": angle,
                "angle_error": angle_err,
                "fill_ratio": fill_ratio,
            })

        if not candidates:
            return [], clean, rejected

        # 过滤相对过窄的碎片
        widths = np.asarray([c["width_px"] for c in candidates], dtype=np.float32)
        med_w = float(np.median(widths))
        filtered: List[dict] = []
        for c in candidates:
            if c["width_px"] < med_w * 0.32:
                rejected.append(("narrow_vs_median", c["area"]))
                continue
            filtered.append(c)

        # 按中心 x 分组合并共线碎片
        filtered = sorted(filtered, key=lambda r: r["center_x"])
        groups: List[List[dict]] = []
        min_center_gap = max(6.0, med_w * 0.65)
        for c in filtered:
            if groups and abs(
                c["center_x"] - np.median([g["center_x"] for g in groups[-1]])
            ) < min_center_gap:
                groups[-1].append(c)
            else:
                groups.append([c])

        regular: List[dict] = []
        for group in groups:
            if len(group) == 1:
                regular.append(group[0])
                continue

            centers_x = np.asarray(
                [g["center_x"] for g in group], dtype=np.float32,
            )
            widths_g = np.asarray(
                [g["width_px"] for g in group], dtype=np.float32,
            )
            angles_g = np.asarray(
                [g["angle"] for g in group], dtype=np.float32,
            )
            areas_g = np.asarray([g["area"] for g in group], dtype=np.float32)

            x1 = min([g["bbox"][0] for g in group])
            y1 = min([g["bbox"][1] for g in group])
            x2 = max([g["bbox"][0] + g["bbox"][2] for g in group])
            y2 = max([g["bbox"][1] + g["bbox"][3] for g in group])

            center_x = float(
                np.average(centers_x, weights=np.maximum(areas_g, 1.0))
            )
            center_y = float((y1 + y2) / 2.0)
            width_px = float(np.median(widths_g))
            angle = float(np.median(angles_g))
            length_px = float(max(
                1.0,
                (y2 - y1) / max(abs(np.sin(np.deg2rad(angle))), 0.25),
            ))
            merged = dict(group[int(np.argmax(areas_g))])
            merged.update({
                "bbox": (int(x1), int(y1), int(x2 - x1), int(y2 - y1)),
                "area": float(np.sum(areas_g)),
                "center_x": center_x,
                "center_y": center_y,
                "width_px": width_px,
                "length_px": length_px,
                "angle": angle,
                "angle_error": self._vertical_angle_error(angle),
            })
            regular.append(merged)

        # 构造 edge 字段（供后续绘制与间距计算）
        for idx, r in enumerate(regular):
            box = self._box_from_center_angle(
                r["center_x"], r["center_y"],
                r["width_px"], r["length_px"], r["angle"],
            )
            ys = box[:, 1]
            top = int(max(0, np.min(ys)))
            bottom = int(min(h - 1, np.max(ys)))
            width_px = float(r["width_px"])
            center_x = float(r["center_x"])
            r["box"] = box
            r["edge"] = {
                "left_x": center_x - width_px / 2.0,
                "right_x": center_x + width_px / 2.0,
                "center_x": center_x,
                "width_px": width_px,
                "y_top": top,
                "y_bottom": bottom,
            }
            r["id"] = f"H{idx + 1}"

        return regular, clean, rejected

    # ------------------------------------------------------------------
    # 内部：主钢筋过滤（迁移自 V53 _filter_main_rebars_for_spacing）
    # ------------------------------------------------------------------

    @staticmethod
    def _center_x_at_y(rebar: dict, y_value: float) -> float:
        """计算钢筋在指定 y 处的中心 x（迁移自 V53 _center_x_at_y）。"""
        angle = float(rebar.get("angle", 90.0))
        theta = np.deg2rad(angle)
        sin_t = float(np.sin(theta))
        cos_t = float(np.cos(theta))
        if abs(sin_t) < 1e-3:
            return float(rebar["center_x"])
        return float(
            rebar["center_x"]
            + (float(y_value) - float(rebar["center_y"])) * cos_t / sin_t
        )

    def _filter_main_rebars_for_spacing(
        self, rebars: List[dict], img_h: int,
    ) -> Tuple[List[dict], int]:
        """过滤主钢筋用于间距计算（迁移自 V53 _filter_main_rebars_for_spacing）。

        :return: (main_rebars, measure_y)
        """
        if not rebars:
            return [], int(img_h * 0.50)

        rebars = [r for r in rebars if abs(float(r.get("angle_error", 0.0))) <= 18.0]
        if not rebars:
            return [], int(img_h * 0.50)

        lengths = np.asarray(
            [r["length_px"] for r in rebars], dtype=np.float32,
        )
        med_len = float(np.median(lengths))
        min_main_len = max(img_h * self._min_height_ratio_filter, med_len * 0.45)

        long_rebars = [r for r in rebars if r["length_px"] >= min_main_len]
        if not long_rebars:
            long_rebars = list(rebars)

        centers_y = np.asarray(
            [r["center_y"] for r in long_rebars], dtype=np.float32,
        )
        measure_y = int(round(float(np.median(centers_y))))
        measure_y = max(int(img_h * 0.18), min(int(img_h * 0.82), measure_y))

        margin = max(8, int(img_h * 0.015))
        min_crossing_len = max(img_h * 0.18, med_len * 0.25)
        main: List[dict] = []
        for r in rebars:
            if r["length_px"] < min_crossing_len:
                continue
            edge = r["edge"]
            if edge["y_top"] + margin <= measure_y <= edge["y_bottom"] - margin:
                main.append(r)

        if len(main) < 2:
            main = long_rebars

        # 重排并按 measure_y 处的中心 x 排序
        for r in main:
            width_px = float(r["width_px"])
            cx_at_y = self._center_x_at_y(r, measure_y)
            r["edge"]["center_x"] = cx_at_y
            r["edge"]["left_x"] = cx_at_y - width_px / 2.0
            r["edge"]["right_x"] = cx_at_y + width_px / 2.0

        main = sorted(main, key=lambda item: item["edge"]["center_x"])
        for idx, r in enumerate(main):
            r["id"] = f"H{idx + 1}"

        return main, measure_y

    # ------------------------------------------------------------------
    # 内部：按间距插补缺失钢筋（迁移自 V53 _insert_missing_rebars_by_gap）
    # ------------------------------------------------------------------

    def _insert_missing_rebars_by_gap(
        self, rebars: List[dict], img_h: int, measure_y: int,
    ) -> Tuple[List[dict], int]:
        """按中位间距插补缺失钢筋（迁移自 V53 _insert_missing_rebars_by_gap）。

        :return: (merged_rebars, inserted_count)
        """
        if len(rebars) < 4:
            return rebars, 0

        rebars = sorted(rebars, key=lambda item: item["edge"]["center_x"])
        center_gaps = np.asarray(
            [
                rebars[i]["edge"]["center_x"] - rebars[i - 1]["edge"]["center_x"]
                for i in range(1, len(rebars))
            ],
            dtype=np.float32,
        )
        center_gaps = center_gaps[center_gaps > 0]
        if len(center_gaps) < 3:
            return rebars, 0

        med_pitch = float(np.median(center_gaps))
        if med_pitch <= 1:
            return rebars, 0

        widths = np.asarray(
            [r["edge"]["width_px"] for r in rebars], dtype=np.float32,
        )
        width_px = float(np.median(widths))
        y_top = int(max(0, min(r["edge"]["y_top"] for r in rebars)))
        y_bottom = int(min(img_h - 1, max(r["edge"]["y_bottom"] for r in rebars)))

        inserted: List[dict] = []
        for i in range(1, len(rebars)):
            prev = rebars[i - 1]
            curr = rebars[i]
            prev_cx = float(prev["edge"]["center_x"])
            curr_cx = float(curr["edge"]["center_x"])
            gap = curr_cx - prev_cx
            ratio = gap / med_pitch
            if ratio < 1.55 or ratio > 3.10:
                continue

            missing_count = int(round(ratio)) - 1
            if missing_count <= 0 or missing_count > 2:
                continue

            step = gap / float(missing_count + 1)
            for k in range(missing_count):
                cx = prev_cx + step * float(k + 1)
                left = cx - width_px / 2.0
                right = cx + width_px / 2.0
                box = np.array(
                    [[left, y_top], [right, y_top],
                     [right, y_bottom], [left, y_bottom]],
                    dtype=np.float32,
                )
                angle = 90.0
                inserted.append({
                    "center_x": float(cx),
                    "center_y": float((y_top + y_bottom) / 2.0),
                    "width_px": float(width_px),
                    "length_px": float(y_bottom - y_top),
                    "angle": float(angle),
                    "angle_error": self._vertical_angle_error(float(angle)),
                    "box": box,
                    "edge": {
                        "left_x": float(left),
                        "right_x": float(right),
                        "center_x": float(cx),
                        "width_px": float(width_px),
                        "y_top": int(y_top),
                        "y_bottom": int(y_bottom),
                    },
                    "synthetic": True,
                })

        if not inserted:
            return rebars, 0

        merged = sorted(rebars + inserted, key=lambda item: item["edge"]["center_x"])
        for idx, r in enumerate(merged):
            r["id"] = f"H{idx + 1}"
        return merged, len(inserted)

    # ------------------------------------------------------------------
    # 内部：国标对齐（迁移自 V53 _normalize_rebar_diameter）
    # ------------------------------------------------------------------

    def _normalize_rebar_diameter(self, d_mm: float) -> int:
        """就近对齐国标规格（迁移自 V53 _normalize_rebar_diameter）。

        :return: 国标规格整数（mm）
        """
        idx = int(np.argmin(np.abs(_STANDARD_DIAMETERS_ARR - d_mm)))
        return int(_STANDARD_DIAMETERS_ARR[idx])

    # ------------------------------------------------------------------
    # 内部：标注图绘制
    # ------------------------------------------------------------------

    def _draw_annotated_frame(
        self,
        rgb_bgr: Optional[np.ndarray],
        label_mask: np.ndarray,
        rebars: List[dict],
        spacings_mm: List[float],
        measure_y: int,
        h: int,
        w: int,
    ) -> np.ndarray:
        """绘制标注图：原图 + 掩码叠加 + 直径/间距标注。

        :param rgb_bgr: 原始 BGR 帧；为 None 时用掩码生成灰度背景
        :return: BGR 标注图
        """
        if rgb_bgr is not None:
            vis = rgb_bgr.copy()
        else:
            # 无原图时：掩码灰度背景 + 纵向钢筋红色着色
            gray = cv2.cvtColor(label_mask * 80, cv2.COLOR_GRAY2BGR)
            vis = gray

        font_scale = max(0.42, w / 2100.0)
        text_thick = max(1, int(w / 1000))
        line_thick = max(2, int(w / 800))

        # 绘制每根钢筋 + 直径标签
        for r in rebars:
            self._draw_regular_rebar(vis, r, alpha=0.72)
            edge = r["edge"]
            std_mm = r.get("standard_mm")
            if std_mm is not None:
                label = f"D:{int(std_mm)}"
            else:
                label = f"D:{r['diameter_mm']:.1f}mm"
            tx = int(edge["center_x"])
            ty = max(24, min(h - 8, int(edge["y_top"] + 24)))
            (tw, _), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_thick + 1,
            )
            tx = max(4, min(w - tw - 4, tx - tw // 2))
            cv2.putText(
                vis, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                font_scale, (0, 255, 255), text_thick + 1, lineType=cv2.LINE_AA,
            )

        # 绘制相邻间距标注
        for i in range(1, len(rebars)):
            prev = rebars[i - 1]["edge"]
            curr = rebars[i]["edge"]
            spacing_mm = spacings_mm[i - 1] if i - 1 < len(spacings_mm) else None
            if spacing_mm is None:
                continue
            x1 = int(round(prev["right_x"]))
            x2 = int(round(curr["left_x"]))
            y_line = int(measure_y)
            if x2 <= x1:
                continue
            cv2.line(
                vis, (x1, y_line), (x2, y_line),
                (0, 255, 255), line_thick, lineType=cv2.LINE_AA,
            )
            cv2.circle(vis, (x1, y_line), line_thick + 2, (0, 0, 255), -1, lineType=cv2.LINE_AA)
            cv2.circle(vis, (x2, y_line), line_thick + 2, (0, 0, 255), -1, lineType=cv2.LINE_AA)

            gap_label = f"{int(round(spacing_mm))}mm"
            mid_x = (x1 + x2) // 2
            label_y = y_line - 18 if (i % 2 == 1) else y_line + 34
            label_y = max(28, min(h - 8, label_y))
            (gw, gh), _ = cv2.getTextSize(
                gap_label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_thick + 2,
            )
            gx = max(2, min(w - gw - 2, mid_x - gw // 2))
            pad = 4
            cv2.rectangle(
                vis, (gx - pad, label_y - gh - pad),
                (gx + gw + pad, label_y + pad), (255, 255, 255), -1,
            )
            cv2.rectangle(
                vis, (gx - pad, label_y - gh - pad),
                (gx + gw + pad, label_y + pad), (0, 0, 0), 1,
            )
            cv2.putText(
                vis, gap_label, (gx, label_y), cv2.FONT_HERSHEY_SIMPLEX,
                font_scale, (0, 0, 0), text_thick + 2, lineType=cv2.LINE_AA,
            )

        return vis

    def _draw_regular_rebar(self, vis: np.ndarray, rebar: dict, alpha: float = 0.72) -> None:
        """绘制单根规则化钢筋（红色半透明填充 + 绿色描边，迁移自 V53 _draw_regular_rebar）。"""
        overlay = vis.copy()
        box = np.int32(np.round(rebar["box"]))
        cv2.fillConvexPoly(overlay, box, (0, 0, 255), lineType=cv2.LINE_AA)
        cv2.addWeighted(overlay, alpha, vis, 1 - alpha, 0, dst=vis)
        cv2.polylines(vis, [box], True, (0, 255, 0), 2, lineType=cv2.LINE_AA)
