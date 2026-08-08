"""
推理器（RebarPredictor）：封装完整的「预处理 → GPU 前向 → 后处理」流水线。

设计要点：
  - 无状态设计：每次 infer() 独立调用，不缓存中间结果
  - 异常安全：内部捕获所有异常并返回 None，不抛出到 gRPC 框架
  - 与检测节点 状态机解耦：predictor 不关心状态转换，由 grpc_server 驱动

单次推理耗时预计 1~3 秒（Jetson Nano CUDA @ 640×640）。

接口契约（Step 2）：
  RebarPredictor(model_path, use_cuda) → infer(jpeg_bytes) → Optional[Tuple[ndarray, int, int]]
  返回 (mask, width, height) 或 None（推理失败）
"""

import io
import logging
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image

from inference.constants import (
    NUM_CLASSES,
    INPUT_SIZE,
    IMAGENET_MEAN,
    IMAGENET_STD,
    CLASS_NAMES,
)
from inference.model import UNetModel

logger = logging.getLogger("node_detect.inference.predictor")


class RebarPredictor:
    """
    UNet 钢筋分割推理器。

    职责：
      1. 启动时一次性加载权重到 GPU
      2. 接收 JPEG 字节流 → 解码 → 预处理 → CUDA 推理 → 后处理
      3. 输出 uint8 类别掩码（0/1/2）与原图尺寸
    """

    def __init__(self, model_path: str, use_cuda: bool = True) -> None:
        """
        初始化推理器。

        Args:
            model_path: 权重文件绝对路径（.pth）
            use_cuda: 是否使用 CUDA 推理。Jetson Nano 上必须为 True。

        Raises:
            FileNotFoundError: 权重文件不存在
            RuntimeError: CUDA 不可用但 use_cuda=True，或权重结构不匹配
        """
        self._model_path: str = model_path
        self._use_cuda: bool = use_cuda
        self._input_w: int = INPUT_SIZE[0]
        self._input_h: int = INPUT_SIZE[1]

        # ---- 设备初始化 ----
        if self._use_cuda and not torch.cuda.is_available():
            raise RuntimeError(
                f"配置启用 CUDA 但 torch.cuda.is_available()=False。"
                f"请检查 Jetson PyTorch 安装。"
            )
        self._device = torch.device("cuda" if self._use_cuda else "cpu")
        logger.info(f"推理设备: {self._device}")

        # ---- 模型加载 ----
        self._model = UNetModel(num_classes=NUM_CLASSES)
        self._degraded: bool = False       # 降级标志：模型权重部分加载失败时置 True
        self._degraded_reason: str = ""    # 降级原因（供日志与 journalctl 排查）
        try:
            state_dict = torch.load(self._model_path, map_location=self._device)
            state_dict = self._migrate_legacy_state_dict_keys(state_dict)
            # strict=False：允许迁移后仍有个别不匹配（人工接管源头修复前保持服务可启动）
            missing_keys, unexpected_keys = self._model.load_state_dict(
                state_dict, strict=False
            )
            if missing_keys:
                logger.warning(f"state_dict 缺 key: {missing_keys[:5]}{'...' if len(missing_keys) > 5 else ''}")
                self._degraded = True
                self._degraded_reason = f"缺 {len(missing_keys)} 个权重 key"
            if unexpected_keys:
                logger.warning(f"state_dict 多余 key（已忽略）: {unexpected_keys[:5]}{'...' if len(unexpected_keys) > 5 else ''}")
        except FileNotFoundError:
            logger.critical(f"权重文件未找到: {self._model_path}")
            raise
        except Exception as e:
            logger.error(f"模型加载异常（进入降级模式）: {e}", exc_info=True)
            self._degraded = True
            self._degraded_reason = f"load 异常: {type(e).__name__}: {e}"

        self._model.to(self._device)
        self._model.eval()
        if self._degraded:
            logger.warning(f"模型进入降级模式（仍可启动，推理请求将返回 None）: {self._degraded_reason}")
        else:
            logger.info(f"模型加载成功: {self._model_path} ({NUM_CLASSES} 类)")

    @staticmethod
    def _migrate_legacy_state_dict_keys(state_dict: dict) -> dict:
        """
        兼容迁移旧版权重（new-predict.py 训练的权重）到当前 model.py 命名。

        旧版命名：
          - up_concat{N}.conv1.{weight|bias}
          - up_concat{N}.conv2.{weight|bias}
          - up_conv.{1|3}.bias   (nn.Conv2d，带 bias=True)

        当前命名（model.py）：
          - up{N}.conv.{0|2}.weight   (nn.Sequential，Conv2d + ReLU + Conv2d + ReLU)
          - up_conv.{1|3}.weight       (同上，bias=False)

        处理规则：
          1. up_concat{N}.conv1.weight → up{N}.conv.0.weight
          2. up_concat{N}.conv2.weight → up{N}.conv.2.weight
          3. 含 bias 且目标层 bias=False 的 key 丢弃（load_state_dict strict=False 已兜底，此处显式丢弃更干净）
          4. up_conv.{1|3}.bias → 丢弃（当前模型 bias=False）
          5. 其他 key 保持不变（resnet encoder / bn / runtime keys 等）

        返回新 state_dict（原 dict 不修改，避免副作用）。
        """
        migrated = {}
        for key, value in state_dict.items():
            new_key = key

            # 规则 4：丢弃 up_conv 的 bias（新模型 bias=False）
            if new_key.startswith("up_conv.") and new_key.endswith(".bias"):
                logger.debug(f"[兼容迁移] 丢弃旧权重的 bias 项: {key}")
                continue

            # 规则 1/2：重命名 up_concatN.conv{M} → upN.conv.{idx}
            if new_key.startswith("up_concat"):
                # 提取 N：up_concat{N}.xxx → up{N}.xxx
                head, _, rest = new_key.partition(".")
                idx = head[len("up_concat"):]
                new_key = f"up{idx}.{rest}" if rest else f"up{idx}"

                # conv1 → conv.0，conv2 → conv.2
                if ".conv1." in new_key:
                    new_key = new_key.replace(".conv1.", ".conv.0.")
                elif ".conv2." in new_key:
                    new_key = new_key.replace(".conv2.", ".conv.2.")

                # 含 bias 的 up_concatN 项在新模型中无对应（bias=False），丢弃
                if new_key.endswith(".bias"):
                    logger.debug(f"[兼容迁移] 丢弃旧权重的 bias 项: {key} → up{idx}")
                    continue

            migrated[new_key] = value

        logger.info(
            f"[兼容迁移] 旧权重重命名完成："
            f"原始 {len(state_dict)} key → 迁移后 {len(migrated)} key "
            f"（丢弃 {len(state_dict) - len(migrated)} 个 bias / 不匹配 key）"
        )
        return migrated

    def infer(self, jpeg_bytes: bytes) -> Optional[Tuple[np.ndarray, int, int]]:
        """
        执行单帧推理（完整流水线）。

        Args:
            jpeg_bytes: JPEG 编码的 RGB 图像字节流

        Returns:
            成功: (mask, width, height)
              - mask: uint8 ndarray, shape=(height, width), 取值 0/1/2
              - width: 原图宽度（像素）
              - height: 原图高度（像素）
            失败: None（异常已记录到日志）
        """
        # 降级模式（weights 不兼容）：直接返回 None，日志记录原因（人工在 journalctl 查看）
        if self._degraded:
            logger.warning(
                f"[降级模式] 拒绝推理请求：{self._degraded_reason} | "
                f"需人工对齐 UnetModel 结构与权重后重新加载"
            )
            return None

        t_start = time.time()
        try:
            # 1. 解码 JPEG
            image = self._decode_jpeg(jpeg_bytes)
            if image is None:
                return None

            orig_w, orig_h = image.size
            logger.debug(f"输入图像尺寸: {orig_w}x{orig_h}")

            # 2. 预处理 → [1, 3, 640, 640] 张量
            tensor, nw, nh = self._preprocess(image)

            # 3. CUDA 前向推理
            with torch.no_grad():
                tensor = tensor.to(self._device)
                output = self._model(tensor)  # [1, num_classes, H_out, W_out]

            # 4. 后处理 → uint8 掩码
            mask = self._postprocess(output, nw, nh, orig_w, orig_h)

            # 5. 类别分布统计（DEBUG 级别）
            unique, counts = np.unique(mask, return_counts=True)
            for cls_id, cnt in zip(unique, counts):
                logger.debug(f"类别 {cls_id}（{CLASS_NAMES[cls_id]}）: {cnt} px")

            elapsed_ms = (time.time() - t_start) * 1000
            logger.info(
                f"推理完成: mask={orig_w}x{orig_h}, 耗时={elapsed_ms:.1f}ms"
            )
            return mask, orig_w, orig_h

        except Exception as e:
            logger.error(f"推理异常: {e}", exc_info=True)
            return None

    # ============================================================
    # 私有方法
    # ============================================================

    @staticmethod
    def _decode_jpeg(jpeg_bytes: bytes) -> Optional[Image.Image]:
        """
        解码 JPEG 字节流为 PIL Image（RGB）。

        Returns:
            PIL Image 或 None（解码失败）
        """
        try:
            buf = io.BytesIO(jpeg_bytes)
            image = Image.open(buf).convert("RGB")
            return image
        except Exception as e:
            logger.error(f"JPEG 解码失败: {e}")
            return None

    def _preprocess(self, image: Image.Image) -> Tuple[torch.Tensor, int, int]:
        """
        预处理流水线：
        1. Letterbox 缩放装入 640×640（灰边填充，画面不变形）
        2. 归一化到 [0, 1] + ImageNet 均值/方差标准化
        3. 转 [1, 3, 640, 640] 张量

        Returns:
            (tensor, nw, nh) — nw/nh 为缩放后实际图像区域尺寸（不含灰边）
        """
        target_w, target_h = self._input_w, self._input_h

        # Letterbox 缩放（保持宽高比）
        orig_w, orig_h = image.size
        scale = min(target_w / orig_w, target_h / orig_h)
        nw = int(orig_w * scale)
        nh = int(orig_h * scale)
        resized = image.resize((nw, nh), Image.BICUBIC)

        # 灰边填充至目标尺寸（灰色 RGB=128）
        new_image = Image.new("RGB", (target_w, target_h), (128, 128, 128))
        offset_x = (target_w - nw) // 2
        offset_y = (target_h - nh) // 2
        new_image.paste(resized, (offset_x, offset_y))

        # numpy → 归一化 → ImageNet 标准化 → 转张量
        img_array = np.array(new_image, dtype=np.float32) / 255.0  # [0, 1]
        mean = np.array(IMAGENET_MEAN, dtype=np.float32)
        std = np.array(IMAGENET_STD, dtype=np.float32)
        img_array = (img_array - mean) / std  # ImageNet 标准化

        # [H, W, C] → [C, H, W] → [1, C, H, W]
        tensor = torch.from_numpy(img_array.transpose(2, 0, 1)).unsqueeze(0)
        return tensor, nw, nh

    @staticmethod
    def _postprocess(
        output: torch.Tensor,
        nw: int, nh: int,
        orig_w: int, orig_h: int
    ) -> np.ndarray:
        """
        后处理流水线：
        1. softmax → 概率图
        2. 裁剪灰边区域（保留实际图像区域）
        3. 插值放大回原图尺寸
        4. argmax → 类别掩码（uint8, 取值 0/1/2）

        Args:
            output: [1, C, H_out, W_out] logits 张量（CUDA 或 CPU）
            nw, nh: letterbox 缩放后图像区域尺寸
            orig_w, orig_h: 原图尺寸

        Returns:
            uint8 数组, shape=(orig_h, orig_w), 取值 0/1/2
        """
        # Softmax → 概率图
        prob = F.softmax(output, dim=1)  # [1, C, H_out, W_out]
        prob = prob.squeeze(0).permute(1, 2, 0).cpu().numpy()  # [H_out, W_out, C]

        out_h, out_w = prob.shape[:2]

        # 裁剪灰边区域（与 letterbox 填充参数一致）
        offset_x = (out_w - nw) // 2
        offset_y = (out_h - nh) // 2
        crop = prob[offset_y: offset_y + nh, offset_x: offset_x + nw, :]

        # 插值回原图尺寸
        mask_prob = cv2.resize(crop, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)

        # argmax → 类别掩码
        mask = mask_prob.argmax(axis=-1).astype(np.uint8)  # [orig_h, orig_w]
        return mask

    @property
    def is_ready(self) -> bool:
        """推理器是否就绪（模型已加载且未处于降级模式）"""
        return hasattr(self, '_model') and self._model is not None and not self._degraded
