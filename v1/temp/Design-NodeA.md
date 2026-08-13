# 节点 A 设计方案 — AI 推理节点（Jetson Nano）

> 本文档是节点 A（AI 推理节点）的详细设计方案，面向开发者与 AI 编码助手。
> 上层架构规范见项目根目录 `AGENTS.md`，本文档仅包含节点 A 专属的设计细节。

> **⚠️ 项目阶段边界声明：本项目当前为"原型技术验证 Demo"阶段。**
> - 阶段目标仅为**验证端到端技术链路可行**（采集 → 推理 → 测量 → 显示）
> - 精度/延迟指标、流式推理、TensorRT/FP16、自动化测试等 **C 类要素本阶段一律不引入**
> - 本文档所有设计约定以"最快打通链路、可演示"为优先原则

---

## 目录

1. [节点职责与运行环境](#1-节点职责与运行环境)
2. [工程目录结构](#2-工程目录结构)
3. [核心模块与类设计](#3-核心模块与类设计)
4. [并发与线程模型](#4-并发与线程模型)
5. [配置加载与启动自检流程](#5-配置加载与启动自检流程)
6. [日志与异常处理策略](#6-日志与异常处理策略)

---

## 1. 节点职责与运行环境

### 1.1 节点定位

节点 A 是**专用 AI 计算节点**，在整个系统中承担唯一的职责：**接收节点 B 发送的 RGB 图像，执行 UNet 语义分割推理，返回类别掩码**。

```
[节点B 主控] ──JPEG 帧──→ [节点A AI推理] ──PNG 掩码──→ [节点B 主控]
                              │
                         UNet (ResNet50)
                         CUDA @ 640×640
```

### 1.2 职责边界（强制约束）

| 职责 | 是否归属节点 A | 说明 |
|------|---------------|------|
| UNet 模型加载与前向推理 | **是** | 节点 A 唯一核心职责 |
| gRPC 服务端（接收请求、返回响应） | **是** | 通信层服务 |
| 启停与环境自检 | **是** | 启动门控 |
| 物理测量换算（直径/间距） | **否** | 属于节点 B `measure` 模块 |
| 视频采集（336L 相机） | **否** | 属于节点 B `camera` 模块 |
| 激光测距数据采集 | **否** | 属于节点 B `serial` 模块 |
| GUI 渲染与用户交互 | **否** | 属于节点 B `ui` 模块 |
| 深度图（Depth）处理 | **否** | 原型阶段不接入 |

**设计理由**：测量换算在数据使用方（节点 B）执行，节点 A 无需持有相机内参与传感器状态，保持无状态、可替换。gRPC 请求中携带的 `camera_distance_mm` 仅随帧记录（用于日志追溯与帧同步校验），不参与任何计算。

### 1.3 硬件环境

| 属性 | 说明 |
|------|------|
| 硬件 | NVIDIA Jetson Nano（Maxwell 架构 128 CUDA core，4GB 共享显存） |
| 操作系统 | Ubuntu 22.04 LTS（NVIDIA Jetson 官方定制内核，原生兼容 CUDA 生态） |
| CUDA 版本 | JetPack 自带 CUDA（通常 10.2+），需实测确认 |
| Python 版本 | Python 3.8+（Jetson 官方镜像自带） |
| PyTorch | Jetson 专用 wheel 安装（NVIDIA 官方提供，需对应 JetPack 版本） |

### 1.4 性能约束

- **4GB 共享显存**：CPU/GPU 共享内存，模型加载后余量有限，节点 A **不应再承担其他服务**
- **推理模式**：按需单次推理（触发间隔 >=10s），不承诺逐帧实时
- **单帧耗时**：ResNet50-UNet @ 640x640 on CUDA，预计 1~3 秒（Jetson Nano 算力有限）
- **模型权重**：`logs/Unet_resnet50.pth`，约 100MB+ 量级，启动时一次性加载

### 1.5 触发模式

原型阶段采用**按需触发**：用户确认测量时由节点 B 发送单次推理请求。任意相邻两次推理请求的间隔不少于 10 秒，不做逐帧连续推理。

> 理由：Jetson Nano 算力有限，逐帧推理无法保证帧率且会持续占满 GPU；>=10 秒的间隔预算下，单帧推理为突发负载，双方都有充裕时间完成。

---

## 2. 工程目录结构

```
DEMO/
├── AGENTS.md                          # 项目总体架构规范（双节点共享）
├── Design-NodeA.md                    # 本文档 — 节点 A 设计方案
├── node_a/                            # 节点 A 工程根目录
│   ├── __init__.py
│   ├── main.py                        # 入口：启动自检 → 启动 gRPC 服务 → 阻塞等待
│   ├── config/                        # 配置文件（节点 A 专属副本）
│   │   ├── network.json               # 网络参数：local_ip, remote_ip, grpc_port
│   │   └── inference.json             # 推理参数：model_path, input_shape, cuda
│   ├── proto/                         # Protocol Buffers 定义与生成桩代码
│   │   ├── rebar_inference.proto     # gRPC 服务与消息定义
│   │   ├── rebar_inference_pb2.py    # protoc 生成的消息类
│   │   └── rebar_inference_pb2_grpc.py  # protoc 生成的服务桩
│   ├── inference/                     # 推理核心模块
│   │   ├── __init__.py
│   │   ├── model.py                   # UNet 模型定义（从 new-predict.py 迁移）
│   │   ├── predictor.py               # 推理器：预处理 → 前向 → 后处理
│   │   └── constants.py               # 分割类别常量、ImageNet 均值/方差
│   ├── grpc_server/                   # gRPC 服务端模块
│   │   ├── __init__.py
│   │   ├── servicer.py                # RebarInferenceServicer 实现
│   │   └── server_factory.py          # gRPC server 创建与生命周期管理
│   ├── system/                        # 系统级模块
│   │   ├── __init__.py
│   │   ├── config_loader.py           # JSON 配置文件加载与校验
│   │   ├── self_check.py              # 启动环境自检（IP / 端口 / CUDA）
│   │   └── logger.py                  # 全局日志初始化（一机一日志）
│   ├── logs/                          # 日志输出目录
│   │   └── node_a.log                 # 节点 A 全局日志文件
│   └── weights/                       # 模型权重目录
│       └── .gitkeep                   # 部署时人工放入 Unet_resnet50.pth
├── node_b/                            # 节点 B 工程根目录（独立设计，不在本文档范围）
├── shared/                            # 共享常量与工具（双节点共用）
│   └── constants.py                   # 帧格式常量、国标规格表等
└── requirements_node_a.txt            # 节点 A 专属依赖清单
```

### 目录设计说明

- `inference/`、`grpc_server/`、`system/` 严格按 AGENTS.md §3.1 的模块命名
- `proto/` 包含 `.proto` 源文件与生成的 `_pb2.py`、`_pb2_grpc.py`，确保双节点使用同一份 proto
- `config/` 仅放节点 A 需要的配置（网络 + 推理），不放相机/内参配置（属节点 B）
- `weights/` 目录在代码仓库中仅保留 `.gitkeep`，模型权重不入代码目录（见 AGENTS.md §8.4）
- 共享常量（帧格式、国标规格等）放 `shared/` 目录，通过 `sys.path` 或包引用

---

## 3. 核心模块与类设计

### 3.1 `inference/constants.py` — 分割类别与预处理常量

```python
"""
推理常量定义：分割类别、ImageNet 标准化参数。
单一来源，节点 A 与节点 B 共享同一份定义。
"""

from typing import Tuple, List

# 分割类别定义
CLASS_BACKGROUND: int = 0      # 背景
CLASS_REBAR_VERTICAL: int = 1  # 纵向钢筋
CLASS_REBAR_HORIZONTAL: int = 2  # 横向钢筋

NUM_CLASSES: int = 3

# 类别名称（用于日志与调试）
CLASS_NAMES: List[str] = ["背景", "纵向钢筋", "横向钢筋"]

# 推理输入尺寸（宽, 高）
INPUT_SIZE: Tuple[int, int] = (640, 640)

# ImageNet 均值与方差（预处理标准化用）
IMAGENET_MEAN: Tuple[float, float, float] = (0.485, 0.456, 0.406)
IMAGENET_STD: Tuple[float, float, float] = (0.229, 0.224, 0.225)
```

> **与现有代码的差异**：`new-predict.py` 的 `preprocess_input` 仅做 `/255.0`，缺少 ImageNet 均值/方差标准化。新实现必须补充标准化步骤，否则与训练时的预处理不一致，导致推理精度下降。

### 3.2 `inference/model.py` — UNet 模型定义

**设计决策**：从 `new-predict.py` 整体迁移网络定义代码（`BasicBlock`、`BottleNeck`、`ResNet`、`UNetModel` 等），不做结构修改，仅做以下调整：

1. 删除 `VGG16` 相关代码（仅保留 `resnet50` backbone，减少代码量）
2. 删除 `onnx` 导出相关逻辑（原型阶段不需要）
3. 删除打印配置、颜色映射等可视化相关代码（节点 A 不做可视化）

```python
"""
UNet（ResNet50 backbone）模型定义。
从 new-predict.py 迁移，仅保留 ResNet50 backbone 推理所需的最小网络结构。
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Tuple

from inference.constants import NUM_CLASSES


def conv3x3(in_planes: int, out_planes: int, stride: int = 1,
            groups: int = 1, dilation: int = 1) -> nn.Conv2d:
    return nn.Conv2d(in_planes, out_planes, kernel_size=3, stride=stride,
                     padding=dilation, groups=groups, bias=False, dilation=dilation)


def conv1x1(in_planes: int, out_planes: int, stride: int = 1) -> nn.Conv2d:
    return nn.Conv2d(in_planes, out_planes, kernel_size=1, stride=stride, bias=False)


class Bottleneck(nn.Module):
    expansion: int = 4

    def __init__(self, inplanes: int, planes: int, stride: int = 1,
                 downsample: nn.Module = None) -> None:
        super().__init__()
        self.conv1 = conv1x1(inplanes, planes)
        self.bn1 = nn.BatchNorm2d(planes)
        self.conv2 = conv3x3(planes, planes, stride)
        self.bn2 = nn.BatchNorm2d(planes)
        self.conv3 = conv1x1(planes, planes * self.expansion)
        self.bn3 = nn.BatchNorm2d(planes * self.expansion)
        self.relu = nn.ReLU(inplace=True)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        identity = x
        out = self.relu(self.bn1(self.conv1(x)))
        out = self.relu(self.bn2(self.conv2(out)))
        out = self.bn3(self.conv3(out))
        if self.downsample is not None:
            identity = self.downsample(x)
        return self.relu(out + identity)


class ResNet(nn.Module):
    """ResNet50 backbone：输出 4 级特征图供 UNet 编码器使用。"""

    def __init__(self, block: nn.Module, layers: list) -> None:
        super().__init__()
        self.inplanes = 64
        self.conv1 = nn.Conv2d(3, 64, kernel_size=7, stride=2, padding=3, bias=False)
        self.bn1 = nn.BatchNorm2d(64)
        self.relu = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        self.layer1 = self._make_layer(block, 64, layers[0])    # 输出 1/4
        self.layer2 = self._make_layer(block, 128, layers[1], stride=2)  # 1/8
        self.layer3 = self._make_layer(block, 256, layers[2], stride=2)  # 1/16
        self.layer4 = self._make_layer(block, 512, layers[3], stride=2)  # 1/32

    def _make_layer(self, block: nn.Module, planes: int,
                    blocks: int, stride: int = 1) -> nn.Sequential:
        downsample = None
        if stride != 1 or self.inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                conv1x1(self.inplanes, planes * block.expansion, stride),
                nn.BatchNorm2d(planes * block.expansion),
            )
        layers = [block(self.inplanes, planes, stride, downsample)]
        self.inplanes = planes * block.expansion
        for _ in range(1, blocks):
            layers.append(block(self.inplanes, planes))
        return nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, ...]:
        x = self.relu(self.bn1(self.conv1(x)))
        x = self.maxpool(x)
        feat1 = self.layer1(x)   # 256 channels
        feat2 = self.layer2(feat1)  # 512 channels
        feat3 = self.layer3(feat2)  # 1024 channels
        feat4 = self.layer4(feat3)  # 2048 channels
        return feat1, feat2, feat3, feat4


class UNetUp(nn.Module):
    """UNet 解码器上采样块：拼接 + 两次卷积。"""

    def __init__(self, in_size: int, out_size: int) -> None:
        super().__init__()
        self.up = nn.UpsamplingBilinear2d(scale_factor=2)
        self.conv = nn.Sequential(
            conv3x3(in_size, out_size),
            nn.ReLU(inplace=True),
            conv3x3(out_size, out_size),
            nn.ReLU(inplace=True),
        )

    def forward(self, low: torch.Tensor, high: torch.Tensor) -> torch.Tensor:
        up = self.up(high)
        # 尺寸对齐（处理奇偶差异）
        if up.shape[2:] != low.shape[2:]:
            up = F.interpolate(up, size=low.shape[2:], mode='bilinear', align_corners=False)
        return self.conv(torch.cat([low, up], dim=1))


class UNetModel(nn.Module):
    """UNet 分割网络：ResNet50 encoder + UNet decoder。"""

    def __init__(self, num_classes: int = NUM_CLASSES) -> None:
        super().__init__()
        self.resnet = ResNet(Bottleneck, [3, 4, 6, 3])  # ResNet50
        in_filters = [256, 512, 1024, 2048]  # ResNet50 4 级特征通道数
        out_filters = [64, 128, 256, 512]
        self.up4 = UNetUp(in_filters[3] + out_filters[3], out_filters[3])
        self.up3 = UNetUp(in_filters[2] + out_filters[3], out_filters[2])
        self.up2 = UNetUp(in_filters[1] + out_filters[2], out_filters[1])
        self.up1 = UNetUp(in_filters[0] + out_filters[1], out_filters[0])
        self.up_conv = nn.Sequential(
            nn.UpsamplingBilinear2d(scale_factor=2),
            conv3x3(out_filters[0], out_filters[0]),
            nn.ReLU(inplace=True),
            conv3x3(out_filters[0], out_filters[0]),
            nn.ReLU(inplace=True),
        )
        self.final = nn.Conv2d(out_filters[0], num_classes, kernel_size=1)

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        feat1, feat2, feat3, feat4 = self.resnet(inputs)
        up4 = self.up4(feat3, feat4)
        up3 = self.up3(feat2, up4)
        up2 = self.up2(feat1, up3)
        up1 = self.up1(feat1, up2)
        up1 = self.up_conv(up1)
        return self.final(up1)
```

> **注意**：上述 `UNetUp` 与 `UNetModel` 的实现与原 `new-predict.py` 有细微差异。原版的 `unetUp` 输入通道数标注为 `[192, 512, 1024, 3072]` 与实际 ResNet50 输出不一致——这里已修正为 ResNet50 实际通道数。迁移时需逐层核对通道维度，确保拼接操作维度对齐。

### 3.3 `inference/predictor.py` — 推理器

这是节点 A 的核心推理类，封装完整的「预处理 → GPU 推扫 → 后处理」流水线。

```python
"""
推理器：封装 UNet 推理的完整流水线。
单次推理耗时预计 1~3 秒（Jetson Nano CUDA）。
"""

import io
import logging
from typing import Optional, Tuple

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image

from inference.model import UNetModel
from inference.constants import (
    NUM_CLASSES, INPUT_SIZE, IMAGENET_MEAN, IMAGENET_STD, CLASS_NAMES
)

logger = logging.getLogger("node_a.inference")


class RebarPredictor:
    """
    UNet 钢筋分割推理器。

    职责：加载权重 → 预处理 JPEG 帧 → CUDA 推理 → 后处理 → 输出 uint8 掩码。
    做无状态设计：每次 infer() 独立，不缓存中间结果。
    """

    def __init__(self, model_path: str, use_cuda: bool = True) -> None:
        """
        初始化推理器。

        Args:
            model_path: 权重文件绝对路径，如 "/home/jetson/node_a/weights/Unet_resnet50.pth"
            use_cuda: 是否使用 CUDA 推理。Jetson Nano 上必须为 True。

        Raises:
            FileNotFoundError: 权重文件不存在
            RuntimeError: CUDA 不可用但 use_cuda=True
        """
        self._model_path: str = model_path
        self._use_cuda: bool = use_cuda
        self._input_size: Tuple[int, int] = INPUT_SIZE  # (640, 640)

        # --- 初始化设备 ---
        if self._use_cuda and not torch.cuda.is_available():
            raise RuntimeError(
                f"配置启用 CUDA 但 torch.cuda.is_available()=False。"
                f"请检查 Jetson PyTorch 安装。"
            )
        self._device = torch.device("cuda" if self._use_cuda else "cpu")
        logger.info(f"推理设备: {self._device}")

        # --- 加载模型 ---
        self._model = UNetModel(num_classes=NUM_CLASSES)
        try:
            state_dict = torch.load(self._model_path, map_location=self._device)
            self._model.load_state_dict(state_dict)
        except FileNotFoundError:
            logger.error(f"权重文件未找到: {self._model_path}")
            raise
        except RuntimeError as e:
            logger.error(f"权重加载失败（可能结构不匹配）: {e}")
            raise

        self._model.to(self._device)
        self._model.eval()
        logger.info(f"模型加载成功: {self._model_path}")

    def infer(self, jpeg_bytes: bytes) -> Optional[Tuple[np.ndarray, int, int]]:
        """
        执行单帧推理。

        Args:
            jpeg_bytes: JPEG 编码的 RGB 图像字节流（来自节点 B）。

        Returns:
            (mask, width, height) 三元组：
                - mask: uint8 数组，取值 0/1/2，shape=(height, width)
                - width: 原图宽度
                - height: 原图高度
            推理失败返回 None。
        """
        try:
            # 1. 解码 JPEG → PIL Image
            image = self._decode_jpeg(jpeg_bytes)
            if image is None:
                return None

            orig_w, orig_h = image.size
            logger.debug(f"输入图像尺寸: {orig_w}x{orig_h}")

            # 2. 预处理：letterbox 缩放 + ImageNet 标准化 → [1,3,640,640] 张量
            tensor, nw, nh = self._preprocess(image)

            # 3. 前向推理
            with torch.no_grad():
                tensor = tensor.to(self._device)
                output = self._model(tensor)   # [1, 3, H, W] logits

            # 4. 后处理：softmax → 裁剪灰边 → 插值回原图 → argmax → uint8 mask
            mask = self._postprocess(output, nw, nh, orig_w, orig_h)

            # 5. 统计类别分布（DEBUG 级别日志）
            unique, counts = np.unique(mask, return_counts=True)
            for cls_id, cnt in zip(unique, counts):
                logger.debug(f"类别 {cls_id}（{CLASS_NAMES[cls_id]}）: {cnt} px")

            return mask, orig_w, orig_h

        except Exception as e:
            logger.error(f"推理异常: {e}", exc_info=True)
            return None

    # ---------------- 私有方法 ----------------

    @staticmethod
    def _decode_jpeg(jpeg_bytes: bytes) -> Optional[Image.Image]:
        """解码 JPEG 字节流为 PIL Image（RGB）。"""
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
        1. Letterbox 缩放装入 640×640（灰边填充）
        2. 归一化到 [0,1] + ImageNet 均值/方差标准化
        3. 转 [1, 3, 640, 640] 张量

        Returns:
            (tensor, nw, nh) — nw/nh 为缩放后实际图像区域（不含灰边），供后处理裁剪用。
        """
        target_w, target_h = self._input_size

        # Letterbox 缩放
        orig_w, orig_h = image.size
        scale = min(target_w / orig_w, target_h / orig_h)
        nw = int(orig_w * scale)
        nh = int(orig_h * scale)
        resized = image.resize((nw, nh), Image.BICUBIC)

        # 灰边填充至目标尺寸
        new_image = Image.new("RGB", (target_w, target_h), (128, 128, 128))
        new_image.paste(resized, ((target_w - nw) // 2, (target_h - nh) // 2))

        # numpy → 归一化 → ImageNet 标准化 → 张量
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
        1. Softmax → 概率图
        2. 裁剪灰边区域（保留实际图像区域）
        3. 插值放大回原图尺寸
        4. argmax → 类别掩码

        Returns:
            uint8 数组，shape=(orig_h, orig_w)，取值 0/1/2。
        """
        # output: [1, C, H, W] → softmax → [H, W, C]
        prob = F.softmax(output, dim=1)  # [1, C, H, W]
        prob = prob.squeeze(0).permute(1, 2, 0).cpu().numpy()  # [H, W, C]

        inp_h, inp_w = prob.shape[:2]  # 640, 640

        # 裁剪灰边区域
        top = (inp_h - nh) // 2
        left = (inp_w - nw) // 2
        crop = prob[top: top + nh, left: left + nw, :]

        # 插值回原图尺寸
        mask_prob = cv2.resize(crop, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)

        # argmax → 类别
        mask = mask_prob.argmax(axis=-1).astype(np.uint8)  # [orig_h, orig_w]
        return mask

    @property
    def is_ready(self) -> bool:
        """推理器是否就绪（模型已加载）。"""
        return hasattr(self, '_model') and self._model is not None
```

### 3.4 `grpc_server/servicer.py` — gRPC 服务实现

```python
"""
gRPC 服务实现：RebarInference 的 Infer 方法。

请求-响应流程：
  1. 解析 InferRequest（JPEG 帧 + frame_id + timestamp + distance）
  2. 调用 RebarPredictor.infer() 获取掩码
  3. 将掩码编码为 PNG 字节
  4. 构造 InferResponse 返回
"""

import logging
import time
from typing import Optional

import cv2
import numpy as np

from inference.predictor import RebarPredictor

# 生成的桩代码（需先 protoc 编译）
from proto import rebar_inference_pb2 as pb2
from proto import rebar_inference_pb2_grpc import RebarInferenceServicer

logger = logging.getLogger("node_a.grpc_server")


class RebarInferenceServicer(RebarInferenceServicer):
    """
    RebarInference gRPC 服务实现。

    每个 Infer 请求独立处理，节点 A 不做请求排队或批处理。
    节点 B 端已在 gRPC channel 层设置 deadline，超时由 gRPC 框架自动取消。
    """

    def __init__(self, predictor: RebarPredictor) -> None:
        """
        Args:
            predictor: 已就绪的 RebarPredictor 实例。
        """
        super().__init__()
        self._predictor = predictor

    def Infer(self, request: pb2.InferRequest, context) -> pb2.InferResponse:
        """
        执行单帧推理并返回分割掩码。

        流程伪代码：
        ```
        接收请求:
            frame_id   = request.frame_id
            timestamp  = request.timestamp_ms
            distance   = request.camera_distance_mm   # 仅记录，不参与计算
            jpeg_bytes = request.image

        日志记录: frame_id, timestamp, distance（随帧绑定追溯）

        result = predictor.infer(jpeg_bytes)
        if result is None:
            return error (掩码为空)

        mask, width, height = result
        png_bytes = encode_png(mask)

        return InferResponse(
            label_mask = png_bytes,
            frame_id   = frame_id,       # 回传请求 frame_id
            timestamp_ms = timestamp,    # 回传请求时间戳
            width  = width,
            height = height,
        )
        ```

        错误处理：
        - 推理失败：记录 ERROR 日志，返回空掩码（width=0, height=0）
        - 不抛出异常到 gRPC 框架（避免连接中断），以服务端内部错误码处理
        """
        t_start = time.time()
        frame_id = request.frame_id
        timestamp_ms = request.timestamp_ms
        distance_mm = request.camera_distance_mm

        logger.info(
            f"收到推理请求: frame_id={frame_id}, timestamp={timestamp_ms}ms, "
            f"distance={distance_mm}mm, image_size={len(request.image)} bytes"
        )

        # 记录距离标量（不参与计算，仅供追溯与帧同步校验）
        logger.debug(f"帧 {frame_id} 绑定的激光测距值: {distance_mm}mm")

        # 执行推理
        result: Optional[tuple] = self._predictor.infer(request.image)

        if result is None:
            logger.error(f"推理失败: frame_id={frame_id}，返回空响应")
            return pb2.InferResponse(
                label_mask=b"",
                frame_id=frame_id,
                timestamp_ms=timestamp_ms,
                width=0,
                height=0,
            )

        mask, width, height = result

        # 编码掩码为 PNG 字节
        try:
            success, png_bytes = cv2.imencode('.png', mask)
            if not success:
                raise RuntimeError("cv2.imencode PNG 失败")
            png_data = png_bytes.tobytes()
        except Exception as e:
            logger.error(f"掩码 PNG 编码失败: frame_id={frame_id}, error={e}")
            return pb2.InferResponse(
                label_mask=b"",
                frame_id=frame_id,
                timestamp_ms=timestamp_ms,
                width=0,
                height=0,
            )

        elapsed_ms = (time.time() - t_start) * 1000
        logger.info(
            f"推理完成: frame_id={frame_id}, "
            f"mask={width}x{height}, png_size={len(png_data)} bytes, "
            f"耗时={elapsed_ms:.1f}ms"
        )

        return pb2.InferResponse(
            label_mask=png_data,
            frame_id=frame_id,
            timestamp_ms=timestamp_ms,
            width=width,
            height=height,
        )
```

### 3.5 `grpc_server/server_factory.py` — gRPC 服务端生命周期

```python
"""
gRPC 服务端创建与生命周期管理。

单线程同步服务器（无流式调用），使用 ThreadPoolExecutor
处理并发请求（虽然触发间隔 >=10s，理论上不会出现并发，
但线程池提供基本的并发隔离）。
"""

import logging
from concurrent import futures
from typing import Optional

import grpc

from grpc_server.servicer import RebarInferenceServicer
from inference.predictor import RebarPredictor

# 生成的桩代码
from proto import rebar_inference_pb2_grpc as pb2_grpc

logger = logging.getLogger("node_a.grpc_server")


class GRPCServer:
    """
    gRPC 服务端封装。

    生命周期：create() → start() → wait_for_termination() → stop()
    """

    def __init__(
        self,
        predictor: RebarPredictor,
        listen_address: str,   # 如 "0.0.0.0:50051"
        max_workers: int = 4,  # 线程池大小（原型阶段 4 足够）
    ) -> None:
        """
        Args:
            predictor: 已就绪的 RebarPredictor 实例。
            listen_address: 监听地址，格式 "IP:PORT" 或 "0.0.0.0:PORT"。
            max_workers: 线程池工作线程数。
        """
        self._predictor = predictor
        self._listen_address = listen_address
        self._max_workers = max_workers
        self._server: Optional[grpc.Server] = None

    def start(self) -> None:
        """
        创建并启动 gRPC 服务。

        伪代码：
        ```
        servicer = RebarInferenceServicer(predictor)
        server = grpc.server(ThreadPoolExecutor(max_workers))
        add_servicer_to_server(servicer, server)
        server.add_insecure_port(listen_address)
        server.start()
        log: "gRPC 服务已启动，监听 {listen_address}"
        ```
        """
        servicer = RebarInferenceServicer(self._predictor)
        self._server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=self._max_workers)
        )
        pb2_grpc.add_RebarInferenceServicer_to_server(servicer, self._server)
        self._server.add_insecure_port(self._listen_address)
        self._server.start()
        logger.info(f"gRPC 服务已启动，监听 {self._listen_address}")

    def wait_for_termination(self) -> None:
        """阻塞当前线程，直到服务终止。"""
        if self._server:
            self._server.wait_for_termination()

    def stop(self, grace: float = 5.0) -> None:
        """
        优雅停止 gRPC 服务。

        Args:
            grace: 等待现有 RPC 完成的宽限期（秒）。
        """
        if self._server:
            self._server.stop(grace)
            logger.info("gRPC 服务已停止")
```

### 3.6 `system/config_loader.py` — 配置加载

```python
"""
JSON 配置文件加载与校验。

读取 config/network.json 与 config/inference.json，
合并为统一的 AppConfig 数据对象。
"""

import json
import logging
import os
from dataclasses import dataclass, field
from typing import Any, Dict

logger = logging.getLogger("node_a.system")


@dataclass
class AppConfig:
    """节点 A 完整配置。"""

    # network.json 字段
    local_ip: str = ""
    remote_ip: str = ""
    grpc_port: int = 50051

    # inference.json 字段
    model_path: str = ""
    input_width: int = 640
    input_height: int = 640
    use_cuda: bool = True

    # 日志配置
    log_dir: str = "logs"
    log_level: str = "INFO"
    log_file: str = "node_a.log"


def load_config(config_dir: str) -> AppConfig:
    """
    加载并校验配置文件。

    Args:
        config_dir: 配置目录绝对路径。

    Returns:
        填充好的 AppConfig 实例。

    Raises:
        FileNotFoundError: 配置文件不存在
        KeyError: 缺少必要字段
        ValueError: 字段值无效（如端口号超出范围）

    伪代码：
    ```
    network_conf = read_json(config_dir/network.json)
    inference_conf = read_json(config_dir/inference.json)

    merge into AppConfig

    validate:
        - grpc_port in (1024, 65535)
        - input_width > 0 and input_height > 0
        - use_cuda is bool
        - local_ip is valid IPv4 format

    return config
    """
    network_path = os.path.join(config_dir, "network.json")
    inference_path = os.path.join(config_dir, "inference.json")

    if not os.path.isfile(network_path):
        raise FileNotFoundError(f"网络配置文件不存在: {network_path}")
    if not os.path.isfile(inference_path):
        raise FileNotFoundError(f"推理配置文件不存在: {inference_path}")

    with open(network_path, "r", encoding="utf-8") as f:
        net_conf: Dict[str, Any] = json.load(f)
    with open(inference_path, "r", encoding="utf-8") as f:
        inf_conf: Dict[str, Any] = json.load(f)

    config = AppConfig(
        local_ip=net_conf["local_ip"],
        remote_ip=net_conf["remote_ip"],
        grpc_port=int(net_conf["grpc_port"]),
        model_path=inf_conf["model_path"],
        input_width=int(inf_conf.get("input_width", 640)),
        input_height=int(inf_conf.get("input_height", 640)),
        use_cuda=bool(inf_conf.get("use_cuda", True)),
        log_dir=inf_conf.get("log_dir", "logs"),
        log_level=inf_conf.get("log_level", "INFO"),
        log_file=inf_conf.get("log_file", "node_a.log"),
    )

    # 校验
    if not (1024 <= config.grpc_port <= 65535):
        raise ValueError(f"gRPC 端口号无效: {config.grpc_port}（有效范围 1024-65535）")
    if config.input_width <= 0 or config.input_height <= 0:
        raise ValueError(f"输入尺寸无效: {config.input_width}x{config.input_height}")
    if not _is_valid_ipv4(config.local_ip):
        raise ValueError(f"本机 IP 格式无效: {config.local_ip}")
    if not _is_valid_ipv4(config.remote_ip):
        raise ValueError(f"对端 IP 格式无效: {config.remote_ip}")

    # 检查权重文件是否存在
    if not os.path.isfile(config.model_path):
        raise FileNotFoundError(f"模型权重文件不存在: {config.model_path}")

    logger.info(f"配置加载成功: local_ip={config.local_ip}, "
                f"grpc_port={config.grpc_port}, model={config.model_path}")
    return config


def _is_valid_ipv4(ip: str) -> bool:
    """简单的 IPv4 格式校验。"""
    parts = ip.split(".")
    if len(parts) != 4:
        return False
    try:
        return all(0 <= int(p) <= 255 for p in parts)
    except ValueError:
        return False
```

### 3.7 `system/self_check.py` — 启动环境自检

```python
"""
启动环境自检。

在 gRPC 服务启动前同步执行，不通过则安全退出进程。
检查项：本机 IP 一致性、端口可用性、CUDA 可用性、模型权重可加载性。
"""

import logging
import os
import socket
import subprocess
import sys
from typing import Tuple

import torch

from system.config_loader import AppConfig

logger = logging.getLogger("node_a.system")


class SelfCheckError(Exception):
    """自检失败异常，携带失败原因。"""
    pass


def run_self_check(config: AppConfig) -> Tuple[bool, str]:
    """
    执行完整的启动环境自检。

    Args:
        config: 已加载的应用配置。

    Returns:
        (passed, message) — passed=True 表示全部通过。

    Raises:
        SelfCheckError: 硬故障（IP 不匹配 / 端口占用 / CUDA 不可用），需安全退出。

    伪代码：
    ```
    passed = True
    messages = []

    # 检查 1：本机 IP 校验
    actual_ip = get_local_ip(config.local_ip)
    if actual_ip != config.local_ip:
        raise SelfCheckError(f"本机 IP 不匹配: 期望 {config.local_ip}, 实际 {actual_ip}")

    # 检查 2：端口可用性
    if not check_port_available(config.grpc_port):
        raise SelfCheckError(f"端口 {config.grpc_port} 已被占用")

    # 检查 3：CUDA 可用性（如果 use_cuda=True）
    if config.use_cuda:
        if not torch.cuda.is_available():
            raise SelfCheckError("CUDA 不可用但配置要求启用 CUDA")
        log GPU info: name, memory

    # 检查 4：模型权重文件可加载性（仅检查文件存在与可读，不实际加载）
    if not os.access(config.model_path, os.R_OK):
        raise SelfCheckError(f"模型权重文件不可读: {config.model_path}")

    return True
    """
    messages = []

    # === 检查 1：本机 IP 校验 ===
    logger.info("自检: 本机 IP 校验...")
    try:
        actual_ip = _get_local_ip_for(config.local_ip)
        if actual_ip != config.local_ip:
            msg = (f"本机 IP 不匹配: 配置文件为 {config.local_ip}, "
                   f"实际网卡 IP 为 {actual_ip}。请检查网络配置脚本是否已执行。")
            logger.error(msg)
            raise SelfCheckError(msg)
        logger.info(f"本机 IP 校验通过: {actual_ip}")
        messages.append(f"本机 IP: {actual_ip} ✓")
    except SelfCheckError:
        raise
    except Exception as e:
        msg = f"本机 IP 校验异常: {e}"
        logger.error(msg)
        raise SelfCheckError(msg)

    # === 检查 2：端口可用性 ===
    logger.info("自检: 端口可用性...")
    if not _check_port_available("0.0.0.0", config.grpc_port):
        msg = f"gRPC 端口 {config.grpc_port} 已被占用，无法启动服务。"
        logger.error(msg)
        raise SelfCheckError(msg)
    logger.info(f"端口 {config.grpc_port} 可用")
    messages.append(f"端口 {config.grpc_port} ✓")

    # === 检查 3：CUDA 可用性 ===
    if config.use_cuda:
        logger.info("自检: CUDA 可用性...")
        if not torch.cuda.is_available():
            msg = "配置要求启用 CUDA，但 torch.cuda.is_available()=False。请检查 JetPy Torch 安装。"
            logger.error(msg)
            raise SelfCheckError(msg)
        gpu_name = torch.cuda.get_device_name(0)
        gpu_mem = torch.cuda.get_device_properties(0).total_mem / (1024 ** 3)
        logger.info(f"CUDA 可用: {gpu_name}, 显存 {gpu_mem:.1f} GB")
        messages.append(f"CUDA: {gpu_name} ({gpu_mem:.1f} GB) ✓")
    else:
        logger.warning("CUDA 已禁用，推理将使用 CPU（性能严重下降）")
        messages.append("CUDA: 已禁用（CPU 模式）")

    # === 检查 4：模型权重可加载性 ===
    logger.info("自检: 模型权重文件...")
    if not os.access(config.model_path, os.R_OK):
        msg = f"模型权重文件不可读: {config.model_path}"
        logger.error(msg)
        raise SelfCheckError(msg)
    file_size_mb = os.path.getsize(config.model_path) / (1024 * 1024)
    logger.info(f"模型权重: {config.model_path} ({file_size_mb:.1f} MB)")
    messages.append(f"模型权重: {os.path.basename(config.model_path)} ({file_size_mb:.1f} MB) ✓")

    return True, "; ".join(messages)


def _get_local_ip_for(expected_ip: str) -> str:
    """
    获取本机与 expected_ip 所在网卡的 IPv4 地址。

    跨平台实现：
    - Linux: 使用 `hostname -I` 或 `ip addr` 解析
    - 通用 fallback: UDP 连接法（不发送数据，仅获取出口 IP）
    """
    # UDP 连接法（最通用）：向目标 IP 发 0 字节 UDP 获取出口 IP
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((expected_ip, 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception:
        pass

    # Linux fallback: hostname -I
    try:
        result = subprocess.run(
            ["hostname", "-I"], capture_output=True, text=True, timeout=5
        )
        ips = result.stdout.strip().split()
        if expected_ip in ips:
            return expected_ip
        return ips[0] if ips else ""
    except Exception:
        return ""


def _check_port_available(host: str, port: int) -> bool:
    """检查指定端口是否可用（未被占用）。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2)
        s.bind((host, port))
        s.close()
        return True
    except OSError:
        return False
```

### 3.8 `system/logger.py` — 全局日志初始化

```python
"""
全局日志初始化（一机一日志）。

节点 A 所有模块统一写入 logs/node_a.log。
格式：时间戳(毫秒) | 级别 | 节点 | 模块 | 消息
"""

import logging
import os
from logging.handlers import RotatingFileHandler
from typing import Optional


NODE_NAME: str = "A"


def init_logger(
    log_dir: str,
    log_file: str,
    log_level: str = "INFO",
    max_bytes: int = 10 * 1024 * 1024,  # 10 MB
    backup_count: int = 5,
) -> None:
    """
    初始化节点 A 全局日志。

    配置：
    - 文件输出：RotatingFileHandler，自动轮转
    - 控制台输出：StreamHandler（开发调试用）
    - 统一格式：%(asctime)s | %(levelname)s | A | %(name)s | %(message)s

    Args:
        log_dir: 日志目录路径。
        log_file: 日志文件名（如 "node_a.log"）。
        log_level: 日志级别字符串（DEBUG/INFO/WARNING/ERROR/CRITICAL）。
        max_bytes: 单个日志文件最大字节数。
        backup_count: 保留的历史日志文件数。
    """
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, log_file)

    # 根日志器
    root_logger = logging.getLogger("node_a")
    root_logger.setLevel(getattr(logging, log_level.upper(), logging.INFO))

    # 避免重复初始化（追加 handler）
    if root_logger.handlers:
        root_logger.handlers.clear()

    # 格式
    formatter = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d | %(levelname)s | %(node)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    # 添加自定义过滤器注入 NODE 字段
    class NodeFilter(logging.Filter):
        def filter(self, record: logging.LogRecord) -> bool:
            record.node = NODE_NAME
            return True

    node_filter = NodeFilter()

    # 文件 handler（轮转）
    file_handler = RotatingFileHandler(
        log_path, maxBytes=max_bytes, backupCount=backup_count,
        encoding="utf-8"
    )
    file_handler.setFormatter(formatter)
    file_handler.addFilter(node_filter)
    root_logger.addHandler(file_handler)

    # 控制台 handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    console_handler.addFilter(node_filter)
    root_logger.addHandler(console_handler)

    root_logger.info(f"日志系统初始化完成: {log_path}")


def get_logger(module_name: str) -> logging.Logger:
    """
    获取模块级日志器。

    Args:
        module_name: 模块名，如 "inference.predictor"、"grpc_server.servicer"、"system.self_check"。

    Returns:
        logging.Logger 实例，日志名称为 "node_a.{module_name}"。
    """
    return logging.getLogger(f"node_a.{module_name}")
```

### 3.9 `main.py` — 入口

```python
"""
节点 A 主入口。

启动流程：
1. 加载配置
2. 初始化日志
3. 执行环境自检（失败则安全退出）
4. 加载模型
5. 启动 gRPC 服务
6. 阻塞等待终止信号
"""

import logging
import os
import signal
import sys

from system.config_loader import load_config
from system.logger import init_logger
from system.self_check import run_self_check, SelfCheckError
from inference.predictor import RebarPredictor
from grpc_server.server_factory import GRPCServer

logger = logging.getLogger("node_a.main")


def main() -> None:
    # --- 确定工程根目录 ---
    project_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(project_dir, "config")

    # --- 1. 加载配置 ---
    try:
        config = load_config(config_dir)
    except (FileNotFoundError, KeyError, ValueError) as e:
        print(f"[FATAL] 配置加载失败: {e}")
        sys.exit(1)

    # --- 2. 初始化日志 ---
    init_logger(
        log_dir=os.path.join(project_dir, config.log_dir),
        log_file=config.log_file,
        log_level=config.log_level,
    )

    logger.info("=" * 60)
    logger.info("节点 A（AI 推理节点）启动中...")
    logger.info("=" * 60)

    # --- 3. 环境自检 ---
    try:
        passed, check_msg = run_self_check(config)
        logger.info(f"环境自检通过: {check_msg}")
    except SelfCheckError as e:
        logger.critical(f"环境自检失败，安全退出: {e}")
        sys.exit(1)

    # --- 4. 加载模型 ---
    try:
        predictor = RebarPredictor(
            model_path=config.model_path,
            use_cuda=config.use_cuda,
        )
    except (FileNotFoundError, RuntimeError) as e:
        logger.critical(f"模型加载失败，安全退出: {e}")
        sys.exit(1)

    # --- 5. 启动 gRPC 服务 ---
    listen_addr = f"0.0.0.0:{config.grpc_port}"
    grpc_server = GRPCServer(
        predictor=predictor,
        listen_address=listen_addr,
        max_workers=4,
    )

    grpc_server.start()
    logger.info(f"节点 A 已就绪，等待节点 B 连接...")

    # --- 6. 注册信号处理，优雅退出 ---
    def _signal_handler(signum, frame):
        logger.info(f"收到信号 {signum}，正在停止服务...")
        grpc_server.stop(grace=5.0)
        logger.info("节点 A 已退出")
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # --- 7. 阻塞 ---
    grpc_server.wait_for_termination()


if __name__ == "__main__":
    main()
```

### 3.10 `proto/rebar_inference.proto` — gRPC 协议定义

```proto
syntax = "proto3";

package rebar.inference;

// 钢筋分割推理服务
service RebarInference {
  rpc Infer(InferRequest) returns (InferResponse);
}

message InferRequest {
  bytes image = 1;                 // JPEG 编码的 RGB 帧
  uint32 frame_id = 2;             // 帧序号（单调递增），帧同步主键
  int64 timestamp_ms = 3;          // 采集时刻时间戳（毫秒），帧同步辅助校验
  double camera_distance_mm = 4;   // 拍摄距离标量（激光测距值），随帧绑定
}

message InferResponse {
  bytes label_mask = 1;     // PNG 编码的单通道 0/1/2 类别掩码
  uint32 frame_id = 2;      // 回传请求 frame_id，帧同步主键
  int64 timestamp_ms = 3;   // 回传请求时间戳，供帧同步校验
  uint32 width = 4;         // 掩码宽（与原图一致）
  uint32 height = 5;        // 掩码高（与原图一致）
}
```

> 格式选择理由：图像用 JPEG（千兆局域网内降低传输量），掩码用 PNG（3 类掩码近乎二值图，PNG 无损且压缩率极高）。

---

## 4. 并发与线程模型

### 4.1 总体策略

节点 A 采用**基于线程的并发模型**（非 asyncio），与节点 B 一致。

由于触发间隔 >=10s，实际上不存在高并发推理请求。gRPC 框架使用 `ThreadPoolExecutor(max_workers=4)` 作为 RPC 执行线程池，提供基本的并发隔离。

### 4.2 线程模型图

```
主线程 (main.py)
  │
  ├── 加载配置
  ├── 初始化日志
  ├── 环境自检
  ├── 加载模型 (一次性，阻塞)
  │
  ├── gRPC Server 线程池 (ThreadPoolExecutor, 4 workers)
  │     ├── Worker-1 ──→ Servicer.Infer() ──→ RebarPredictor.infer()
  │     ├── Worker-2 ──→ Servicer.Infer() ──→ RebarPredictor.infer()
  │     ├── Worker-3 ──→ Servicer.Infer() ──→ RebarPredictor.infer()
  │     └── Worker-4 ──→ Servicer.Infer() ──→ RebarPredictor.infer()
  │
  └── wait_for_termination() (阻塞主线程)
```

### 4.3 锁策略

| 资源 | 是否需要锁 | 说明 |
|------|-----------|------|
| PyTorch 模型 | **否** | 推理为只读操作，`model.eval()` + `torch.no_grad()` 保证线程安全 |
| RebarPredictor 内部状态 | **否** | 无状态设计，每次 infer() 局部变量独立 |
| 日志写入 | **否** | Python `logging` 模块自带线程安全 |
| CUDA stream | **否** | PyTorch 默认 stream 自动管理 |

> 由于 PyTorch 模型推理是只读操作，且每次请求处理都使用局部变量（无共享可变状态），**不需要额外的互斥锁**。这是节点 A 相对简单的设计优势。

### 4.4 GPU 并发考虑

单 GPU 环境下，多个 worker 线程并发调用 `model(input)` 会导致 CUDA kernel 交错执行，实际吞吐不增反降。但考虑到触发间隔 >=10s，实际不会出现并发推理。

若后续出现并发需求（原型阶段不做），可考虑：
- 推理请求队列 + 单推理线程串行处理（C 类要素）

### 4.5 资源释放

```python
# 优雅退出时的资源释放顺序
def graceful_shutdown():
    grpc_server.stop(grace=5.0)    # ① 停止接收新请求，等待现有 RPC 完成（最多 5 秒）
    # PyTorch 模型随进程退出由 OS 回收 GPU 显存
    logger.info("节点 A 已退出")
```

---

## 5. 配置加载与启动自检流程

### 5.1 配置文件清单

#### `config/network.json`

```json
{
  "local_ip": "192.168.10.2",
  "remote_ip": "192.168.10.1",
  "grpc_port": 50051
}
```

#### `config/inference.json`

```json
{
  "model_path": "/home/jetson/node_a/weights/Unet_resnet50.pth",
  "input_width": 640,
  "input_height": 640,
  "use_cuda": true,
  "log_dir": "logs",
  "log_level": "INFO",
  "log_file": "node_a.log"
}
```

### 5.2 启动流程（完整）

```
┌─────────────────────────────────────────────────┐
│  main.py 启动                                    │
├─────────────────────────────────────────────────┤
│                                                  │
│  ① 加载配置 (load_config)                        │
│     ├── 读取 config/network.json                  │
│     ├── 读取 config/inference.json                │
│     ├── 合并 AppConfig                            │
│     └── 校验字段（端口号范围、IP 格式、尺寸正值）   │
│                                                  │
│  ② 初始化日志 (init_logger)                       │
│     ├── 创建 logs/ 目录                           │
│     ├── RotatingFileHandler → logs/node_a.log     │
│     ├── StreamHandler → 控制台                    │
│     └── 格式: 时间戳 | 级别 | A | 模块 | 消息      │
│                                                  │
│  ③ 环境自检 (run_self_check)                      │
│     ├── 本机 IP 校验 ──→ 失败: CRITICAL + exit(1)  │
│     ├── 端口可用性 ──→ 失败: CRITICAL + exit(1)    │
│     ├── CUDA 可用性 ──→ 失败: CRITICAL + exit(1)   │
│     └── 模型权重可读性 ──→ 失败: CRITICAL + exit(1)│
│                                                  │
│  ④ 加载模型 (RebarPredictor.__init__)             │
│     ├── 创建 UNetModel 实例                       │
│     ├── torch.load(state_dict)                    │
│     ├── model.to(device)                          │
│     └── model.eval()                              │
│                                                  │
│  ⑤ 启动 gRPC 服务 (GRPCServer)                    │
│     ├── ThreadPoolExecutor(4)                     │
│     ├── add_servicer_to_server                    │
│     ├── add_insecure_port("0.0.0.0:50051")        │
│     └── server.start()                            │
│                                                  │
│  ⑥ 注册信号处理                                   │
│     ├── SIGINT → graceful_shutdown                │
│     └── SIGTERM → graceful_shutdown               │
│                                                  │
│  ⑦ wait_for_termination() ← 主线程阻塞于此        │
│                                                  │
└─────────────────────────────────────────────────┘
```

### 5.3 自检失败处理策略

| 检查项 | 失败类型 | 处理方式 | 说明 |
|--------|---------|---------|------|
| 本机 IP 不匹配 | **硬故障** | `CRITICAL` 日志 + `sys.exit(1)` | 网络配置脚本未执行 |
| 端口被占用 | **硬故障** | `CRITICAL` 日志 + `sys.exit(1)` | 旧进程未退出 |
| CUDA 不可用 | **硬故障** | `CRITICAL` 日志 + `sys.exit(1)` | Jetson PyTorch 未正确安装 |
| 模型权重不可读 | **硬故障** | `CRITICAL` 日志 + `sys.exit(1)` | 权重文件缺失或权限不足 |

> 节点 A 没有"降级模式"概念；自检失败一律安全退出。降级运行是节点 B 独有的逻辑（节点 B 对端不可达时可降级为仅视频+激光）。

### 5.4 Windows 开发调试兼容

> 节点 A 运行在 Jetson Nano (Linux) 上，不涉及 Windows。Windows 仅作为节点 B 上位机开发环境。
> 若在 Windows 上临时启动节点 A 进行联调测试，IP 校验应提供 bypass 选项：

```python
# config/self_check.json（可选）
{
  "skip_ip_check": false   // Windows 调试时设为 true
}
```

---

## 6. 日志与异常处理策略

### 6.1 日志规范遵循

节点 A 严格遵循 AGENTS.md §7.5 的一机一日志规范：

- **日志文件**：`logs/node_a.log`
- **日志轮转**：单文件最大 10MB，保留 5 个历史文件
- **格式**：`时间戳(毫秒) | 级别 | 节点 | 模块 | 消息`
- **示例**：

```
2026-08-01 10:30:15.456 | INFO | A | system.self_check | 环境自检通过: 本机 IP: 192.168.10.2; 端口 50051; CUDA: NVIDIA Tegra (3.9 GB); 模型权重: Unet_resnet50.pth (102.3 MB)
2026-08-01 10:30:16.789 | INFO | A | grpc_server.server_factory | gRPC 服务已启动，监听 0.0.0.0:50051
2026-08-01 10:35:22.123 | INFO | A | grpc_server.servicer | 收到推理请求: frame_id=42, timestamp=1722494122123ms, distance=812.0mm, image_size=125840 bytes
2026-08-01 10:35:24.456 | INFO | A | grpc_server.servicer | 推理完成: frame_id=42, mask=1280x720, png_size=45230 bytes, 耗时=2333.3ms
```

- **模块命名空间**：`inference.predictor`、`grpc_server.servicer`、`grpc_server.server_factory`、`system.self_check`、`system.config_loader`、`main`

### 6.2 日志级别使用

| 级别 | 使用场景 |
|------|---------|
| `DEBUG` | 推理中间过程（类别像素统计、预处理尺寸信息） |
| `INFO` | 服务启停、推理请求/响应、自检通过 |
| `WARNING` | 非预期但可恢复的情况（如 iPad 模式的 CPU 推理降级） |
| `ERROR` | 推理失败、编码失败、JPEG 解码失败 |
| `CRITICAL` | 自检失败、模型加载失败、需退出进程 |

### 6.3 异常处理策略

| 发生位置 | 异常类型 | 处理方式 | 说明 |
|----------|---------|---------|------|
| `config_loader.py` | `FileNotFoundError` / `KeyError` / `ValueError` | 向上抛至 `main.py`，`exit(1)` | 配置错误即硬故障 |
| `self_check.py` | `SelfCheckError` | 向上抛至 `main.py`，`exit(1)` | 自检失败即硬故障 |
| `predictor.py.infer()` | 任意异常 | 捕获 → 记录 `ERROR` 日志（含 traceback） → 返回 `None` | 推理失败不崩溃，由 gRPC Servicer 构造空响应 |
| `servicer.py.Infer()` | `predictor.infer()` 返回 `None` | 记录 `ERROR` 日志 → 返回 `label_mask=b"", width=0, height=0` | 节点 B 侧检测到 width=0 即判为推理失败 |
| `grpc_server.py` | `RuntimeError`（端口绑定失败） | 记录 `CRITICAL` 日志 → `exit(1)` | 服务启动失败 |

### 6.4 关键设计原则

1. **推理异常不扩散至 gRPC 框架**：`RebarPredictor.infer()` 内部捕获所有异常并返回 `None`，`Servicer.Infer()` 捕获 `None` 后构造空响应，**不抛出 `grpc.RpcError`**。理由：推理失败是业务层面的错误，不应导致 gRPC 连接中断或框架级异常。

2. **超时由节点 B 侧控制**：节点 A 不自行设置推理超时。节点 B gRPC 客户端设置 deadline，超时后自动取消调用；节点 A 端正在执行的推理不受影响（Jetson 算力有限，强行中断可能导致 CUDA 上下文异常）。

3. **不重试**：节点 A 不做重试逻辑。若推理失败，由节点 B 决定是否重新发送请求（可能需要等待下一个测量周期）。

4. **异常日志必须含上下文**：每条 ERROR/CRITICAL 日志必须携带 `frame_id`，便于跨节点与节点 B 的日志对齐排查。

### 6.5 日志轮转策略

```python
RotatingFileHandler(
    "logs/node_a.log",
    maxBytes=10 * 1024 * 1024,  # 10 MB
    backupCount=5,                # 保留 5 个历史文件
    encoding="utf-8"
)
# 最大磁盘占用: 10MB × (1 + 5) = 60MB
```

### 6.6 健康日志输出

为便于排查推理性能问题，每条推理完成日志输出耗时信息：

```
2026-08-01 10:35:24.456 | INFO | A | grpc_server.servicer | 推理完成: frame_id=42, mask=1280x720, png_size=45230 bytes, 耗时=2333.3ms
```

节点 B 侧可据此对比：
- 若推理耗时持续超过 5 秒，说明 Jetson Nano 算力可能受温度/功耗限制降频
- 若推理耗时突然跳变，可能是 GPU 内存不足触发 OOM

---

## 附录 A：从 `new-predict.py` 迁移清单

| `new-predict.py` 组件 | 迁移至 | 说明 |
|----------------------|-------|------|
| `resize_image()` | `predictor.py._preprocess()` | 保留 letterbox 逻辑，补充 ImageNet 标准化 |
| `cvtColor()` | `predictor.py._decode_jpeg()` | 边界处理，确保 3 通道 RGB |
| `preprocess_input()` | `predictor.py._preprocess()` | **仅做了 `/255.0`，需补充 ImageNet 均值/方差标准化** |
| `Unet._defaults` 中的参数 | `config/inference.json` | 配置化 |
| `Unet.generate()` | `RebarPredictor.__init__()` | 权重加载逻辑 |
| `Unet.detect_label()` | `RebarPredictor.infer()` | 推理核心逻辑（需去掉 `detect_image` 可视化部分） |
| `Unet.detect_image()` | **不迁移** | 可视化分割节点 B 负责 |
| `UNetModel`（ResNet50 版） | `model.py` | 仅保留 ResNet50 backbone 路径 |
| `BasicBlock` | **删除** | ResNet50 用 Bottleneck |
| `Bottleneck` → 重命名 | `model.py.Bottleneck` | 保留 |
| `ResNet` | `model.py.ResNet` | 保留 |
| `unetUp` | `model.py.UNetUp` | 保留，注意通道数核对 |
| `RebarMeasureV53` | **不迁移** | 属于节点 B |

## 附录 B：依赖清单 (`requirements_node_a.txt`)

```
# gRPC
grpcio>=1.50.0
grpcio-tools>=1.50.0
protobuf>=4.21.0

# 深度学习（Jetson 专用 wheel 需单独安装，不在此列）
# torch — 通过 NVIDIA 官方 Jetson PyTorch wheel 安装
# torchvision — 与 torch 版本匹配

# 图像处理
opencv-python-headless>=4.6.0
Pillow>=9.0.0
numpy>=1.21.0

# 工具
tqdm>=4.60.0  # 仅用于启动时的加载进度条（可选）
```

> **注意**：Jetson Nano 上的 PyTorch 必须通过 NVIDIA 官方提供的 wheel 安装（`pip` 安装的是 x86 版本，在 ARM 上不可用）。

## 附录 C：已知风险与缓解

| 风险 | 影响 | 缓解措施 |
|------|------|---------|
| Jetson Nano 4GB 共享显存 OOM | 推理崩溃 | 启动时仅加载推理服务，不启动其他进程 |
| 长时间推理导致温度降频 | 推理变慢 | 推理间隔 >=10s 留有散热余量 |
| gRPC Jetson 侧 `grpcio` 版本不兼容 | 服务启动失败 | 测试时使用与 CPU 架构匹配的 whl |
| weight 文件与 model 结构不匹配 | 加载时 RuntimeError | `load_state_dict(strict=True)` + 捕获异常 + 安全退出 |
| ImageNet 标准化缺失（与训练不一致） | 分割精度下降 | 实现 `_preprocess()` 时强制加入均值/方差标准化 |
