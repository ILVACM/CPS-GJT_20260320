"""
UNet（ResNet50 backbone）分割模型定义。

从 temp/new-predict.py 迁移，仅保留 ResNet50 backbone 推理所需的最小网络结构。

关键修正（对齐 Design-AI_detect.md §3.2）：
  1. in_filters 修正为 ResNet50 实际输出通道数 [256, 512, 1024, 2048]
     （原版错误标注为 [192, 512, 1024, 3072]）
  2. ResNet forward 仅返回 4 级特征（feat1-4），不返回 avgpool 后的全局特征
  3. 删除 VGG16、DataParallel、onnx 导出、可视化相关代码
  4. 删除 self.backbone 分支判断，硬编码 ResNet50 路径

通道维度核对清单（ResNet50 + UNet decoder）：
  ResNet50 encoder:
    conv1+bn1+relu+maxpool: 3 → 64 (stride 4)
    layer1 (feat1): 64 → 256  (1/4 分辨率)
    layer2 (feat2): 256 → 512 (1/8)
    layer3 (feat3): 512 → 1024 (1/16)
    layer4 (feat4): 1024 → 2048 (1/32)

  UNet decoder（拼接方式: cat(low, up(high))）：
    up4: cat(feat3[1024], up(feat4)[2048]) = 3072 → out 512
    up3: cat(feat2[512], up(up4)[512]) = 1024 → out 256
    up2: cat(feat1[256], up(up3)[256]) = 512 → out 128
    up1: cat(feat1[256], up(up2)[128]) = 384 → out 64
    up_conv: 64 → 64 (upsample ×2)
    final: 64 → num_classes (1×1 conv)
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Tuple

from inference.constants import NUM_CLASSES


# ============================================================
# 基础模块
# ============================================================


def conv3x3(in_planes: int, out_planes: int, stride: int = 1,
            groups: int = 1, dilation: int = 1) -> nn.Conv2d:
    """3×3 卷积（带 dilation 支持）"""
    return nn.Conv2d(in_planes, out_planes, kernel_size=3, stride=stride,
                     padding=dilation, groups=groups, bias=False, dilation=dilation)


def conv1x1(in_planes: int, out_planes: int, stride: int = 1) -> nn.Conv2d:
    """1×1 卷积（通道变换）"""
    return nn.Conv2d(in_planes, out_planes, kernel_size=1, stride=stride, bias=False)


class Bottleneck(nn.Module):
    """
    ResNet50 Bottleneck 块。

    结构：1×1(降维) → 3×3(空间) → 1×1(升维) + 残差
    输出通道数 = planes × 4（expansion = 4）
    """

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


# ============================================================
# ResNet50 编码器
# ============================================================


class ResNet(nn.Module):
    """
    ResNet50 backbone：输出 4 级特征图（feat1-4）供 UNet 解码器使用。

    与标准 ResNet50 的区别：
      - 删除 avgpool 和 fc 分类头
      - forward 仅返回 4 级特征（不返回全局池化后的特征向量）
    """

    def __init__(self, block: nn.Module = Bottleneck, layers: list = None) -> None:
        super().__init__()
        if layers is None:
            layers = [3, 4, 6, 3]  # ResNet50

        self.inplanes = 64

        # 初始卷积 + 池化（stride=4 总降采样）
        self.conv1 = nn.Conv2d(3, 64, kernel_size=7, stride=2, padding=3, bias=False)
        self.bn1 = nn.BatchNorm2d(64)
        self.relu = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)

        # 4 个残差层
        self.layer1 = self._make_layer(block, 64, layers[0])    # 输出 1/4, 256ch
        self.layer2 = self._make_layer(block, 128, layers[1], stride=2)   # 1/8, 512ch
        self.layer3 = self._make_layer(block, 256, layers[2], stride=2)   # 1/16, 1024ch
        self.layer4 = self._make_layer(block, 512, layers[3], stride=2)   # 1/32, 2048ch

    def _make_layer(self, block: nn.Module, planes: int,
                    blocks: int, stride: int = 1) -> nn.Sequential:
        downsample = None
        if stride != 1 or self.inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                conv1x1(self.inplanes, planes * block.expansion, stride),
                nn.BatchNorm2d(planes * block.expansion),
            )

        layers_list = [block(self.inplanes, planes, stride, downsample)]
        self.inplanes = planes * block.expansion
        for _ in range(1, blocks):
            layers_list.append(block(self.inplanes, planes))
        return nn.Sequential(*layers_list)

    def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, ...]:
        """
        前向传播，返回 4 级特征图。

        Returns:
            (feat1, feat2, feat3, feat4)
            - feat1: [B, 256, H/4, W/4]
            - feat2: [B, 512, H/8, W/8]
            - feat3: [B, 1024, H/16, W/16]
            - feat4: [B, 2048, H/32, W/32]
        """
        x = self.relu(self.bn1(self.conv1(x)))
        x = self.maxpool(x)

        feat1 = self.layer1(x)   # 256 channels
        feat2 = self.layer2(feat1)  # 512 channels
        feat3 = self.layer3(feat2)  # 1024 channels
        feat4 = self.layer4(feat3)  # 2048 channels

        return feat1, feat2, feat3, feat4


# ============================================================
# UNet 解码器
# ============================================================


class UNetUp(nn.Module):
    """
    UNet 解码器上采样块。

    拼接（低层特征 + 上层上采样特征）→ 两次 3×3 卷积
    """

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
        """
        上采样并拼接。

        Args:
            low: 低层高分辨率特征（encoder 端）
            high: 低分辨率高层特征（待上采样）

        Returns:
            融合后的特征 [B, out_size, H_low, W_low]
        """
        up = self.up(high)
        # 尺寸对齐（处理奇偶差异）
        if up.shape[2:] != low.shape[2:]:
            up = F.interpolate(up, size=low.shape[2:], mode='bilinear', align_corners=False)
        return self.conv(torch.cat([low, up], dim=1))


# ============================================================
# UNet 完整模型
# ============================================================


class UNetModel(nn.Module):
    """
    UNet 分割网络：ResNet50 encoder + UNet decoder。

    输入：[B, 3, H, W]（RGB，会被预处理为 640×640）
    输出：[B, num_classes, H, W]（logits，未 softmax）
    """

    def __init__(self, num_classes: int = NUM_CLASSES) -> None:
        super().__init__()
        self.resnet = ResNet(Bottleneck, [3, 4, 6, 3])

        # ResNet50 4 级特征通道数（实际输出，与原版 new-predict.py 不同）
        in_filters = [256, 512, 1024, 2048]
        out_filters = [64, 128, 256, 512]

        # 解码器：
        # 拼接逻辑 cat(low, up(high)) — low 为 encoder 端特征，high 为 decoder 端特征
        # up4: cat(feat3[1024], up(feat4)[2048]) = 3072 → 512
        # up3: cat(feat2[512], up(up4)[512]) = 1024 → 256
        # up2: cat(feat1[256], up(up3)[256]) = 512 → 128
        # up1: cat(feat1[256], up(up2)[128]) = 384 → 64
        self.up4 = UNetUp(in_filters[2] + in_filters[3], out_filters[3])  # 1024+2048=3072 → 512
        self.up3 = UNetUp(in_filters[1] + out_filters[3], out_filters[2])  # 512+512=1024 → 256
        self.up2 = UNetUp(in_filters[0] + out_filters[2], out_filters[1])  # 256+256=512 → 128
        self.up1 = UNetUp(in_filters[0] + out_filters[1], out_filters[0])  # 256+128=384 → 64

        # 最终上采样 + 分类头
        self.up_conv = nn.Sequential(
            nn.UpsamplingBilinear2d(scale_factor=2),
            conv3x3(out_filters[0], out_filters[0]),
            nn.ReLU(inplace=True),
            conv3x3(out_filters[0], out_filters[0]),
            nn.ReLU(inplace=True),
        )
        self.final = nn.Conv2d(out_filters[0], num_classes, kernel_size=1)

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        """
        完整前向传播。

        Args:
            inputs: [B, 3, H, W] RGB 输入（H=W=640 来自预处理）

        Returns:
            [B, num_classes, H_out, W_out] logits（H_out≈H, W_out≈W，经 decoder 恢复）
        """
        # ResNet50 encoder
        feat1, feat2, feat3, feat4 = self.resnet(inputs)

        # UNet decoder
        up4 = self.up4(feat3, feat4)
        up3 = self.up3(feat2, up4)
        up2 = self.up2(feat1, up3)
        up1 = self.up1(feat1, up2)   # 与 Design-AI_detect.md 一致：up1 使用 feat1

        # 最终上采样 + 1×1 分类
        up1 = self.up_conv(up1)
        return self.final(up1)
