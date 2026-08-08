# -*- coding: utf-8 -*-
"""
V53: UNet prediction + regularized rebar measurement.

This version keeps the V52 measurement logic and writes:
1. The final measurement image.
2. A five-column CSV report for each image.
3. A combined CSV report when folder mode is used.
"""

import os
import sys
import csv
import time
import cv2
import numpy as np
from PIL import Image
from tqdm import tqdm


# ==============================
#  Single-file UNet network and prediction code.
#  It no longer depends on external unet.py / nets / utils modules.
#  A trained weights file is still required, for example logs/Unet_resnet50.pth.
# ==============================

import colorsys
import copy
import math
import random

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.utils.model_zoo as model_zoo
from torch.hub import load_state_dict_from_url


def cvtColor(image):
    if len(np.shape(image)) == 3 and np.shape(image)[2] == 3:
        return image
    else:
        return image.convert("RGB")


def resize_image(image, size):
    iw, ih = image.size
    w, h = size
    scale = min(w / iw, h / ih)
    nw = int(iw * scale)
    nh = int(ih * scale)
    image = image.resize((nw, nh), Image.BICUBIC)
    new_image = Image.new("RGB", size, (128, 128, 128))
    new_image.paste(image, ((w - nw) // 2, (h - nh) // 2))
    return new_image, nw, nh


def preprocess_input(image):
    image /= 255.0
    return image


def show_config(**kwargs):
    print("Configurations:")
    print("-" * 70)
    print("|%25s | %40s|" % ("keys", "values"))
    print("-" * 70)
    for key, value in kwargs.items():
        print("|%25s | %40s|" % (str(key), str(value)))
    print("-" * 70)


def conv3x3(in_planes, out_planes, stride=1, groups=1, dilation=1):
    return nn.Conv2d(
        in_planes, out_planes, kernel_size=3, stride=stride,
        padding=dilation, groups=groups, bias=False, dilation=dilation
    )


def conv1x1(in_planes, out_planes, stride=1):
    return nn.Conv2d(in_planes, out_planes, kernel_size=1, stride=stride, bias=False)


class BasicBlock(nn.Module):
    expansion = 1

    def __init__(self, inplanes, planes, stride=1, downsample=None, groups=1,
                 base_width=64, dilation=1, norm_layer=None):
        super(BasicBlock, self).__init__()
        if norm_layer is None:
            norm_layer = nn.BatchNorm2d
        if groups != 1 or base_width != 64:
            raise ValueError("BasicBlock only supports groups=1 and base_width=64")
        if dilation > 1:
            raise NotImplementedError("Dilation > 1 not supported in BasicBlock")
        self.conv1 = conv3x3(inplanes, planes, stride)
        self.bn1 = norm_layer(planes)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = conv3x3(planes, planes)
        self.bn2 = norm_layer(planes)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        identity = x
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)
        out = self.conv2(out)
        out = self.bn2(out)
        if self.downsample is not None:
            identity = self.downsample(x)
        out += identity
        out = self.relu(out)
        return out


class Bottleneck(nn.Module):
    expansion = 4

    def __init__(self, inplanes, planes, stride=1, downsample=None, groups=1,
                 base_width=64, dilation=1, norm_layer=None):
        super(Bottleneck, self).__init__()
        if norm_layer is None:
            norm_layer = nn.BatchNorm2d
        width = int(planes * (base_width / 64.0)) * groups
        self.conv1 = conv1x1(inplanes, width)
        self.bn1 = norm_layer(width)
        self.conv2 = conv3x3(width, width, stride, groups, dilation)
        self.bn2 = norm_layer(width)
        self.conv3 = conv1x1(width, planes * self.expansion)
        self.bn3 = norm_layer(planes * self.expansion)
        self.relu = nn.ReLU(inplace=True)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        identity = x
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)
        out = self.conv2(out)
        out = self.bn2(out)
        out = self.relu(out)
        out = self.conv3(out)
        out = self.bn3(out)
        if self.downsample is not None:
            identity = self.downsample(x)
        out += identity
        out = self.relu(out)
        return out


class ResNet(nn.Module):
    def __init__(self, block, layers, num_classes=1000):
        self.inplanes = 64
        super(ResNet, self).__init__()
        self.conv1 = nn.Conv2d(3, 64, kernel_size=7, stride=2, padding=3, bias=False)
        self.bn1 = nn.BatchNorm2d(64)
        self.relu = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=0, ceil_mode=True)
        self.layer1 = self._make_layer(block, 64, layers[0])
        self.layer2 = self._make_layer(block, 128, layers[1], stride=2)
        self.layer3 = self._make_layer(block, 256, layers[2], stride=2)
        self.layer4 = self._make_layer(block, 512, layers[3], stride=2)
        self.avgpool = nn.AvgPool2d(7)
        self.fc = nn.Linear(512 * block.expansion, num_classes)

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                n = m.kernel_size[0] * m.kernel_size[1] * m.out_channels
                m.weight.data.normal_(0, math.sqrt(2.0 / n))
            elif isinstance(m, nn.BatchNorm2d):
                m.weight.data.fill_(1)
                m.bias.data.zero_()

    def _make_layer(self, block, planes, blocks, stride=1):
        downsample = None
        if stride != 1 or self.inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                nn.Conv2d(self.inplanes, planes * block.expansion,
                          kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(planes * block.expansion),
            )
        layers = [block(self.inplanes, planes, stride, downsample)]
        self.inplanes = planes * block.expansion
        for _ in range(1, blocks):
            layers.append(block(self.inplanes, planes))
        return nn.Sequential(*layers)

    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        feat1 = self.relu(x)

        x = self.maxpool(feat1)
        feat2 = self.layer1(x)

        feat3 = self.layer2(feat2)
        feat4 = self.layer3(feat3)
        feat5 = self.layer4(feat4)
        return [feat1, feat2, feat3, feat4, feat5]


def resnet50(pretrained=False, **kwargs):
    model = ResNet(Bottleneck, [3, 4, 6, 3], **kwargs)
    if pretrained:
        model.load_state_dict(
            model_zoo.load_url(
                "https://s3.amazonaws.com/pytorch/models/resnet50-19c8e357.pth",
                model_dir="model_data"
            ),
            strict=False
        )
    del model.avgpool
    del model.fc
    return model


class VGG(nn.Module):
    def __init__(self, features, num_classes=1000):
        super(VGG, self).__init__()
        self.features = features
        self.avgpool = nn.AdaptiveAvgPool2d((7, 7))
        self.classifier = nn.Sequential(
            nn.Linear(512 * 7 * 7, 4096),
            nn.ReLU(True),
            nn.Dropout(),
            nn.Linear(4096, 4096),
            nn.ReLU(True),
            nn.Dropout(),
            nn.Linear(4096, num_classes),
        )
        self._initialize_weights()

    def forward(self, x):
        feat1 = self.features[:4](x)
        feat2 = self.features[4:9](feat1)
        feat3 = self.features[9:16](feat2)
        feat4 = self.features[16:23](feat3)
        feat5 = self.features[23:-1](feat4)
        return [feat1, feat2, feat3, feat4, feat5]

    def _initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.Linear):
                nn.init.normal_(m.weight, 0, 0.01)
                nn.init.constant_(m.bias, 0)


def make_layers(cfg, batch_norm=False, in_channels=3):
    layers = []
    for v in cfg:
        if v == "M":
            layers += [nn.MaxPool2d(kernel_size=2, stride=2)]
        else:
            conv2d = nn.Conv2d(in_channels, v, kernel_size=3, padding=1)
            if batch_norm:
                layers += [conv2d, nn.BatchNorm2d(v), nn.ReLU(inplace=True)]
            else:
                layers += [conv2d, nn.ReLU(inplace=True)]
            in_channels = v
    return nn.Sequential(*layers)


cfgs = {
    "D": [64, 64, "M", 128, 128, "M", 256, 256, 256, "M", 512, 512, 512, "M", 512, 512, 512, "M"]
}


def VGG16(pretrained, in_channels=3, **kwargs):
    model = VGG(make_layers(cfgs["D"], batch_norm=False, in_channels=in_channels), **kwargs)
    if pretrained:
        state_dict = load_state_dict_from_url(
            "https://download.pytorch.org/models/vgg16-397923af.pth",
            model_dir="./model_data"
        )
        model.load_state_dict(state_dict)
    del model.avgpool
    del model.classifier
    return model


class unetUp(nn.Module):
    def __init__(self, in_size, out_size):
        super(unetUp, self).__init__()
        self.conv1 = nn.Conv2d(in_size, out_size, kernel_size=3, padding=1)
        self.conv2 = nn.Conv2d(out_size, out_size, kernel_size=3, padding=1)
        self.up = nn.UpsamplingBilinear2d(scale_factor=2)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, inputs1, inputs2):
        outputs = torch.cat([inputs1, self.up(inputs2)], 1)
        outputs = self.conv1(outputs)
        outputs = self.relu(outputs)
        outputs = self.conv2(outputs)
        outputs = self.relu(outputs)
        return outputs


class UNetModel(nn.Module):
    def __init__(self, num_classes=21, pretrained=False, backbone="vgg"):
        super(UNetModel, self).__init__()
        if backbone == "vgg":
            self.vgg = VGG16(pretrained=pretrained)
            in_filters = [192, 384, 768, 1024]
        elif backbone == "resnet50":
            self.resnet = resnet50(pretrained=pretrained)
            in_filters = [192, 512, 1024, 3072]
        else:
            raise ValueError("Unsupported backbone - `{}`, Use vgg, resnet50.".format(backbone))
        out_filters = [64, 128, 256, 512]
        self.up_concat4 = unetUp(in_filters[3], out_filters[3])
        self.up_concat3 = unetUp(in_filters[2], out_filters[2])
        self.up_concat2 = unetUp(in_filters[1], out_filters[1])
        self.up_concat1 = unetUp(in_filters[0], out_filters[0])

        if backbone == "resnet50":
            self.up_conv = nn.Sequential(
                nn.UpsamplingBilinear2d(scale_factor=2),
                nn.Conv2d(out_filters[0], out_filters[0], kernel_size=3, padding=1),
                nn.ReLU(),
                nn.Conv2d(out_filters[0], out_filters[0], kernel_size=3, padding=1),
                nn.ReLU(),
            )
        else:
            self.up_conv = None

        self.final = nn.Conv2d(out_filters[0], num_classes, 1)
        self.backbone = backbone

    def forward(self, inputs):
        if self.backbone == "vgg":
            feat1, feat2, feat3, feat4, feat5 = self.vgg.forward(inputs)
        elif self.backbone == "resnet50":
            feat1, feat2, feat3, feat4, feat5 = self.resnet.forward(inputs)

        up4 = self.up_concat4(feat4, feat5)
        up3 = self.up_concat3(feat3, up4)
        up2 = self.up_concat2(feat2, up3)
        up1 = self.up_concat1(feat1, up2)

        if self.up_conv is not None:
            up1 = self.up_conv(up1)

        final = self.final(up1)
        return final


class Unet(object):
    _defaults = {
        "model_path": "logs/Unet_resnet50.pth",
        # Training classes: background, 1 Rebar, 2 X-Rebar.
        "num_classes": 3,
        "backbone": "resnet50",
        "input_shape": [640, 640],
        # mix_type=1 outputs the segmentation image for downstream extraction.
        "mix_type": 1,
        "cuda": False,
    }

    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults)
        for name, value in kwargs.items():
            setattr(self, name, value)

        if self.num_classes <= 21:
            self.colors = [
                (0, 0, 0), (128, 0, 0), (0, 128, 0), (128, 128, 0),
                (0, 0, 128), (128, 0, 128), (0, 128, 128), (128, 128, 128),
                (64, 0, 0), (192, 0, 0), (64, 128, 0), (192, 128, 0),
                (64, 0, 128), (192, 0, 128), (64, 128, 128), (192, 128, 128),
                (0, 64, 0), (128, 64, 0), (0, 192, 0), (128, 192, 0),
                (0, 64, 128), (128, 64, 12)
            ]
        else:
            hsv_tuples = [(x / self.num_classes, 1.0, 1.0) for x in range(self.num_classes)]
            self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
            self.colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), self.colors))

        self.generate()
        show_config(
            model_path=self.model_path,
            num_classes=self.num_classes,
            backbone=self.backbone,
            input_shape=self.input_shape,
            mix_type=self.mix_type,
            cuda=self.cuda,
        )

    def generate(self, onnx=False):
        self.net = UNetModel(num_classes=self.num_classes, backbone=self.backbone)
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.net.load_state_dict(torch.load(self.model_path, map_location=device))
        self.net = self.net.eval()
        print("{} model, and classes loaded.".format(self.model_path))
        if not onnx:
            if self.cuda:
                self.net = nn.DataParallel(self.net)
                self.net = self.net.cuda()

    def detect_image(self, image, count=False, name_classes=None):
        image = cvtColor(image)
        old_img = copy.deepcopy(image)
        orininal_h = np.array(image).shape[0]
        orininal_w = np.array(image).shape[1]

        image_data, nw, nh = resize_image(image, (self.input_shape[1], self.input_shape[0]))
        image_data = np.expand_dims(
            np.transpose(preprocess_input(np.array(image_data, np.float32)), (2, 0, 1)),
            0
        )

        with torch.no_grad():
            images = torch.from_numpy(image_data)
            if self.cuda:
                images = images.cuda()

            pr = self.net(images)[0]
            pr = F.softmax(pr.permute(1, 2, 0), dim=-1).cpu().numpy()
            pr = pr[
                int((self.input_shape[0] - nh) // 2): int((self.input_shape[0] - nh) // 2 + nh),
                int((self.input_shape[1] - nw) // 2): int((self.input_shape[1] - nw) // 2 + nw)
            ]
            pr = cv2.resize(pr, (orininal_w, orininal_h), interpolation=cv2.INTER_LINEAR)
            pr = pr.argmax(axis=-1)

        if count:
            classes_nums = np.zeros([self.num_classes])
            total_points_num = orininal_h * orininal_w
            print("-" * 63)
            print("|%25s | %15s | %15s|" % ("Key", "Value", "Ratio"))
            print("-" * 63)
            for i in range(self.num_classes):
                num = np.sum(pr == i)
                ratio = num / total_points_num * 100
                if num > 0 and name_classes is not None:
                    print("|%25s | %15s | %14.2f%%|" % (str(name_classes[i]), str(num), ratio))
                    print("-" * 63)
                classes_nums[i] = num
            print("classes_nums:", classes_nums)

        seg_img = np.reshape(np.array(self.colors, np.uint8)[np.reshape(pr, [-1])], [orininal_h, orininal_w, -1])
        image = Image.fromarray(np.uint8(seg_img))

        if self.mix_type == 0:
            image = Image.blend(old_img, image, 0.7)
        elif self.mix_type == 1:
            pass
        elif self.mix_type == 2:
            seg_img = (np.expand_dims(pr != 0, -1) * np.array(old_img, np.float32)).astype("uint8")
            image = Image.fromarray(np.uint8(seg_img))

        return image

    def detect_label(self, image, count=False, name_classes=None):
        """Return the raw class mask predicted by UNet: 0=background, 1=rebar, 2=x-rebar."""
        image = cvtColor(image)
        orininal_h = np.array(image).shape[0]
        orininal_w = np.array(image).shape[1]

        image_data, nw, nh = resize_image(image, (self.input_shape[1], self.input_shape[0]))
        image_data = np.expand_dims(
            np.transpose(preprocess_input(np.array(image_data, np.float32)), (2, 0, 1)),
            0
        )

        with torch.no_grad():
            images = torch.from_numpy(image_data)
            if self.cuda:
                images = images.cuda()

            pr = self.net(images)[0]
            pr = F.softmax(pr.permute(1, 2, 0), dim=-1).cpu().numpy()
            pr = pr[
                int((self.input_shape[0] - nh) // 2): int((self.input_shape[0] - nh) // 2 + nh),
                int((self.input_shape[1] - nw) // 2): int((self.input_shape[1] - nw) // 2 + nw)
            ]
            pr = cv2.resize(pr, (orininal_w, orininal_h), interpolation=cv2.INTER_LINEAR)
            pr = pr.argmax(axis=-1).astype(np.uint8)

        if count:
            classes_nums = np.zeros([self.num_classes])
            total_points_num = orininal_h * orininal_w
            for i in range(self.num_classes):
                num = np.sum(pr == i)
                ratio = num / total_points_num * 100
                if num > 0 and name_classes is not None:
                    print("%s: %d, %.2f%%" % (str(name_classes[i]), int(num), ratio))
                classes_nums[i] = num
            print("classes_nums:", classes_nums)

        return pr



class RebarMeasureV53:
    def __init__(
        self,
        camera_distance_mm=500.0,
        fx=950.0,
        fy=950.0,
        cx=641.420,
        cy=362.843,
        min_component_area=120,
        min_height_ratio=0.12,
        merge_x_ratio=0.012,
        use_standard_diameter=True,
        min_diameter_mm=5.0,
        max_diameter_mm=45.0,
        scanline_ratio=0.50,
        unify_diameter_per_image=False,
        show_edge_debug=True,
        spacing_scale_factor=1.50,
        small_rebar_nominal_mm=10.0,
    ):
        self.camera_distance_mm = float(camera_distance_mm)
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)

        self.mm_per_pixel_x = self.camera_distance_mm / self.fx
        self.mm_per_pixel_y = self.camera_distance_mm / self.fy
        self.mm_per_pixel = self.mm_per_pixel_x
        self.spacing_scale_factor = float(spacing_scale_factor)
        self.spacing_mm_per_pixel = self.mm_per_pixel * self.spacing_scale_factor

        self.min_component_area = int(min_component_area)
        self.min_height_ratio = float(min_height_ratio)
        self.merge_x_ratio = float(merge_x_ratio)
        self.use_standard_diameter = bool(use_standard_diameter)
        self.min_diameter_mm = float(min_diameter_mm)
        self.max_diameter_mm = float(max_diameter_mm)
        self.scanline_ratio = float(scanline_ratio) if scanline_ratio is not None else -1.0
        self.unify_diameter_per_image = bool(unify_diameter_per_image)
        self.show_edge_debug = bool(show_edge_debug)
        self.small_rebar_nominal_mm = float(small_rebar_nominal_mm) if small_rebar_nominal_mm else None
        self.min_diameter_px = self.min_diameter_mm / max(self.mm_per_pixel, 1e-6)
        self.max_diameter_px = self.max_diameter_mm / max(self.mm_per_pixel, 1e-6)

        self.standard_diameters = np.array(
            [6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40],
            dtype=np.float32,
        )

    @staticmethod
    def _safe_to_bgr(img):
        if img is None:
            return None
        if len(img.shape) == 2:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if img.shape[2] == 4:
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img

    @staticmethod
    def _read_image_unicode(path):
        data = np.fromfile(path, dtype=np.uint8)
        img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
        return RebarMeasureV53._safe_to_bgr(img)

    @staticmethod
    def _write_image_unicode(path, img):
        ext = os.path.splitext(path)[1]
        if ext == "":
            ext = ".jpg"
            path = path + ext
        ok, buf = cv2.imencode(ext, img)
        if ok:
            buf.tofile(path)
        else:
            raise RuntimeError(f"Failed to save image: {path}")

    def _get_red_rebar_mask(self, pred_bgr):
        """
        Extract the Rebar class from a UNet color prediction.
        The expected palette is background=(0,0,0), Rebar=(128,0,0),
        X-Rebar=(0,128,0). After PIL-to-OpenCV conversion, Rebar is near
        BGR=(0,0,128). A color range is used instead of Otsu thresholding
        to avoid treating red text or overlays as rebar.
        """
        b, g, r = cv2.split(pred_bgr)

        # In the color mask, Rebar is dark red. Keep some tolerance for edges.
        mask = ((r >= 80) & (g <= 60) & (b <= 60)).astype(np.uint8) * 255

        # Remove tiny noise without over-connecting neighboring rebars.
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

        # Apply a small vertical repair. Larger kernels over-merge fragments.
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

        return mask

    def _find_raw_rebars(self, mask):
        h, w = mask.shape[:2]
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        raw = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_component_area:
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)

            if bh < h * self.min_height_ratio:
                continue
            if bw < max(2, int(self.min_diameter_px * 0.35)):
                continue
            if bw > min(w * 0.12, self.max_diameter_px * 1.8):
                continue
            if bh / (bw + 1e-6) < 2.2:
                continue

            rect = cv2.minAreaRect(cnt)
            (cx, cy), (d1, d2), angle = rect
            width = min(d1, d2)
            length = max(d1, d2)
            if width <= 1:
                continue
            if width > self.max_diameter_px * 1.8:
                continue
            if length / (width + 1e-6) < 2.2:
                continue

            box = np.int32(cv2.boxPoints(rect))
            raw.append(
                {
                    "cx": float(cx),
                    "cy": float(cy),
                    "rect": rect,
                    "box": box,
                    "bbox": (x, y, bw, bh),
                    "area": float(area),
                }
            )

        return raw

    def _combine_group(self, group):
        if len(group) == 1:
            return group[0]
        pts = []
        for item in group:
            x, y, w, h = item["bbox"]
            pts.append(np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype=np.int32))
        pts = np.vstack(pts)
        rect = cv2.minAreaRect(pts)
        box = np.int32(cv2.boxPoints(rect))
        x, y, w, h = cv2.boundingRect(pts)
        (cx, cy), _, _ = rect
        return {
            "cx": float(cx),
            "cy": float(cy),
            "rect": rect,
            "box": box,
            "bbox": (x, y, w, h),
            "area": float(sum([g.get("area", 0.0) for g in group])),
        }

    def _merge_collinear_segments(self, raw_rebars, img_w):
        """Merge collinear split segments for the legacy path."""
        if not raw_rebars:
            return []

        raw_rebars = sorted(raw_rebars, key=lambda r: r["cx"])
        x_tol = max(6.0, img_w * self.merge_x_ratio)

        merged = []
        group = [raw_rebars[0]]
        for item in raw_rebars[1:]:
            prev_cx = np.median([g["cx"] for g in group])
            if abs(item["cx"] - prev_cx) <= x_tol:
                group.append(item)
            else:
                merged.append(self._combine_group(group))
                group = [item]
        merged.append(self._combine_group(group))
        return merged

    def _scan_edges_for_one_rebar(self, mask, bbox, padding=6, min_valid_width=4):
        """
        Scan one vertical bar in multiple rows and estimate stable left/right edges.
        """
        img_h, img_w = mask.shape[:2]
        x, y, w, h = bbox
        x1 = max(0, x - padding)
        y1 = max(0, y)
        x2 = min(img_w, x + w + padding)
        y2 = min(img_h, y + h)

        roi = mask[y1:y2, x1:x2]
        if roi.size == 0:
            return None

        rh, rw = roi.shape[:2]
        if rh < 10 or rw < 3:
            return None

        # Ignore unstable top/bottom regions and measure the stable middle band.
        top = int(rh * 0.15)
        bottom = int(rh * 0.85)
        roi_mid = roi[top:bottom, :]

        lefts, rights, centers, widths = [], [], [], []
        for row in roi_mid:
            xs = np.where(row > 0)[0]
            if len(xs) < min_valid_width:
                continue
            left = float(xs[0])
            right = float(xs[-1])
            width = right - left + 1.0
            lefts.append(left)
            rights.append(right)
            centers.append((left + right) / 2.0)
            widths.append(width)

        if len(widths) < 8:
            return None

        widths = np.asarray(widths, dtype=np.float32)
        med_w = float(np.median(widths))

        valid_idx = np.where(
            (widths > med_w * 0.60) &
            (widths < med_w * 1.35) &
            (widths >= self.min_diameter_px * 0.45) &
            (widths <= self.max_diameter_px * 1.35)
        )[0]
        if len(valid_idx) < 8:
            valid_idx = np.arange(len(widths))

        lefts = np.asarray(lefts, dtype=np.float32)[valid_idx]
        rights = np.asarray(rights, dtype=np.float32)[valid_idx]
        centers = np.asarray(centers, dtype=np.float32)[valid_idx]
        widths = widths[valid_idx]

        left_x = float(np.median(lefts)) + x1
        right_x = float(np.median(rights)) + x1
        center_x = float(np.median(centers)) + x1
        width_px = float(np.median(widths))

        # Representative vertical range used for drawing.
        y_top = y1 + top
        y_bottom = y1 + bottom

        return {
            "left_x": left_x,
            "right_x": right_x,
            "center_x": center_x,
            "width_px": width_px,
            "y_top": int(y_top),
            "y_bottom": int(y_bottom),
            "bbox": bbox,
        }

    def _normalize_rebar_diameter(self, d_mm):
        idx = int(np.argmin(np.abs(self.standard_diameters - d_mm)))
        return float(self.standard_diameters[idx])

    def _draw_red_bar(self, vis, edge, alpha=0.70):
        """Draw one regular red bar for legacy visualization."""
        overlay = vis.copy()
        x1 = int(round(edge["left_x"]))
        x2 = int(round(edge["right_x"]))
        y1 = int(edge["y_top"])
        y2 = int(edge["y_bottom"])
        x1 = max(0, min(vis.shape[1] - 1, x1))
        x2 = max(0, min(vis.shape[1] - 1, x2))
        y1 = max(0, min(vis.shape[0] - 1, y1))
        y2 = max(0, min(vis.shape[0] - 1, y2))
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 255), -1)
        cv2.addWeighted(overlay, alpha, vis, 1 - alpha, 0, dst=vis)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2, lineType=cv2.LINE_AA)

    def _get_green_mask(self, pred_bgr):
        """Extract x-rebar mask from legacy color prediction."""
        b, g, r = cv2.split(pred_bgr)
        return ((g >= 80) & (r <= 70) & (b <= 70)).astype(np.uint8) * 255

    @staticmethod
    def _intervals_from_col_has(col_has, min_w=3, max_gap=2):
        """Extract intervals from a boolean projection."""
        intervals = []
        n = len(col_has)
        i = 0
        while i < n:
            while i < n and not col_has[i]:
                i += 1
            if i >= n:
                break
            s = i
            last_true = i
            gap = 0
            i += 1
            while i < n:
                if col_has[i]:
                    last_true = i
                    gap = 0
                else:
                    gap += 1
                    if gap > max_gap:
                        break
                i += 1
            e = last_true
            if e - s + 1 >= min_w:
                intervals.append([s, e])
        return intervals

    @staticmethod
    def _merge_close_intervals(intervals, gap_tol=4):
        if not intervals:
            return []
        intervals = sorted(intervals, key=lambda x: x[0])
        merged = [intervals[0]]
        for s, e in intervals[1:]:
            if s - merged[-1][1] <= gap_tol:
                merged[-1][1] = max(merged[-1][1], e)
            else:
                merged.append([s, e])
        return merged

    def _select_measurement_scanline(self, red_mask, green_mask):
        """
        Select a horizontal scanline for the legacy measurement path.
        """
        h, w = red_mask.shape[:2]

        # Prefer a fixed scanline ratio for a fixed camera position.
        if 0.05 <= self.scanline_ratio <= 0.95:
            y = int(round(h * self.scanline_ratio))
            band_half = max(4, int(h * 0.010))
            y1 = max(0, y - band_half)
            y2 = min(h, y + band_half + 1)
            col_has = np.any(red_mask[y1:y2, :] > 0, axis=0)
            intervals = self._intervals_from_col_has(
                col_has,
                min_w=max(3, int(self.min_diameter_px * 0.35)),
                max_gap=2,
            )
            intervals = self._merge_close_intervals(intervals, gap_tol=3)
            good = []
            for s, e in intervals:
                ww = e - s + 1
                if self.min_diameter_px * 0.35 <= ww <= self.max_diameter_px * 1.45:
                    good.append([s, e])
            return y, good

        y_min = int(h * 0.22)
        y_max = int(h * 0.82)
        band_half = max(4, int(h * 0.010))

        best = None
        best_score = -1e18

        for y in range(y_min, y_max, 3):
            y1 = max(0, y - band_half)
            y2 = min(h, y + band_half + 1)

            red_band = red_mask[y1:y2, :] > 0
            green_band = green_mask[y1:y2, :] > 0

            # Skip rows dominated by horizontal bars to avoid intersections.
            green_ratio = float(np.mean(green_band))
            if green_ratio > 0.08:
                continue

            col_has = np.any(red_band, axis=0)
            intervals = self._intervals_from_col_has(
                col_has,
                min_w=max(3, int(self.min_diameter_px * 0.35)),
                max_gap=2,
            )
            intervals = self._merge_close_intervals(intervals, gap_tol=3)

            good = []
            for s, e in intervals:
                ww = e - s + 1
                if self.min_diameter_px * 0.35 <= ww <= self.max_diameter_px * 1.45:
                    good.append([s, e])

            if len(good) < 2:
                continue

            # Score rows by rebar coverage while penalizing horizontal bars.
            red_pixels = int(np.sum(red_band))
            score = len(good) * 10000 + red_pixels - green_ratio * 50000
            if score > best_score:
                best_score = score
                best = (y, good)

        # Fallback to a row near the image center.
        if best is None:
            y = h // 2
            y1 = max(0, y - 6)
            y2 = min(h, y + 7)
            col_has = np.any(red_mask[y1:y2, :] > 0, axis=0)
            intervals = self._intervals_from_col_has(col_has, min_w=3, max_gap=2)
            intervals = self._merge_close_intervals(intervals, gap_tol=3)
            best = (y, intervals)

        return best

    def _refine_interval_width_near_scanline(self, red_mask, green_mask, interval, y_scan):
        """
        Estimate width around the scanline and use a median to reduce the
        influence of burrs and local mask noise.
        """
        h, w = red_mask.shape[:2]
        s, e = interval
        center = (s + e) / 2.0
        search_half_y = max(12, int(h * 0.025))
        y1 = max(0, int(y_scan - search_half_y))
        y2 = min(h, int(y_scan + search_half_y + 1))

        widths, lefts, rights = [], [], []
        for y in range(y1, y2):
            if np.mean(green_mask[max(0, y-2):min(h, y+3), :] > 0) > 0.08:
                continue

            row = red_mask[y, :] > 0
            intervals = self._intervals_from_col_has(row, min_w=2, max_gap=2)
            intervals = self._merge_close_intervals(intervals, gap_tol=3)

            candidates = []
            for a, b in intervals:
                if a - 5 <= center <= b + 5:
                    ww = b - a + 1
                    if self.min_diameter_px * 0.35 <= ww <= self.max_diameter_px * 1.45:
                        candidates.append((abs((a+b)/2.0 - center), a, b))
            if not candidates:
                continue
            _, a, b = min(candidates, key=lambda x: x[0])
            lefts.append(a)
            rights.append(b)
            widths.append(b - a + 1)

        if len(widths) < 5:
            a, b = interval
            return float(a), float(b), float(b - a + 1)

        widths = np.asarray(widths, dtype=np.float32)
        med_w = float(np.median(widths))
        valid = np.where((widths > med_w * 0.65) & (widths < med_w * 1.35))[0]
        if len(valid) < 5:
            valid = np.arange(len(widths))

        left = float(np.median(np.asarray(lefts, dtype=np.float32)[valid]))
        right = float(np.median(np.asarray(rights, dtype=np.float32)[valid]))
        width = float(np.median(widths[valid]))
        return left, right, width

    def _get_vertical_span_at_interval(self, red_mask, interval, y_scan):
        """
        Get the vertical display span for the legacy scanline interval.
        """
        h, w = red_mask.shape[:2]
        left, right = int(round(interval[0])), int(round(interval[1]))
        pad = 4
        x1 = max(0, left - pad)
        x2 = min(w, right + pad + 1)

        row_has = np.any(red_mask[:, x1:x2] > 0, axis=1)

        runs = []
        i = 0
        while i < h:
            while i < h and not row_has[i]:
                i += 1
            if i >= h:
                break
            s = i
            while i < h and row_has[i]:
                i += 1
            e = i - 1
            runs.append((s, e))

        if not runs:
            return max(0, y_scan - 60), min(h - 1, y_scan + 60)

        contain = [r for r in runs if r[0] <= y_scan <= r[1]]
        if contain:
            s, e = max(contain, key=lambda r: r[1] - r[0])
        else:
            s, e = min(runs, key=lambda r: min(abs(r[0] - y_scan), abs(r[1] - y_scan)))

        if e - s < 40:
            cy = (s + e) // 2
            s = max(0, cy - 40)
            e = min(h - 1, cy + 40)

        return int(s), int(e)

    def _draw_scanline_bar(self, vis, edge, alpha=0.65):
        left = int(round(edge["left_x"]))
        right = int(round(edge["right_x"]))
        y_top = int(edge["y_top"])
        y_bottom = int(edge["y_bottom"])
        overlay = vis.copy()
        cv2.rectangle(overlay, (left, y_top), (right, y_bottom), (0, 0, 255), -1)
        cv2.addWeighted(overlay, alpha, vis, 1 - alpha, 0, dst=vis)
        cv2.rectangle(vis, (left, y_top), (right, y_bottom), (0, 255, 0), 2, lineType=cv2.LINE_AA)

    def _overlay_prediction_mask(self, vis, red_mask, green_mask, alpha=0.60):
        """
        Overlay legacy masks for comparison only.
        """
        overlay = vis.copy()
        overlay[red_mask > 0] = (0, 0, 255)
        overlay[green_mask > 0] = (0, 255, 0)
        cv2.addWeighted(overlay, alpha, vis, 1 - alpha, 0, dst=vis)

    @staticmethod
    def _rect_long_angle(rect):
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
    def _vertical_angle_error(angle):
        return abs(90.0 - abs(angle))

    @staticmethod
    def _box_from_center_angle(cx, cy, width, length, angle_deg):
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

    def _robust_component_width(self, component_mask, bbox):
        x, y, bw, bh = bbox
        roi = component_mask[y:y + bh, x:x + bw]
        widths = []
        for row in roi:
            xs = np.where(row > 0)[0]
            if len(xs) >= 3:
                widths.append(float(xs[-1] - xs[0] + 1))
        if len(widths) < 8:
            return None
        widths = np.asarray(widths, dtype=np.float32)
        med = float(np.median(widths))
        valid = widths[(widths >= med * 0.55) & (widths <= med * 1.35)]
        if len(valid) >= 6:
            widths = valid
        return float(np.median(widths))

    def _label_to_masks(self, label_mask):
        label = np.asarray(label_mask)
        if label.ndim == 3:
            label = label[:, :, 0]
        red_mask = (label == 1).astype(np.uint8) * 255
        green_mask = (label == 2).astype(np.uint8) * 255
        return red_mask, green_mask

    def _regularize_rebar_mask(self, red_mask):
        mask = (red_mask > 0).astype(np.uint8) * 255
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        kernel_close_v = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 23))
        kernel_close_h = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close_v)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close_h)
        mask = cv2.medianBlur(mask, 3)
        return mask

    def _build_regular_rebars(self, red_mask, green_mask):
        h, w = red_mask.shape[:2]
        clean = self._regularize_rebar_mask(red_mask)
        contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        rejected = []
        min_len = max(45.0, h * self.min_height_ratio)
        max_len = h * 1.20
        min_width = max(3.0, self.min_diameter_px * 0.45)
        max_width = min(w * 0.10, self.max_diameter_px * 1.35)

        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < self.min_component_area:
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
            green_overlap = float(np.mean(green_mask[y:y + bh, x:x + bw] > 0)) if bw > 0 and bh > 0 else 0.0

            reason = None
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

            candidates.append(
                {
                    "rect": rect,
                    "box": self._box_from_center_angle(cx, cy, robust_width, long_side, angle),
                    "bbox": (x, y, bw, bh),
                    "area": area,
                    "center_x": float(cx),
                    "center_y": float(cy),
                    "width_px": robust_width,
                    "length_px": long_side,
                    "angle": angle,
                    "angle_error": angle_err,
                    "fill_ratio": fill_ratio,
                }
            )

        if not candidates:
            return [], clean, rejected

        widths = np.asarray([c["width_px"] for c in candidates], dtype=np.float32)
        med_w = float(np.median(widths))
        filtered = []
        for c in candidates:
            # Keep genuinely thick rebars. Only discard hairline fragments that are
            # far thinner than the image's main rebar population.
            if c["width_px"] < med_w * 0.32:
                rejected.append(("narrow_vs_median", c["area"]))
                continue
            filtered.append(c)

        filtered = sorted(filtered, key=lambda r: r["center_x"])
        groups = []
        min_center_gap = max(6.0, med_w * 0.65)
        for c in filtered:
            if groups and abs(c["center_x"] - np.median([g["center_x"] for g in groups[-1]])) < min_center_gap:
                groups[-1].append(c)
            else:
                groups.append([c])

        regular = []
        for group in groups:
            if len(group) == 1:
                regular.append(group[0])
                continue

            centers_x = np.asarray([g["center_x"] for g in group], dtype=np.float32)
            widths_g = np.asarray([g["width_px"] for g in group], dtype=np.float32)
            angles_g = np.asarray([g["angle"] for g in group], dtype=np.float32)
            areas_g = np.asarray([g["area"] for g in group], dtype=np.float32)

            x1 = min([g["bbox"][0] for g in group])
            y1 = min([g["bbox"][1] for g in group])
            x2 = max([g["bbox"][0] + g["bbox"][2] for g in group])
            y2 = max([g["bbox"][1] + g["bbox"][3] for g in group])

            center_x = float(np.average(centers_x, weights=np.maximum(areas_g, 1.0)))
            center_y = float((y1 + y2) / 2.0)
            width_px = float(np.median(widths_g))
            angle = float(np.median(angles_g))
            length_px = float(max(1.0, (y2 - y1) / max(abs(np.sin(np.deg2rad(angle))), 0.25)))
            merged = dict(group[int(np.argmax(areas_g))])
            merged.update(
                {
                    "bbox": (int(x1), int(y1), int(x2 - x1), int(y2 - y1)),
                    "area": float(np.sum(areas_g)),
                    "center_x": center_x,
                    "center_y": center_y,
                    "width_px": width_px,
                    "length_px": length_px,
                    "angle": angle,
                    "angle_error": self._vertical_angle_error(angle),
                }
            )
            regular.append(merged)

        for idx, r in enumerate(regular):
            box = self._box_from_center_angle(
                r["center_x"], r["center_y"], r["width_px"], r["length_px"], r["angle"]
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

    def _draw_regular_rebar(self, vis, rebar, alpha=0.72):
        overlay = vis.copy()
        box = np.int32(np.round(rebar["box"]))
        cv2.fillConvexPoly(overlay, box, (0, 0, 255), lineType=cv2.LINE_AA)
        cv2.addWeighted(overlay, alpha, vis, 1 - alpha, 0, dst=vis)
        cv2.polylines(vis, [box], True, (0, 255, 0), 2, lineType=cv2.LINE_AA)

    @staticmethod
    def _center_x_at_y(rebar, y_value):
        angle = float(rebar.get("angle", 90.0))
        theta = np.deg2rad(angle)
        sin_t = float(np.sin(theta))
        cos_t = float(np.cos(theta))
        if abs(sin_t) < 1e-3:
            return float(rebar["center_x"])
        return float(rebar["center_x"] + (float(y_value) - float(rebar["center_y"])) * cos_t / sin_t)

    def _filter_main_rebars_for_spacing(self, rebars, img_h):
        if not rebars:
            return [], int(img_h * 0.50)

        rebars = [r for r in rebars if abs(float(r.get("angle_error", 0.0))) <= 18.0]
        if not rebars:
            return [], int(img_h * 0.50)

        lengths = np.asarray([r["length_px"] for r in rebars], dtype=np.float32)
        med_len = float(np.median(lengths))
        min_main_len = max(img_h * 0.34, med_len * 0.45)

        long_rebars = [r for r in rebars if r["length_px"] >= min_main_len]
        if not long_rebars:
            long_rebars = list(rebars)

        centers_y = np.asarray([r["center_y"] for r in long_rebars], dtype=np.float32)
        measure_y = int(round(float(np.median(centers_y))))
        measure_y = max(int(img_h * 0.18), min(int(img_h * 0.82), measure_y))

        margin = max(8, int(img_h * 0.015))
        min_crossing_len = max(img_h * 0.18, med_len * 0.25)
        main = []
        for r in rebars:
            if r["length_px"] < min_crossing_len:
                continue
            edge = r["edge"]
            if edge["y_top"] + margin <= measure_y <= edge["y_bottom"] - margin:
                main.append(r)

        if len(main) < 2:
            main = long_rebars

        for idx, r in enumerate(sorted(main, key=lambda item: self._center_x_at_y(item, measure_y))):
            width_px = float(r["width_px"])
            cx_at_y = self._center_x_at_y(r, measure_y)
            r["edge"]["center_x"] = cx_at_y
            r["edge"]["left_x"] = cx_at_y - width_px / 2.0
            r["edge"]["right_x"] = cx_at_y + width_px / 2.0
            r["id"] = f"H{idx + 1}"

        main = sorted(main, key=lambda item: item["edge"]["center_x"])
        return main, measure_y

    def _fit_bar_box_from_local_mask(self, red_mask, center_x, width_px, y_top, y_bottom, search_radius):
        h, w = red_mask.shape[:2]
        x0 = int(max(0, round(center_x - search_radius)))
        x1 = int(min(w - 1, round(center_x + search_radius)))
        points = []

        for y in range(max(0, int(y_top)), min(h - 1, int(y_bottom)) + 1, 2):
            xs = np.where(red_mask[y, x0:x1 + 1] > 0)[0]
            if len(xs) < 3:
                continue
            xs = xs + x0
            breaks = np.where(np.diff(xs) > 2)[0]
            starts = np.r_[0, breaks + 1]
            ends = np.r_[breaks, len(xs) - 1]
            segments = []
            for s, e in zip(starts, ends):
                seg = xs[s:e + 1]
                if len(seg) >= 3:
                    segments.append(seg)
            if not segments:
                continue
            seg = min(segments, key=lambda arr: abs(float(np.mean(arr)) - float(center_x)))
            seg_center = float((int(seg[0]) + int(seg[-1])) / 2.0)
            if abs(seg_center - center_x) <= search_radius * 0.90:
                points.append((float(y), seg_center))

        if len(points) < 20:
            return None

        pts = np.asarray(points, dtype=np.float32)
        ys = pts[:, 0]
        xs = pts[:, 1]
        med_x = float(np.median(xs))
        keep = np.abs(xs - med_x) < max(width_px * 2.5, search_radius * 0.55)
        if int(np.count_nonzero(keep)) >= 20:
            ys = ys[keep]
            xs = xs[keep]

        slope, intercept = np.polyfit(ys, xs, 1)
        y_a = float(max(0, y_top))
        y_b = float(min(h - 1, y_bottom))
        x_a = float(slope * y_a + intercept)
        x_b = float(slope * y_b + intercept)
        x_mid = float(slope * float((y_a + y_b) / 2.0) + intercept)

        dx = x_b - x_a
        dy = y_b - y_a
        length = max(float(np.hypot(dx, dy)), 1.0)
        nx = -dy / length
        ny = dx / length
        half_w = float(width_px) / 2.0
        box = np.array(
            [
                [x_a + nx * half_w, y_a + ny * half_w],
                [x_a - nx * half_w, y_a - ny * half_w],
                [x_b - nx * half_w, y_b - ny * half_w],
                [x_b + nx * half_w, y_b + ny * half_w],
            ],
            dtype=np.float32,
        )
        angle = float(np.degrees(np.arctan2(dy, dx)))
        if angle < 0:
            angle += 180.0
        return box, x_mid, angle

    def _insert_missing_rebars_by_gap(self, rebars, img_h, measure_y, candidates=None, red_mask=None):
        if len(rebars) < 4:
            return rebars, 0

        rebars = sorted(rebars, key=lambda item: item["edge"]["center_x"])
        center_gaps = np.asarray(
            [rebars[i]["edge"]["center_x"] - rebars[i - 1]["edge"]["center_x"] for i in range(1, len(rebars))],
            dtype=np.float32,
        )
        center_gaps = center_gaps[center_gaps > 0]
        if len(center_gaps) < 3:
            return rebars, 0

        med_pitch = float(np.median(center_gaps))
        if med_pitch <= 1:
            return rebars, 0

        widths = np.asarray([r["edge"]["width_px"] for r in rebars], dtype=np.float32)
        width_px = float(np.median(widths))
        y_top = int(max(0, min(r["edge"]["y_top"] for r in rebars)))
        y_bottom = int(min(img_h - 1, max(r["edge"]["y_bottom"] for r in rebars)))
        candidate_centers = []
        if candidates:
            existing_centers = [float(r["edge"]["center_x"]) for r in rebars]
            for c in candidates:
                c_x = float(c.get("center_x", c.get("edge", {}).get("center_x", 0.0)))
                if c_x <= 0:
                    continue
                if any(abs(c_x - e_x) < width_px * 1.5 for e_x in existing_centers):
                    continue
                candidate_centers.append(c_x)

        inserted = []

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
                expected_cx = prev_cx + step * float(k + 1)
                usable_candidates = [
                    c_x for c_x in candidate_centers
                    if prev_cx + width_px * 2.0 < c_x < curr_cx - width_px * 2.0
                    and abs(c_x - expected_cx) < med_pitch * 0.35
                ]
                cx = min(usable_candidates, key=lambda c_x: abs(c_x - expected_cx)) if usable_candidates else expected_cx
                left = cx - width_px / 2.0
                right = cx + width_px / 2.0
                box = np.array(
                    [
                        [left, y_top],
                        [right, y_top],
                        [right, y_bottom],
                        [left, y_bottom],
                    ],
                    dtype=np.float32,
                )
                angle = 90.0
                left = cx - width_px / 2.0
                right = cx + width_px / 2.0
                inserted.append(
                    {
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
                    }
                )

        if not inserted:
            return rebars, 0

        merged = sorted(rebars + inserted, key=lambda item: item["edge"]["center_x"])
        for idx, r in enumerate(merged):
            r["id"] = f"H{idx + 1}"
        return merged, len(inserted)

    def measure_from_prediction(self, original_bgr, pred_bgr, save_dir, base_name):
        """
        Legacy color-prediction measurement path kept for comparison.
        """
        os.makedirs(save_dir, exist_ok=True)

        original_bgr = self._safe_to_bgr(original_bgr)
        pred_bgr = self._safe_to_bgr(pred_bgr)
        h, w = original_bgr.shape[:2]

        if pred_bgr.shape[:2] != original_bgr.shape[:2]:
            pred_bgr = cv2.resize(pred_bgr, (w, h), interpolation=cv2.INTER_NEAREST)

        red_mask = self._get_red_rebar_mask(pred_bgr)
        green_mask = self._get_green_mask(pred_bgr)

        y_scan, intervals = self._select_measurement_scanline(red_mask, green_mask)
        intervals = self._merge_close_intervals(intervals, gap_tol=3)

        final = []
        for interval in intervals:
            left, right, width_px = self._refine_interval_width_near_scanline(
                red_mask, green_mask, interval, y_scan
            )
            diameter_mm = width_px * self.mm_per_pixel

            if diameter_mm < self.min_diameter_mm or diameter_mm > self.max_diameter_mm:
                continue

            y_top, y_bottom = self._get_vertical_span_at_interval(red_mask, (left, right), y_scan)
            if y_bottom - y_top < h * 0.08:
                continue

            standard_mm = self._normalize_rebar_diameter(diameter_mm) if self.use_standard_diameter else None
            final.append(
                {
                    "edge": {
                        "left_x": left,
                        "right_x": right,
                        "center_x": (left + right) / 2.0,
                        "width_px": width_px,
                        "y_top": y_top,
                        "y_bottom": y_bottom,
                    },
                    "diameter_mm": float(diameter_mm),
                    "standard_mm": standard_mm,
                }
            )

        final = sorted(final, key=lambda r: r["edge"]["center_x"])

        # Bars in one image usually share the same nominal diameter. Raw mask
        # widths can be affected by burrs, slant, and occlusion, so keep raw
        # values while also deriving a per-image display diameter.
        image_raw_diameter_mm = None
        image_standard_mm = None
        if len(final) > 0:
            raw_ds = np.asarray([r["diameter_mm"] for r in final], dtype=np.float32)
            med_d = float(np.median(raw_ds))
            good = raw_ds[(raw_ds > med_d * 0.65) & (raw_ds < med_d * 1.35)]
            if len(good) >= 2:
                med_d = float(np.median(good))
            image_raw_diameter_mm = med_d
            image_standard_mm = self._normalize_rebar_diameter(med_d) if self.use_standard_diameter else med_d

            if self.unify_diameter_per_image:
                for r in final:
                    r["raw_diameter_mm"] = r["diameter_mm"]
                    r["diameter_mm"] = float(image_standard_mm if self.use_standard_diameter else image_raw_diameter_mm)
                    r["standard_mm"] = image_standard_mm if self.use_standard_diameter else None
            else:
                for r in final:
                    r["raw_diameter_mm"] = r["diameter_mm"]

        vis = original_bgr.copy()

        # Legacy path: overlay the prediction mask directly on the image.
        # Measurement still uses scanline edges; display is not regularized.
        self._overlay_prediction_mask(vis, red_mask, green_mask, alpha=0.60)

        font_scale = max(0.42, w / 2100.0)
        text_thick = max(1, int(w / 1000))
        line_thick = max(2, int(w / 800))

        # Draw the automatically selected scanline for debugging.
        cv2.line(vis, (0, int(y_scan)), (w - 1, int(y_scan)), (255, 180, 0), 1, lineType=cv2.LINE_AA)

        diameters = []
        standards = []
        spacings = [None] * len(final)

        for i, r in enumerate(final):
            edge = r["edge"]

            # Mark the left/right edges used for diameter measurement.
            y0 = int(y_scan)
            lx = int(round(edge["left_x"]))
            rx = int(round(edge["right_x"]))
            if self.show_edge_debug:
                cv2.line(vis, (lx, y0), (rx, y0), (255, 255, 255), 4, lineType=cv2.LINE_AA)
                cv2.line(vis, (lx, y0), (rx, y0), (255, 0, 0), 2, lineType=cv2.LINE_AA)
                cv2.line(vis, (lx, y0 - 18), (lx, y0 + 18), (0, 255, 0), 2, lineType=cv2.LINE_AA)
                cv2.line(vis, (rx, y0 - 18), (rx, y0 + 18), (0, 255, 0), 2, lineType=cv2.LINE_AA)

            d_mm = r["diameter_mm"]
            std_mm = r["standard_mm"]
            diameters.append(d_mm)
            standards.append(std_mm)

            if self.use_standard_diameter and std_mm is not None:
                raw_d = r.get("raw_diameter_mm", d_mm)
                if self.unify_diameter_per_image:
                    label = f"D:{std_mm:.0f}({raw_d:.1f})"
                else:
                    label = f"D:{d_mm:.1f}mm/S{std_mm:.0f}"
            else:
                label = f"D:{d_mm:.1f}mm"

            tx = int(edge["center_x"])
            ty = int(y_scan - 28 if (i % 2 == 0) else y_scan + 28)
            ty = max(25, min(h - 10, ty))
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_thick + 1)
            tx = max(5, min(w - tw - 5, tx - tw // 2))
            cv2.putText(
                vis,
                label,
                (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (0, 255, 255),
                text_thick + 1,
                lineType=cv2.LINE_AA,
            )

        for i in range(1, len(final)):
            prev = final[i - 1]["edge"]
            curr = final[i]["edge"]
            # Two clear-spacing modes:
            # measured_gap_mm uses mask edge-to-edge distance directly.
            # gap_mm may use center distance minus a unified diameter.
            measured_gap_px = curr["left_x"] - prev["right_x"]
            center_gap_px = curr["center_x"] - prev["center_x"]
            if center_gap_px <= 0:
                spacings[i] = None
                continue

            measured_gap_mm = float(measured_gap_px * self.mm_per_pixel) if measured_gap_px > 0 else None
            if self.unify_diameter_per_image and image_standard_mm is not None:
                gap_mm = float(center_gap_px * self.mm_per_pixel - image_standard_mm)
            else:
                gap_mm = measured_gap_mm

            if gap_mm is None or gap_mm <= 0:
                spacings[i] = None
                continue
            spacings[i] = gap_mm

            if measured_gap_px > 0:
                x1 = int(round(prev["right_x"]))
                x2 = int(round(curr["left_x"]))
            else:
                x1 = int(round(prev["center_x"]))
                x2 = int(round(curr["center_x"]))
            y_line = int(y_scan)

            cv2.line(vis, (x1, y_line), (x2, y_line), (0, 255, 255), line_thick, lineType=cv2.LINE_AA)
            cv2.circle(vis, (x1, y_line), line_thick + 2, (0, 0, 255), -1, lineType=cv2.LINE_AA)
            cv2.circle(vis, (x2, y_line), line_thick + 2, (0, 0, 255), -1, lineType=cv2.LINE_AA)

            gap_label = f"{gap_mm:.1f}mm"
            mid_x = (x1 + x2) // 2
            label_y = y_line - 22 if (i % 2 == 1) else y_line + 38
            label_y = max(30, min(h - 10, label_y))
            (gw, gh), _ = cv2.getTextSize(gap_label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_thick + 2)
            gx = max(2, min(w - gw - 2, mid_x - gw // 2))
            pad = 4

            cv2.rectangle(
                vis,
                (gx - pad, label_y - gh - pad),
                (gx + gw + pad, label_y + pad),
                (255, 255, 255),
                -1,
            )
            cv2.rectangle(
                vis,
                (gx - pad, label_y - gh - pad),
                (gx + gw + pad, label_y + pad),
                (0, 0, 0),
                1,
            )
            cv2.putText(
                vis,
                gap_label,
                (gx, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (0, 0, 0),
                text_thick + 2,
                lineType=cv2.LINE_AA,
            )

        result_path = os.path.join(save_dir, f"result_v51_{base_name}.jpg")
        pred_path = os.path.join(save_dir, f"predict_v51_{base_name}.png")
        mask_path = os.path.join(save_dir, f"debug_mask_v51_{base_name}.png")
        csv_path = os.path.join(save_dir, f"report_v51_{base_name}.csv")

        self._write_image_unicode(result_path, vis)
        self._write_image_unicode(pred_path, pred_bgr)
        self._write_image_unicode(mask_path, red_mask)

        rows = []
        for i, r in enumerate(final):
            edge = r["edge"]
            if i + 1 < len(final):
                adj = f"H{i + 1}~H{i + 2}"
                gap_val = f"{spacings[i + 1]:.2f}" if spacings[i + 1] is not None else "-"
            else:
                adj = "-"
                gap_val = "-"

            rows.append(
                [
                    base_name,
                    f"H{i + 1}",
                    f"{edge['left_x']:.2f}",
                    f"{edge['right_x']:.2f}",
                    f"{edge['center_x']:.2f}",
                    f"{r.get('raw_diameter_mm', r['diameter_mm']):.2f}",
                    f"{r['diameter_mm']:.2f}",
                    f"{r['standard_mm']:.0f}" if r["standard_mm"] is not None else "-",
                    adj,
                    gap_val,
                    f"{self.camera_distance_mm:.2f}",
                    f"{self.mm_per_pixel:.6f}",
                    f"{y_scan}",
                ]
            )

        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "image_name",
                    "rebar_id",
                    "left_edge_px",
                    "right_edge_px",
                    "center_px",
                    "raw_diameter_mm",
                    "display_diameter_mm",
                    "standard_diameter_mm",
                    "adjacent_pair",
                    "edge_to_edge_clear_spacing_mm",
                    "camera_distance_mm",
                    "mm_per_pixel",
                    "scanline_y_px",
                ]
            )
            writer.writerows(rows)

        print(f"  结果图: {result_path}")
        print(f"  预测图: {pred_path}")
        print(f"  mask图: {mask_path}")
        print(f"  CSV: {csv_path}")
        print(f"  扫描线 y={y_scan}, 检测到 {len(final)} 根纵筋")

        return {
            "rebars": final,
            "spacings": spacings,
            "rows": rows,
            "result_path": result_path,
            "predict_path": pred_path,
            "mask_path": mask_path,
            "csv_path": csv_path,
            "scanline_y": y_scan,
        }

    def measure_from_label(self, original_bgr, label_mask, save_dir, base_name):
        """Measure only from raw class labels. The label image must use 0/1/2 class ids."""
        os.makedirs(save_dir, exist_ok=True)

        original_bgr = self._safe_to_bgr(original_bgr)
        h, w = original_bgr.shape[:2]
        label_mask = np.asarray(label_mask)
        if label_mask.shape[:2] != (h, w):
            label_mask = cv2.resize(label_mask, (w, h), interpolation=cv2.INTER_NEAREST)
        if label_mask.ndim == 3:
            label_mask = label_mask[:, :, 0]
        label_mask = label_mask.astype(np.uint8)

        red_mask, green_mask = self._label_to_masks(label_mask)
        final, clean_mask, rejected = self._build_regular_rebars(red_mask, green_mask)
        final = sorted(final, key=lambda r: r["edge"]["center_x"])
        all_candidates = list(final)
        before_main_filter = len(final)
        final, measure_y = self._filter_main_rebars_for_spacing(final, h)
        filtered_by_main_line = max(0, before_main_filter - len(final))
        final, inserted_missing = self._insert_missing_rebars_by_gap(final, h, measure_y, all_candidates, red_mask)

        image_raw_diameter_mm = None
        image_standard_mm = None
        if len(final) > 0:
            raw_ds = np.asarray([r["edge"]["width_px"] * self.mm_per_pixel for r in final], dtype=np.float32)
            med_d = float(np.median(raw_ds))
            good = raw_ds[(raw_ds > med_d * 0.70) & (raw_ds < med_d * 1.30)]
            if len(good) >= 2:
                med_d = float(np.median(good))
            image_raw_diameter_mm = med_d
            image_standard_mm = self._normalize_rebar_diameter(med_d) if self.use_standard_diameter else med_d

        for r in final:
            raw_d = float(r["edge"]["width_px"] * self.mm_per_pixel)
            r["raw_diameter_mm"] = raw_d
            if self.unify_diameter_per_image and image_raw_diameter_mm is not None:
                r["diameter_mm"] = float(image_standard_mm if self.use_standard_diameter else image_raw_diameter_mm)
                r["standard_mm"] = image_standard_mm if self.use_standard_diameter else None
            elif (
                self.use_standard_diameter
                and self.small_rebar_nominal_mm is not None
                and image_raw_diameter_mm is not None
                and image_raw_diameter_mm <= self.small_rebar_nominal_mm * 1.15
                and raw_d <= self.small_rebar_nominal_mm * 1.35
            ):
                r["diameter_mm"] = float(self.small_rebar_nominal_mm)
                r["standard_mm"] = float(self.small_rebar_nominal_mm)
            elif (
                self.use_standard_diameter
                and image_raw_diameter_mm is not None
                and image_standard_mm is not None
                and image_standard_mm <= 12
                and image_raw_diameter_mm * 0.75 <= raw_d <= image_raw_diameter_mm * 1.35
            ):
                r["diameter_mm"] = float(image_standard_mm)
                r["standard_mm"] = image_standard_mm
            else:
                r["diameter_mm"] = raw_d
                r["standard_mm"] = self._normalize_rebar_diameter(raw_d) if self.use_standard_diameter else None

        vis = original_bgr.copy()
        font_scale = max(0.42, w / 2100.0)
        text_thick = max(1, int(w / 1000))
        line_thick = max(2, int(w / 800))

        for i, r in enumerate(final):
            self._draw_regular_rebar(vis, r, alpha=0.72)
            edge = r["edge"]
            std_mm = r["standard_mm"]
            if self.use_standard_diameter and std_mm is not None:
                label = f"D:{std_mm:.0f}"
            else:
                label = f"D:{r['diameter_mm']:.1f}mm"
            tx = int(edge["center_x"])
            ty = max(24, min(h - 8, int(edge["y_top"] + 24)))
            (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_thick + 1)
            tx = max(4, min(w - tw - 4, tx - tw // 2))
            cv2.putText(
                vis, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                font_scale, (0, 255, 255), text_thick + 1, lineType=cv2.LINE_AA
            )

        spacings = [None] * len(final)
        center_spacings = [None] * len(final)
        for i in range(1, len(final)):
            prev = final[i - 1]["edge"]
            curr = final[i]["edge"]
            center_gap_px = curr["center_x"] - prev["center_x"]
            measured_gap_px = curr["left_x"] - prev["right_x"]
            if center_gap_px <= 0 or measured_gap_px <= 0:
                continue
            center_spacing_mm = float(center_gap_px * self.spacing_mm_per_pixel)
            clear_spacing_mm = float(measured_gap_px * self.spacing_mm_per_pixel)
            if clear_spacing_mm <= 0:
                continue
            gap_int = int(round(clear_spacing_mm))
            spacings[i] = gap_int
            center_spacings[i] = int(round(center_spacing_mm))

            x1 = int(round(prev["right_x"]))
            x2 = int(round(curr["left_x"]))
            y_line = int(measure_y)

            cv2.line(vis, (x1, y_line), (x2, y_line), (0, 255, 255), line_thick, lineType=cv2.LINE_AA)
            cv2.circle(vis, (x1, y_line), line_thick + 2, (0, 0, 255), -1, lineType=cv2.LINE_AA)
            cv2.circle(vis, (x2, y_line), line_thick + 2, (0, 0, 255), -1, lineType=cv2.LINE_AA)

            gap_label = f"{gap_int}mm"
            mid_x = (x1 + x2) // 2
            label_y = y_line - 18 if (i % 2 == 1) else y_line + 34
            label_y = max(28, min(h - 8, label_y))
            (gw, gh), _ = cv2.getTextSize(gap_label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_thick + 2)
            gx = max(2, min(w - gw - 2, mid_x - gw // 2))
            pad = 4
            cv2.rectangle(vis, (gx - pad, label_y - gh - pad), (gx + gw + pad, label_y + pad), (255, 255, 255), -1)
            cv2.rectangle(vis, (gx - pad, label_y - gh - pad), (gx + gw + pad, label_y + pad), (0, 0, 0), 1)
            cv2.putText(
                vis, gap_label, (gx, label_y), cv2.FONT_HERSHEY_SIMPLEX,
                font_scale, (0, 0, 0), text_thick + 2, lineType=cv2.LINE_AA
            )

        result_path = os.path.join(save_dir, f"result_v53_{base_name}.jpg")

        self._write_image_unicode(result_path, vis)

        rows = []
        for i, r in enumerate(final):
            edge = r["edge"]
            if i + 1 < len(final):
                adj = f"H{i + 1}~H{i + 2}"
                gap_val = f"{spacings[i + 1]:.0f}" if spacings[i + 1] is not None else "-"
                center_val = "-" if center_spacings[i + 1] is None else f"{center_spacings[i + 1]:.0f}"
            else:
                adj = "-"
                gap_val = "-"
                center_val = "-"
            rows.append(
                [
                    base_name,
                    f"H{i + 1}",
                    f"{edge['left_x']:.2f}",
                    f"{edge['right_x']:.2f}",
                    f"{edge['center_x']:.2f}",
                    f"{r['raw_diameter_mm']:.2f}",
                    f"{r['diameter_mm']:.2f}",
                    f"{r['standard_mm']:.0f}" if r["standard_mm"] is not None else "-",
                    f"{r['angle_error']:.2f}",
                    adj,
                    gap_val,
                    center_val,
                    f"{self.camera_distance_mm:.2f}",
                    f"{self.spacing_mm_per_pixel:.6f}",
                    f"{self.mm_per_pixel:.6f}",
                ]
            )

        # --------------------------------------------------------------
        # Save the simplified CSV report requested for Excel.
        # One row corresponds to one detected longitudinal rebar.
        # UTF-8 with BOM prevents Chinese headers from becoming garbled
        # when the CSV file is opened directly in Microsoft Excel.
        # --------------------------------------------------------------
        csv_headers = [
            "图片名称",
            "钢筋编号",
            "检测直径/mm",
            "相邻区间",
            "检测间距/mm",
        ]
        csv_rows = []

        for i, r in enumerate(final):
            if self.use_standard_diameter and r.get("standard_mm") is not None:
                diameter_value = int(round(float(r["standard_mm"])))
            else:
                diameter_value = round(float(r["diameter_mm"]), 1)

            if i + 1 < len(final):
                adjacent_interval = f"H{i + 1}~H{i + 2}"
                spacing_value = spacings[i + 1] if spacings[i + 1] is not None else "-"
            else:
                adjacent_interval = "-"
                spacing_value = "-"

            csv_rows.append(
                [
                    base_name,
                    f"H{i + 1}",
                    diameter_value,
                    adjacent_interval,
                    spacing_value,
                ]
            )

        csv_path = os.path.join(save_dir, f"report_v53_{base_name}.csv")
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow(csv_headers)
            writer.writerows(csv_rows)

        reject_summary = {}
        for reason, _ in rejected:
            reject_summary[reason] = reject_summary.get(reason, 0) + 1
        if filtered_by_main_line > 0:
            reject_summary["not_on_main_measure_line"] = filtered_by_main_line
        if inserted_missing > 0:
            reject_summary["inserted_missing_rebar"] = inserted_missing
        print(f"  V53 measurement: {len(final)} valid bars, {len(rejected)} rejected {reject_summary}")
        print(f"  CSV: {csv_path}")

        return {
            "rebars": final,
            "spacings": spacings,
            "center_spacings": center_spacings,
            "rows": rows,
            "csv_headers": csv_headers,
            "csv_rows": csv_rows,
            "csv_path": csv_path,
            "result_path": result_path,
            "rejected": rejected,
            "measure_y": measure_y,
            "inserted_missing": inserted_missing,
        }

class PredictAndMeasureV53:
    def __init__(
        self,
        camera_distance_mm=500.0,
        use_standard_diameter=True,
        model_path="logs/Unet_resnet50.pth",
        scanline_ratio=0.50,
        unify_diameter_per_image=False,
        show_edge_debug=True,
        spacing_scale_factor=1.50,
        small_rebar_nominal_mm=10.0,
    ):
        self.model = Unet(
            model_path=model_path,
            num_classes=3,
            backbone="resnet50",
            input_shape=[640, 640],
            mix_type=1,
            cuda=False,
        )

        self.measurer = RebarMeasureV53(
            camera_distance_mm=camera_distance_mm,
            use_standard_diameter=use_standard_diameter,
            scanline_ratio=scanline_ratio,
            unify_diameter_per_image=unify_diameter_per_image,
            show_edge_debug=show_edge_debug,
            spacing_scale_factor=spacing_scale_factor,
            small_rebar_nominal_mm=small_rebar_nominal_mm,
        )

    def predict_one(self, image_path):
        """Read the source image and return original_bgr plus a 0/1/2 label mask."""
        image_path = image_path.strip().strip('"').strip("'")
        pil_img = Image.open(image_path).convert("RGB")

        original_rgb = np.array(pil_img)
        original_bgr = cv2.cvtColor(original_rgb, cv2.COLOR_RGB2BGR)
        label_mask = self.model.detect_label(pil_img)
        return original_bgr, label_mask

    def process_single_image(self, image_path, save_dir):
        image_path = image_path.strip().strip('"').strip("'")
        if not os.path.exists(image_path):
            print(f"File not found: {image_path}")
            return None

        base_name = os.path.splitext(os.path.basename(image_path))[0]
        os.makedirs(save_dir, exist_ok=True)

        print(f"Processing: {base_name}")
        t0 = time.time()
        original_bgr, label_mask = self.predict_one(image_path)
        ret = self.measurer.measure_from_label(original_bgr, label_mask, save_dir, base_name)
        dt = time.time() - t0

        print(f"  Bars: {len(ret['rebars'])}")
        print(f"  Result: {ret['result_path']}")
        print(f"  CSV: {ret['csv_path']}")
        print(f"  Time: {dt:.2f}s")
        return ret

    def process_folder(self, input_dir, save_dir):
        input_dir = input_dir.strip().strip('"').strip("'")
        if not os.path.isdir(input_dir):
            print(f"Input folder not found: {input_dir}")
            return

        os.makedirs(save_dir, exist_ok=True)
        valid_exts = (".bmp", ".dib", ".png", ".jpg", ".jpeg", ".pbm", ".pgm", ".ppm", ".tif", ".tiff")
        img_names = [f for f in os.listdir(input_dir) if f.lower().endswith(valid_exts)]
        img_names = [f for f in img_names if not f.lower().startswith(("result_", "predict_", "debug_"))]

        processed = 0
        summary_headers = [
            "图片名称",
            "钢筋编号",
            "检测直径/mm",
            "相邻区间",
            "检测间距/mm",
        ]
        summary_rows = []
        print(f"找到 {len(img_names)} 张图片")

        for img_name in tqdm(img_names):
            image_path = os.path.join(input_dir, img_name)
            ret = self.process_single_image(image_path, save_dir)
            if ret is None:
                continue
            processed += 1
            summary_rows.extend(ret.get("csv_rows", []))

        summary_csv_path = os.path.join(save_dir, "report_v53_summary.csv")
        with open(summary_csv_path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow(summary_headers)
            writer.writerows(summary_rows)

        print(f"\nV53完成：已生成 {processed} 张测距图，保存到 {save_dir}")
        print(f"批量汇总CSV: {summary_csv_path}")


def input_float(prompt, default_value):
    text = input(prompt).strip()
    if text == "":
        return float(default_value)
    try:
        return float(text)
    except ValueError:
        print(f"Invalid input, using default value: {default_value}")
        return float(default_value)


def input_yes_no(prompt, default=False):
    text = input(prompt).strip().lower()
    if text == "":
        return bool(default)
    return text in ["y", "yes", "1", "true", "是", "开", "开启"]


def input_text(prompt, default_value=""):
    text = input(prompt).strip().strip('"').strip("'")
    return text if text != "" else default_value


if __name__ == "__main__":
    print("=" * 72)
    print("  V53：钢筋批量识别、测量与 CSV 输出")
    print("=" * 72)
    print("只需要输入两个参数：")
    print("1. 原始图片文件夹路径")
    print("2. 拍摄距离（mm）")
    print("其余参数均已固定，结果自动保存到原图文件夹下的 v53_out。")
    print("=" * 72)

    # ==============================================================
    # 固定参数：通常不需要在每次运行时修改
    # ==============================================================
    USE_STANDARD_DIAMETER = True       # 按测量值匹配最近的标准钢筋规格
    SCANLINE_RATIO = 0.50              # 测量位置比例
    UNIFY_DIAMETER_PER_IMAGE = False   # 保留当前逐根规格判定逻辑
    SPACING_SCALE_FACTOR = 1.50        # 间距比例修正系数
    SHOW_EDGE_DEBUG = False            # 不显示旧版边界调试线
    SMALL_REBAR_NOMINAL_MM = None      # 不再强制把小直径钢筋归一为 D:10

    # 只输入原始图片文件夹
    input_dir = input("请输入原始图片文件夹路径: ").strip().strip('"').strip("'")
    if not os.path.isdir(input_dir):
        print(f"原始图片文件夹不存在: {input_dir}")
        sys.exit(1)

    # 只输入拍摄距离，直接回车时使用 500 mm
    camera_distance_mm = input_float("请输入拍摄距离/mm，默认500: ", 500.0)
    if camera_distance_mm <= 0:
        print("拍摄距离必须大于0，已自动使用默认值500 mm。")
        camera_distance_mm = 500.0

    # 输出文件夹自动建立在原图文件夹内
    save_dir = os.path.join(input_dir, "v53_out")

    # 权重路径自动查找，不再要求手动输入
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_candidates = [
        os.path.join(script_dir, "logs", "Unet_resnet50.pth"),
        os.path.join(os.getcwd(), "logs", "Unet_resnet50.pth"),
        os.path.join(os.path.dirname(input_dir), "logs", "Unet_resnet50.pth"),
    ]
    model_path = next((p for p in model_candidates if os.path.isfile(p)), None)

    if model_path is None:
        print("未找到权重文件 Unet_resnet50.pth。")
        print("请将权重放到脚本同级目录的 logs 文件夹中：")
        print(os.path.join(script_dir, "logs", "Unet_resnet50.pth"))
        sys.exit(1)

    print(f"\n原图文件夹: {input_dir}")
    print(f"拍摄距离: {camera_distance_mm:.1f} mm")
    print(f"权重文件: {model_path}")
    print(f"结果文件夹: {save_dir}\n")

    system = PredictAndMeasureV53(
        camera_distance_mm=camera_distance_mm,
        use_standard_diameter=USE_STANDARD_DIAMETER,
        model_path=model_path,
        scanline_ratio=SCANLINE_RATIO,
        unify_diameter_per_image=UNIFY_DIAMETER_PER_IMAGE,
        show_edge_debug=SHOW_EDGE_DEBUG,
        spacing_scale_factor=SPACING_SCALE_FACTOR,
        small_rebar_nominal_mm=SMALL_REBAR_NOMINAL_MM,
    )

    system.process_folder(input_dir, save_dir)
