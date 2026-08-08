# Orbbec Gemini 336L Python SDK 集成参考

本文档基于 `demo/3/` 实现项目，面向需要快速集成 Orbbec 336L 深度相机的开发者。

---

## 1. 项目概述

这是一个在 ARM64 openEuler 22 desktop 环境下运行的 **Orbbec Gemini 336L RGB 实时显示 Demo**，演示了从设备枚举、流采集、帧解码到 GUI 渲染的完整调用链。核心定位是作为参考示例，帮助团队快速上手 PyOrbbecSDK v2.x 的基础用法，涵盖：

- 设备识别与参数读取
- Color 传感器数据流获取与解码
- tkinter 非阻塞渲染（~30fps）
- 拍照保存功能

运行环境：ARM64 / openEuler 22.03 SP4 / Python 3.9+ / uv 包管理工具链。

---

## 2. 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                        主线程 (GUI)                          │
│                                                             │
│   ┌──────────────┐    ┌──────────────┐    ┌──────────────┐ │
│   │ ViewerApp    │───▶│ content_frame│───▶│ Label        │ │
│   │              │    │ (expand=True)│    │ (图像显示)   │ │
│   └──────┬───────┘    └──────────────┘    └──────────────┘ │
│          │                   │                              │
│          │              ┌────▼────┐                         │
│          │              │info_label│ ← 型号/PID/SN/分辨率  │
│          │              └─────────┘                         │
│          │                                                   │
│          │              ┌─────────┐                         │
│          │              │btn_frame │ ← [📷 拍照] 按钮       │
│          │              └────┬────┘                         │
│          │                   │ click ──▶ _on_snapshot()     │
│          │                   │                             │
│          └───────────────────┘                              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                           ▲
                           │ queue.put(rgb)
                           │
┌──────────────────────────┴──────────────────────────────────┐
│                后台采集线程 (daemon Thread)                  │
│                                                             │
│   FrameCapture.run()                                        │
│      └─ Pipeline.start(cfg)                                │
│      └─ wait_for_frames(1000) → frames                      │
│      └─ get_color_frame() → ColorFrame                     │
│      └─ frame_to_bgr_image() → numpy array                 │
│      └─ cv2.cvtColor(BGR→RGB)                              │
│      └─ queue.put_nowait(rgb)                               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**核心模块对应文件：**

| 模块 | 源文件 | 关键类/函数 |
|------|--------|------------|
| 设备扫描 | `viewer.py` | `scan_device()` |
| 帧采集线程 | `viewer.py` | `FrameCapture` |
| GUI 主窗口 | `viewer.py` | `ViewerApp` |
| 帧格式转换 | `utils.py` | `frame_to_bgr_image()` |
| 拍照保存 | `viewer.py` | `ViewerApp._on_snapshot()` |

---

## 3. 环境与依赖

### 硬件要求
- Orbbec Gemini 336L 相机（USB 3.0 接口，PID `0x0807`）
- ARM64 开发板（如香橙派）或 x86_64 Linux 主机

### 软件要求
- OS: openEuler 22.03 SP4 (desktop) 或 Ubuntu 20.04+
- Python: 3.9+
- 包管理: `uv`（推荐）或 `pip`
- USB 权限: udev 规则文件 `source/99-obsensor-libusb.rules`

### Python 依赖（requirements.txt）
```txt
opencv-python>=4.8
numpy<2.0               # ⚠️ pyorbbecsdk v2.1.1 与 numpy 2.x 不兼容
Pillow                   # PIL + ImageTk
```

### SDK Wheel（本地安装）
```bash
# Linux ARM64 (openEuler)
uv pip install source/pyorbbecsdk2-2.1.1-cp39-cp39-linux_aarch64.whl

# Linux x86_64（如有需要）
uv pip install source/pyorbbecsdk2-2.1.1-cp39-cp39-linux_x86_64.whl
```

### udev 规则安装
```bash
sudo ./source/install_udev_rules.sh
# 执行后重新插拔相机即可免 sudo 访问
```

---

## 4. 快速启动（Quick Start）

以下命令假设已在 `demo/3/` 目录中：

```bash
# Step 1: 激活虚拟环境
source ../.venv/bin/activate

# Step 2: 安装依赖（国内网络建议用阿里云镜像）
uv pip install --default-index https://mirrors.aliyun.com/pypi/simple -r requirements.txt

# Step 3: 安装本地 SDK wheel（如需更新）
uv pip install source/pyorbbecsdk2-2.1.1-cp39-cp39-linux_aarch64.whl

# Step 4: 安装 udev 规则（首次使用或更换设备时）
sudo ./source/install_udev_rules.sh

# Step 5: 运行程序
python viewer.py
```

**预期输出：**
```
==================================================
  Orbbec Gemini 336L RGB Viewer (Demo 3)
  平台: ARM64 / openEuler 22 desktop
==================================================
[扫描] 已识别设备:
  型号  : Orbbec Gemini 336L
  PID   : 0x0807
  序列号: CPCG853000MT
  连接  : USB3.2
[采集] Color 默认 profile: 1280x720 @ 30fps
[采集] Pipeline 已启动，开始取帧 ...
```

退出方式：按 `q` 键、`ESC` 键或关闭窗口。

---

## 5. 目录结构说明

```
demo/3/
├── source/                         # 部署资源（迁移时整体复制）
│   ├── 99-obsensor-libusb.rules    # udev 规则：授予 USB 设备访问权限
│   └── install_udev_rules.sh       # udev 安装脚本（需 sudo 执行）
├── doc/                            # 归档文档
│   ├── PLAN.md                     # 开发实现方案
│   └── quickstart.md               # 快速上手指南（部署后编写）
├── temp/                           # 测试临时文件
│   └── output.txt                  # 运行日志备份
├── img/                            # 拍照保存目录（运行时自动创建）
├── AGENTS.md                       # 项目约束文档（本 demo 的全局规范）
├── requirements.txt                # Python 依赖声明
├── utils.py                        # 帧格式转换工具（支持 MJPG/YUYV/I420/RGB/NV12/NV21）
└── viewer.py                       # 主程序入口（约 430 行）
```

---

## 6. 核心代码走读

### 6.1 设备枚举与参数读取

**来源：** `viewer.py` 第 32~65 行，函数 `scan_device()`

```python
def scan_device():
    ctx = Context()
    dev_list = ctx.query_devices()
    if dev_list.get_count() == 0:
        return None

    dev = dev_list[0]
    info = dev.get_device_info()

    name   = info.get_name()            # "Orbbec Gemini 336L"
    pid    = info.get_pid()             # int, 如 0x0807
    serial = info.get_serial_number()   # "CPCG853000MT"
    conn   = info.get_connection_type() # "USB3.2"

    print(f"  型号  : {name}")
    print(f"  PID   : 0x{pid:04X}")
    print(f"  序列号: {serial}")
    print(f"  连接  : {conn}")

    return dev
```

**用途：** 验证 V1-V3 API 调用，返回 Device 对象供后续使用。

---

### 6.2 Profile 获取与 Pipeline 启动

**来源：** `viewer.py` 第 130~160 行，方法 `FrameCapture.run()`

```python
pipeline = Pipeline()
cfg = Config()

# 获取 ColorSensor 默认 profile
profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
color_profile = profiles.get_default_video_stream_profile()
self._rgb_w = color_profile.get_width()   # 1280
self._rgb_h = color_profile.get_height()  # 720

# 启用 Color 流
cfg.enable_stream(color_profile)

# 设置帧聚合模式（可选，确保同时获取多路帧）
cfg.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)

# 启动采集
pipeline.start(cfg)
print(f"[采集] Color 默认 profile: {w}x{h} @ {fps}fps")
```

**用途：** 配置相机流参数并启动采集线程。

---

### 6.3 帧解码与入队

**来源：** `viewer.py` 第 165~195 行，方法 `FrameCapture.run()`

```python
while not self._stopped:
    frames = pipeline.wait_for_frames(1000)  # 超时 1000ms
    if frames is None:
        continue

    color_frame = frames.get_color_frame()
    if color_frame is None:
        continue

    # 调用通用转换函数（utils.py）
    bgr = frame_to_bgr_image(color_frame)
    if bgr is None:
        continue

    # BGR → RGB（tkinter 需要 RGB）
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    # 非阻塞入队，丢弃旧帧
    try:
        self._queue.put_nowait(rgb)
    except queue.Full:
        pass
```

**用途：** 后台线程持续采集并解码，保证主线程不被阻塞。

---

### 6.4 tkinter 渲染循环

**来源：** `viewer.py` 第 280~295 行，方法 `ViewerApp._refresh()`

```python
def _refresh(self):
    img = self.capture.get_latest_frame()
    if img is not None:
        pil_img = Image.fromarray(img)
        self.tk_photo = ImageTk.PhotoImage(image=pil_img)  # ← 必须保留引用！
        self.label.configure(image=self.tk_photo)
    self.root.after(33, self._refresh)  # ~30fps
```

**关键陷阱：**
- `self.tk_photo` 必须保存为实例变量，否则 Python GC 会回收底层 C 图像数据，导致黑屏。
- 不能写 `tk.PhotoImage(image=pil_img)`，PhotoImage 构造器不接受 PIL Image 对象。
- 正确写法是使用 `ImageTk.PhotoImage(image=pil_img)`（来自 PIL）。

---

### 6.5 拍照保存功能

**来源：** `viewer.py` 第 230~270 行，方法 `ViewerApp._on_snapshot()`

```python
def _on_snapshot(self):
    img = self.capture.get_latest_frame()
    if img is None:
        print("[拍照] 警告：当前无可用帧")
        return

    # 生成唯一文件名：YYYYMMDD_HHMMSS_mmm.jpg
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    filename = f"{timestamp}.jpg"
    filepath = os.path.join(self.save_dir, filename)

    # 保存前转换回 BGR（cv2.imwrite 接受 BGR）
    bgr_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    success = cv2.imwrite(filepath, bgr_img, [cv2.IMWRITE_JPEG_QUALITY, 95])

    if success:
        print(f"[拍照] 已保存: {filepath}")
        self.info_label.config(text=f"✅ 已保存: {filename}")
        self.root.after(2000, lambda: self.info_label.config(text=self._build_info_text()))
    else:
        print(f"[拍照] 失败: 无法写入 {filepath}")
```

**用途：** 捕获当前显示帧，异步保存为 JPEG 文件（不阻塞预览）。

---

## 7. 集成抽取指南

### 场景 a) 只要 RGB 画面（不要深度，不要 GUI）

**抽取来源：** `viewer.py` `FrameCapture` 类 + `utils.py`

**最小可运行代码：**
```python
import queue
import threading
import numpy as np
import cv2
from pyorbbecsdk import *
from utils import frame_to_bgr_image

class RgbCapture(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self._queue = queue.Queue(maxsize=1)
        self._stopped = False

    def run(self):
        pipeline = Pipeline()
        cfg = Config()
        profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        profile = profiles.get_default_video_stream_profile()
        cfg.enable_stream(profile)
        pipeline.start(cfg)

        while not self._stopped:
            frames = pipeline.wait_for_frames(1000)
            if frames is None:
                continue
            color = frames.get_color_frame()
            if color is None:
                continue
            bgr = frame_to_bgr_image(color)
            if bgr is None:
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            try:
                self._queue.put_nowait(rgb)
            except queue.Full:
                pass

    def stop(self):
        self._stopped = True

    def get_latest_frame(self):
        try:
            return self._queue.get_nowait()
        except queue.Empty:
            return None

# 使用示例
capture = RgbCapture()
capture.start()
while True:
    frame = capture.get_latest_frame()
    if frame is not None:
        print(f"Got frame: {frame.shape}")
    # 处理 frame...
capture.stop()
```

---

### 场景 b) 只要深度数据（不要显示）

**说明：** 本项目仅验证 RGB 流，深度数据采集需额外配置 DepthSensor profile。

**参考示例（PyOrbbecSDK 官方）：**
```python
from pyorbbecsdk import *

pipeline = Pipeline()
cfg = Config()

# 获取 Depth 默认 profile
profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
depth_profile = profiles.get_default_video_stream_profile()
cfg.enable_stream(depth_profile)
pipeline.start(cfg)

frames = pipeline.wait_for_frames(1000)
depth_frame = frames.get_depth_frame()
depth_data = depth_frame.get_data()  # numpy array, 单位 mm
```

---

### 场景 c) 只要相机设备信息（型号、序列号、固件版本等）

**抽取来源：** `viewer.py` `scan_device()` 函数（第 32~65 行）

**最小可运行代码：**
```python
from pyorbbecsdk import *

ctx = Context()
dev_list = ctx.query_devices()
if dev_list.get_count() == 0:
    print("No device found")
else:
    dev = dev_list[0]
    info = dev.get_device_info()
    print(f"Name  : {info.get_name()}")
    print(f"PID   : 0x{info.get_pid():04X}")
    print(f"SN    : {info.get_serial_number()}")
    print(f"Conn  : {info.get_connection_type()}")
    # 其他可用字段: get_firmware_version(), get_name()
```

---

### 场景 d) 只要拍照保存能力（集成到其他 GUI 程序）

**抽取来源：** `viewer.py` `ViewerApp._on_snapshot()` 方法（第 230~270 行）

**最小可运行代码：**
```python
import os
import cv2
from datetime import datetime

def save_snapshot(frame_rgb, save_dir="img", quality=95):
    """
    将 RGB 帧保存为 JPEG 文件。
    
    Args:
        frame_rgb: numpy array, shape (H, W, 3), dtype uint8
        save_dir: 保存目录，相对路径或绝对路径
        quality: JPEG 质量 (1-100)
    
    Returns:
        str: 保存的文件路径，失败返回 None
    """
    if frame_rgb is None:
        return None
    
    os.makedirs(save_dir, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    filename = f"{timestamp}.jpg"
    filepath = os.path.join(save_dir, filename)
    
    # cv2.imwrite 接受 BGR，需转换
    bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    success = cv2.imwrite(filepath, bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    
    return filepath if success else None
```

---

## 8. 常见问题与排错（FAQ）

### Q1: `ImportError: No module named pyorbbecsdk`
- **现象：** 运行时报找不到 SDK 模块
- **原因：** venv 未激活或 wheel 路径错误
- **排查：**
  ```bash
  which python
  pip list | grep orbbec
  ```
- **解决：** 确认 `.venv` 已激活，wheel 已正确安装

---

### Q2: `Bus error` / SDK 崩溃
- **现象：** 程序异常退出，报 Bus error
- **原因：** USB 3.0 接口不稳定或 libusb 问题
- **排查：**
  ```bash
  lsusb | grep 2bc5  # 确认设备枚举
  dmesg | tail -20   # 查看内核日志
  ```
- **解决：** 更换 USB 3.0 接口，确保使用高质量数据线

---

### Q3: tkinter 黑屏 / 白屏
- **现象：** 窗口打开但无图像
- **原因 A：** PhotoImage 引用被 GC 回收
  - **解决：** 保存为实例变量 `self.tk_photo = ...`
- **原因 B：** PIL/Pillow 未安装
  - **解决：** `pip install Pillow`
- **原因 C：** 颜色空间错误
  - **解决：** 确认转换链：`BGR → RGB → PIL.Image.fromarray() → ImageTk.PhotoImage`

---

### Q4: numpy 版本冲突
- **现象：** 导入 SDK 时报错或运行时崩溃
- **原因：** pyorbbecsdk v2.1.1 与 numpy 2.x 不兼容
- **解决：**
  ```bash
  pip install "numpy<2.0"
  ```

---

### Q5: 拍照按钮不可见
- **现象：** 按钮不在窗口可视区域内
- **原因：** pack 布局顺序错误，label 先占据全部空间
- **解决：** 使用容器 Frame 包裹 label + info_label，按钮固定到底部（详见 v1.6 修复）

---

### Q6: PyPI 下载慢
- **现象：** `pip install` 卡住或超时
- **解决：** 使用阿里云镜像加速（临时，不修改系统配置）：
  ```bash
  pip install --index-url https://mirrors.aliyun.com/pypi/simple/ package_name
  ```

---

### Q7: SELinux 拒绝 USB 访问
- **现象：** 程序无报错但无法获取设备
- **原因：** openEuler SELinux enforcing 模式限制
- **解决：**
  ```bash
  sudo setenforce 0  # 临时关闭（仅作诊断）
  # 生产环境请配置正确的 SELinux 策略
  ```

---

### Q8: `AttributeError: 'PhotoImage' object has no attribute 'paste'`
- **现象：** 运行时抛出此异常
- **原因：** 误用 `tk.PhotoImage` 的 paste 方法（该方法不存在）
- **解决：** 改用 `ImageTk.PhotoImage(image=pil_img)`

---

## 9. 参考链接

| 资源 | 链接 |
|------|------|
| PyOrbbecSDK GitHub | https://github.com/orbbec/pyorbbecsdk |
| PyOrbbecSDK 官方文档 | https://orbbecdev.github.io/pyorbbecsdk/ |
| OrbbecSDK C++ 示例 | `OrbbecSDK_v2.8.7_*/examples/src/` |
| openEuler 官方文档 | https://www.openeuler.org/ |
| udev 规则参考 | https://wiki.archlinux.org/title/udev |

---

## 10. 奥比中光 336L 关键参数扫描参考

### 10.1 脚本定位

`doc/scan_orbbec_336l.py` 是一个独立的设备参数扫描工具，用于快速诊断和查看 Orbbec Gemini 336L 相机的关键参数。与 `viewer.py`（实时显示 Demo）不同，本脚本：

- **无 GUI 依赖**，可在 SSH 远程或 headless 环境运行
- **无需相机实时采集**，仅读取设备静态参数
- **输出结构化表格**，每个参数同时显示原始值与解析结果

适用场景：
- 快速确认设备连接状态
- 获取 PID、SN、固件版本等关键参数
- 查看所有支持的分辨率配置
- 多设备并行检测（自动过滤目标型号）

---

### 10.2 关键实现点

#### SDK 初始化与设备枚举

```python
from pyorbbecsdk import Context, OBError, OBSensorType, Pipeline

ctx = Context()
dev_list = ctx.query_devices()
count = dev_list.get_count()

for i in range(count):
    dev = dev_list[i]
    info = dev.get_device_info()
    # ...
```

**说明**：
- `Context()` 是 SDK 的入口点，初始化 USB 通信
- `query_devices()` 返回所有已连接的 Orbbec 设备列表
- 若返回空列表，表示未检测到设备（检查 USB 连接和 udev 权限）

---

#### 目标设备识别

```python
TARGET_KEYWORD = "336L"

def find_target_device(devices, keyword=TARGET_KEYWORD):
    for dev, info in devices:
        name = info.get_name()
        if isinstance(name, str) and keyword in name:
            return (dev, info)
    return None
```

**说明**：
- 自动筛选型号包含关键词的设备
- 支持多设备场景（如同时连接多台 336L）
- 未找到时给出友好提示并列出所有已检测到的设备

---

#### 参数获取方式

| 参数 | SDK API | 原始类型 | 解析结果 |
|------|---------|---------|---------|
| 品牌/Vendor | 通过 PID 反推 | `int` (PID) | `"Orbbec (USB VID: 0x2BC5)"` |
| 设备型号 | `info.get_name()` | `str` | 去除空白字符后的字符串 |
| PID | `info.get_pid()` | `int` | `"0x{:04X}".format(pid)` |
| 序列号 | `info.get_serial_number()` | `str` | 直接输出 |
| 固件版本 | `info.get_firmware_version()` | `str` | 直接输出 |
| 连接类型 | `info.get_connection_type()` | `str` | 直接输出 |
| 深度流分辨率 | `Pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)` | `list[dict]` | `"{w}x{h}@{fps}fps ({fmt})"` |
| 彩色流分辨率 | `Pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)` | `list[dict]` | `"{w}x{h}@{fps}fps ({fmt})"` |

**注意**：
- SDK 无直接 vendor 字段，需根据 PID 反推（Orbbec 统一为 `0x2BC5`）
- 分辨率数据需通过临时创建 `Pipeline` 对象查询 profile 列表获取

---

### 10.3 输出格式说明

脚本采用三列表格格式输出，便于阅读和日志比对：

```
========================================================================
  参数名称                       | 原始字符串                                   | 解析结果
========================================================================
  品牌/Vendor                    | PID=0x0807                                   | Orbbec (USB VID: 0x2BC5)
  设备型号                       | 'Orbbec Gemini 336L'                        | Orbbec Gemini 336L
  PID                            | 2055                                         | 0x0807
  ...
------------------------------------------------------------------------
```

| 列名 | 说明 |
|------|------|
| **参数名称** | 人类可读的参数描述 |
| **原始字符串** | SDK/API 返回的原始值（保留引号、类型信息） |
| **解析结果** | 经过格式化、清洗后的可读值 |

对于分辨率列表，每项配置占一行，展示宽×高@帧率+格式。

---

### 10.4 可复用代码片段

以下函数可直接移植到其他项目：

#### 设备枚举函数

**来源：** `doc/scan_orbbec_336l.py` `enumerate_devices()`（第 75~105 行）

```python
from pyorbbecsdk import Context, OBError

def enumerate_orbbec_devices():
    """枚举所有已连接的 Orbbec 设备，返回 (Device, DeviceInfo) 列表。"""
    try:
        ctx = Context()
        dev_list = ctx.query_devices()
        if dev_list.get_count() == 0:
            return []
        
        devices = []
        for i in range(dev_list.get_count()):
            dev = dev_list[i]
            info = dev.get_device_info()
            devices.append((dev, info))
        return devices
    except OBError as e:
        print(f"[错误] SDK 初始化失败: {e}")
        return []
```

---

#### 目标设备查找函数

**来源：** `doc/scan_orbbec_336l.py` `find_target_device()`（第 108~120 行）

```python
def find_device_by_model(devices, keyword="336L"):
    """从设备列表中查找型号包含关键词的目标设备。"""
    for dev, info in devices:
        name = info.get_name()
        if isinstance(name, str) and keyword in name:
            return (dev, info)
    return None
```

---

#### Profile 查询函数

**来源：** `doc/scan_orbbec_336l.py` `get_supported_profiles()`（第 123~145 行）

```python
from pyorbbecsdk import Pipeline, OBSensorType

def get_sensor_profiles(device, sensor_type):
    """获取指定传感器类型的所有支持分辨率配置。"""
    profiles = []
    pipeline = Pipeline()
    profile_list = pipeline.get_stream_profile_list(sensor_type)
    
    for i in range(profile_list.get_count()):
        sp = profile_list.get_video_stream_profile(i)
        profiles.append({
            "width": sp.get_width(),
            "height": sp.get_height(),
            "fps": sp.get_fps(),
            "format": str(sp.get_format()),
        })
    return profiles
```

---

### 10.5 依赖与运行命令

**依赖：**
- `pyorbbecsdk >= 2.0.18`（与 demo3 一致）
- Python 3.9+
- 无其他第三方依赖（仅需 stdlib）

**安装命令：**
```bash
# 激活虚拟环境
source ../.venv/bin/activate

# 安装 SDK wheel
uv pip install source/pyorbbecsdk2-2.1.1-cp39-cp39-linux_aarch64.whl
```

**运行命令：**
```bash
cd demo/3
python doc/scan_orbbec_336l.py
```

或从项目根目录运行：
```bash
cd /path/to/EBD-Orbbec
python demo/3/doc/scan_orbbec_336l.py
```

---

### 10.6 示例输出

```
============================================================
  Orbbec Gemini 336L 设备信息扫描工具
  平台: ARM64 / openEuler 22 desktop
  SDK : pyorbbecsdk (与 demo3 一致)
============================================================
[信息] 共检测到 1 台 Orbbec 设备。
[确认] 已定位目标设备: Orbbec Gemini 336L

========================================================================
  设备标识: SN=CPCG8530...
========================================================================
  参数名称                       | 原始字符串                                   | 解析结果
========================================================================
  品牌/Vendor                    | PID=0x0807                                   | Orbbec (USB VID: 0x2BC5)
  设备型号                       | 'Orbbec Gemini 336L'                        | Orbbec Gemini 336L
  PID                            | 2055                                         | 0x0807
  序列号/SN                      | 'CPCG853000MT'                              | CPCG853000MT
  固件版本                       | '1.8.10.100'                                | 1.8.10.100
  连接类型                       | 'USB3.2'                                     | USB3.2
------------------------------------------------------------------------

  [深度流 Depth Stream]
========================================================================
  参数名称                       | 原始字符串                                   | 解析结果
========================================================================
  分辨率配置                     | w=640, h=480, fps=30, fmt=OBFormat.MJPG      | 640x480@30fps (OBFormat.MJPG)
  分辨率配置                     | w=1280, h=720, fps=30, fmt=OBFormat.MJPG     | 1280x720@30fps (OBFormat.MJPG)
  分辨率配置                     | w=640, h=480, fps=60, fmt=OBFormat.MJPG      | 640x480@60fps (OBFormat.MJPG)
------------------------------------------------------------------------

  [彩色流 Color Stream]
========================================================================
  参数名称                       | 原始字符串                                   | 解析结果
========================================================================
  分辨率配置                     | w=640, h=480, fps=30, fmt=OBFormat.MJPG      | 640x480@30fps (OBFormat.MJPG)
  分辨率配置                     | w=1280, h=720, fps=30, fmt=OBFormat.MJPG     | 1280x720@30fps (OBFormat.MJPG)
  分辨率配置                     | w=1920, h=1080, fps=30, fmt=OBFormat.MJPG    | 1920x1080@30fps (OBFormat.MJPG)
------------------------------------------------------------------------

============================================================
  扫描完成。
============================================================
```

---

### 10.7 注意事项

#### 多设备场景

脚本会自动枚举所有 Orbbec 设备，并按关键词过滤。未找到目标设备时，会列出所有已检测到的设备供用户核对：

```
[提示] 未找到型号包含 '336L' 的设备。
       当前检测到的设备列表：
         - Orbbec Astra Mini  (PID: 0x069D)
         - Orbbec Gemini 335L (PID: 0x0800)
```

#### 未连接设备

若输出 `[提示] 未检测到任何 Orbbec 设备`，请按以下步骤排查：

```bash
lsusb | grep 2bc5          # 确认设备被系统识别
dmesg | tail -20           # 查看内核日志
sudo ./source/install_udev_rules.sh  # 重新安装 udev 规则
```

#### USB 权限不足

报错 `Bus error` 或 `OBError: ...` 时，通常是 USB 权限问题：

```bash
# 检查当前用户是否有 USB 访问权限
ls -la /dev/bus/usb/*/00*

# 重新加载 udev 规则
sudo udevadm control --reload && sudo udevadm trigger
```

#### 回退获取方式

若 SDK 无法获取某些字段（如品牌），脚本提供了回退逻辑：
- **品牌**：通过 PID 反推（Orbbec 固定为 `0x2BC5`）
- **固件版本**：若 SDK 返回异常，显示 `(SDK 未提供)`

*文档版本：v1.1 | 基于 demo/3/doc/scan_orbbec_336l.py v1.9 | 最后更新：2026-08-04*
