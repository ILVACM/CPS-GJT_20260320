# Demo 3 — AGENTS.md（全局指导约束）

> 本文档定义 `demo/3/` 的开发目标、技术边界、API 验证清单及复用规范，
> 所有后续修改和扩展均以此为准。

---

## 一、项目定位

| 维度 | 说明 |
|------|------|
| **直接目的** | 在 ARM64 + openEuler 22 desktop 环境下，验证 Orbbec SDK 基础调用链可用 |
| **长期价值** | 为后续集成到其他程序提供可复用的 API 范式、字段名格式规范、错误处理模板；配套独立参数扫描工具（`scan_orbbec_336l.py`）供快速设备诊断 |
| **代码性质** | 精简但不粗糙，注释指向 SDK 官方文档与参考示例 |
| **运行平台** | 华为香橙派（ARM64 aarch64），openEuler 22.03 SP4 (desktop)，USB 直连 Gemini 336L |

---

## 二、功能需求（本次范围）

| # | 需求 | 技术验证意义 |
|---|------|-------------|
| F1 | 设备扫描识别，打印品牌/型号/PID/序列号/连接类型 | 验证 `Context.query_devices()` 及 `DeviceInfo` 各字段名与返回值类型 |
| F2 | RGB 实时显示（使用 ColorSensor 默认 profile，~30fps） | 验证 `Pipeline` / `Config` / `enable_stream()` / `wait_for_frames()` / `get_color_frame()` 链式调用 |
| F3 | 画面下方文本叠加显示设备关键参数 | 验证 tkinter 非阻塞渲染架构及字段拼接格式规范 |
| F4 | 按键 `q` 或 `ESC` 退出 | 验证 tkinter 事件绑定兼容性 |
| F5 | 关闭窗口正常退出 | 验证线程安全清理（`pipeline.stop` + 队列排空） |

### 不在范围内（后续迭代）

- Depth / 左红外 / 右红外 三路同时显示
- 点云 3D 渲染
- TCP 推流服务器
- PySide6 GUI

---

## 三、目录结构

```
demo/3/
├── source/                              # 部署资源（迁移时整体复制）
│   ├── 99-obsensor-libusb.rules         # udev 规则文件（免 sudo 访问 USB）
│   ├── install_udev_rules.sh            # udev 安装脚本（独立运行）
│   └── scan_orbbec_336l.py              # 设备参数扫描工具（独立脚本，无 GUI 依赖）
├── doc/                                 # 归档文档目录
│   ├── PLAN.md                          # 实现方案（开发前需人工确认）
│   ├── quickstart.md                    # 快速上手指引（部署完成后编写）
│   ├── Project-reference.md             # 集成参考文档（面向新开发者，含架构/抽离指南/FAQ）
│   └── scan_tool-readme.md              # 扫描工具使用说明与示例输出
├── temp/                                # 测试临时文件（调试产出，不纳入版本控制）
│   └── output.txt                       # 运行日志备份
├── img/                                 # 拍照保存目录（运行时自动创建，不纳入版本控制）
├── AGENTS.md                            # 本文件（全局指导约束，任何改动须同步更新）
├── requirements.txt                     # 运行时依赖声明
├── utils.py                             # 帧格式转换工具（复制自 demo/1/utils.py，禁止跨目录 import）
└── viewer.py                            # 主程序入口（~430 行，含扫描/采集/GUI/拍照四类职责）
```

> **注**：`img/` 目录在运行时由 `_on_snapshot()` 自动创建（`os.makedirs(..., exist_ok=True)`），不纳入版本控制。

---

## 四、技术约束

### 4.1 线程模型（强制）

```
┌──────────────────────────────────────────────────────┐
│                  tkinter 主线程                       │
│  root.mainloop()                                      │
│      ↓                                                │
│  _refresh()（每 33ms by after()）                     │
│      └─ queue.get_nowait() → 更新 Label               │
│      └─ bind("<KeyPress-q>") → root.quit()            │
│      └─ WM_DELETE_WINDOW → root.quit()                │
└──────────────────┬───────────────────────────────────┘
                   │ queue.put(img)
┌──────────────────▼───────────────────────────────────┐
│              后台采集线程（daemon Thread）             │
│  pipeline.wait_for_frames(1000)                      │
│      ↓                                                │
│  解码 + 入队（有界队列 maxsize=1，丢弃旧帧）           │
│  异常捕获 → 继续循环，不崩溃                           │
└──────────────────────────────────────────────────────┘
```

**关键原则**：
- `wait_for_frames()` 是阻塞调用，**严禁**放在 tkinter 主线程内（会卡死 UI）
- 队列必须是有界的（`maxsize=1`），确保始终展示最新帧
- 退出时先 `capture.stop()` 再 `root.quit()`，顺序不可颠倒

### 4.2 颜色空间转换链（固定）

```
SDK VideoFrame
    → frame_to_bgr_image()     （utils.py，处理 MJPG/YUYV/I420/RGB 等格式）
    → cv2.cvtColor(BGR→RGB)    （tkinter PhotoImage 需要 RGB）
    → PIL.Image.fromarray()    （转换为 tkinter 可接受的 Image 对象）
    → ImageTk.PhotoImage(image=…)  （⚠️ 必须保留引用到实例变量，否则被 GC 回收导致黑屏）
```

### 4.3 依赖声明（禁止随意增减）

```txt
opencv-python>=4.8
numpy<2.0                 # numpy 2.x 与 pyorbbecsdk v2.1.1 存在兼容性问题
Pillow                   # PIL.Image.fromarray() → tkinter PhotoImage
```

> `pyorbbecsdk` 通过本地 wheel 安装，**不写入** requirements.txt（wheel 路径由部署脚本指定）。

---

## 五、API 验证清单

在板上运行成功后，以下每项应被打勾确认。此清单也是后续集成的字段名/格式规范基准。

| # | 验证项 | 对应代码位置 | 通过标准 |
|---|--------|------------|---------|
| V1 | `Context().query_devices()` 成功 | `scan_device()` | 控制台打印出设备型号/PID/SN |
| V2 | `DeviceInfo.get_pid()` 返回 int，可格式化 hex | `scan_device()` | `0x{:04X}` 输出正确（如 `0x3360`） |
| V3 | `get_connection_type()` 返回字符串描述 | `scan_device()` | 打印出 `"USB"` / `"PCIe"` 等，非枚举对象 |
| V4 | `get_default_video_stream_profile()` 成功 | `FrameCapture.run()` | 获取到 1280×720@30 或同类默认 profile |
| V5 | `Config.enable_stream(profile)` 无异常 | `FrameCapture.run()` | 无 `OBError` |
| V6 | `FULL_FRAME_REQUIRE` 模式生效 | `FrameCapture.run()` | 帧集中同时含 Color 和 Depth 帧（可后续验证） |
| V7 | `wait_for_frames(1000)` 超时返回 None 正确处理 | `FrameCapture.run()` | 不会卡死，继续下一轮循环 |
| V8 | `get_color_frame()` 返回 `ColorFrame`（`VideoFrame` 子类） | `FrameCapture.run()` | 不为 None，有 `get_data()` 方法 |
| V9 | `frame_to_bgr_image()` 对 MJPG 格式处理正确 | `utils.py` + `FrameCapture.run()` | RGB 画面色彩正常（非偏蓝/偏绿） |
| V10 | tkinter 窗口以原始分辨率显示，无拉伸变形 | `ViewerApp.__init__()` | 画面填满窗口，无黑边/裁剪/变形 |
| V11 | 按 `q` 或 `ESC` 退出，线程安全清理 | `ViewerApp.run()` + 入口清理 | 无僵尸线程，无 SDK 报错残留 |
| V12 | 关闭 tkinter 窗口正常退出 | `protocol("WM_DELETE_WINDOW")` | 同 V11 |

---

## 六、部署步骤（板上执行）

```bash
# 1. 进入项目目录
cd /path/to/EBD-Orbbec

# 2. 创建 uv venv（首次）
uv venv --python python3 .venv
source .venv/bin/activate          # Windows 上改为：.venv\Scripts\activate

# 3. 安装 SDK wheel（从 Windows 开发机 scp 上传后）
uv pip install source/pyorbbecsdk2-2.1.1-cp39-cp39-linux_aarch64.whl

# 4. 安装运行依赖
#
#    【推荐】如网络较慢（国内访问 PyPI 不稳定），使用阿里云镜像加速：
#    --default-index 仅作用于当前命令，不修改 ~/.pip/pip.conf 等系统配置。
#    uv pip install --default-index https://mirrors.aliyun.com/pypi/simple -r demo/3/requirements.txt
#
#    【备选】网络正常时使用默认源：
#    uv pip install -r demo/3/requirements.txt
#
#    ⚠️ 注意：SDK wheel 为本地文件，安装时不需要也不应该加 --default-index。

# 5. 安装 udev 规则（USB 免 sudo 访问）
#    udev 相关文件位于 demo/3/source/ 目录
sudo ./demo/3/source/install_udev_rules.sh

# 6. 运行 Demo 3
cd demo/3
python viewer.py
```

### 6.1 常见问题排查

| 现象 | 可能原因 | 排查命令 |
|------|---------|---------|
| `ImportError: No module named pyorbbecsdk` | venv 未激活或 wheel 路径错误 | `which python`、`pip list \| grep orbbec` |
| `Bus error` / SDK 崩溃 | USB 连接不稳定或 libusb 问题 | `lsusb` 确认设备枚举，检查 USB 3.0 接口 |
| tkinter 黑屏 | PIL/Pillow 未安装或 PhotoImage 引用丢失 | `python -c "from PIL import Image"`；检查 `self.tk_photo` 引用 |
| 窗口标题乱码 | 终端编码非 UTF-8 | `export PYTHONIOENCODING=utf-8` |
| SELinux 拒绝 USB 访问 | openEuler SELinux enforcing 模式 | `sudo setenforce 0` 临时验证（不改策略仅作诊断） |

---

## 七、后续复用方向

本程序的以下组件可直接迁移到未来集成程序中，无需重写：

| 组件 | 复用方式 | 适用场景 |
|------|---------|---------|
| `scan_device()` | 复制到新项目入口函数 | 任何需要设备识别的场景 |
| `FrameCapture` 线程模型 | 继承基类，扩展 Depth/IR 流 | 多路流采集程序 |
| `_refresh()` + `after()` 模式 | 直接复用 | 所有 tkinter + OpenCV 显示场景 |
| `utils.frame_to_bgr_image()` | 整模块复制 | 帧格式转换通用工具 |
| 设备信息文本格式 | 作为状态栏统一规范 | 所有 GUI 应用的底部信息栏 |

---

## 八、项目复用指南

本项目定位为**供团队其他项目参考的示例项目**。如需将本项目中的能力集成到自己的程序中，请按以下指引操作。

### 8.1 复用入口

首次接触本项目、或计划将本项目能力集成到其他项目时，请优先阅读：

> **`doc/Project-reference.md`** — 涵盖项目概述、架构说明、快速启动、核心代码走读、集成抽取指南（含最小可运行代码片段）、常见问题 FAQ 及参考链接。

### 8.2 常见复用场景导航

| 复用需求 | 对应章节 |
|---------|---------|
| 只要 RGB 画面（不要深度，不要 GUI） | `Project-reference.md` §7.a |
| 只要深度数据（不要显示） | `Project-reference.md` §7.b |
| 只要相机设备信息（型号、序列号、固件版本等） | `Project-reference.md` §7.c / §10 |
| 只要拍照保存能力（集成到其他 GUI 程序） | `Project-reference.md` §7.d |
| 快速扫描设备参数（无 GUI 依赖） | `Project-reference.md` §10 |

### 8.3 环境前提提醒

复用前请确认满足以下基本条件：

- **硬件：** Orbbec Gemini 336L（或同系列设备），USB 3.0 接口
- **OS：** Linux ARM64（openEuler 22.03+）或 x86_64
- **Python：** 3.9+，推荐使用 `uv` 管理虚拟环境
- **SDK：** `pyorbbecsdk` v2.1.1（通过本地 wheel 安装）
- **依赖：** `opencv-python>=4.8`、`numpy<2.0`、`Pillow`

详细环境与安装步骤见 `Project-reference.md` §3、§4。

### 8.4 注意事项

- 本项目是**参考示例**，集成时需根据自身项目调整布局、性能优化、错误处理等细节。
- 遇到问题请先查阅 `Project-reference.md` §8（FAQ），大多数常见问题已在其中给出解决方案。
- 本项目中的 `utils.py`（帧格式转换工具）和 `scan_device()`（设备枚举函数）可直接复制到其他项目使用，无需引入 tkinter 依赖。

### 8.5 设备参数扫描工具

本项目配套提供独立的设备参数扫描脚本 `source/scan_orbbec_336l.py`，用于快速诊断设备状态：

| 特性 | 说明 |
|------|------|
| **运行环境** | 无 GUI 依赖，可在 SSH 远程或 headless 环境运行 |
| **核心功能** | 枚举设备、识别 336L 型号、打印参数表格 |
| **输出格式** | 三列表格：参数名 \| 原始字符串 \| 解析结果 |
| **复用价值** | 完整展示 SDK DeviceInfo 字段用法，可直接移植 |

详细使用说明见 `doc/scan_tool-readme.md`。

---

## 九、变更记录

| 日期 | 版本 | 变更内容 |
|------|------|---------|
| 2026-08-04 | v1.0 | 初始版本，确立 Demo 3 功能范围与技术约束 |
| 2026-08-04 | v1.1 | udev 规则文件移至 source/ 子目录，更新文档与脚本路径引用 |
| 2026-08-04 | v1.2 | 部署命令新增阿里云 PyPI 镜像加速选项（临时使用，不修改系统配置） |
| 2026-08-04 | v1.3 | 优化镜像命令呈现方式：提升为推荐路径，补充 `--default-index` 作用说明，区分 SDK wheel（本地文件）与 PyPI 依赖的安装差异 |
| 2026-08-04 | v1.4 | 修复 tkinter 白屏：引入 `ImageTk` 桥接，`self.tk_photo = ImageTk.PhotoImage(image=pil_img)`（根本原因：tk.PhotoImage 无 paste 方法，需通过 PIL ImageTk 转换） |
| 2026-08-04 | v1.5 | 新增拍照功能：GUI 增加"📷 拍照"按钮，点击保存当前帧至 demo/3/img/ 目录，文件名格式 YYYYMMDD_HHMMSS_mmm.jpg，JPEG 质量 95% |
| 2026-08-04 | v1.6 | 修复拍照按钮不可见 BUG：重构布局，将 label+info_label 包入 content_frame（expand=True），btn_frame 固定到底部（side="bottom"），窗口高度调整为 h+75 |
| 2026-08-04 | v1.7 | 新增 §八 项目复用指南章节，补充 doc/Project-reference.md 文件索引 |
| 2026-08-04 | v1.8 | 完善复用指南：补全常见场景导航表（§8.2）、环境前提提醒（§8.3）及注意事项（§8.4） |
| 2026-08-04 | v1.9 | 新增设备参数扫描工具 source/scan_orbbec_336l.py，补充 §8.5 小节说明及依赖安装指引 |