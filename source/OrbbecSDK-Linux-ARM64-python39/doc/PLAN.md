# Demo 3 — 实现方案

> 本文档基于 `../AGENTS.md` 约束，描述 `viewer.py`、`utils.py`、`requirements.txt` 的具体实现细节，供人工确认后进入编码阶段。
> 所有设计决策须符合 AGENTS.md 第四节的「技术约束」与第五节的「API 验证清单」。

---

## 一、实现范围

```
demo/3/
├── source/                              # 部署资源（迁移时整体复制）
│   ├── 99-obsensor-libusb.rules         # udev 规则文件（免 sudo 访问 USB）
│   ├── install_udev_rules.sh            # udev 安装脚本（独立运行）
│   └── scan_orbbec_336l.py              # 设备参数扫描工具（独立脚本，无 GUI 依赖）
├── doc/                                 # 归档文档目录
│   ├── PLAN.md                          # 本文件（实现方案，开发前需人工确认）
│   ├── quickstart.md                    # 快速上手指引（部署完成后编写）
│   ├── Project-reference.md             # 集成参考文档（面向新开发者，含架构/抽离指南/FAQ）
│   └── scan_tool-readme.md              # 扫描工具使用说明与示例输出
├── temp/                                # 测试临时文件（调试产出，不纳入版本控制）
│   └── output.txt                       # 运行日志备份
├── img/                                 # 拍照保存目录（运行时自动创建，不纳入版本控制）
├── AGENTS.md                            # 全局指导约束（当前目录外部）
├── requirements.txt                     # 运行时依赖声明
├── utils.py                             # 帧格式转换工具（复制自 demo/1/utils.py，禁止跨目录 import）
└── viewer.py                            # 主程序入口（~430 行，含扫描/采集/GUI/拍照四类职责）
```

| 文件 | 路径 | 行数估计 | 职责 |
|------|------|---------|------|
| `viewer.py` | `demo/3/viewer.py` | ~430 行 | 主程序：设备扫描 + 后台采集线程 + tkinter GUI + 拍照 |
| `utils.py` | `demo/3/utils.py` | ~150 行 | 帧格式转换（复制自 demo/1/utils.py，零修改） |
| `requirements.txt` | `demo/3/requirements.txt` | ~4 行 | 依赖声明 |
| `99-obsensor-libusb.rules` | `demo/3/source/99-obsensor-libusb.rules` | ~30 行 | udev 规则（免 sudo 访问 USB 设备） |
| `install_udev_rules.sh` | `demo/3/source/install_udev_rules.sh` | ~30 行 | udev 安装脚本（独立运行，不依赖 SDK 子目录） |
| `scan_orbbec_336l.py` | `demo/3/source/scan_orbbec_336l.py` | ~290 行 | 设备参数扫描工具（无 GUI 依赖） |
| `quickstart.md` | `demo/3/doc/quickstart.md` | ~80 行 | 部署与启动指引（开发完成后编写） |
| `Project-reference.md` | `demo/3/doc/Project-reference.md` | ~670 行 | 集成参考文档（面向新开发者） |
| `scan_tool-readme.md` | `demo/3/doc/scan_tool-readme.md` | ~170 行 | 扫描工具使用说明 |

---

## 二、模块分解与接口定义

### 2.1 utils.py — 帧转换工具

**策略**：直接复制 `demo/1/utils.py`，不做任何修改。

**理由**：
- 该文件已在 demo/1 中验证可用
- 跨目录 import 在包结构混乱时容易出错（`from ..utils` 不可靠）
- AGENTS.md 明确要求"禁止跨目录 import"

**需重点验证的函数**（对应 V9 验证项）：
```python
frame_to_bgr_image(frame: VideoFrame) -> Optional[np.ndarray]
```
- 输入：`ColorFrame`（即 `VideoFrame` 子类），格式为 `OBFormat.RGB` / `OBFormat.MJPG` / `OBFormat.YUYV` / `OBFormat.I420` 等
- 输出：`np.ndarray`，shape `(height, width, 3)`，dtype `uint8`，BGR 顺序
- 异常处理：格式不支持时打印警告并返回 `None`，不抛出异常

---

### 2.2 viewer.py — 主程序

#### 2.2.1 顶层入口

```python
if __name__ == "__main__":
    # ① 扫描设备（同时做 V1~V3 验证）
    device = scan_device()
    if device is None:
        sys.exit(1)

    dev_info = device.get_device_info()

    # ② 启动后台采集线程
    capture = FrameCapture(device)
    capture.start()

    # ③ 启动 tkinter GUI
    app = ViewerApp(capture, dev_info)
    app.run()

    # ④ 退出清理（顺序不可颠倒）
    capture.stop()
    capture.join(timeout=2.0)
    cv2.destroyAllWindows()
    print("[退出] 程序已正常结束。")
```

---

#### 2.2.2 `scan_device()` — 设备扫描函数

**目标**：完成 V1–V3 三项验证，返回 `Device` 对象或 `None`。

**实现细节**：
```python
def scan_device():
    ctx = Context()
    dev_list = ctx.query_devices()
    
    if dev_list.get_count() == 0:
        print("[扫描] 未检测到 Orbbec 设备，请检查 USB 连接后重试。")
        return None
    
    dev = dev_list[0]
    info = dev.get_device_info()
    
    name   = info.get_name()           # → str
    pid    = info.get_pid()            # → int，验证 hex 格式化
    serial = info.get_serial_number()  # → str
    conn   = info.get_connection_type()  # → str，非枚举
    
    print(f"[扫描] 已识别设备:")
    print(f"  型号  : {name}")
    print(f"  PID   : 0x{pid:04X}")
    print(f"  序列号: {serial}")
    print(f"  连接  : {conn}")
    
    return dev
```

**参考**：`pyorbbecsdk-v2.0.18/examples/enumerate.py`

---

#### 2.2.3 `FrameCapture` — 后台采集线程类

**目标**：完成 V4–V9 五项验证，提供非阻塞帧队列接口。

**属性**：
| 名称 | 类型 | 说明 |
|------|------|------|
| `_device` | `Device` | SDK 设备对象，生命周期内保持引用 |
| `_stopped` | `bool` | 停止标志 |
| `_queue` | `queue.Queue(maxsize=1)` | 有界帧队列，只保最新帧 |
| `_rgb_w` / `_rgb_h` | `int` | RGB 分辨率，用于窗口大小设置 |
| `_pipeline` | `Pipeline`（可选）| 保存引用以便后续扩展（当前版本无需显式 stop） |

**`run()` 方法流程**：
```
1. pipeline = Pipeline()
2. cfg = Config()
3. profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
   └─ 验证 V4：get_default_video_stream_profile() 成功
4. profile = profiles.get_default_video_stream_profile()
5. self._rgb_w, self._rgb_h = profile.get_width(), profile.get_height()
6. cfg.enable_stream(profile)        └─ 验证 V5：无 OBError
7. cfg.set_frame_aggregate_output_mode(FULL_FRAME_REQUIRE)  └─ 验证 V6
8. pipeline.start(cfg)
9. while not self._stopped:
       frames = pipeline.wait_for_frames(1000)   └─ 验证 V7
       if frames is None: continue
       color = frames.get_color_frame()            └─ 验证 V8
       if color is None: continue
       fmt = color.get_format()                    └─ 验证格式合法性
       if fmt not in supported_formats: skip
       bgr = frame_to_bgr_image(color)             └─ 验证 V9
       if bgr is None: continue
       rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
       try:
           self._queue.put_nowait(rgb)             └─ 满则跳过（丢弃旧帧）
       except queue.Full:
           pass
```

**`stop()` 方法**：
```python
def stop(self):
    self._stopped = True
```

**`get_latest_frame()` 方法**（供 GUI 调用）：
```python
def get_latest_frame(self):
    try:
        return self._queue.get_nowait()
    except queue.Empty:
        return None
```

**`size` 属性**：
```python
@property
def size(self):
    return (self._rgb_w, self._rgb_h)
```

**关键设计决策**：
- 使用 `daemon=True` 创建线程，确保主线程退出时自动回收
- `put_nowait()` + 捕获 `Full` 异常，避免阻塞采集循环
- 异常全部在内部消化（`except Exception as e: continue`），不向上传播

---

#### 2.2.4 `ViewerApp` — tkinter 主窗口类

**目标**：完成 V10–V12 三项验证。

**初始化**：
```python
class ViewerApp:
    def __init__(self, capture: FrameCapture, dev_info):
        self.capture = capture
        self.info = dev_info
        
        self.root = tk.Tk()
        self.root.title("Orbbec Test Display")
        
        w, h = capture.size
        self.root.geometry(f"{w}x{h+75}")    # 图像区 + 信息栏(40px) + 按钮(35px)
        self.root.minsize(w, h + 75)
        
        # 内容容器 Frame
        content_frame = tk.Frame(self.root)
        content_frame.pack(expand=True, fill="both")
        
        # 图像 Label（pack expand 填满剩余空间）
        self.label = tk.Label(content_frame)
        self.label.pack(expand=True, fill="both")
        
        # 设备信息文本栏
        self.info_label = tk.Label(
            content_frame,
            text=self._build_info_text(),
            font=("Consolas", 10),
            fg="#333333",
            anchor="w",
            height=2,
        )
        self.info_label.pack(fill="x")
        
        # 拍照按钮
        btn_frame = tk.Frame(self.root, bg="#f0f0f0", height=35)
        btn_frame.pack(fill="x", side="bottom")
        
        self.snapshot_btn = tk.Button(
            btn_frame,
            text="📷 拍照",
            font=("Microsoft YaHei", 12),
            width=10,
            command=self._on_snapshot,
            relief="flat",
            bg="#4a90e2",
            fg="white",
            activebackground="#357abd",
        )
        self.snapshot_btn.pack(expand=True, pady=5)
        
        # 按键绑定：q / ESC 退出（V11）
        self.root.bind("<Escape>", lambda e: self.root.quit())
        self.root.bind("<KeyPress-q>", lambda e: self.root.quit())
        
        # 窗口关闭事件：点击 × 按钮退出（V12）
        self.root.protocol("WM_DELETE_WINDOW", self.root.quit)
```

**`_build_info_text()` 方法**：
```python
def _build_info_text(self) -> str:
    """构造底部状态栏文本，展示设备关键参数。"""
    return (
        f"型号: {self.info.get_name()}  |  "
        f"PID: 0x{self.info.get_pid():04X}  |  "
        f"SN: {self.info.get_serial_number()}  |  "
        f"RGB: {self.capture.size[0]}×{self.capture.size[1]}"
    )
```

**`_refresh()` 方法**（核心渲染循环）：
```python
def _refresh(self):
    """每 33ms 被 after() 调用一次，非阻塞取帧并刷新 Label。"""
    img = self.capture.get_latest_frame()
    if img is not None:
        pil_img = Image.fromarray(img)
        # ⚠️ 必须保留引用，否则 PIL Image 和 PhotoImage 都会被 GC 回收导致黑屏
        self.tk_photo = ImageTk.PhotoImage(image=pil_img)
        self.label.configure(image=self.tk_photo)
    self.root.after(33, self._refresh)   # ~30fps 驱动
```

**`run()` 方法**：
```python
def run(self):
    self._refresh()
    self.root.mainloop()
```

---

## 三、关键实现细节

### 3.1 PhotoImage 引用陷阱

tkinter 的 `PhotoImage` 采用 C 层面引用计数管理图片数据。若只在局部变量中使用：
```python
# ❌ 错误写法：局部变量，函数返回后被 GC 回收，Label 显示空白
img = tk.PhotoImage(image=pil_img)
label.configure(image=img)
```
必须保存为实例变量：
```python
# ✅ 正确写法：实例变量持有引用，防止 GC
self.tk_photo = ImageTk.PhotoImage(image=pil_img)
self.label.configure(image=self.tk_photo)
```

### 3.2 队列满时丢弃旧帧

使用 `queue.Queue(maxsize=1)` + `put_nowait()`，当队列已满时抛 `queue.Full`，捕获后 `pass` 即可丢弃当前帧（此时队列中已有更新的帧）：
```python
try:
    self._queue.put_nowait(rgb)
except queue.Full:
    pass  # 已有更新帧，丢弃当前帧
```

### 3.3 退出顺序（强制）

```
capture.stop()          # 1. 先设停止标志
capture.join(timeout=2) # 2. 等待采集线程自然退出（最多 2s）
root.quit()             # 3. 再终止 mainloop（实际已由键盘/关闭窗口触发）
```

若顺序颠倒，`root.quit()` 先执行则 `mainloop()` 退出，但采集线程可能仍在运行，导致：
- SDK 资源未及时释放
- 控制台残留报错

---

## 四、代码注释规范

每条注释需说明以下之一：
1. **验证目的**：为何此处调用此 API（对应哪个 V 编号）
2. **注意事项**：常见陷阱或平台差异
3. **参考来源**：指向官方示例文件路径

示例：
```python
# V5: 验证 Config.enable_stream() 无异常
cfg.enable_stream(profile)

# ⚠️ PhotoImage 必须保留引用到实例变量，否则被 GC 回收导致黑屏
self.tk_photo = ImageTk.PhotoImage(image=pil_img)

# 参考：pyorbbecsdk-v2.0.18/examples/color.py
color_profile = profiles.get_default_video_stream_profile()
```

---

## 五、验收标准

满足以下全部条件视为验收通过：

| # | 验收项 | 判定方式 |
|---|--------|---------|
| A1 | 设备上电插入后，控制台 3 秒内打印设备信息 | 手动观察 |
| A2 | tkinter 窗口以 1280×720 分辨率打开（或 ColorSensor 默认分辨率） | 视觉确认 |
| A3 | 画面内容与实际场景一致，色彩正常（无偏蓝/偏绿） | 视觉确认 |
| A4 | 按 `q` 或 `ESC`，程序在 1 秒内完全退出，无僵尸进程 | 任务管理器确认 |
| A5 | 点击窗口关闭按钮，同 A4 | 同上 |
| A6 | 信息栏显示型号/PID/序列号/分辨率，与实际设备匹配 | 与 `lsusb` 对比 |
| A7 | 连续运行 10 分钟无内存泄漏（RSS 增长 < 50MB） | `top` 监控 |

---

## 六、风险与缓解措施

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|---------|
| Pillow 未预装在 openEuler | 高 | V10 失败（黑屏） | requirements.txt 显式声明；板上安装前 `python -c "from PIL import Image"` 预检 |
| `get_connection_type()` 返回值与预期不同 | 中 | V3 失败（打印格式异常） | 打印时不做假设，原样输出；调试期加 `type(conn)` 确认 |
| MJPG 格式解码失败 | 低 | V9 失败（画面偏色或黑屏） | `frame_to_bgr_image()` 内部已处理，若仍失败则降级为 `cv2.imdecode` fallback |
| tkinter 在 SSH + X11 转发下延迟高 | 中 | V10 失败（画面卡顿） | 测试时用桌面直连；若远程则改用 `export DISPLAY=:0` 或直接 SSH X11 forwarding |
| SDK v2.1.1 与 v2.0.18 stubs API 有差异 | 低 | 运行时 `AttributeError` | 保留完整 traceback，对照实际 wheel 内的 `pyorbbecsdk.pyi` 修正 |

---

## 七、开发步骤

| 步骤 | 操作 | 产出 |
|------|------|------|
| S0 | 准备 udev 文件（复制到 `demo/3/source/`） | `99-obsensor-libusb.rules` / `install_udev_rules.sh` |
| S1 | 复制 `demo/1/utils.py` → `demo/3/utils.py`（零修改） | `utils.py` |
| S2 | 编写 `viewer.py` 框架（import、常量、入口） | `viewer.py` 骨架 |
| S3 | 实现 `scan_device()` | V1–V3 通过 |
| S4 | 实现 `FrameCapture` 类 | V4–V9 通过 |
| S5 | 实现 `ViewerApp` 类（含 `_refresh()`） | V10–V12 通过 |
| S6 | 联调测试（本地 Windows 模拟逻辑，板上实测） | A1–A7 通过 |
| S7 | 编写 `quickstart.md`（见第八节） | `quickstart.md` |
| S8 | 编写 `scan_orbbec_336l.py` | 独立设备诊断工具 |
| S9 | 编写 `Project-reference.md` | 集成参考文档 |
| S10 | 更新 AGENTS.md 变更记录 | 文档同步 |

---

## 八、quickstart.md 编写要求

**编写时机**：S6（联调测试通过）之后、S7 执行。

**文件位置**：`demo/3/doc/quickstart.md`

**目标读者**：首次在 openEuler ARM64 开发板上部署 Demo 3 的开发者，阅读时间不超过 5 分钟。

### 内容大纲

```markdown
# Demo 3 — Quick Start

## 前置条件
- 华为香橙派（ARM64 aarch64），openEuler 22.03 SP4 desktop（带 GUI）
- Gemini 336L 已通过 **USB 3.0** 线缆连接到开发板
- `uv` 已安装（终端执行 `uv --version` 可确认）
- Python 3.9（系统自带，无需额外安装）

## 首次部署（仅执行一次）

### 1. 进入项目目录
```bash
cd /path/to/EBD-Orbbec
```

### 2. 创建虚拟环境并激活
```bash
uv venv --python python3 .venv
source .venv/bin/activate
```

### 3. 安装 SDK wheel + 运行依赖

#### 3.1 安装 SDK wheel（本地文件，无需联网）
```bash
uv pip install source/pyorbbecsdk2-2.1.1-cp39-cp39-linux_aarch64.whl
```

#### 3.2 安装运行依赖（opencv-python / numpy / Pillow）
> **推荐：使用阿里云 PyPI 镜像加速。**
>
> `--default-index` 标志仅在本次命令中生效，**不会修改** `~/.pip/pip.conf` 或系统全局配置，退出 venv 后即失效。
```bash
uv pip install --default-index https://mirrors.aliyun.com/pypi/simple -r requirements.txt
```

> **备选：网络正常时可直接使用默认源。**
>
> ```bash
> uv pip install -r requirements.txt
> ```

### 4. 安装 udev 规则（首次必需，免 sudo 访问 USB 设备）
> `demo/3/source/` 目录已内嵌 udev 规则文件与安装脚本，便于独立迁移部署。
```bash
sudo bash ./source/install_udev_rules.sh
```

### 5. 验证 SDK 可用
```bash
python -c "from pyorbbecsdk import *; print('SDK ok')"
```

---

## 运行 Demo 3

```bash
cd demo/3
python viewer.py
```

**预期效果**：
- 控制台打印设备信息（型号 / PID / 序列号 / 连接类型）
- 弹出「Orbbec Test Display」窗口，实时显示 RGB 画面
- 窗口底部状态栏显示设备参数

---

## 运行扫描工具

```bash
cd demo/3
python source/scan_orbbec_336l.py
```

**预期效果**：
- 控制台打印结构化表格，包含设备基本信息和各流分辨率配置

---

## 退出方式
- 按键：`q` 或 `ESC`
- 点击窗口关闭按钮（×）

---

## 常见问题排查

| 现象 | 排查命令 | 解决 |
|------|---------|------|
| `ImportError: No module named pyorbbecsdk` | `which python` / `pip list \| grep orbbec` | 确认 venv 已激活，wheel 路径正确 |
| 窗口黑屏（有标题无画面） | `python -c "from PIL import Image"` | 安装 Pillow：`uv pip install Pillow` |
| 设备未识别 | `lsusb \| grep -i orbbec` | 检查 USB 连接；重新执行 `sudo ./demo/3/source/install_udev_rules.sh` |
| 画面偏蓝/偏绿 | — | 属正常，取决于相机实际输出格式（RGB/MJPG/YUYV） |
| SELinux 拒绝 USB 访问 | `sudo getenforce` | `sudo setenforce 0` 临时关闭（诊断用，不改策略） |
| tkinter 提示 "no display" | `echo $DISPLAY` | 确保在桌面环境下运行，或通过 VNC/X11 forwarding 连接 |
```

### 编写原则
- 命令块可直接复制粘贴执行
- 每一步都有明确的预期输出说明
- 只保留最简路径，不展开原理性解释（原理在 AGENTS.md 和 PLAN.md 中）
- 错误排查表放在最后，方便快速定位

---

## 九、后续扩展预留点（不在本次范围，代码中留注释标记）

| 位置 | 预留内容 |
|------|---------|
| `FrameCapture.run()` | `enable_stream(LEFT_IR_SENSOR)` / `enable_stream(RIGHT_IR_SENSOR)` — 待扩展双目 IR |
| `FrameCapture` | 添加 `depth_queue` 和 `ir_queue`，支持多路流并行采集 |
| `ViewerApp` | 扩展为 2×2 网格布局，叠加 Depth/IR 画面 |
| `ViewerApp._build_info_text()` | 补充深度分辨率、帧率等字段 |

---

方案至此。确认后开始编码（S0→S10 依次执行）。
