# Demo 3 — Quick Start

> 面向首次在 openEuler ARM64 开发板上部署 Demo 3 的开发者。阅读时间 ≤ 5 分钟，所有命令可直接复制粘贴执行。

---

## 前置条件

- 华为香橙派（ARM64 aarch64），openEuler 22.03 SP4 desktop（带 GUI）
- Gemini 336L 已通过 **USB 3.0** 线缆连接到开发板
- `uv` 已安装（终端执行 `uv --version` 可确认）
- Python 3.9（系统自带，无需额外安装）

---

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

> **推荐：使用阿里云 PyPI 镜像加速**
>
> `--default-index` 标志仅在本次命令中生效，**不会修改** `~/.pip/pip.conf` 或系统全局配置，退出 venv 后即失效。

```bash
uv pip install --default-index https://mirrors.aliyun.com/pypi/simple -r requirements.txt
```

> **备选：网络正常时可直接使用默认源**
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

# 或使用扫描工具
python source/scan_orbbec_336l.py
```

**预期效果：**
- 控制台打印设备信息（型号 / PID / 序列号 / 连接类型）
- 弹出「Orbbec Test Display」窗口，实时显示 RGB 画面
- 窗口底部状态栏显示设备参数

---

## 退出方式

- 按键盘 `q` 或 `ESC`
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