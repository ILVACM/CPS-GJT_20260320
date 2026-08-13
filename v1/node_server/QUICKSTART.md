# 服务节点 快速启动指南（QUICKSTART）

> **适用对象**：刚接手本项目的实施人员 / 部署工程师
> **节点定位**：系统主控端（HUAWEI OrangePi KunPeng，openEuler 22 LTS）
> **核心职责**：采集相机画面与激光测距、向检测节点 转发推理任务、执行测量换算与画面拼接、本地 GUI 显示与交互
> **对应规范**：`AGENTS.md` §2.3 / `docs/Design-server.md`
> **工具链红线**：本项目统一使用 **`uv`** 作为 Python 包管理与运行工具，**严禁** `pip` / `conda` / `virtualenv` 直装

---

## 一、环境准备

### 1.1 硬件清单

| 项 | 规格 | 说明 |
|----|------|------|
| 主机 | HUAWEI OrangePi KunPeng（ARM Cortex-A，无需 GPU） | 主控节点，需稳定多外设接入 |
| 网络 | RJ45 网口 | 用于与检测节点 网线直连 |
| 深度相机 | Orbbec Gemini 336L | USB 直连服务节点 |
| 激光测距 | STP23L ×4 → 轮趣 S21C 主控板汇总 → Type-C 串口接入 | CH9102 USB 转串口芯片，波特率 115200 |
| 显示器 | HDMI 显示器一台 | GUI 必须，AGENTS.md §8.2；openEuler 必须有 GUI 桌面环境 |
| 电源 | 官方配置电源适配器 | 务必使用官方配置 |

### 1.2 软件版本要求

| 项 | 版本要求 | 说明 |
|----|----------|------|
| OS | openEuler 22 LTS / Ubuntu 22.04 LTS | 必须有 GUI 桌面环境（project_memory 硬约束） |
| 发行版支持 | Debian 系（Ubuntu/Debian/Raspbian）/ RHEL 系（openEuler/CentOS/RHEL/Fedora/Rocky/AlmaLinux/Kylin） | 部署脚本自适应识别，自动映射 DEB/RPM 包名 |
| Python | 3.9+ | 系统自带 |
| uv | ≥ 0.4.0 | 见 §二 安装步骤 |
| tkinter | `python3-tk` / `python3-tkinter` | Debian 系 `apt install python3-tk`；RHEL 系 `dnf install python3-tkinter`（AGENTS.md §8.2） |
| X11 / XWayland | 必须可用 | Wayland 会话下需 `GDK_BACKEND=x11` 兼容 tkinter |

### 1.3 系统级依赖（一次性预装，自适应发行版）

⚠️ **以下依赖必须先于 `uv sync` 安装**，否则 OpenCV/grpcio 会在 ARM64 上触发源码编译，弱性能 OrangePi 实测会卡死 2-4 小时甚至 OOM。

#### 1.3.1 包名映射表（DEB 系 vs RPM 系）

部署脚本 `deploy_node_server.sh` 内置 `detect_distro()` 自动识别发行版，并经 `install_pkgs()` 按下表自动映射包名。手动安装时请对照本表选择对应包名：

| 逻辑用途 | Debian 系（apt） | RHEL 系（dnf/yum） | 说明 |
|----------|------------------|---------------------|------|
| OpenGL 运行时 | `libgl1` | `mesa-libGL` | OpenCV 必须 |
| OpenGL GLU | `libglu1-mesa` | `mesa-libGLU` | OpenCV 必须 |
| OpenGL EGL | `libegl1` | `mesa-libEGL` | OpenCV 必须 |
| glib2 运行时 | `libglib2.0-0` | `glib2` | OpenCV 必须 |
| glib2 开发头 | `libglib2.0-dev` | `glib2-devel` | 编译期 |
| libSM | `libsm6` | `libSM` | OpenCV 必须 |
| libXext | `libxext6` | `libXext` | OpenCV 必须 |
| libXrender | `libxrender1` | `libXrender` | OpenCV 必须 |
| libXtst | `libxtst6` | `libXtst` | OpenCV 必须 |
| libXi | `libxi6` | `libXi` | OpenCV 必须 |
| libgthread | `libgthread2.0-0` | **（空）** | RHEL 系由 `glib2` 主包提供，无需独立安装 |
| GTK2 运行时 | `libgtk2.0-0` | `gtk2` | OpenCV highgui |
| GTK3 运行时 | `libgtk-3-0` | `gtk3` | OpenCV highgui |
| 构建工具链 | `build-essential` | `gcc gcc-c++ make cmake` | grpcio 源码编译 |
| pkg-config | `pkg-config` | `pkgconfig` | 编译期 |
| Python 开发头 | `python3-dev` | `python3-devel` | 编译期 |
| 内核头 | `linux-headers-$(uname -r)` | `kernel-devel` | 编译期 |
| tkinter | `python3-tk` | `python3-tkinter` | GUI 必须（AGENTS.md §8.2） |
| X11 Server | `xorg` | `xorg-x11-server-Xorg` | tkinter 显示后端 |
| XWayland | `xwayland` | `xorg-x11-server-Xwayland` | Wayland 兼容 |
| X11 工具 | `x11-utils` | `xorg-x11-utils` | 诊断 |
| xauth | `xauth` | `xorg-x11-xauth` | SSH -X 转发 |
| USB 工具 | `usbutils` | `usbutils` | 串口诊断 |
| 用户管理 | `passwd` | `shadow-utils` | 精简镜像补全 |
| sudo | `sudo` | `sudo` | 精简镜像补全 |

#### 1.3.2 Debian 系手动安装（Ubuntu / Debian）

```bash
# 1. 更新 apt 索引（推荐配置阿里云镜像源加速）
sudo apt update -y

# 2. 安装 tkinter（GUI 必须，AGENTS.md §8.2 明确要求）
sudo apt install -y python3-tk

# 3. 安装 X11 / Wayland 兼容层（tkinter 显示后端）
sudo apt install -y xorg xwayland x11-utils xauth

# 4. 安装 OpenCV 运行时依赖（避免 ImportError: libGL.so.1 / libgthread-2.0.so.0）
sudo apt install -y \
    libgl1 libglu1-mesa libegl1 \
    libglib2.0-0 libglib2.0-dev \
    libsm6 libxext6 libxrender1 libxtst6 libxi6 \
    libgthread2.0-0 libgtk2.0-0 libgtk-3-0

# 5. 安装构建工具链（grpcio / numpy 无 wheel 时回退源码编译必需）
sudo apt install -y \
    build-essential pkg-config \
    python3-dev linux-headers-$(uname -r)

# 6. 安装串口与 USB 工具（CH9102 ch341 内核驱动 + 串口诊断）
sudo apt install -y usbutils util-linux

# 7. 安装 sudo 与用户管理工具（若精简镜像缺失）
sudo apt install -y sudo passwd
```

#### 1.3.3 RHEL 系手动安装（openEuler / CentOS / RHEL / Fedora）

```bash
# 1. 更新 dnf 索引（推荐配置阿里云镜像源加速）
sudo dnf update -y

# 2. 安装 tkinter（GUI 必须，AGENTS.md §8.2 明确要求）
sudo dnf install -y python3-tkinter

# 3. 安装 X11 / Wayland 兼容层（tkinter 显示后端）
sudo dnf install -y \
    xorg-x11-server-Xorg xorg-x11-server-Xwayland \
    xorg-x11-utils xorg-x11-xauth

# 4. 安装 OpenCV 运行时依赖（避免 ImportError: libGL.so.1 / libgthread-2.0.so.0）
#    注：libgthread 在 RHEL 系由 glib2 主包提供，无需独立安装
sudo dnf install -y \
    mesa-libGL mesa-libGLU mesa-libEGL \
    glib2 glib2-devel \
    libSM libXext libXrender libXtst libXi \
    gtk2 gtk3

# 5. 安装构建工具链（grpcio / numpy 无 wheel 时回退源码编译必需）
sudo dnf install -y \
    gcc gcc-c++ make cmake pkgconfig \
    python3-devel kernel-devel

# 6. 安装串口与 USB 工具（CH9102 ch341 内核驱动 + 串口诊断）
sudo dnf install -y \
    usbutils util-linux \
    kernel-modules-extra

# 7. 安装 sudo 与用户管理工具（若精简镜像缺失）
sudo dnf install -y sudo shadow-utils
```

⚠️ **避坑提示**：
- `libgthread2 libgtk2 libgtk3` 是**错误包名**（既非标准 DEB 也非标准 RPM），RHEL 系安装会报 `No match for argument`；正确写法见上表
- RHEL 系的 `libgthread-2.0.so.0` 由 `glib2` 主包提供，无需独立安装
- 部署脚本已内置自适应逻辑，**推荐直接使用 `sudo bash scripts/deploy_node_server.sh`**，无需手动执行上述命令

### 1.4 系统权限配置（一次性，部署期完成）

```bash
# 1. 将用户加入 video 组（摄像头权限）
sudo usermod -aG video $USER

# 2. 将用户加入 dialout 组（串口权限）
sudo usermod -aG dialout $USER

# 3. 重新登录使组权限生效（必须）
exit
# 重新 SSH 登录或本地登录

# 4. Wayland 会话下兼容 tkinter（可选，写入 ~/.bashrc 永久生效）
export GDK_BACKEND=x11

# 5. 确认 ch341 / cdc_acm 内核驱动加载（CH9102 串口芯片）
#    CH9102 在 CDC ACM 模式下由 cdc_acm 驱动（节点 /dev/ttyACM*），
#    在 UART 模式下由 ch341 驱动（节点 /dev/ttyUSB*），两种均需支持
lsmod | grep -E 'ch341|cdc_acm'
# 若未加载：
sudo modprobe ch341 || true
sudo modprobe cdc_acm || true

# 6. 安装 Orbbec udev 规则（USB 设备免 sudo，P1/F2/A15）
#    来源：source/install_udev_rules.sh + source/99-obsensor-libusb.rules
#    install_udev_rules.sh 内部已执行 udevadm control --reload + udevadm trigger（R7）
#    部署脚本 deploy_node_server.sh 会自动执行此步，手动部署时需显式调用
sudo bash source/install_udev_rules.sh
```

⚠️ **避坑提示**：
- openEuler 默认镜像可能精简，**必须**先完成 §1.3 + §1.4 才能进入后续步骤
- Wayland 会话下 tkinter 可能无法启动，建议直接切换到 X11 会话登录
- 用户组改动**必须重新登录生效**，sudo 内执行 usermod 不影响当前会话

---

## 二、手动部署（调试模式）

> **强制结构**：以下步骤以表格形式呈现，每一步均独立可验证。建议在调试期按表执行，便于定位失败步骤。

### 2.1 部署步骤表

| 步骤 | Bash 命令 | 预期输出/说明 | 避坑提示 |
|:-----|:----------|:---------------|:---------|
| **1. 部署代码到 /opt** | `sudo mkdir -p /opt/node_server && sudo cp -r /path/to/DEMO/node_server/* /opt/node_server/` | 目录 `/opt/node_server` 下含 `main.py` `config/` `proto/` 等 | ⚠️ **路径必须为 `/opt/node_server`**，systemd unit 默认指向此路径；若自定义路径需同步改 `service.json` |
| **2. 进入项目目录** | `cd /opt/node_server` | 当前工作目录切换 | 后续所有命令均在此目录执行 |
| **3. 安装 uv** | `curl -LsSf https://astral.sh/uv/install.sh \| sh` | 安装完成提示 `uv` 已加入 `~/.local/bin` | ⚠️ 安装后执行 `source ~/.bashrc` 或重开终端使 PATH 生效；ARM64 平台确认 `uv --version` 可执行 |
| **4. 配置 uv 镜像源（可选，阿里云加速）** | `export UV_HTTP_TIMEOUT=600`<br>`export UV_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/` | 环境变量生效 | ⚠️ **可选**，openEuler ARM64 上 PyPI 默认源访问慢甚至超时时配置；永久生效写入 `~/.bashrc`；一键部署脚本可加 `--aliyun` 参数临时启用（见 §四）；如需配置 dnf 阿里云源参考 https://developer.aliyun.com/mirror/openeuler |
| **5. 创建项目虚拟环境** | `uv venv --python /usr/bin/python3` | 输出 `Using Python 3.9.x` 与 `.venv` 路径 | ⚠️ openEuler 22.03 SP4 系统 Python 为 3.9.9，直接使用系统 Python 避免 uv 下载 python-build-standalone 3.10（其 _tkinter 依赖 Tcl/Tk 9.0，openEuler 仅有 8.6，会导致 GUI 崩溃）；部署脚本会自动校验系统 Python 版本并清理 uv 缓存的 3.10，确保设备内 Python 完全统一为 3.9 |
| **6. 安装 Python 依赖（uv sync）** | `uv sync` | 逐包解析并安装，最终 `Resolved N packages`，生成 `uv.lock` | ⚠️ 如卡死在 opencv 编译，确认 §1.3 系统依赖已装；项目同时保留 `requirements.txt` 供人工临时安装：`uv pip install -r requirements.txt`（双轨兼容） |
| **7. 编译 proto 文件** | `cd /opt/node_server && sudo uv run python -m grpc_tools.protoc \<br>    --proto_path=. \<br>    --python_out=. \<br>    --grpc_python_out=. \<br>    proto/rebar_inference.proto` | `proto/rebar_inference_pb2.py` 与 `rebar_inference_pb2_grpc.py` 重新生成 | ⚠️ proto 更新后必须重编译；必须用 `--proto_path=.` 让 protoc 识别 proto 为包，生成包内导入（`from . import`）；使用 `uv run` 而非裸 `python` 以调用虚拟环境内的 grpcio-tools；node_server 降级 Python 3.9 后 gencode 为 6.x，与 runtime 6.33.6 匹配 |
| **8. 配置串口设备** | `ls -l /dev/ttyUSB* /dev/ttyACM*` | 出现 `/dev/ttyUSB0` 或 `/dev/ttyACM0` 等节点 | ⚠️ 首次插入 S21C 主控板时确认；CH9102 在 CDC ACM 模式下节点为 `/dev/ttyACM*`（cdc_acm 驱动），UART 模式下为 `/dev/ttyUSB*`（ch341 驱动）；若无节点，检查 `lsmod \| grep -E 'ch341\|cdc_acm'` 与 `lsusb`；兼容 USB 直连与扩展坞接入 |
| **9. 环境自检** | `sudo uv run main.py self-check` | 输出 `[自检] ...` | ⚠️ 必须 `sudo`（root 权限绑定 50051 端口）；自检以**诊断模式**运行（网络硬故障仅记日志不退出），相机/串口扫描与网络监测在启动后异步进行，失败可重试 |
| **10. 前台启动（调试模式）** | `sudo uv run main.py` | 见 §三 预期现象 | 调试期推荐前台运行，便于实时查看日志；正式部署见 §四 |

### 2.2 关键依赖说明

| 包 | 版本 | 用途 | 避坑提示 |
|----|------|------|----------|
| `opencv-python` | ≥ 4.5 | 图像处理、相机采集、画面渲染 | ⚠️ openEuler ARM64 上可能无预编译 wheel，触发源码编译；务必先装 §1.3 系统依赖 |
| `pyorbbecsdk` | 2.1.1 | Orbbec Gemini 336L SDK（Pipeline + 设备枚举） | ⚠️ **仅 Linux ARM64 wheel**，须将 `pyorbbecsdk2-2.1.1-cp39-cp39-linux_aarch64.whl` 在部署前放 `source/`（P1/K4）；缺失时相机功能不可用（D3 懒导入兜底） |
| `pyserial` | ≥ 3.5 | S21C 串口通信（CH9102） | 通常有 wheel，无坑 |
| `Pillow` | ≥ 8.0 | 图像格式转换（JPEG 编码） | 通常有 wheel |
| `grpcio` / `grpcio-tools` | ≥ 1.50 | gRPC 通信与 proto 编译 | ⚠️ ARM64 上可能源码编译，需 `gcc-c++` `python3-devel` |
| `protobuf` | ≥ 4.21 | protobuf 运行时 | 通常有 wheel |
| `numpy` | ≥ 1.21 | 数值计算 | 通常有 wheel |
| `tkinter` | 系统包 | GUI 主框架 | ⚠️ **不通过 uv 安装**，必须 `sudo dnf install python3-tkinter` |

---

## 三、预期现象

### 3.1 启动成功标志

| 检查项 | 预期 |
|--------|------|
| 控制台日志 | 出现 `[INFO] [UI] GUI 主循环启动` 字样 |
| GUI 主窗口 | 弹出 tkinter 窗口，显示"摄像头选择阶段"（未连接状态） |
| 端口监听 | `sudo ss -tlnp \| grep 50051` 显示 LISTEN（服务节点 gRPC 服务端，接收 D→S 心跳与 Shutdown） |
| 心跳接收 | 每 5 秒日志输出 `[心跳] 收到`（检测节点 在线时） |
| 节点状态指示 | GUI 顶部显示"推理服务在线"（绿色，检测节点 在线时） |

### 3.2 识别工作阶段

用户点击"识别"按钮后，按钮状态转换：

```
READY（就绪）→ LOADING（识别中）→ RESULT（结果展示）→ READY
```

每次识别后：
- 主界面叠加显示钢筋直径与间距标注
- 结果自动保存至 `./result/{YYYYMMDD-HHMMSS}/` 目录（含 `ClassMask.png` `report.csv` `result.jpg`）
- 同秒内重复触发，新结果覆盖旧结果（project_memory 硬约束）

### 3.3 降级模式

检测节点 不可达（启动时未探测到，或运行中连续 15 秒未收到心跳）时：
- GUI 顶部显示"推理服务离线"（红色）
- 识别按钮置灰禁用
- 视频预览与激光测距数据**仍正常显示**（仅推理功能不可用）
- 检测节点 恢复后心跳自动触发"恢复在线"，无需重启服务节点

### 3.4 优雅退出（D→S 接收方）

> ⚠️ **方向纠正**：根据 `AGENTS.md` §5.5 / §5.7.3，`Shutdown` RPC 方向为 **D→S（检测节点 → 服务节点）**。服务节点 是 **grpc_server 接收方**，不是发起方。原文档中"S→D 退出通知"为历史遗留错误描述，已在新版中纠正。

服务节点 退出流程：

1. GUI 主窗口关闭按钮 / `Ctrl+C` / `systemctl stop` 触发
2. 检查检测节点 是否在线
3. 若在线，等待检测节点 的 Shutdown 通知（best-effort，超时 2-3 秒）
4. 停止所有后台线程（相机采集 / 串口读取 / 心跳监测）
5. 释放相机与串口资源
6. 关闭 gRPC 服务端与客户端
7. 进程退出

> **注**：服务节点 退出时不主动向检测节点 发指令，而是被动接收检测节点 的 Shutdown 通知。检测节点 自身退出时会主动通过 grpc_client 发送 Shutdown RPC 到服务节点。

---

## 四、一键部署（生产模式）

> 生产环境部署推荐使用项目提供的 `scripts/deploy_node_server.sh` 自动化脚本，含环境检查、镜像源配置、错误处理。

```bash
# 1. 赋予执行权限
cd /opt/node_server
sudo chmod +x scripts/deploy_node_server.sh

# 2. 执行一键部署（默认使用系统当前软件源）
sudo bash scripts/deploy_node_server.sh

# 2'. 或：本次运行临时使用阿里云镜像源（dnf/apt + uv 全覆盖，不修改系统配置）
sudo bash scripts/deploy_node_server.sh --aliyun

# 脚本内部依次完成：
#   ① root 权限检查 / ARM64 架构检查 / 发行版识别
#   ② 软件源配置（默认系统源；--aliyun 临时复制系统 repo 替换为阿里云 host）
#   ③ 刷新包管理器元数据（dnf makecache / apt update，不做全系统升级）+ 安装系统级依赖
#   ④ 配置用户组（video / dialout）
#   ⑤ 安装 uv 工具链（uv 镜像源：默认沿用系统配置；--aliyun 临时 export 阿里云）
#   ⑥ uv venv 创建 + uv sync 安装 Python 依赖
#   ⑦ proto 编译（uv run python -m grpc_tools.protoc）
#   ⑧ 提示用户人工插入 S21C 串口与 336L 相机
#   ⑨ 注册 systemd 服务（rebar-node-server）并启动
```

⚠️ **避坑提示**：
- 脚本含 `set -e`，任一步骤失败立即停止，不产生半安装状态
- **默认使用系统当前软件源**，不修改 `/etc`（不写 `/etc/environment`、不替换 `/etc/yum.repos.d`）；`--aliyun` 仅本次运行临时生效，退出后自动清理临时 repo 文件
- 相机与 S21C 串口设备**由人工插入**，脚本不自动检测外设
- 如系统源损坏或过慢，加 `--aliyun` 参数临时切阿里云；若需永久配置阿里云源请手动参照 https://developer.aliyun.com/mirror/openeuler

### 4.1 systemd 服务管理

```bash
# 查看服务状态
sudo systemctl status rebar-node-server

# 启动 / 停止 / 重启
sudo systemctl start rebar-node-server
sudo systemctl stop rebar-node-server
sudo systemctl restart rebar-node-server

# 查看实时日志
sudo journalctl -u rebar-node-server -f
# 或查看应用日志
sudo tail -f /opt/node_server/logs/node_server.log

# 卸载服务（停止 + 取消开机自启 + 删除 unit 文件）
sudo bash scripts/service-manager.sh uninstall
```

---

## 五、排错手册（针对服务节点特有问题）

### 5.1 常见启动失败原因

| 现象 | 原因 | 解决办法 |
|------|------|----------|
| **`[自检] 网络异常（诊断日志，不退出）`** | 本机 IP 不匹配 / 端口冲突 / 对端不可达 | **网络自检已是诊断模式，不阻断启动**。① 本机 IP 与 `local_ip` 不一致？→ 执行 `sudo /opt/node_server/.venv/bin/python /opt/node_server/main.py setup-network` 自动配置静态 IP（详见 §5.4）；② 对端（检测节点）不可达？→ 系统进入**降级模式**（仅视频+激光，禁用识别按钮），由 `NetworkMonitor` 后台周期探测，网络恢复后自动回到在线；③ 50051 端口被占用？→ `sudo lsof -i :50051` 后 `kill -9 <PID>`并重启 |
| **GUI 不弹出 / `_tkinter.TclError: no display`** | 无 DISPLAY 环境变量 / Wayland 会话不兼容 | ① 本地登录需有桌面环境；② SSH 调试用 `ssh -X`；③ Wayland 下 `export GDK_BACKEND=x11`；④ 安装 XWayland：`sudo dnf install xorg-x11-server-Xwayland` |
| **`ModuleNotFoundError: No module named 'tkinter'`** | tkinter 未安装 | `sudo dnf install python3-tkinter`（AGENTS.md §8.2）；⚠️ **不通过 uv 安装**，tkinter 是系统包 |
| **`ModuleNotFoundError: No module named 'cv2'`** | opencv 安装失败 / libGL 缺失 | ① 确认 §1.3 系统依赖已装；② `uv pip list \| grep opencv` 确认安装；③ 临时改用 `opencv-python-headless`（但 GUI 显示受影响） |
| **摄像头打不开 / `Permission denied`** | 用户未加入 video 组 | `sudo usermod -aG video $USER`，**重新登录生效**（AGENTS.md §8.2） |
| **串口打不开 / `Permission denied: /dev/ttyUSB0`** | 用户未加入 dialout 组 / 驱动未加载 | ① `sudo usermod -aG dialout $USER`，**重新登录生效**；② `sudo modprobe ch341` 与 `sudo modprobe cdc_acm`；③ `lsmod \| grep -E 'ch341\|cdc_acm'` 确认 |
| **CH9102 串口芯片未识别** | Linux 内核 ch341 / cdc_acm 驱动未加载 | `lsmod \| grep -E 'ch341\|cdc_acm'`；若未加载，`sudo modprobe ch341` 与 `sudo modprobe cdc_acm`；CH9102 在 CDC ACM 模式下节点为 `/dev/ttyACM*`，UART 模式下为 `/dev/ttyUSB*`；首次接入实测确认（AGENTS.md §8.2） |
| **检测节点 不可达（降级模式）** | 检测节点 未启动 / IP 配置不一致 / 网线未插 | ① 先启动检测节点；② 核对双节点 `network.json` 中 IP 对称性；③ 检查网线物理连接；④ `ping 192.168.10.2` 测试连通性 |
| **`uv sync` 卡死在 opencv/grpc 编译** | ARM64 无预编译 wheel，触发源码编译 | 先执行 §1.3 系统级 dnf 依赖安装；默认系统源慢时加 `--aliyun` 参数重跑部署脚本，或手动 `export UV_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/` |
| **心跳超时（连续 15 秒未收到）** | 检测节点 崩溃 / 网络中断 / Shutdown 未送达 | ① 检查检测节点 进程：`ssh node_detect 'systemctl status node-detect-inference'`；② 检查网线；③ 等待检测节点 重启后心跳自动恢复 |
| **`uv: command not found`** | uv 未加入 PATH | `source ~/.bashrc` 或执行 `export PATH="$HOME/.local/bin:$PATH"` |
| **`[gRPC] 启动失败: 端口被占用`** | 已有进程占用 50051 | `sudo lsof -i :50051` 后 `kill -9 <PID>`，或修改 `network.json` 中 `grpc_port`（双节点同步改） |

### 5.2 诊断命令速查

```bash
# 查看实时应用日志
sudo tail -f /opt/node_server/logs/node_server.log

# 检查端口监听（应有 LISTEN，接收 D→S 心跳与 Shutdown）
sudo ss -tlnp | grep 50051

# 测试与检测节点 的网络连通性
ping -c 4 192.168.10.2

# 列出 USB 设备（确认相机与串口芯片）
lsusb

# 列出串口设备节点（CH9102 可能为 /dev/ttyUSB* 或 /dev/ttyACM*）
ls -l /dev/ttyUSB* /dev/ttyACM*

# 检查用户组归属
groups $USER
# 应包含 video 与 dialout

# 检查 DISPLAY 环境变量
echo $DISPLAY
# 本地登录应显示 :0 或 :1；SSH -X 应显示 localhost:10.0

# 检查 Wayland / X11 会话
echo $XDG_SESSION_TYPE
# 建议为 x11；若为 wayland 需 export GDK_BACKEND=x11

# 检查 ch341 / cdc_acm 驱动加载（CH9102 串口芯片）
lsmod | grep -E 'ch341|cdc_acm'

# 五项自检脚本（Windows 开发环境用 Mock 替代真实硬件）
cd /opt/node_server
sudo uv run python test_node_server.py

# 查看 systemd 服务状态
sudo systemctl status rebar-node-server

# 查看 systemd 实时日志
sudo journalctl -u rebar-node-server -f
```

### 5.3 联调验证步骤

1. **检测节点 单机自检**：在检测节点 上执行 `sudo uv run python tests/test_predictor_offline.py`，确认推理全流程正常
2. **服务节点 单机自检**：在服务节点 上执行 `sudo uv run python test_node_server.py`，确认五项（相机/串口/GUI/通信/推理测量）通过
3. **双节点联调**：
   - 先启动检测节点：`sudo uv run main.py run`
   - 再启动服务节点：`sudo uv run main.py`
   - 服务节点 GUI 应显示"推理服务在线"
   - 点击"识别"按钮，应在 3-5 秒内返回测量结果并叠加显示
4. **降级模式验证**：启动服务节点 后立即关闭检测节点，服务节点 应在 15 秒内进入降级模式
5. **优雅退出验证**：关闭检测节点，应在 2-3 秒内看到服务节点 日志输出 `[Shutdown] 收到退出通知`，并立即标记离线

### 5.4 网络配置（IP 不匹配时使用）

当启动日志出现 `[诊断] 本机 IP 校验未通过`（IP 与 `local_ip` 不一致）时，说明需配置静态 IP。网络自检仅是**诊断日志，不阻断启动**（系统仍进入降级模式运行）；如需恢复完整推理功能，使用 `setup-network` 子命令自动完成备份 + 配置静态 IP。该子命令独立于启动流程，由人工主动触发（对齐 AGENTS.md §4.2）。

#### 5.4.1 命令用法

```bash
# 方式一：直接调用 Python 子命令（推荐）
sudo /opt/node_server/.venv/bin/python /opt/node_server/main.py setup-network [OPTIONS]

# 方式二：Shell 包装脚本（同样支持透传 OPTIONS）
sudo bash /opt/node_server/scripts/setup_network.sh [OPTIONS]
```

> ⚠️ **必须以 root 权限运行**（脚本内部也会校验），否则直接退出。
> ⚠️ **修改 IP 后当前 SSH 会话可能断开**，请确保已通过本地终端或带外方式连接设备。

**可选参数（全部可省略，省略时走默认行为）**：

| 参数 | 说明 | 默认行为 |
|------|------|----------|
| `--ip <addr>` | 临时指定 IP（仅本次生效） | 取 `network.json` 的 `local_ip` |
| `--netmask <mask>` | 临时指定子网掩码（仅本次生效） | 取 `network.json` 的 `netmask` |
| `--gateway <addr>` | 临时指定网关（仅本次生效） | 取 `network.json` 的 `gateway`；未配置则自动推导为 IP 网段 `.1` |
| `--restore` | 复原模式：从备份恢复原配置 | 否（默认设置模式） |
| `--no-start-interface` | 配置后不自动启动网络接口 | 自动启动 |
| `--no-restart-service` | 配置后不自动重启网络服务 | 自动重启 |

> ⚠️ `--restore` 与其他设置参数互斥：复原模式下忽略 `--ip/--netmask/--gateway`，直接从备份恢复。
> 显式参数（`--ip` 等）**仅本次执行生效，不修改任何配置文件**。

#### 5.4.2 执行流程

一键式（无交互确认，四步连跑；失败即停，不自动回滚但输出已做步骤）：

1. **备份当前网络配置**（仅保留最新一份，先写临时目录再原子 rename，避免中途失败损坏既有备份）：
   - `ip addr` / `ip route` / `nmcli dev show` / `nmcli connection show` 命令输出
   - `/etc/resolv.conf` / `/etc/network/interfaces` 配置文件
   - `/etc/sysconfig/network-scripts/` 整目录（RHEL 系）
   - `/etc/netplan/` 整目录（Debian 系）
   - `backup_manifest.json` 清单文件（含 hostname / 备份文件列表 / 时间戳）
   - 固定目录 `logs/network_backup/`（非时间戳目录）
2. **写入静态 IP**：自动检测物理网卡（遍历 `/sys/class/net/` 排除 `lo` / `docker` / `virbr` / 无线网卡，优先 UP 状态；或取 `network_interface` 字段），自适应发行版：
   - RHEL 系（openEuler/CentOS/RHEL/Fedora/Rocky/AlmaLinux/Kylin）：写入 `/etc/sysconfig/network-scripts/ifcfg-<iface>`（含 IPADDR/NETMASK/GATEWAY）
   - Debian 系（Ubuntu/Debian/Raspbian）：写入 netplan（`/etc/netplan/01-rebar-static.yaml`，含 routes 网关）或 `/etc/network/interfaces`
3. **启动网络接口**：`ip link set <iface> up`（`--no-start-interface` 跳过）
4. **重启网络服务**：RHEL 系 `systemctl restart NetworkManager`（失败回退 `network`）、Debian 系 `netplan apply`，适配失败回退 `networking`（`--no-restart-service` 跳过）

> 网关取 `--gateway` > `network.json.gateway` > 自动推导为 IP 网段 `.1`；IP/网关非法时**报错退出**而非静默使用错误值。

#### 5.4.3 配置文件字段

`config/network.json` 相关字段：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `network_interface` | string | `""`（空） | 指定网卡名；留空时自动检测物理网卡 |
| `netmask` | string | `"255.255.255.252"` | 子网掩码；`/30` 双节点直连够用（仅 4 个地址），`/24`（`255.255.255.0`）更通用 |
| `gateway` | string | 无（自动推导 `.1`） | 网关地址；未配置时由工具按 `local_ip` 网段自动推导 `<网段>.1` |

#### 5.4.4 恢复网络配置（`--restore` 一键复原）

配置失败或需回退时，从备份一键复原（先校验备份完整性与 Manifest 存在，备份缺失/损坏即报错）：

```bash
# 一键复原（自动识别发行版并从备份复制回 netplan / ifcfg / interfaces）
sudo /opt/node_server/.venv/bin/python /opt/node_server/main.py setup-network --restore
# 或
sudo bash /opt/node_server/scripts/setup_network.sh --restore

# 主动查看备份内容
ls -l /opt/node_server/logs/network_backup/
cat /opt/node_server/logs/network_backup/backup_manifest.json
```

#### 5.4.5 触发场景示例

网络 IP 不匹配时的实际日志形态（**诊断模式，不退出**）：

```
INFO | ... | ./执行启动环境自检（诊断模式，不阻塞启动）...
WARNING | ... | ./[诊断] 本机 IP 校验未通过: 本机 IP=192.168.10.5 != 配置 local_ip=192.168.10.1
INFO | ... | ./[诊断] gRPC 端口可用性通过
...
WARNING | ... | ./对端 192.168.10.2 不可达，进入降级模式（仅视频+激光，禁用识别）
INFO | ... | ./NetworkMonitor 启动，周期探测检测节点 可达性
```

此时**服务节点 仍正常启动**（进入降级模式，推理按钮禁用）。若需恢复完整推理功能，执行 IP 配置后重启：

```
sudo /opt/node_server/.venv/bin/python /opt/node_server/main.py setup-network
或：sudo bash /opt/node_server/scripts/setup_network.sh
配置完成后重启服务：sudo systemctl restart rebar-node-server
```

网络恢复后，`NetworkMonitor` 会在周期探测中探测到对端可达并自动切回在线模式（无需重启）。

---

## 六、关键约束速查

| 约束 | 来源 | 说明 |
|------|------|------|
| 必须使用 uv 工具链 | spec.md | 严禁 `pip` / `conda` / `virtualenv` |
| 必须有 GUI 桌面环境 | project_memory | openEuler 必须安装桌面环境 |
| tkinter 通过 dnf 安装 | AGENTS.md §8.2 | 不通过 uv 安装，是系统包 |
| 启动第一步读 JSON 配置 | project_memory | 严禁硬编码，所有参数配置化 |
| 启动第二步网络自检 | project_memory | 网络为诊断模式（仅日志不退出），配合 NetworkMonitor 运行时动态降级；仅配置文件加载失败才 CRITICAL 退出 |
| 必须 root/sudo 启动 | project_memory | 网络配置与端口绑定需要 root 权限 |
| GUI 主线程不阻塞 | AGENTS.md §7.2 | 测量在工作线程执行，结果经 `root.after` 回主线程 |
| 副页面打开时禁用识别按钮 | project_memory | 按钮灰显 + 提示文本 |
| 结果默认保存 result.jpg | project_memory | 无论用户设置如何，可视化结果图必须保存 |
| 掩码图 PNG 无损 | project_memory | 8-bit 单通道 PNG，禁止 JPEG |
| 同秒触发覆盖旧结果 | project_memory | 时间戳精度到秒，同秒覆盖 |
| 一机一日志 | AGENTS.md §7.5 | 所有模块写入 `logs/node_server.log` |
| 退出通知方向 D→S | AGENTS.md §5.7 | 服务节点 是 **grpc_server 接收方**，不主动发起 |
| systemd ExecStart 调用 uv 虚拟环境 Python | spec.md | 路径为 `/opt/node_server/.venv/bin/python`，非 `/usr/bin/python3` |

---

## 七、配置文件清单

| 配置文件 | 内容 | 关键字段 |
|----------|------|----------|
| `config/network.json` | 网络参数 | `local_ip` / `remote_ip` / `grpc_port` / `heartbeat_interval_seconds` / `heartbeat_timeout_count` / `network_monitor_interval_seconds` / `network_interface`（可选）/ `netmask`（可选）/ `gateway`（可选，未配置时 setup-network 自动推导网段 `.1`）（见 §5.4.3） |
| `config/camera.json` | 相机参数 | `rgb_width` / `rgb_height` / `rgb_fps` / `device_index` / `compatible_models` |
| `config/intrinsics.json` | 相机内参 | `fx` / `fy` / `cx` / `cy` |
| `config/inference.json` | 推理节奏 | `inference_interval_seconds`（默认 3.0） |
| `config/serial.json` | 串口参数（S21C） | `device_path` / `auto_scan` / `vid_whitelist` / `pid_whitelist` / `baudrate` / `scan_interval_seconds` / `max_age_seconds`（见 §7.7） |
| `config/logging.json` | 日志参数 | `max_bytes`（10MB）/ `backup_count`（3）/ `level` |
| `config/service.json` | systemd 参数 | `service_name` / `exec_start`（uv venv Python）/ `working_directory` |

### 7.1 网络参数示例 `config/network.json`

```json
{
  "local_ip": "192.168.10.1",
  "remote_ip": "192.168.10.2",
  "grpc_port": 50051,
  "heartbeat_interval_seconds": 5,
  "heartbeat_timeout_count": 3,
  "network_monitor_interval_seconds": 5.0,
  "network_interface": "",
  "netmask": "255.255.255.252"
}
```

**关键填写规则**：
- `local_ip` 必须与检测节点 的 `remote_ip` **完全一致**
- `remote_ip` 必须与检测节点 的 `local_ip` **完全一致**
- 部署前由人工分配 IP 段，双节点网线直连
- `network_interface` 留空时自动检测；若自动检测不准（如多网卡场景），手动指定网卡名（如 `eth0` / `ens33`），详见 §5.4.3
- `netmask` 默认 `255.255.255.252`（`/30`，双节点直连够用），可改为 `255.255.255.0`（`/24`）以接入更大子网
- `gateway` **可选**，配置后 `setup-network` 使用该网关；未配置时由工具自动推导为 `local_ip` 网段 `.1`（如 `192.168.10.10` → `192.168.10.1`）。可在运行时用 `--gateway` 临时覆盖，仅本次生效
- 当本机 IP 与 `local_ip` 不一致时，执行 `setup-network` 子命令自动配置（详见 §5.4）
- `network_monitor_interval_seconds`：**网络动态监测周期**（默认 5.0 秒）。启动后由 `NetworkMonitor` 后台线程周期探测检测节点 可达性，不阻塞启动。对端不可达时系统进入降级模式（仅视频+激光，禁用识别按钮），网络恢复后自动回到在线模式；检测节点 心跳与网络状态任一离线即降级（详见 §五 排错手册）

### 7.2 相机参数示例 `config/camera.json`

```json
{
  "rgb_width": 1920,
  "rgb_height": 1080,
  "rgb_fps": 30,
  "rgb_format": "MJPG",
  "depth_width": 640,
  "depth_height": 480,
  "depth_fps": 30,
  "device_index": 0,
  "compatible_devices": [
    {
      "brand": "Orbbec",
      "model": "Gemini 336L",
      "pid": "0x0807",
      "vid": "0x2bc5"
    },
    {
      "brand": "Orbbec",
      "model": "Astra",
      "pid": "0x0636",
      "vid": "0x2bc5"
    }
  ]
}
```

**关键填写规则**：
- `device_index` 为 USB 相机索引，单设备接入时为 0
- 原型阶段仅使用 RGB 通道参与推理与测量，Depth 留作后续精度增强项
- 兼容设备由 `compatible_devices` 结构化数组定义（已替代旧版 `compatible_models` 纯字符串列表），每项含 `brand` / `model` / `pid` / `vid` 四字段
- `pid` / `vid` 使用十六进制字符串（例如 `"0x0807"`），`config_loader.py` 负责 hex→int 解析；新增兼容设备时请查阅厂商 USB 描述符
- 旧版 `compatible_models` 字段存在时 `config_loader.py` 会输出 WARNING 并忽略

### 7.3 相机内参示例 `config/intrinsics.json`

```json
{
  "fx": 950.0,
  "fy": 950.0,
  "cx": 960.0,
  "cy": 540.0
}
```

**关键填写规则**：
- 默认值沿用现有约定（AGENTS.md §8.3），后续由人工根据实际标定结果优化替换
- `mm/px` 换算公式：`mm/px = 工作距离(mm) ÷ fx`

### 7.4 推理节奏示例 `config/inference.json`

```json
{
  "inference_interval_seconds": 3.0
}
```

**控制权归属**：仅在服务节点 侧执行——服务节点 收到用户触发后，先读取此值进行时间戳门控，未达间隔阻止请求发出。检测节点 侧不做任何时间间隔检查（AGENTS.md §3.3）。

### 7.5 日志参数示例 `config/logging.json`

```json
{
  "max_bytes": 10485760,
  "backup_count": 3,
  "level": "INFO"
}
```

### 7.6 systemd 服务参数示例 `config/service.json`

```json
{
  "service_name": "rebar-node-server",
  "exec_start": "/opt/node_server/.venv/bin/python /opt/node_server/main.py",
  "working_directory": "/opt/node_server",
  "user": "root",
  "restart": "always"
}
```

⚠️ **关键变更**：`exec_start` 已从 `/usr/bin/python3` 改为 **`/opt/node_server/.venv/bin/python`**（uv 创建的虚拟环境 Python），与 spec.md MODIFIED Requirement 一致。

### 7.7 串口参数示例 `config/serial.json`

```json
{
  "device_path": "",
  "auto_scan": true,
  "vid_whitelist": [6790],
  "pid_whitelist": [21972, 21973],
  "description_keywords": ["Serial", "CH910", "CH340"],
  "path_prefix_whitelist": ["/dev/ttyUSB", "/dev/ttyACM"],
  "path_prefix_blacklist": ["/dev/ttyAMA", "/dev/ttyS", "/dev/ttyconsole"],
  "baudrate": 115200,
  "select_first_when_multiple": true,
  "reconnect_interval_seconds": 3.0,
  "max_reconnect_attempts": 5,
  "scan_interval_seconds": 2.0,
  "max_age_seconds": 2.0,
  "usb_unstable_threshold": 5
}
```

**关键字段说明**：

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `device_path` | `""`（空） | 非空时直接使用指定设备路径，跳过自动扫描；留空时启用自动扫描 |
| `auto_scan` | `true` | 是否启用 VID/PID 白名单自动扫描 S21C 设备 |
| `vid_whitelist` | `[6790]` | WCH 厂商 ID（0x1A86=**6790**，注意：0x1A86 心算为 4290 是错误的，见 config_loader 校验告警） |
| `pid_whitelist` | `[21972, 21973]` | CH9102 系列 PID（0x55D4=21972 CDC ACM 模式，0x55D5=21973 UART 模式） |
| `path_prefix_whitelist` | `["/dev/ttyUSB", "/dev/ttyACM"]` | 设备节点路径前缀白名单，兼容 USB 直连（ttyUSB）与扩展坞接入（ttyACM） |
| `path_prefix_blacklist` | `["/dev/ttyAMA", "/dev/ttyS", "/dev/ttyconsole"]` | 排除系统内置串口（AMA/S/console） |
| `baudrate` | `115200` | S21C 串口波特率（AGENTS.md §5.1） |
| `select_first_when_multiple` | `true` | 扫描到多个 S21C 时默认使用第一个（按 device_path 字典序） |
| `reconnect_interval_seconds` | `3.0` | 断线重扫的退避间隔（兼容旧字段，现由 `scan_interval_seconds` 统一控制扫描周期） |
| `max_reconnect_attempts` | `5` | ⚠️ **已弃用**：S21C 已改为**热插拔无限重连**，不再受此上限约束；保留字段仅为兼容旧配置 |
| `scan_interval_seconds` | `2.0` | **热插拔扫描间隔**：持续扫描 S21C 设备的轮询周期（发现即自动连接，断线后自动重扫重连） |
| `max_age_seconds` | `2.0` | 数据新鲜度阈值，超过此秒数的激光数据视为过期 |
| `usb_unstable_threshold` | `5` | 5 分钟内断线次数超此阈值时告警供电问题 |

**设备识别策略**（对齐 AGENTS.md §5.1 / §8.2）：

1. `device_path` 非空 → 直接使用指定路径
2. `auto_scan=true` → 按 VID+PID 白名单扫描，兼容 `/dev/ttyUSB*`（ch341 驱动）与 `/dev/ttyACM*`（cdc_acm 驱动）两种节点类型
3. 扫描到多个 S21C 时，`select_first_when_multiple=true` 取字典序首个，其余记 INFO 日志忽略
4. 兼容 USB 直连开发板与通过扩展坞（如供电扩展坞）间接连接两种接入方式

**热插拔持续扫描**（S21C 已从"冷插拔"升级为"实时热插拔扫描"）：

- 服务启动后由 `start_scanning()` 启动后台扫描线程，以 `scan_interval_seconds`（默认 2 秒）为周期持续 `auto_detect()`，**发现设备即自动连接**
- **S21C 可随时接入/拔出**：未接入时持续扫描等待；拔出重插（含换 USB 接口导致节点路径 `/dev/ttyUSB*`→`/dev/ttyACM*` 变化）时自动重新扫描重连，全程无需人工干预
- **无限重连**：`max_reconnect_attempts` 已弃用（不再作上限），断线后无限重扫直至成功；USB 稳定性监测（`usb_unstable_threshold`）保留在 RECONNECTING 态触发供电告警

**手动指定设备路径**：当自动扫描不准时（如多个 CH9102 设备并存），可显式指定：

```json
{
  "device_path": "/dev/ttyACM0",
  "auto_scan": false
}
```

---

## 八、参考文档

- 全局规范：[`AGENTS.md`](file:///d:/swap/CPS-GJT_20260320/DEMO/AGENTS.md)
- 服务节点 详细设计：[`Design-server.md`](file:///d:/swap/CPS-GJT_20260320/DEMO/docs/Design-server.md)
- 检测节点 快速启动：[`node_detect/QUICKSTART.md`](file:///d:/swap/CPS-GJT_20260320/DEMO/node_detect/QUICKSTART.md)
- 系统服务管理脚本参数：[`config/service.json`](file:///d:/swap/CPS-GJT_20260320/DEMO/node_server/config/service.json)
- 一键部署脚本：[`scripts/deploy_node_server.sh`](file:///d:/swap/CPS-GJT_20260320/DEMO/node_server/scripts/deploy_node_server.sh)
- 系统服务管理脚本：[`scripts/service-manager.sh`](file:///d:/swap/CPS-GJT_20260320/DEMO/node_server/scripts/service-manager.sh)
- 网络配置脚本：[`scripts/setup_network.sh`](file:///d:/swap/CPS-GJT_20260320/DEMO/node_server/scripts/setup_network.sh)（详见 §5.4）
- 网络配置模块源码：[`utils/system/network_setup.py`](file:///d:/swap/CPS-GJT_20260320/DEMO/node_server/utils/system/network_setup.py)
