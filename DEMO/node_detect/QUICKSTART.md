# 检测节点 快速启动指南（QUICKSTART）

> **适用对象**：刚接手本项目的实施人员 / 部署工程师
> **节点定位**：AI 推理端（NVIDIA Jetson Nano，Ubuntu 22.04 LTS / JetPack）
> **核心职责**：接收服务节点 发来的 RGB 图像，执行 UNet 语义分割，返回 0/1/2 类别掩码
> **对应规范**：`AGENTS.md` §2.2 / `Design-AI_detect.md`
> **工具链红线**：本项目统一使用 **`uv`** 作为 Python 包管理与运行工具，**严禁** `pip` / `conda` / `virtualenv` 直装

---

## 一、环境准备

### 1.1 硬件清单

| 项 | 规格 | 说明 |
|----|------|------|
| 主机 | NVIDIA Jetson Nano（4GB 内存版本） | 必须为 Jetson 专用镜像，原生兼容 CUDA |
| 网络 | RJ45 网口 | 用于与服务节点 网线直连 |
| 电源 | 官方 5V/4A DC 电源 | ⚠️ 务必使用官方配置，欠压会自动降频导致推理异常 |
| 显示器 | 调试期 HDMI 显示器一台 | 正式部署可不接（headless 服务模式） |
| 存储 | MicroSD ≥ 64GB（推荐 128GB） | PyTorch wheel + 模型权重占用空间大 |

### 1.2 软件版本要求

| 项 | 版本要求 | 说明 |
|----|----------|------|
| OS | Ubuntu 22.04 LTS（Jetson 官方定制内核） | 必须为 Jetson 专用镜像 |
| 发行版支持 | 部署脚本自适应识别 Debian/RHEL 系；Jetson Nano 实际仅 Ubuntu | `detect_distro()` 容错预留，CUDA/JetPack 仅 Debian 系可用 |
| JetPack | ≥ 5.x | 与 CUDA 版本匹配，由 Jetson镜像自带 |
| CUDA | 与 JetPack 匹配 | JetPack 自带，无需单独安装 |
| Python | 3.8+（建议 3.10） | 系统自带 |
| uv | ≥ 0.4.0 | 见 §二 安装步骤 |
| PyTorch | ≥ 2.0（**Jetson 专用 wheel**） | ⚠️ 不能用 PyPI 默认 wheel，见避坑提示 |

### 1.3 系统级依赖（一次性预装，自适应发行版）

⚠️ **以下依赖必须先于 `uv sync` 安装**，否则 OpenCV/grpcio 会在 ARM64 上触发源码编译，弱性能 Jetson Nano 实测会卡死 2-6 小时甚至 OOM。

#### 1.3.1 包名映射表（DEB 系 vs RPM 系）

部署脚本 `deploy_node_detect.sh` 内置 `detect_distro()` 自动识别发行版，并经 `install_pkgs()` 按下表自动映射包名。Jetson Nano 实际仅 Ubuntu 系，下表 RHEL 列仅为完整性预留：

| 逻辑用途 | Debian 系（apt） | RHEL 系（dnf/yum） | 说明 |
|----------|------------------|---------------------|------|
| OpenGL 运行时 | `libgl1` | `mesa-libGL` | OpenCV 必须 |
| glib2 运行时 | `libglib2.0-0` | `glib2` | OpenCV 必须 |
| libSM | `libsm6` | `libSM` | OpenCV 必须 |
| libXext | `libxext6` | `libXext` | OpenCV 必须 |
| libXrender | `libxrender1` | `libXrender` | OpenCV 必须 |
| libgthread | `libgthread2.0-0` | **（空）** | RHEL 系由 `glib2` 主包提供 |
| GTK2 开发头 | `libgtk2.0-dev` | `gtk2-devel` | OpenCV highgui 编译 |
| 构建工具链 | `build-essential` | `gcc gcc-c++ make cmake` | grpcio 源码编译 |
| pkg-config | `pkg-config` | `pkgconfig` | 编译期 |
| Python 开发头 | `python3-dev` | `python3-devel` | 编译期 |
| 内核头 | `linux-headers-$(uname -r)` | `kernel-devel` | 编译期 |
| JetPack 工具 | `nvidia-jetpack` | **（空）** | 仅 Ubuntu/Jetson 有 |

#### 1.3.2 Debian 系手动安装（Ubuntu / Jetson —— 实际唯一路径）

```bash
# 1. 更新 apt 索引（推荐配置阿里云镜像源加速）
sudo apt update && sudo apt upgrade -y

# 2. 安装 OpenCV 运行时依赖（避免 ImportError: libGL.so.1 / libgthread-2.0.so.0）
sudo apt install -y \
    libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 \
    libgthread2.0-0 libgtk2.0-dev

# 3. 安装构建工具链（grpcio / numpy 无 wheel 时回退源码编译必需）
sudo apt install -y \
    build-essential pkg-config \
    python3-dev linux-headers-$(uname -r)

# 4. 安装 Jetson 专用工具与 CUDA 工具链（JetPack 自带，确认存在）
sudo apt install -y \
    nvidia-jetpack \
    cuda-toolkit-<your-version> \
    libcudnn8 libcudnn8-dev

# 5. 验证 CUDA 环境
nvcc --version
echo $CUDA_HOME    # 应为 /usr/local/cuda
```

#### 1.3.3 RHEL 系（仅完整性预留，Jetson 不会走此路径）

```bash
# RHEL 系无 nvidia-jetpack，CUDA 工具链需人工确认
sudo dnf install -y \
    mesa-libGL glib2 \
    libSM libXext libXrender \
    gtk2-devel \
    gcc gcc-c++ make cmake pkgconfig \
    python3-devel kernel-devel
```

⚠️ **避坑提示**：
- `libgthread2 libgtk2 libgtk3` 是**错误包名**（既非标准 DEB 也非标准 RPM），RHEL 系安装会报 `No match for argument`；正确写法见上表
- Jetson Nano（4GB 共享内存）安装依赖期间内存吃紧，建议关闭 GUI 桌面（`sudo systemctl set-default multi-user.target`）后安装，安装完成再切回 `graphical.target`
- 若 `nvcc --version` 报错，将 `export CUDA_HOME=/usr/local/cuda` 与 `export PATH=$CUDA_HOME/bin:$PATH` 写入 `~/.bashrc` 后 `source ~/.bashrc`
- 部署脚本已内置自适应逻辑，**推荐直接使用 `sudo bash scripts/deploy_node_detect.sh`**

---

## 二、手动部署（调试模式）

> **强制结构**：以下步骤以表格形式呈现，每一步均独立可验证。建议在调试期按表执行，便于定位失败步骤。

### 2.1 部署步骤表

| 步骤 | Bash 命令 | 预期输出/说明 | 避坑提示 |
|:-----|:----------|:---------------|:---------|
| **1. 部署代码到 /opt** | `sudo mkdir -p /opt/node_detect && sudo cp -r /path/to/DEMO/node_detect/* /opt/node_detect/` | 目录 `/opt/node_detect` 下含 `main.py` `config/` `proto/` 等 | ⚠️ **路径必须为 `/opt/node_detect`**，systemd unit 默认指向此路径；若自定义路径需同步改 `service_unit.json` |
| **2. 进入项目目录** | `cd /opt/node_detect` | 当前工作目录切换 | 后续所有命令均在此目录执行 |
| **3. 安装 uv** | `curl -LsSf https://astral.sh/uv/install.sh \| sh` | 安装完成提示 `uv` 已加入 `~/.local/bin` | ⚠️ 安装后执行 `source ~/.bashrc` 或重开终端使 PATH 生效；ARM64 平台确认二进制可用：`uv --version` 应输出 `uv 0.4.x` |
| **4. 配置 uv 镜像源（可选，阿里云加速）** | `export UV_HTTP_TIMEOUT=600`<br>`export UV_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/` | 环境变量生效 | ⚠️ **可选**，PyPI 在国内访问慢甚至超时时配置；永久生效写入 `~/.bashrc`；一键部署脚本可加 `--aliyun` 参数临时启用（见 §四）；如需配置 apt 阿里云源参考 https://developer.aliyun.com/mirror/ubuntu |
| **5. 创建项目虚拟环境** | `uv venv --python 3.10` | 输出 `Using Python 3.10.x` 与 `.venv` 路径 | ⚠️ Jetson Nano 上 Python 3.10 兼容性最佳；若系统仅 Python 3.8，`uv` 会自动下载 3.10 |
| **6. 安装 Python 依赖（uv sync）** | `uv sync` | 逐包解析并安装，最终 `Resolved N packages`，生成 `uv.lock` | ⚠️ **PyTorch 不会从 PyPI 安装**；为杜绝"双源竞争"（`uv sync` 从 PyPI 拉 2.13+cu130 覆盖 Jetson whl），`pyproject.toml` 与 `requirements.txt` 中**均注释了** `torch` / `torchvision`，`uv sync` 不解析任何 torch 依赖。torch 的唯一来源是步骤 7 的 Jetson whl。如 `uv sync` 仍卡死 torch，说明 pyproject.toml 中 torch 未正确注释，需回查修复后重跑；开发机 / CI 若需 torch，需自行 `pip install torch`（双轨兼容，见末尾双轨说明） |
| **7. 安装 Jetson 专用 PyTorch wheel** | 见下方 §2.2 单独说明 | torch + torchvision 安装完成 | ⚠️ **核心坑点**，必须使用 NVIDIA Jetson 官方 wheel，PyPI 默认 wheel 不含 CUDA 支持 |
| **8. 编译 proto 文件** | `cd /opt/node_detect && sudo uv run python -m grpc_tools.protoc \<br>    --proto_path=. \<br>    --python_out=. \<br>    --grpc_python_out=. \<br>    proto/rebar_inference.proto` | `proto/rebar_inference_pb2.py` 与 `rebar_inference_pb2_grpc.py` 重新生成 | ⚠️ proto 更新后必须重编译；必须用 `--proto_path=.` 让 protoc 识别 proto 为包，生成包内导入（`from . import`）；使用 `uv run` 而非裸 `python` 以调用虚拟环境内的 grpcio-tools |
| **9. 放置模型权重** | `sudo mkdir -p /opt/node_detect/weights`<br>`sudo cp /path/to/Unet_resnet50.pth /opt/node_detect/weights/` | `ls -l weights/Unet_resnet50.pth` 显示文件 | ⚠️ 权重文件不入版本控制（AGENTS.md §8.4），需人工放置；路径与 `inference.json` 的 `model_path` 一致 |
| **10. 环境自检** | `sudo uv run main.py self-check` | 输出 `[自检] 全部通过` | ⚠️ 必须 `sudo`（root 权限绑定 50051 端口）；自检不过禁止启动服务 |
| **11. 前台启动（调试模式）** | `sudo uv run main.py run` | 见 §三 预期现象 | 调试期推荐前台运行，便于实时查看日志；正式部署见 §四 |

### 2.2 Jetson 专用 PyTorch wheel 安装

Jetson Nano 上 PyTorch **必须使用 NVIDIA 官方 wheel**，PyPI 默认 wheel 不含 CUDA 支持，会导致 `torch.cuda.is_available()` 返回 `False`。

```bash
# 1. 确认 JetPack 版本（决定 wheel 选择）
cat /etc/nv_tegra_release
# 或
dpkg -l | grep nvidia-l4t-core

# 2. 下载匹配 JetPack 版本的 PyTorch wheel（以 JetPack 5.x / Python 3.10 为例）
# 官方下载页：https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetpack-announcements/420
# 假设已下载 torch-2.x-cp310-cp310-linux_aarch64.whl 到 ~/Downloads

# 3. 安装到 uv 虚拟环境（强制跳过 PyPI 解析）
uv pip install ~/Downloads/torch-2.x-cp310-cp310-linux_aarch64.whl

# 4. 安装 torchvision（同样需 Jetson 版本）
uv pip install ~/Downloads/torchvision-0.15.x-cp310-cp310-linux_aarch64.whl

# 5. 验证 CUDA 可用性
uv run python -c "import torch; print('CUDA:', torch.cuda.is_available(), '设备:', torch.cuda.get_device_name(0))"
# 预期输出：CUDA: True 设备: NVIDIA Tegra X1
```

⚠️ **避坑提示**：
- `uv pip install` 与 `uv add` 均可，但 `uv add` 会修改 `pyproject.toml`，对本地 wheel 不友好，建议用 `uv pip install`
- 若 `torch.cuda.is_available()` 返回 `False`，检查 `CUDA_HOME` 与 `LD_LIBRARY_PATH` 是否指向 `/usr/local/cuda`
- PyTorch 与 JetPack 版本必须严格匹配，否则加载权重时报 `RuntimeError: Failed to load c10_cuda`
- 权重文件 `Unet_resnet50.pth` 已修复 `in_filters=[256,512,1024,2048]` 错误（见原 `new-predict.py` 归档说明），如加载失败请确认权重版本

---

## 三、预期现象

### 3.1 启动成功标志

| 检查项 | 预期 |
|--------|------|
| 控制台日志 | 出现 `[INFO] [主循环] 阻塞等待退出信号...` 字样 |
| 端口监听 | `sudo ss -tlnp \| grep 50051` 显示 LISTEN |
| 心跳推送 | 每 5 秒输出一条 `[心跳] 推送` 日志（服务节点 在线时返回 `accepted=true`） |
| 服务节点 视图 | 服务节点 界面显示"推理服务在线"（绿色） |
| 状态机 | INIT → IDLE（推理就绪） |

### 3.2 推理请求处理

服务节点 触发识别时，检测节点 控制台应输出：

```
[INFO] [Infer] 收到请求 frame_id=1, distance=812.5mm
[INFO] [Infer] 完成 frame_id=1, size=1920x1080, 耗时=320ms, status=1
```

### 3.3 优雅退出

按 `Ctrl+C` 或 `systemctl stop` 后，应依次：

1. 收到 SIGTERM/SIGINT 信号
2. 状态机切换到 `SHUTTING_DOWN`
3. 通过 gRPC 向服务节点 发送 `Shutdown` 通知（**D→S 方向**，best-effort，2 秒超时）
4. 停止心跳推送线程
5. 停止 gRPC 服务端
6. 释放 CUDA 缓存（`torch.cuda.empty_cache()`）
7. 进程退出

> ⚠️ **方向纠正**：根据 `AGENTS.md` §5.5 / §5.7.3，`Shutdown` RPC 方向为 **D→S（检测节点 → 服务节点）**，检测节点 是发起方，服务节点 是 grpc_server 接收方。

---

## 四、一键部署（生产模式）

> 生产环境部署推荐使用项目提供的 `scripts/deploy_node_detect.sh` 自动化脚本，含环境检查、镜像源配置、错误处理。

```bash
# 1. 赋予执行权限
cd /opt/node_detect
sudo chmod +x scripts/deploy_node_detect.sh

# 2. 执行一键部署（默认使用系统当前软件源）
sudo bash scripts/deploy_node_detect.sh

# 2'. 或：本次运行临时使用阿里云镜像源（apt + uv 全覆盖，不修改系统配置）
sudo bash scripts/deploy_node_detect.sh --aliyun

# 脚本内部依次完成：
#   ① root 权限检查 / ARM64 架构检查 / Jetson 特性检查
#   ② 软件源配置（默认系统源；--aliyun 临时复制系统 repo 替换为阿里云 host）
#   ③ 刷新包管理器元数据（apt update，不做全系统升级）+ 安装系统级依赖
#   ④ 安装 uv 工具链（uv 镜像源：默认沿用系统配置；--aliyun 临时 export 阿里云）
#   ⑤ uv venv 创建 + uv sync 安装 Python 依赖
#   ⑥ proto 编译（uv run python -m grpc_tools.protoc）
#   ⑦ 提示用户人工放置模型权重到 weights/Unet_resnet50.pth
#   ⑧ 注册 systemd 服务（node-detect-inference）并启动
```

⚠️ **避坑提示**：
- 脚本含 `set -e`，任一步骤失败立即停止，不产生半安装状态
- **默认使用系统当前软件源**，不修改 `/etc`（不写 `/etc/environment`、不替换 `/etc/apt/sources.list`）；`--aliyun` 仅本次运行临时生效，退出后自动清理临时 repo 文件
- 模型权重文件**必须由人工放置**，脚本不会自动下载（AGENTS.md §8.4 约定）
- 如系统源损坏或过慢，加 `--aliyun` 参数临时切阿里云；若需永久配置阿里云源请手动参照 https://developer.aliyun.com/mirror/ubuntu
- `--aliyun` 在 Jetson 上会排除 NVIDIA 私有 repo，`nvidia-jetpack` 可能不可用（脚本已容错跳过，因 JetPack 通常已预装）

### 4.1 systemd 服务管理

```bash
# 查看服务状态
sudo systemctl status node-detect-inference

# 启动 / 停止 / 重启
sudo systemctl start node-detect-inference
sudo systemctl stop node-detect-inference
sudo systemctl restart node-detect-inference

# 查看实时日志
sudo journalctl -u node-detect-inference -f
# 或查看应用日志
sudo tail -f /opt/node_detect/logs/node_detect.log

# 卸载服务（停止 + 取消开机自启 + 删除 unit 文件）
sudo bash scripts/service-manager.sh uninstall
```

---

## 五、排错手册（针对推理端特有问题）

### 5.1 常见启动失败原因

| 现象 | 原因 | 解决办法 |
|------|------|----------|
| **`[自检] 失败 — 安全退出`** | 环境自检未通过 | 查看具体失败项：① `local_ip` 与本机网卡 IP 不一致？→ 执行 `sudo /opt/node_detect/.venv/bin/python /opt/node_detect/main.py setup-network` 自动配置静态 IP（详见 §5.3）；② 50051 端口被占用？→ `sudo lsof -i :50051` 后 `kill -9 <PID>`；③ CUDA 不可用？→ 见下方 CUDA 行；④ `model_path` 文件不存在？→ 见下方模型行 |
| **`RuntimeError: CUDA is not available`** | PyTorch 未启用 CUDA / 系统无 CUDA | ① `uv run python -c "import torch; print(torch.cuda.is_available())"` 应为 `True`；② 若 `False`，重装 Jetson 专用 PyTorch wheel（见 §2.2）；③ 检查 `echo $CUDA_HOME` 是否为 `/usr/local/cuda` |
| **`RuntimeError: Failed to load c10_cuda`** | PyTorch wheel 与 JetPack 版本不匹配 | 重新下载匹配 JetPack 版本的 wheel，参考 [NVIDIA 官方指南](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html) |
| **`[模型] 权重文件不存在`** | `inference.json` 中 `model_path` 路径错误 | 改为绝对路径，确认文件存在：`ls -l /opt/node_detect/weights/Unet_resnet50.pth` |
| **`[模型] 加载失败: state_dict 不匹配`** | 权重与模型结构不一致 | 确认 `in_filters=[256,512,1024,2048]`（修复过原版错误）；权重文件需用修复后版本 |
| **`uv sync` 卡死在 torch 安装** | PyPI 默认 wheel 不支持 Jetson CUDA | 用 `uv sync --no-deps` 跳过 torch，单独按 §2.2 安装 Jetson 专用 wheel |
| **`uv sync` 卡死在 opencv/grpc 编译** | ARM64 无预编译 wheel，触发源码编译 | 先执行 §1.3 系统级 apt 依赖安装；配置 uv 镜像源（步骤 4） |
| **`[gRPC] 启动失败: 端口被占用`** | 已有进程占用 50051 | `sudo lsof -i :50051` 后 `kill -9 <PID>`，或修改 `network.json` 中 `grpc_port`（双节点同步改） |
| **心跳推送持续 WARNING** | 服务节点 未启动 / IP 配置不一致 | ① 启动服务节点；② 核对双节点 `network.json` 中 IP 对称性 |
| **`uv: command not found`** | uv 未加入 PATH | `source ~/.bashrc` 或执行 `export PATH="$HOME/.local/bin:$PATH"` |

### 5.2 诊断命令速查

```bash
# 查看实时应用日志
sudo tail -f /opt/node_detect/logs/node_detect.log

# 检查端口监听
sudo ss -tlnp | grep 50051

# 检查 CUDA 可用性
uv run python -c "import torch; print('CUDA:', torch.cuda.is_available(), '设备:', torch.cuda.get_device_name(0))"

# 检查 JetPack 版本
cat /etc/nv_tegra_release

# 检查 Python 环境
uv run python --version
uv pip list | grep -E "torch|grpcio|opencv"

# 检查网卡 IP
ip addr show eth0

# 离线验证 predictor（无需启动服务，无需相机/网络）
cd /opt/node_detect
sudo uv run python tests/test_predictor_offline.py

# 用 mock_client 模拟服务节点 发送推理请求（用于联调）
cd /opt/node_detect
sudo uv run python tests/mock_client.py --port 50051 --frames 3

# 监控 GPU 利用率（推理时确认 CUDA 真在跑）
sudo tegrastats
# 关注 GR3D_FREQ 字段，推理时应接近 99%
```

### 5.3 网络配置（IP 不匹配时使用）

当 systemd 启动报"本机 IP 与配置 local_ip 不一致"硬故障时，使用 `setup-network` 子命令自动完成备份 + 配置静态 IP。该子命令独立于 systemd 启动流程，由人工主动触发（对齐 AGENTS.md §4.2）。

#### 5.3.1 命令用法

```bash
# 方式一：直接调用 Python 子命令（推荐）
sudo /opt/node_detect/.venv/bin/python /opt/node_detect/main.py setup-network

# 方式二：Shell 包装脚本
sudo bash /opt/node_detect/scripts/setup_network.sh
```

> ⚠️ **必须以 root 权限运行**（脚本内部也会校验），否则直接退出。
> ⚠️ **修改 IP 后当前 SSH 会话可能断开**，请确保已通过本地终端或带外方式连接设备。

#### 5.3.2 执行流程

1. **备份当前网络配置**：保存到 `logs/network_backup_YYYYMMDD-HHMMSS/` 目录
   - `ip addr` / `ip route` / `nmcli dev show` / `nmcli connection show` 命令输出
   - `/etc/resolv.conf` / `/etc/network/interfaces` 配置文件
   - `/etc/sysconfig/network-scripts/` 整目录（RHEL 系，Jetson 通常不会走此路径）
   - `/etc/netplan/` 整目录（Debian 系）
   - `backup_manifest.json` 清单文件（含 hostname / 备份文件列表 / 时间戳）
2. **自动检测物理网卡**：遍历 `/sys/class/net/` 排除 `lo` / `docker` / `virbr` / 无线网卡，优先选状态 UP 的物理网卡
3. **用户确认**：提示"即将修改 IP，SSH 会断开"，必须输入 `yes` 确认继续
4. **配置静态 IP**：自适应发行版
   - RHEL 系（openEuler/CentOS 等）：写入 `/etc/sysconfig/network-scripts/ifcfg-<iface>`，重启 `NetworkManager`（失败时回退 `network` 服务）
   - Debian 系（Ubuntu/Jetson/Debian）：写入 netplan（`/etc/netplan/01-rebar-static.yaml`）或 `/etc/network/interfaces`，重启 `networking`
5. **验证新 IP**：等待 3 秒后执行 `ip addr show <iface>` 确认新 IP 生效
6. **输出下一步指引**：提示用新 IP 重连 SSH + 重启 systemd 服务

#### 5.3.3 配置文件字段

`config/network.json` 新增可选字段：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `network_interface` | string | `""`（空） | 指定网卡名；留空时自动检测物理网卡 |
| `netmask` | string | `"255.255.255.252"` | 子网掩码；`/30` 双节点直连够用（仅 4 个地址），`/24`（`255.255.255.0`）更通用 |

#### 5.3.4 恢复网络配置

若配置失败需恢复，从备份目录手动恢复：

```bash
# 查看备份目录
ls -l /opt/node_detect/logs/network_backup_*/

# 查看备份清单
cat /opt/node_detect/logs/network_backup_*/backup_manifest.json

# 恢复 ifcfg 文件（RHEL 系）
sudo cp /opt/node_detect/logs/network_backup_*/ifcfg-backup/ifcfg-eth0 /etc/sysconfig/network-scripts/
sudo systemctl restart NetworkManager

# 恢复 netplan 配置（Ubuntu / Jetson）
sudo cp /opt/node_detect/logs/network_backup_*/netplan-backup/*.yaml /etc/netplan/
sudo netplan apply

# 恢复 /etc/network/interfaces（Debian）
sudo cp /opt/node_detect/logs/network_backup_*/interfaces.bak /etc/network/interfaces
sudo systemctl restart networking
```

#### 5.3.5 触发场景示例

systemd 启动失败日志典型形态：

```
[CRITICAL] [自检] 失败 — 安全退出: 启动自检失败: 本机 IP 校验失败: 192.168.10.2
[CRITICAL] ============================================================
[CRITICAL] 修复指引：本机 IP 与配置 local_ip 不一致
[CRITICAL] ============================================================
[CRITICAL] 请执行以下命令自动配置静态 IP（详见 QUICKSTART.md §5.3）：
[CRITICAL]   sudo /opt/node_detect/.venv/bin/python /opt/node_detect/main.py setup-network
[CRITICAL]   或：sudo bash /opt/node_detect/scripts/setup_network.sh
[CRITICAL] 配置完成后重启服务：sudo systemctl restart node-detect-inference
[CRITICAL] ============================================================
```

按上述日志中的命令执行即可。

---

## 六、关键约束速查

| 约束 | 来源 | 说明 |
|------|------|------|
| 必须使用 uv 工具链 | spec.md | 严禁 `pip` / `conda` / `virtualenv` |
| 必须启用 CUDA | AGENTS.md §8.1 | Jetson Nano 上 CPU 推理不可接受 |
| 必须使用 Jetson 专用 PyTorch wheel | NVIDIA 官方 | PyPI 默认 wheel 不含 CUDA |
| 模型权重不入版本控制 | AGENTS.md §8.4 | 部署时人工放置到 `weights/` |
| 配置文件统一 JSON 格式 | AGENTS.md §7.1 | 禁止 YAML/TOML/INI（systemd unit 除外） |
| 启动前环境自检 | AGENTS.md §4.3 | 自检不过禁止启动 |
| 必须 root/sudo 启动 | project_memory | 网络配置与端口绑定需要 root 权限 |
| 一机一日志 | AGENTS.md §7.5 | 所有模块写入 `logs/node_detect.log` |
| 优雅退出通知 | AGENTS.md §5.7 | 退出前向服务节点 发送 Shutdown RPC（**D→S 方向**，best-effort，2 秒超时） |
| systemd ExecStart 调用 uv 虚拟环境 Python | spec.md | 路径为 `/opt/node_detect/.venv/bin/python`，非 `/usr/bin/python3` |

### 6.1 依赖"双轨兼容"策略（Jetson whl vs PyPI）

为同时支持"Jetson 部署（必须用 Jetson 官方 whl）"与"开发机 / CI（通常用 PyPI）"两种环境，本项目对 PyTorch 采用双轨策略：

| 渠道 | 内容 | 说明 |
|:-----|:-----|:-----|
| `pyproject.toml` `[project.dependencies]` | **注释** `torch>=2.0.0` 与 `torchvision>=0.15.0` | 禁止 `uv sync` 从 PyPI 拉取 2.13+cu130（cu13x 要求 CUDA ≥ 13.0 / 驱动 > 12060）。torch 的唯一来源为部署脚本步骤 6 的 Jetson whl 安装 |
| `requirements.txt` | **同上注释** | 与 pyproject.toml 同步；避免开发机/CI 人工 `uv pip install -r requirements.txt` 拉错版本 |
| `scripts/deploy_node_detect.sh` 步骤 6 | `uv pip install source/torch-2.5-cp310-cp310-linux_aarch64.whl` | Jetson 部署唯一 whl 来源；安装后自动校验 CUDA 可用性、审计 torch 版本非 cu13x，并锁定 `uv.lock` |
| 开发机 / CI | 需自行安装，如 `pip install torch` | 开发机镜像可走 PyPI 通用版本；部署脚本不处理此路径，按环境自选 |

⚠️ **维护红线**：
- **永远不要**重新启用 `pyproject.toml` 或 `requirements.txt` 中的 `torch` / `torchvision` 行（取消注释）。一旦启用，`uv sync` 将覆盖 Jetson whl → 部署后立即复现 "CUDA driver too old" 错误。
- 如确需调整 torch 最低版本，**仅在 `scripts/deploy_node_detect.sh` 中修改 whl 文件名/pattern**，whl 是唯一来源。

---

## 七、配置文件清单

| 配置文件 | 内容 | 关键字段 |
|----------|------|----------|
| `config/network.json` | 网络参数 | `local_ip` / `remote_ip` / `grpc_port` / `heartbeat_interval_seconds` / `heartbeat_timeout_count` / `network_interface`（可选）/ `netmask`（可选，见 §5.3.3） |
| `config/inference.json` | 推理参数 | `model_path`（绝对路径）/ `input_width` / `input_height` / `use_cuda`（必须 `true`）/ `log_level` |
| `config/service_unit.json` | systemd unit 参数 | `exec_start`（uv 虚拟环境 Python）/ `working_directory` / `restart` |

### 7.1 网络参数示例 `config/network.json`

```json
{
  "local_ip": "192.168.10.2",
  "remote_ip": "192.168.10.1",
  "grpc_port": 50051,
  "heartbeat_interval_seconds": 5,
  "heartbeat_timeout_count": 3,
  "network_interface": "",
  "netmask": "255.255.255.252"
}
```

**关键填写规则**：
- `local_ip` 必须与服务节点 的 `remote_ip` **完全一致**
- `remote_ip` 必须与服务节点 的 `local_ip` **完全一致**
- 部署前由人工分配 IP 段，双节点网线直连，**禁止复用 DHCP**
- `network_interface` 留空时自动检测；若自动检测不准（如多网卡场景），手动指定网卡名（如 `eth0`），详见 §5.3.3
- `netmask` 默认 `255.255.255.252`（`/30`，双节点直连够用），可改为 `255.255.255.0`（`/24`）以接入更大子网
- 当本机 IP 与 `local_ip` 不一致时，执行 `setup-network` 子命令自动配置（详见 §5.3）

### 7.2 systemd 服务参数示例 `config/service_unit.json`

```json
{
  "unit_name": "node-detect-inference",
  "description": "检测节点 钢筋直径推理服务",
  "exec_start": "/opt/node_detect/.venv/bin/python /opt/node_detect/main.py run",
  "working_directory": "/opt/node_detect",
  "restart": "on-failure",
  "restart_sec": 5,
  "user": "root",
  "after": "network.target",
  "wanted_by": "multi-user.target"
}
```

⚠️ **关键变更**：`exec_start` 已从 `/usr/bin/python3` 改为 **`/opt/node_detect/.venv/bin/python`**（uv 创建的虚拟环境 Python），与 spec.md MODIFIED Requirement 一致。

---

## 八、参考文档

- 全局规范：[`AGENTS.md`](file:///d:/swap/CPS-GJT_20260320/DEMO/AGENTS.md)
- 检测节点 详细设计：[`Design-AI_detect.md`](file:///d:/swap/CPS-GJT_20260320/DEMO/Design-AI_detect.md)
- 服务节点 快速启动：[`node_server/QUICKSTART.md`](file:///d:/swap/CPS-GJT_20260320/DEMO/node_server/QUICKSTART.md)
- 系统服务管理脚本参数：[`config/service_unit.json`](file:///d:/swap/CPS-GJT_20260320/DEMO/node_detect/config/service_unit.json)
- 一键部署脚本：[`scripts/deploy_node_detect.sh`](file:///d:/swap/CPS-GJT_20260320/DEMO/node_detect/scripts/deploy_node_detect.sh)
- 网络配置脚本：[`scripts/setup_network.sh`](file:///d:/swap/CPS-GJT_20260320/DEMO/node_detect/scripts/setup_network.sh)（详见 §5.3）
- 网络配置模块源码：[`system/network_setup.py`](file:///d:/swap/CPS-GJT_20260320/DEMO/node_detect/system/network_setup.py)
- Jetson PyTorch 安装指南：[NVIDIA 官方文档](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
