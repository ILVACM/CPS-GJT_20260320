# 检测节点 — 钢筋直径测量推理服务

检测节点 是钢筋直径测量设备的 **AI 推理端**，部署于 NVIDIA Jetson Nano。接收服务节点 发送的 JPEG 图像，执行 UNet 语义分割，返回 PNG 类别掩码。

## 目录结构

```
node_detect/
├── main.py                          启动入口（7 个子命令：run/start/status/stop/install/self-check/setup-network）
├── config/
│   ├── network.json                 网络参数（IP/端口/心跳配置）
│   ├── inference.json               推理参数（模型路径/尺寸/CUDA）
│   └── service_unit.json            systemd unit 参数源
├── proto/                           gRPC 协议（protobuffer 定义 + 生成桩）
├── inference/                       推理核心（UNet 模型定义 + 预测器）
│   ├── model.py                     UNet（ResNet50 backbone, 3 类分割 @ 640x640）
│   └── predictor.py                推理器：预处理 → GPU 前向 → 后处理
├── system/                         基础设旄（日志/配置/自检/状态机）
├── grpc_server/                     gRPC 服务端（Infer RPC，S→D 方向）
├── grpc_client/                     gRPC 客户端（Heartbeat/Shutdown，D→S 方向）
├── tests/                           测试脚本（mock_client / test_predictor_offline）
├── weights/                         模型权重目录（不入版本控制）
├── logs/                            运行时日志
└── README.md                        本文件
```

## 快速启动

### 1. 前置条件

- Python 3.10+（推荐）
- uv 工具链 ≥ 0.4.0（项目红线，严禁裸 `pip` / `conda` / `virtualenv`）
- PyTorch（Jetson 专用 wheel，参见 `requirements.txt` 注释）
- grpcio / grpcio-tools（proto 编译）
- opencv-python-headless / Pillow / numpy

### 2. 安装依赖

> **工具链红线**：本项目统一使用 **`uv`** 作为 Python 包管理工具。
> 完整首次部署步骤（含 uv 安装、镜像源配置、Jetson 专用 PyTorch wheel）请参考 [`QUICKSTART.md`](QUICKSTART.md) §2.1 表格步骤 3-7。

```bash
# 创建虚拟环境并安装依赖（默认使用系统/uv 当前镜像源）
uv venv --python 3.10
uv sync

# 如 PyPI 访问慢或超时，可选临时启用阿里云镜像源（仅当前会话生效）
export UV_HTTP_TIMEOUT=600
export UV_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/

# 批量测试 source/ 下所有 whl 的 CUDA 兼容性（推荐先跑一遍再部署）
bash scripts/test_torch_whl.sh --source-dir source/

# 如需单独安装 Jetson 专用 PyTorch wheel，参考 QUICKSTART.md §2.2
# 一键部署脚本支持 --aliyun 参数临时启用阿里云源，详见 QUICKSTART.md §四
```

> **⚠️ Jetson whl 文件名校验说明**：NVIDIA 官方的 PyTorch wheel 文件名
> 含完整 JetPack 构建号（如 `nv24.7.16234504`），但内部 METADATA 版本号
> 简化为 `nv24.7`，导致 `uv` 默认校验报 `Wheel version does not match filename`。
> 这是 NVIDIA 的打包惯例而非损坏，`deploy_node_detect.sh` 与 `test_torch_whl.sh`
> 安装 whl 时均已自动设置 `UV_SKIP_WHEEL_FILENAME_CHECK=1` 跳过文件名校验，
> 无需人工干预。

### 3. 编译 proto（首次或 proto 更新时）

```bash
cd node_detect
uv run python -m grpc_tools.protoc \
    --proto_path=. \
    --python_out=. \
    --grpc_python_out=. \
    proto/rebar_inference.proto
```

### 4. 放置模型权重

将 `Unet_resnet50.pth` 放入 `weights/` 目录，然后更新 `config/inference.json` 中的 `model_path` 字段。

```json
{
  "model_path": "D:/.../DEMO/node_detect/weights/Unet_resnet50.pth"
}
```

### 5. 配置网络参数

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

> 当本机 IP 与 `local_ip` 不一致时，执行 `sudo uv run main.py setup-network` 自动备份并配置静态 IP（详见 QUICKSTART.md §5.3）。
> - `network_interface`：留空自动检测；多网卡场景手动指定（如 `eth0`）
> - `netmask`：默认 `/30`（双节点直连够用），可改 `/24` 接入更大子网

### 6. 运行

```bash
# 前台开发模式
sudo uv run main.py run

# 仅执行自检（不启动服务）
sudo uv run main.py self-check

# 配置本机静态 IP（IP 不匹配时使用，详见 QUICKSTART.md §5.3）
sudo uv run main.py setup-network

# 查看帮助
sudo uv run main.py --help
```

启动成功时控制台应输出：

```
[INFO] 检测节点 (node-detect) 启动序列开始
[INFO] [自检] 全部通过
[INFO] [状态机] INIT → IDLE（推理就绪）
[INFO] [GRPCServer] 启动成功: 0.0.0.0:50051
[INFO] [心跳] 推送线程已启动（间隔 5s）
[INFO] [主循环] 阻塞等待退出信号...
```

### 7. 冒烟测试

```bash
# 在另一个终端中运行 mock_client 模拟服务节点 发送推理请求
sudo uv run python tests/mock_client.py --port 50051 --frames 3

# 离线验证 predictor 全流程（不需要运行服务）
sudo uv run python tests/test_predictor_offline.py
```

### 8. 优雅退出

- 按 `Ctrl+C` 发送 SIGINT
- 或从另一终端执行 `sudo systemctl stop node-detect-inference` 发送 SIGTERM

退出流程：`SHUTDOWN → 推送 Shutdown 通知 → 停止心跳线程 → 停止 gRPC → 释放 CUDA 缓存`

## 系统架构

```
服务节点 ─── gRPC Infer (S→D) ──→ [gRPC Server :50051]
                                ↓
                          RebarInferenceServicer
                                ↓
                          StateMachine (IDLE → INFERENCING → RETURNING → IDLE)
                                ↓
                          RebarPredictor.infer(JPEG)
                                ↓
                          UNet Model (ResNet50 backbone, 640×640)
                                ↓
                          3类掩码 (0=背景/1=纵向钢筋/2=横向钢筋)
                                ↓
                          PNG 编码 + 状态捎带 → 响应返回
```

心跳方向（D→S，独立 gRPC channel）：

```
Heartbeat Worker (daemon, 5s)
        ↓
NodeServerClient.send_heartbeat() ──→ 服务节点 gRPC Server :50051
```

## 故障排查

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| `[自检] 本机 IP 校验失败` | 本机网卡 IP 与 `local_ip` 不一致 | 执行 `sudo uv run main.py setup-network` 自动备份 + 配置静态 IP（详见 QUICKSTART.md §5.3） |
| 端口被占用 | 已有进程占用 50051 | 检查 `lsof -i :50051` 后 kill 或直接换端口 |
| CUDA 不可用 | JetPy CUDA 配置问题 | 确认 JetPack 版本与 PyTorch wheel 匹配 |
| 权重加载失败 | 模型结构与权重不匹配 | 确认 in_filters=[256,512,1024,2048]（修复过原版错误） |
| 心跳推送失败（WARNING） | 服务节点 未启动 | 仅警告不阻塞，启动服务节点 后恢复 |
| proto 编译版本冲突 | protobuf 版本差异 | 确保 grpcio-tools 与 grpcio 版本一致 |

## 工程规范

- 日志统一写入 `logs/node_detect.log`（一机一日志，10MB × 5 轮转）
- 日志格式：`时间戳(毫秒) | 级别 | 节点(D) | 模块 | 消息`
- 模块命名空间：`grpc_server.servicer` / `inference.predictor` / `system.lifecycle` 等

## License

本项目为内部原型验证 Demo，未经授权不得外传。
