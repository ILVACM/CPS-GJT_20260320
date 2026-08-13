# 钢筋直径测量设备（双节点边缘系统）

> 基于边缘计算的钢筋直径与间距自动检测设备原型。AI 语义分割 + 激光测距 + 国标规格对齐，本地闭环运行。

---

## 一句话定位

由深度摄像机采集现场画面，AI 语义分割识别图中钢筋，结合激光测距完成像素到物理尺寸的换算，自动输出每根钢筋的直径（就近对齐国标规格）与相邻间距，并在本地实时显示叠加了测量结果的画面。

---

## 核心特性

- **双节点架构**：AI 推理（Jetson Nano）与系统主控（OrangePi KunPeng）物理隔离，互不干扰
- **AI 语义分割**：UNet（ResNet50 骨干）3 类分割 @ 640×640，识别纵向/横向钢筋
- **激光测距融合**：4 路 STP23L 激光探头 + S21C 主控汇总，支持多种融合策略
- **国标对齐**：自动就近对齐 6~40mm 国标规格系列
- **gRPC 通信**：双节点间强类型 protobuf 协议，帧同步机制保障数据一致性
- **本地 GUI**：tkinter 实时显示测量画面，CLI 交互式融合策略选择

---

## 技术栈 [已校验一致]

| 层次 | 选型 |
|------|------|
| 开发语言 | Python 3 |
| AI 模型 | PyTorch，UNet（ResNet50 backbone，3 类语义分割） |
| 视觉与测量 | OpenCV、NumPy |
| 节点间通信 | gRPC（Protocol Buffers） |
| 深度相机 | Orbbec Gemini 336L（RGB + Depth） |
| 激光测距 | STP23L ×4 → S21C 主控 → Type-C 串口 |
| 本地 GUI | tkinter |
| 操作系统 | 检测节点：Ubuntu 22.04 LTS；服务节点：openEuler 22 LTS |

---

## 项目结构概览 [已校验一致]

```
DEMO/
├── AGENTS.md              # 项目架构规范（面向 Agent）
├── README.md              # 本文件 — 项目门户（面向人类）
├── docs/                  # 知识库内容目录
│   ├── INDEX.md           # 知识库索引与路由
│   ├── Design-AI_detect.md    # 检测节点专项设计
│   └── Design-server.md       # 服务节点专项设计
├── node_detect/           # 检测节点（Jetson Nano，AI 推理端）
│   ├── main.py
│   ├── inference/         # UNet 推理核心
│   ├── grpc_server/       # gRPC 服务端（Infer RPC）
│   ├── grpc_client/       # gRPC 客户端（Heartbeat/Shutdown）
│   ├── config/            # 配置文件（JSON）
│   └── ...
├── node_server/           # 服务节点（OrangePi，主控端）
│   ├── main.py
│   ├── utils/camera/      # 相机输入模块
│   ├── utils/serial/      # 串口通信与激光解析
│   ├── utils/grpc_client/ # gRPC 客户端
│   ├── utils/measure/     # 测量换算
│   ├── utils/ui/          # GUI 交互
│   ├── utils/system/      # 启停与健康监测
│   ├── config/            # 配置文件（JSON）
│   └── ...
├── _temp/                 # 校验与推测中间产物（供审计追溯）
└── temp/                  # 工作目录（含历史脚本与归档）
```

---

## 快速开始提示

### 检测节点（Jetson Nano）

```bash
cd node_detect
# 安装依赖（使用 uv）
uv sync
# 启动推理服务
uv run python main.py run
```

详见 [`node_detect/QUICKSTART.md`](node_detect/QUICKSTART.md)

### 服务节点（OrangePi）

```bash
cd node_server
# 安装依赖（使用 uv）
uv sync
# 启动主控服务
uv run python main.py
```

详见 [`node_server/QUICKSTART.md`](node_server/QUICKSTART.md)

---

## 项目阶段 [已校验一致]

本项目当前为 **"原型技术验证 Demo" 阶段**：
- 阶段目标仅为**验证端到端技术链路可行**（采集 → 推理 → 测量 → 显示）
- 精度/延迟指标、流式推理、TensorRT/FP16、自动化测试与 CI 等属 C 类要素，本阶段不引入

---

## 关联文档

| 文档 | 说明 |
|------|------|
| [`AGENTS.md`](AGENTS.md) | 项目架构规范（全局权威，面向 Agent） |
| [`docs/INDEX.md`](docs/INDEX.md) | 知识库索引与路由 |
| [`docs/Design-AI_detect.md`](docs/Design-AI_detect.md) | 检测节点专项设计 |
| [`docs/Design-server.md`](docs/Design-server.md) | 服务节点专项设计 |
| [`node_detect/QUICKSTART.md`](node_detect/QUICKSTART.md) | 检测节点快速启动指南 |
| [`node_server/QUICKSTART.md`](node_server/QUICKSTART.md) | 服务节点快速启动指南 |

---

> 本文件由知识库体系自动维护，详细信息请参阅 [`docs/`](docs/) 目录。
>
> **可信度声明**：本文件关键信息标注可信度标签（如 `[已校验一致]`），标签含义请参阅 [`AGENTS.md`](AGENTS.md) §12。
