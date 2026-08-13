# 服务节点 详细设计方案 — 系统主控节点（OrangePi KunPeng）

> **文档版本**：v1.0
> **创建日期**：2026-08-28
> **关联文档**：`AGENTS.md`（项目总览与设计约定）、`./Design-AI_detect.md`（检测节点 设计方案）
> **适用范围**：服务节点 原型阶段的工程实现，覆盖模块划分、类接口、并发模型、自检流程、日志与异常策略

---

## 目录

- [0. 程序总体流程逻辑](#0-程序总体流程逻辑)
- [1. 节点职责与运行环境](#1-节点职责与运行环境)
- [2. 工程目录结构](#2-工程目录结构)
- [3. 核心模块与类设计](#3-核心模块与类设计)
- [4. 并发与线程模型](#4-并发与线程模型)
- [5. 配置加载与启动自检流程](#5-配置加载与启动自检流程)
- [6. 日志与异常处理策略](#6-日志与异常处理策略)
- [7. 推理端状态监测与记录机制](#7-推理端状态监测与记录机制)
- [8. 识别结果存储规范](#8-识别结果存储规范)
- [9. 系统服务管理脚本](#9-系统服务管理脚本)
- [10. 双退出流程](#10-双退出流程)
- [附录 C：一致性与已知冲突说明](#附录-c一致性与已知冲突说明)

---

## 0. 程序总体流程逻辑

> 本章节以"五阶段全生命周期"组织服务节点 程序的总体流程，作为各模块行为的顶层编排基准。每阶段列出触发条件、行为与状态流转；模块级细节见 §3，并发模型见 §4，启动自检见 §5。
>
> 配置加载与权限检查的顺序约束：**第一步读取加载 JSON 配置 → 第二步环境权限检查**，二者不可颠倒。理由：权限检查项（本机 IP、端口占用等）依赖配置文件中的参数（`local_ip`、`grpc_port`），配置未加载则检查无依据。

### 0.1 启动阶段

| 属性 | 说明 |
|------|------|
| 触发条件 | 用户以 `root` / `sudo` 执行 `main.py` 启动程序 |
| 前置状态 | 程序未运行 |
| 行为 | ① **第一步：读取加载 JSON 配置**（`config/*.json`，统一 `json` 模块解析）<br>② **第二步：环境权限检查**（管理员/root 权限验证、网络配置权限、gRPC 端口可用性，详见 §5）<br>③ 软硬件前置：对端检测节点 可达性 TCP 探测 |
| 状态流转 | 通过 → 进入 0.2；网络相关（本机 IP 不匹配 / 端口冲突 / 对端 A 不可达）均为**诊断日志，不退出**（由 NetworkMonitor 运行时监测，见 §5）；仅配置文件加载失败 → CRITICAL + 安全退出 |

> **网络硬故障已放开**：本次迭代将本机 IP 不匹配、端口冲突、对端节点不可达**全部改为诊断模式**（仅记录日志，不退出启动），由 `NetworkMonitor` 后台周期探测接管动态监测。对端不可达 / 网络异常时系统进入**降级模式**（仅视频 + 激光 + 本地测量/保存，识别按钮禁用），网络恢复后自动切回在线。仅配置文件加载失败等真正致命错误才 `sys.exit(1)`。详见 §5。

### 0.2 摄像头初始化阶段

| 属性 | 说明 |
|------|------|
| 触发条件 | 启动阶段环境自检通过（UI 已完成基础初始化） |
| 前置状态 | 配置已加载、环境权限已通过 |
| 行为 | ① 自动扫描本地摄像头（枚举 `/dev/video*` 或 Windows DirectShow 索引），按配置白名单自适应加载（详见 §3.3.5）<br>② 在 GUI 摄像头选择界面列出扫描结果，并支持用户手动添加网络摄像头（TCP，默认 `192.168.1.100:8080`，详见 §3.3.4）<br>③ 用户在列表中选择一个摄像头并点击"连接" |
| 状态流转 | 连接成功 → 进入 0.3；连接失败 → 界面提示 + 允许重试或重新选择 |

### 0.3 数据接入阶段

| 属性 | 说明 |
|------|------|
| 触发条件 | 摄像头连接成功 |
| 前置状态 | 相机已就绪 |
| 行为 | ① 同步接收主控板经串口（S21C，波特率 115200）上报的 4 路激光测距数据（11 字节帧协议见 §3.4 / §5.1）<br>② 显示实时 RGB 画面与激光参数供用户校对<br>③ 后台启动健康检查线程监测检测节点 存活性（心跳机制见 §7） |
| 状态流转 | 数据正常 → 进入 0.4 待机；激光全无效 → 界面提示"激光测距无效"，等待恢复；A 离线 → 降级模式 |

### 0.4 识别工作阶段（九步核心循环）

| 属性 | 说明 |
|------|------|
| 触发条件 | 用户点击主界面"识别"按钮（按需单帧触发，非逐帧连续） |
| 前置状态 | 摄像头与激光数据就绪、推理服务在线（非降级）、副页面均已关闭 |
| 行为 | 见下方九步核心循环 |

**九步核心循环**：

```
① 用户点击"识别"按钮
        ↓
② 冻结主画面（停止实时刷新）+ 按钮转"加载中"状态（详见 §3.7.6 状态机）
        ↓
③ 锁定帧：保证 RGB 原图 ↔ 拍摄距离 ↔ 类别掩码 三者一一对应
   - RGB：当前冻结帧
   - 拍摄距离：由用户在设置项中选取的测距方式确定（取指定路/均值/最大值/最小值/手动输入，见 §3.7.5）
   - 类别掩码：待推理返回
        ↓
④ 发送 RGB 帧至检测节点 推理取类别掩码（gRPC Infer，S→D，详见 §3.5）
   - frame_id / timestamp_ms 由服务节点 生成，检测节点 原样回传，服务节点 校验一致性（帧同步见 §4.4）
        ↓
⑤ 综合三者计算最终结果（RGB + 拍摄距离 + 类别掩码 → RebarMeasure，详见 §3.6）
   - mm/px = 拍摄距离 ÷ fx
   - 逐根提取钢筋直径 + 就近对齐国标规格 + 相邻间距
        ↓
⑥ 按用户设置的保存方式输出结果（详见结果存储规范）
        ↓
⑦ 主画面显示可视化结果图（叠加掩码 + 直径/间距标注，见 §3.7.7）
        ↓
⑧ 按钮由"加载中"转"准备就绪"
        ↓
⑨ 再次点击识别按钮 → 解除冻结，回到实时画面（返回 ①）
```

> **拍摄距离来源说明**：当前阶段统一使用激光单点测距（AGENTS.md §5.3 已定稿）。4 路测距值的融合策略由用户在 GUI 设置副页面中选取（五选一，见 §3.7.5），不使用 336L 的 Depth 深度图参与测量换算。

### 0.5 退出阶段

服务节点 退出分两种路径，**统一行为**为：检查检测节点 在线状态 → 若在线则经 gRPC 向检测节点 发送退出指令（S→D，暂记 `RequestPeerShutdown`，**待 AGENTS.md proto 扩展确认**）→ 在时限内等反馈 → 收到正常退出反馈则正常退出；超时仍未收到则仍退出，但告警检测节点 异常并写入日志。

| 退出方式 | 触发条件 | 行为特征 |
|----------|----------|----------|
| **GUI 退出（正常）** | 用户点击 GUI 关闭按钮 / 退出菜单 | 正常流程：保存状态 → 通知 A → 等反馈 → 释放资源 → 退出 |
| **CLI 退出（紧急）** | 用户在终端执行 CLI 退出命令（见 §3.10） / 收到 `SIGTERM`/`SIGINT` | 紧急流程：跳过部分状态保存 → 通知 A（best-effort）→ 释放资源 → 立即退出 |

> **S→D 退出通知说明**：该 RPC 暂记为 `RequestPeerShutdown`（与现有 D→S `Shutdown` 语义区分，避免方向歧义），**标注为"待 AGENTS.md proto 扩展确认"**。详细双退出流程（含 deadline、超时告警落盘）由后续章节补充。检测节点 自身的退出行为以 `AGENTS.md` §5.7 为准。
>
> **异常退出兜底**：若 B 异常退出导致退出指令未发出，检测节点 通过心跳超时（默认连续丢失 3 次 / 15 秒）自行感知并降级（见 §7）。

### 0.6 总体流程图

```
[启动 root/sudo]
       │
       ▼
[① 加载 JSON 配置] ──失败──→ 退出
       │
       ▼
[② 环境权限检查] ──硬故障──→ 退出
       │              └─软故障(A不可达)──→ 降级模式
       ▼
[摄像头初始化: 扫描+白名单+网络摄像头手动添加]
       │
       ▼
[用户选择并连接摄像头] ──失败──→ 重试
       │
       ▼
[数据接入: 4路激光(S21C串口) + 实时RGB校对]
       │
       ▼
[识别工作九步循环] ←─────────────┐
   (冻结→锁定帧→推理→测量→保存→可视化→就绪)
       │                          │
       ▼                          │ 再次点击
[退出: 检查D→S→A退出指令→等反馈→退出] ─┘
```

---

## 1. 节点职责与运行环境

### 1.1 节点角色定位

服务节点 是系统的**主控与交互枢纽**，承担以下四项核心职责：

| 编号 | 职责 | 输入 | 输出 |
|------|------|------|------|
| R1 | 深度相机 RGB 帧采集 | Orbbec 336L USB 视频流 | JPEG 编码帧（供推理） |
| R2 | 激光测距数据采集 | S21C 主控串口 11 字节帧 | 4 路 uint16 距离值（mm） |
| R3 | 推理任务调度与帧同步 | RGB 帧 + 距离标量 + 用户触发 | gRPC 请求；InferResponse 解析 |
| R4 | 测量换算 + 画面渲染 + 用户交互 | 掩码 + 距离 + 内参 | 标注画面、直径/间距数据、交互反馈 |

> **职责扩展声明**：除上述四项核心职责外，服务节点 同时具备以下能力——
> - **GUI 与 CLI 双交互能力**：GUI（tkinter）为正常运维主入口，CLI（`argparse`，见 §3.10）为紧急备用手段，二者共享同一份运行时状态（CLI 为备用，正常运维优先使用 GUI）。
> - **本地 USB 与网络摄像头 TCP 双接入**：相机层以 `BaseCameraInput` 抽象（§3.3.1）统一两种接入方式，`Orbbec336LInput`（USB 直连）为主、`NetworkCameraInput`（TCP 网络摄像头，见 §3.3.4）为辅，上层逻辑无需感知接入差异。
> - **识别结果存储职责**：按 §8 识别结果存储规范将每次推理结果落盘至 `./result/{YYYYMMDD-HHMMSS}/`，包含可视化结果图、CSV 明细表、类别掩码图。
> - **系统服务管理职责**：经 `scripts/service-manager.sh`（§9）提供开机自启注册/卸载能力，并经 GUI 设置页与 CLI 命令向用户暴露同等能力。

### 1.2 运行环境约束

| 维度 | 说明 |
|------|------|
| 硬件 | HUAWEI OrangePi KunPeng（ARM Cortex-A 系列，无 GPU） |
| 操作系统 | openEuler 22 LTS |
| Python | 3.9+（PEP 8，Type Hints） |
| 关键依赖 | `opencv-python`、`pyserial`、`Pillow`、`grpcio`、`protobuf`、`numpy` |
| 显示 | 本地 GUI（tkinter）；需 `DISPLAY` 环境变量或 XWayland |
| 桌面环境 | 服务端保证所部署系统带 GUI 桌面环境（openEuler 桌面版或 XWayland），确保 tkinter 正常渲染；纯命令行环境仅 CLI 可用、GUI 不可用 |
| 权限 | 摄像头需 `video` 组；串口需 `dialout` 组；启动需 `root`/`sudo`（注册/卸载系统服务、绑定特权端口需 root，见 §9.3 / §0.1） |
| 网络 | 静态 IP（如 `192.168.10.1`），经网线直连检测节点 |

### 1.3 平台兼容声明

- **生产环境**：openEuler 22 LTS，所有功能完整启用
- **开发调试环境**：Windows 10/11，仅负责代码开发与调试
  - 摄像头后端使用 DirectShow（`cv2.CAP_DSHOW`）
  - 串口使用 CH9102 驱动虚拟串口
  - 环境自检中 IP 校验 bypass 或仅warning
  - 检测节点 不可达时进入降级模式而非退出

### 1.4 触发模式约束（配置化）

推理触发为**手动按需触发**：用户在 UI 点击"测量"后执行单帧推理。

**推理请求的最小时间间隔由配置文件 `config/inference.json` 的 `inference_interval_seconds` 字段控制**，默认值为 **3.0 秒**。服务节点 的 UI/调度模块在收到用户触发后，先读取该配置值进行时间戳门控：若距上次成功触发的时间不足配置值，则弹提示并阻止请求发送到检测节点。

> **控制权归属（已定稿）**：间隔控制在**服务节点 侧执行**，检测节点 侧不做时间限制。部署时可根据 Jetson Nano 的具体性能余量，通过修改 `inference_interval_seconds`（如 2.0、3.0、5.0 秒）快速调整，无需重烧系统或修改代码。

---

## 2. 工程目录结构

```
node_server/
├── main.py                          # 程序入口
├── config/
│   ├── camera.json                  # 相机参数配置
│   ├── intrinsics.json              # 相机内参配置
│   └── network.json                 # 网络参数配置
├── proto/
│   ├── rebar_inference.proto        # gRPC Proto 定义
│   ├── rebar_inference_pb2.py       # protoc 生成
│   └── rebar_inference_pb2_grpc.py  # protoc 生成
├── common/
│   ├── __init__.py
│   ├── constants.py                 # 共享常量集中定义
│   └── config_loader.py             # 配置加载器
├── camera/
│   ├── __init__.py
│   ├── base.py                      # BaseCameraInput 抽象基类
│   ├── orbbec_336l.py               # Orbbec336LInput 实现（USB 直连）
│   ├── network_camera.py            # NetworkCameraInput 实现（TCP 网络摄像头）
│   ├── scanner.py                   # LocalCameraScanner 本地摄像头扫描 + 白名单
│   └── mock.py                      # MockCameraInput（调试用）
├── serial/
│   ├── __init__.py
│   ├── serial_buffer.py             # SerialBuffer
│   ├── laser_parser.py              # LaserDataParser + 有效性检查
│   ├── laser_fusion.py              # 4路融合策略
│   └── serial_manager.py            # SerialManager
├── grpc_client/
│   ├── __init__.py
│   └── inference_client.py          # InferenceGrpcClient
├── measure/
│   ├── __init__.py
│   ├── rebar_measure.py             # RebarMeasure 类（源自 RebarMeasureV53）
│   └── intrinsics_manager.py        # 内参管理
├── ui/
│   ├── __init__.py
│   ├── app.py                       # RebarMeasureApp（主应用）
│   ├── cli_fusion_menu.py           # 激光融合策略 CLI 菜单
│   └── render.py                    # 画面渲染与叠加
├── system/
│   ├── __init__.py
│   ├── self_check.py                # 启动环境自检
│   ├── node_monitor.py              # 运行时节点状态监测（被动接收心跳 + 超时判定）
│   ├── service_manager.py           # 系统服务管理脚本调用入口（封装 service-manager.sh）
│   └── bootstrap.py                 # 启动引导
├── logs/
│   └── node_server.log                   # 服务节点 运行日志
├── scripts/
│   └── service-manager.sh           # 系统服务管理脚本（install/uninstall 开机自启）
├── result/                          # 识别结果输出目录（运行时自适应创建，按时间戳子目录组织）
├── requirements.txt                 # 服务节点 专属依赖
└── test_node_server.py                   # 诊断脚本
```

**设计理由**：
- 按 §3.1 模块划分对齐目录
- `proto/` 存放 protobuf 定义与生成代码，**双节点禁止各自硬编码副本**（检测节点 同样引用此目录或独立构建，但 proto 源文件唯一）
- `common/` 存放双节点共享的常量与配置加载逻辑，可被检测节点 引用（如 proto 定义、分割类别常量）
- `requirements.txt` 按节点拆分，避免在 OrangePi 上安装 torch 等推理依赖

---

## 3. 核心模块与类设计

### 3.1 共享常量（`common/constants.py`）

```python
"""
共享常量集中定义，双节点禁止各自硬编码副本。
"""

# ---- 串口帧协议 ----
SERIAL_FRAME_HEADER: int = 0x7B
SERIAL_FRAME_TAIL: int = 0x7D
SERIAL_FRAME_LENGTH: int = 11
SERIAL_BAUDRATE: int = 115200
SERIAL_CHANNELS: int = 4

# ---- 激光测距有效性范围（STP-23L Datasheet §4.1） ----
LASER_MIN_MM: int = 70
LASER_MAX_MM: int = 7500

# ---- 分割类别定义 ----
class SegClass:
    BACKGROUND: int = 0
    REBAR_VERTICAL: int = 1   # 纵向钢筋
    REBAR_HORIZONTAL: int = 2 # 横向钢筋

# ---- 国标规格表（mm） ----
STANDARD_DIAMETERS = [6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40]

# ---- 推理节奏（已迁移至 config/inference.json，不再作为编译期常量） ----
# 服务节点 侧通过 `config/inference.json` 的 `inference_interval_seconds` 字段控制。
# 默认值 3.0 秒；部署时可调整，无需改代码。

# ---- 内参默认值 ----
DEFAULT_FX: float = 950.0
DEFAULT_FY: float = 950.0
```

### 3.2 配置加载器（`common/config_loader.py`）

```python
"""
从 JSON 配置文件读取所有值类参数，严禁硬编码。
"""

import json
import os
from dataclasses import dataclass, field
from typing import Optional

@dataclass
class CameraConfig:
    rgb_width: int = 1920
    rgb_height: int = 1080
    rgb_fps: int = 30
    rgb_format: str = "MJPG"
    depth_width: int = 640
    depth_height: int = 480
    depth_fps: int = 30
    device_index: int = 0  # /dev/video* 或 Windows DirectShow 索引

@dataclass
class IntrinsicsConfig:
    fx: float = 950.0
    fy: float = 950.0
    cx: float = 960.0
    cy: float = 540.0

@dataclass
class NetworkConfig:
    local_ip: str = "192.168.10.1"
    remote_ip: str = "192.168.10.2"
    grpc_port: int = 50051
    grpc_deadline_ms: int = 5000  # gRPC 单次调用 deadline
    health_check_interval_s: float = 5.0

@dataclass
class InferenceConfig:
    """推理节奏控制参数。"""
    inference_interval_seconds: float = 3.0  # 相邻两次推理请求的最小间隔

class ConfigLoader:
    """读取并解析 JSON 配置文件。"""

    @staticmethod
    def load_camera_config(path: str) -> CameraConfig:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return CameraConfig(**data)

    @staticmethod
    def load_intrinsics_config(path: str) -> IntrinsicsConfig:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return IntrinsicsConfig(**data)

    @staticmethod
    def load_network_config(path: str) -> NetworkConfig:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return NetworkConfig(**data)

    @staticmethod
    def load_inference_config(path: str) -> InferenceConfig:
        """加载推理节奏配置（config/inference.json）。"""
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return InferenceConfig(**data)
```

**配置文件示例**：

`config/camera.json`:
```json
{
  "rgb_width": 1920,
  "rgb_height": 1080,
  "rgb_fps": 30,
  "rgb_format": "MJPG",
  "depth_width": 640,
  "depth_height": 480,
  "depth_fps": 30,
  "device_index": 0
}
```

`config/intrinsics.json`:
```json
{
  "fx": 950.0,
  "fy": 950.0,
  "cx": 960.0,
  "cy": 540.0
}
```

`config/network.json`:
```json
{
  "local_ip": "192.168.10.1",
  "remote_ip": "192.168.10.2",
  "grpc_port": 50051,
  "grpc_deadline_ms": 5000,
  "health_check_interval_s": 5.0
}
```

`config/inference.json`:
```json
{
  "inference_interval_seconds": 3.0
}
```

> 上表说明：`inference_interval_seconds` 控制服务节点 侧相邻两次推理请求的最小时间间隔。默认 3.0 秒是在 Jetson Nano 开发机上验证的保守余量，部署时可依据实测性能下调（如 2.0 秒）或上调（如 5.0 秒），修改后无需重启编译、无需改代码，只需重启服务节点 的主应用。

### 3.3 输入模块（`camera/`）

#### 3.3.1 抽象基类

```python
"""camera/base.py"""

from abc import ABC, abstractmethod
from typing import Optional, Dict
import numpy as np


class BaseCameraInput(ABC):
    """
    相机输入抽象基类。
    所有具体实现（Orbbec 336L、Mock 等）必须继承此类。
    """

    @abstractmethod
    def open(self) -> bool:
        """打开设备，返回是否成功。"""
        ...

    @abstractmethod
    def close(self) -> None:
        """关闭设备，释放资源。"""
        ...

    @abstractmethod
    def is_opened(self) -> bool:
        """设备是否已就绪。"""
        ...

    @abstractmethod
    def get_rgb_frame(self) -> Optional[np.ndarray]:
        """
        获取一帧 RGB 图像。
        返回: BGR 格式 numpy 数组 (H, W, 3)，失败返回 None。
        """
        ...

    @abstractmethod
    def get_depth_frame(self) -> Optional[np.ndarray]:
        """
        获取一帧 Depth 深度图。
        原型阶段可不实现，返回 None。
        """
        ...

    @abstractmethod
    def get_intrinsics(self) -> Dict[str, float]:
        """
        获取相机内参。
        返回: {"fx": float, "fy": float, "cx": float, "cy": float}
        """
        ...
```

#### 3.3.2 Orbbec 336L 实现

```python
"""camera/orbbec_336l.py"""

import threading
import time
from typing import Optional, Dict

import cv2
import numpy as np

from .base import BaseCameraInput
from common.config_loader import CameraConfig, IntrinsicsConfig


class Orbbec336LInput(BaseCameraInput):
    """
    Orbbec Gemini 336L 相机实现。
    后台线程持续读帧，取最新帧策略；断线自动重连。
    """

    def __init__(self, camera_cfg: CameraConfig, intrinsics_cfg: IntrinsicsConfig):
        self._camera_cfg = camera_cfg
        self._intrinsics_cfg = intrinsics_cfg
        self._cap: Optional[cv2.VideoCapture] = None
        self._is_opened: bool = False
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        self._read_thread: Optional[threading.Thread] = None
        self._is_running: bool = False
        self._reconnect_interval: float = 3.0
        self._max_reconnect_attempts: int = 5
        self._reconnect_attempts: int = 0

    def open(self) -> bool:
        """打开设备并启动后台读帧线程。"""
        # 伪代码：
        # 根据 platform 选择后端: Windows -> CAP_DSHOW; Linux -> CAP_V4L2
        # self._cap = cv2.VideoCapture(self._camera_cfg.device_index, backend)
        # self._cap.set(CAP_PROP_FRAME_WIDTH, cfg.rgb_width)
        # self._cap.set(CAP_PROP_FRAME_HEIGHT, cfg.rgb_height)
        # self._cap.set(CAP_PROP_FPS, cfg.rgb_fps)
        # self._cap.set(CAP_PROP_BUFFERSIZE, 1)
        # if not self._cap.isOpened(): return False
        # self._is_opened = True; self._is_running = True
        # self._read_thread = Thread(target=self._read_loop, daemon=True)
        # self._read_thread.start()
        return True

    def close(self) -> None:
        """关闭设备并停止线程。"""
        # self._is_running = False
        # if self._read_thread: self._read_thread.join(timeout=2.0)
        # if self._cap: self._cap.release()
        # self._is_opened = False
        ...

    def is_opened(self) -> bool:
        return self._is_opened

    def get_rgb_frame(self) -> Optional[np.ndarray]:
        """获取最新帧的拷贝（线程安全）。"""
        # with self._frame_lock:
        #     return self._latest_frame.copy() if self._latest_frame is not None else None
        return None

    def get_depth_frame(self) -> Optional[np.ndarray]:
        """原型阶段不实现，返回 None。"""
        return None

    def get_intrinsics(self) -> Dict[str, float]:
        """从配置返回内参。"""
        # return {"fx": self._intrinsics_cfg.fx, "fy": self._intrinsics_cfg.fy,
        #         "cx": self._intrinsics_cfg.cx, "cy": self._intrinsics_cfg.cy}
        return {}

    def try_reconnect(self) -> bool:
        """断线重连。"""
        # if self._reconnect_attempts >= self._max_reconnect_attempts: return False
        # self._reconnect_attempts += 1
        # time.sleep(self._reconnect_interval)
        # return self.open()
        return False

    def _read_loop(self):
        """后台线程：持续读取帧，更新 _latest_frame。"""
        # while self._is_running:
        #     if self._cap and self._cap.isOpened():
        #         ret, frame = self._cap.read()
        #         if ret:
        #             with self._frame_lock:
        #                 self._latest_frame = frame
        #             self._reconnect_attempts = 0
        #         time.sleep(0.001)
        #     else:
        #         self._is_opened = False
        #         break
        ...
```

#### 3.3.3 Mock 相机（调试用）

```python
"""camera/mock.py"""

import numpy as np
from typing import Optional, Dict
from .base import BaseCameraInput


class MockCameraInput(BaseCameraInput):
    """模拟相机，返回纯黑测试帧，服务节点 独立联调用。"""

    def __init__(self, width: int = 1920, height: int = 1080):
        self._width = width
        self._height = height
        self._is_opened = False

    def open(self) -> bool:
        self._is_opened = True
        return True

    def close(self) -> None:
        self._is_opened = False

    def is_opened(self) -> bool:
        return self._is_opened

    def get_rgb_frame(self) -> Optional[np.ndarray]:
        return np.zeros((self._height, self._width, 3), dtype=np.uint8)

    def get_depth_frame(self) -> Optional[np.ndarray]:
        return None

    def get_intrinsics(self) -> Dict[str, float]:
        return {"fx": 950.0, "fy": 950.0, "cx": self._width / 2, "cy": self._height / 2}
```

#### 3.3.4 网络摄像头实现（`NetworkCameraInput`）

`NetworkCameraInput` 是基于 TCP 视频流协议的相机输入实现，作为 `Orbbec336LInput`（USB 直连）之外的补充接入方式。通信模型为客户端（服务节点）主动 TCP 连接服务器（网络摄像头端），被动接收推流。完整协议细节参见项目内文档 `orbbec-336L/orbbec-336L-win-protocol.md`。

> **协议要点**（摘自 `orbbec-336L/orbbec-336L-win-protocol.md`）：
> - 应用层握手：客户端发 `HELLO\n` → 服务器回 `OK <port>\n` → 开始推流。服务器 accept 后 5 秒内未收 `HELLO` 则关闭连接。
> - 帧格式：28 字节定长帧头 + 变长 JPEG。帧头 struct 格式 `'<I B B H Q I H H I'`（小端序），字段：magic=`0x4F524242`("ORBB") uint32、version uint8、flags uint8、reserved uint16、timestamp_ms uint64、frame_index uint32、width uint16、height uint16、jpeg_size uint32。
> - 粘包/半包处理：读帧需用 `recv_exactly` 精确读取；magic 不匹配时逐字节扫描重新对齐。

```python
"""camera/network_camera.py"""

import socket
import struct
import threading
import time
from typing import Optional, Dict

import cv2
import numpy as np

from .base import BaseCameraInput
from common.config_loader import IntrinsicsConfig


# 28 字节帧头 struct 格式（小端序）
# '<I B B H Q I H H I' → 4+1+1+2+8+4+2+2+4 = 28 字节
_FRAME_HEADER_FMT = "<IBBHQIHHI"
_FRAME_HEADER_SIZE = 28
_MAGIC_ORBB = 0x4F524242  # "ORBB"
_HELLO = b"HELLO\n"
_HANDSHAKE_TIMEOUT = 5.0  # 服务器 accept 后 5 秒内未收 HELLO 则关闭


class NetworkCameraInput(BaseCameraInput):
    """
    网络摄像头输入（TCP 客户端）。
    主动连接网络摄像头端，握手后被动接收 JPEG 视频推流。
    默认服务器地址 192.168.1.100:8080，允许 GUI 手动指定 host/port。
    """

    def __init__(
        self,
        intrinsics_cfg: IntrinsicsConfig,
        host: str = "192.168.1.100",
        port: int = 8080,
    ):
        self._host = host
        self._port = port
        self._intrinsics_cfg = intrinsics_cfg
        self._sock: Optional[socket.socket] = None
        self._is_opened: bool = False
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        self._read_thread: Optional[threading.Thread] = None
        self._is_running: bool = False
        self._reconnect_interval: float = 3.0
        self._max_reconnect_attempts: int = 5
        self._reconnect_attempts: int = 0

    def open(self) -> bool:
        """建立 TCP 连接 + 应用层握手 + 启动后台读帧线程。"""
        # 伪代码：
        # self._sock = socket.create_connection((self._host, self._port), timeout=5.0)
        # self._sock.sendall(_HELLO)                      # 发送 HELLO
        # resp = self._recv_exactly(len(b"OK \n") + ...)  # 等待 "OK <port>\n"
        # if not resp.startswith(b"OK"): return False
        # self._is_opened = True; self._is_running = True
        # self._read_thread = Thread(target=self._read_loop, daemon=True)
        # self._read_thread.start()
        return True

    def close(self) -> None:
        # self._is_running = False
        # if self._read_thread: self._read_thread.join(timeout=2.0)
        # if self._sock: self._sock.close()
        # self._is_opened = False
        ...

    def is_opened(self) -> bool:
        return self._is_opened

    def get_rgb_frame(self) -> Optional[np.ndarray]:
        """获取最新帧的拷贝（线程安全）。"""
        # with self._frame_lock:
        #     return self._latest_frame.copy() if self._latest_frame is not None else None
        return None

    def get_depth_frame(self) -> Optional[np.ndarray]:
        """原型阶段不实现，返回 None。"""
        return None

    def get_intrinsics(self) -> Dict[str, float]:
        """从配置返回内参。"""
        # return {"fx": self._intrinsics_cfg.fx, "fy": self._intrinsics_cfg.fy,
        #         "cx": self._intrinsics_cfg.cx, "cy": self._intrinsics_cfg.cy}
        return {}

    def _recv_exactly(self, n: int) -> Optional[bytes]:
        """精确读取 n 字节，解决粘包/半包。失败返回 None。"""
        # buf = bytearray()
        # while len(buf) < n:
        #     chunk = self._sock.recv(n - len(buf))
        #     if not chunk: return None
        #     buf.extend(chunk)
        # return bytes(buf)
        return None

    def _read_loop(self) -> None:
        """后台线程：读帧头 → 读 JPEG → 解码 → 更新 _latest_frame。"""
        # while self._is_running:
        #     header = self._recv_exactly(_FRAME_HEADER_SIZE)
        #     if header is None: break
        #     (magic, ver, flags, reserved, ts_ms, fidx, w, h, jpg_sz) = \
        #         struct.unpack(_FRAME_HEADER_FMT, header)
        #     if magic != _MAGIC_ORBB:
        #         # magic 不匹配：丢弃首字节，逐字节扫描重新对齐
        #         continue
        #     jpeg = self._recv_exactly(jpg_sz)
        #     if jpeg is None: break
        #     frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        #     if frame is not None:
        #         with self._frame_lock:
        #             self._latest_frame = frame
        #         self._reconnect_attempts = 0
        ...
```

> **设计说明**：
> - `NetworkCameraInput` 与 `Orbbec336LInput` 并列，均为 `BaseCameraInput` 的具体实现；上层逻辑（主循环、渲染）无需感知接入方式差异。
> - magic 逐字节对齐：当 `_recv_exactly` 读到的帧头 magic 字段不为 `0x4F524242` 时，丢弃缓冲首字节继续扫描，直到重新对齐到有效帧头。
> - 断线重连：TCP 连接中断后按 `_reconnect_interval`（默认 3 秒）重试，超过 `_max_reconnect_attempts`（默认 5 次）则标记设备离线。
> - 默认参数 `192.168.1.100:8080` 来自 `orbbec-336L/orbbec-336L-win-protocol.md`，构造参数允许 GUI 手动指定 host/port。

#### 3.3.5 本地摄像头自动扫描与兼容白名单

本地摄像头（USB 直连，如 Orbbec 336L）在程序启动与摄像头选择阶段需自动扫描枚举，并按配置白名单进行兼容性标注。白名单外型号告警但**不阻塞**连接（允许用户自行选用）。

```python
"""camera/scanner.py"""

from dataclasses import dataclass
from typing import List
import sys


@dataclass
class CameraDeviceInfo:
    """扫描到的摄像头设备信息。"""
    index: int                  # 设备索引（/dev/video* 序号 或 DirectShow 索引）
    name: str                   # 设备名称
    is_whitelisted: bool        # 是否在兼容白名单内
    source: str = "local"       # "local"=本地USB / "network"=网络摄像头


class LocalCameraScanner:
    """
    本地摄像头自动扫描器。
    Linux 枚举 /dev/video*；Windows 枚举 DirectShow 索引。
    """

    def __init__(self, compatible_models: List[str]):
        self._compatible_models = compatible_models  # 来自 config/camera.json

    def scan(self, max_index: int = 10) -> List[CameraDeviceInfo]:
        """
        扫描本地摄像头，返回可用设备列表。
        伪代码：
        # if sys.platform.startswith("linux"):
        #     枚举 /dev/video0..N
        # else:  # Windows
        #     尝试 cv2.VideoCapture(i, CAP_DSHOW) for i in range(max_index)
        # for each 可用设备:
        #     is_whitelisted = any(m in name for m in self._compatible_models)
        #     if not is_whitelisted: log WARNING("型号不在白名单: %s" % name)
        #     results.append(CameraDeviceInfo(index, name, is_whitelisted))
        """
        return []
```

**兼容白名单配置**（`config/camera.json` 扩展字段）：

```json
{
  "rgb_width": 1920,
  "rgb_height": 1080,
  "rgb_fps": 30,
  "rgb_format": "MJPG",
  "device_index": 0,
  "compatible_models": ["Orbbec Gemini 336L", "Orbbec Astra"]
}
```

| 字段 | 说明 |
|------|------|
| `compatible_models` | 兼容型号白名单（字符串列表，按设备名称模糊匹配）。白名单外型号触发 WARNING 日志但允许用户选用 |

> **白名单语义**：白名单用于兼容性提示而非硬性限制。原型阶段硬件适配以"能采集 RGB 即可用"为原则，避免因型号匹配误判阻断可用设备；未在白名单的设备在 GUI 列表中标注"未验证"，由用户自行决定是否选用。

#### 3.3.6 摄像头选择 GUI 状态

摄像头选择界面是主界面在连接前的状态（详见 §3.7.4 主界面两状态），对应 §0.2 流程。该界面提供扫描结果列表与手动添加网络摄像头入口。

| 界面元素 | 说明 |
|----------|------|
| 扫描结果列表 | 显示 `LocalCameraScanner.scan()` 结果：每项含索引、名称、白名单标记（未验证设备标注"未验证"） |
| 手动添加网络摄像头入口 | IP / 端口输入框（默认填充 `192.168.1.100` / `8080`）+ "添加"按钮；添加后以 `NetworkCameraInput` 实例化并加入列表 |
| 选择连接按钮 | 选中列表项后点击"连接"；后台调用对应 `BaseCameraInput.open()`，成功后切到识别工作状态 |
| 刷新扫描按钮 | 重新触发本地摄像头扫描，更新列表 |

```
┌─────────────────────────────────────────────┐
│  摄像头选择                                   │
├─────────────────────────────────────────────┤
│ [刷新扫描]                                    │
│ ┌─────────────────────────────────────────┐ │
│ │ 0  Orbbec Gemini 336L        [已验证]   │ │
│ │ 1  USB Camera (HD)          [未验证]   │ │
│ │ 2  NetworkCamera 192.168.1.100:8080     │ │
│ └─────────────────────────────────────────┘ │
│                                              │
│ 手动添加网络摄像头:                           │
│   IP [192.168.1.100] 端口 [8080] [添加]      │
│                                              │
│                          [选择并连接]         │
└─────────────────────────────────────────────┘
```

> **与流程的对应**：本界面在 §0.2 摄像头初始化阶段呈现；连接成功后主界面切换至识别工作状态（§3.7.4），本界面隐藏。连接失败时停留在本界面并提示错误，允许重试或重新选择。

### 3.4 串口模块（`serial/`）

#### 3.4.1 串口缓冲区（源自现有 `SerialBuffer`）

```python
"""serial/serial_buffer.py"""

import threading
from typing import Optional

from common.constants import (
    SERIAL_FRAME_HEADER, SERIAL_FRAME_TAIL, SERIAL_FRAME_LENGTH
)


class SerialBuffer:
    """
    串口数据缓冲区，处理粘包和断包问题。
    与现有 demo.py 实现完全一致。
    """

    def __init__(self):
        self._buffer = bytearray()
        self._lock = threading.Lock()

    def append(self, data: bytes) -> None:
        """追加原始字节到缓冲区。"""
        with self._lock:
            self._buffer.extend(data)

    def find_frame(self) -> Optional[bytes]:
        """
        从缓冲区中提取一个完整且校验通过的帧（11 字节）。
        返回: 完整帧 bytes，或 None（数据不足 / 无帧）。
        """
        with self._lock:
            while len(self._buffer) >= SERIAL_FRAME_LENGTH:
                # 寻找帧头
                header_index = -1
                for i in range(len(self._buffer) - SERIAL_FRAME_LENGTH + 1):
                    if self._buffer[i] == SERIAL_FRAME_HEADER:
                        header_index = i
                        break

                if header_index == -1:
                    self._buffer.clear()
                    return None

                if header_index > 0:
                    del self._buffer[:header_index]

                if len(self._buffer) < SERIAL_FRAME_LENGTH:
                    return None

                candidate = bytes(self._buffer[:SERIAL_FRAME_LENGTH])

                if candidate[-1] != SERIAL_FRAME_TAIL:
                    del self._buffer[0]
                    continue

                if self._verify_checksum(candidate):
                    del self._buffer[:SERIAL_FRAME_LENGTH]
                    return candidate
                else:
                    del self._buffer[0]

            return None

    @staticmethod
    def _verify_checksum(frame: bytes) -> bool:
        """XOR 校验：前 9 字节异或和等于第 10 字节。"""
        xor_sum = 0
        for i in range(9):
            xor_sum ^= frame[i]
        return xor_sum == frame[9]
```

#### 3.4.2 激光数据解析器 + 有效性检查（源自现有 `LaserDataParser`）

```python
"""serial/laser_parser.py"""

import struct
from dataclasses import dataclass
from typing import List, Tuple

from common.constants import (
    LASER_MIN_MM, LASER_MAX_MM
)


@dataclass
class LaserParseResult:
    """单帧激光数据解析结果。"""
    raw_values: Tuple[int, int, int, int]   # 4 路原始值
    valid_values: List[int]                  # 通过有效性检查的值
    valid_count: int                         # 有效路数


class LaserDataParser:
    """激光测距数据解析器，含有效性检查。"""

    @staticmethod
    def parse(frame: bytes) -> LaserParseResult:
        """
        解析 11 字节帧：4 路大端序 uint16。
        返回 LaserParseResult（包含原始值与有效值列表）。
        """
        # 伪代码：
        # d1 = struct.unpack('>H', frame[1:3])[0]
        # d2 = struct.unpack('>H', frame[3:5])[0]
        # d3 = struct.unpack('>H', frame[5:7])[0]
        # d4 = struct.unpack('>H', frame[7:9])[0]
        # raw = (d1, d2, d3, d4)
        # valid = [v for v in raw if _is_valid(v)]
        # return LaserParseResult(raw, valid, len(valid))
        return LaserParseResult((0, 0, 0, 0), [], 0)

    @staticmethod
    def _is_valid(value: int) -> bool:
        """单值有效性检查。"""
        # if value == 0: return False          # 零值剔除
        # if value < LASER_MIN_MM: return False
        # if value > LASER_MAX_MM: return False
        # return True
        return False
```

#### 3.4.3 4 路融合策略

```python
"""serial/laser_fusion.py"""

import statistics
from enum import Enum
from typing import Optional

from serial.laser_parser import LaserParseResult
from common.constants import SERIAL_CHANNELS


class FusionStrategy(Enum):
    """激光融合策略枚举。"""
    MEAN = 1       # 取均值（默认）
    MEDIAN = 2     # 取中位数
    SPECIFIC = 3   # 取特定一路
    MANUAL = 4     # 手动输入


class LaserFusion:
    """
    4 路激光测距值融合为 1 个标量距离。
    支持运行时策略切换。
    """

    def __init__(self):
        self._strategy: FusionStrategy = FusionStrategy.MEAN
        self._specific_index: int = 0     # 3 路选择时：0~3
        self._manual_value: Optional[float] = None

    def set_strategy(self, strategy: FusionStrategy,
                     specific_index: int = 0,
                     manual_value: Optional[float] = None) -> None:
        """运行时切换策略。"""
        self._strategy = strategy
        self._specific_index = specific_index
        self._manual_value = manual_value

    def get_strategy(self) -> FusionStrategy:
        return self._strategy

    def fuse(self, parse_result: LaserParseResult) -> Optional[float]:
        """
        根据当前策略融合 4 路距离值为 1 个标量。
        返回: 融合结果（mm），全无效返回 None。
        """
        # 伪代码：
        # if self._strategy == FusionStrategy.MANUAL:
        #     return self._manual_value
        #
        # valid = parse_result.valid_values
        # if len(valid) == 0:
        #     return None
        #
        # if self._strategy == FusionStrategy.MEAN:
        #     return sum(valid) / len(valid)
        # elif self._strategy == FusionStrategy.MEDIAN:
        #     return statistics.median(valid)
        # elif self._strategy == FusionStrategy.SPECIFIC:
        #     idx = self._specific_index
        #     raw = parse_result.raw_values
        #     if 0 <= idx < SERIAL_CHANNELS and LaserDataParser._is_valid(raw[idx]):
        #         return float(raw[idx])
        #     return None  # 所选路无效
        return None
```

#### 3.4.4 串口管理器（重构现有 `SerialManager`）

```python
"""serial/serial_manager.py"""

import time
import threading
from typing import Optional, Tuple

import serial
import serial.tools.list_ports

from serial.serial_buffer import SerialBuffer
from serial.laser_parser import LaserDataParser, LaserParseResult
from common.constants import SERIAL_BAUDRATE


class SerialManager:
    """
    串口设备管理类。
    封装设备自动扫描、端口打开、后台读取、帧解析、数据获取与热插拔重连。
    - 热插拔持续扫描：start_scanning() 启动后台 _scan_daemon 线程，
      以 scan_interval_seconds 周期 auto_detect()，发现 S21C 即自动 open_port。
    - 无限重连：断线后自动重新扫描重连（含换 USB 接口导致节点路径变化），
      不再受 max_reconnect_attempts 上限约束（该字段已弃用）。
    """

    def __init__(self, cfg: SerialConfig, timeout: float = 0.1):
        self._cfg = cfg
        self._baudrate = cfg.baudrate
        self._timeout = timeout
        self._serial_conn: Optional[serial.Serial] = None
        self._port_name: str = ""
        self._is_running: bool = False
        self._buffer = SerialBuffer()
        self._data_lock = threading.Lock()
        self._latest_result: Optional[LaserParseResult] = None
        self._data_timestamp: float = 0.0
        self._read_thread: Optional[threading.Thread] = None
        self._scan_thread: Optional[threading.Thread] = None  # 热插拔扫描线程
        self._scan_interval: float = cfg.scan_interval_seconds  # 扫描周期（默认 2.0s）
        self._reconnect_attempts: int = 0
        # 已弃用：max_reconnect_attempts 不再作重连上限（热插拔无限重连），兼容旧配置保留

    def open_port(self, port_name: str) -> Tuple[bool, str]:
        """打开指定串口并启动后台读取线程。"""
        # 伪代码：
        # self._serial_conn = serial.Serial(port=port_name, baudrate=self._baudrate, timeout=self._timeout)
        # self._port_name = port_name
        # self._is_running = True
        # self._buffer = SerialBuffer()
        # self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        # self._read_thread.start()
        # return True, f"成功打开串口 {port_name}"
        return False, "not implemented"

    def close(self) -> None:
        """关闭串口并停止线程。"""
        # self._is_running = False
        # if self._read_thread: self._read_thread.join(timeout=1.0)
        # if self._serial_conn and self._serial_conn.is_open:
        #     self._serial_conn.close()
        ...

    def is_running(self) -> bool:
        return self._is_running

    def get_port_name(self) -> str:
        return self._port_name

    def get_latest(self) -> Tuple[Optional[LaserParseResult], float]:
        """获取最新解析结果与时间戳（线程安全拷贝）。"""
        # with self._data_lock:
        #     return self._latest_result, self._data_timestamp
        return None, 0.0

    def start_scanning(self) -> None:
        """启动热插拔持续扫描线程（幂等）。"""
        # 以 scan_interval_seconds 周期 auto_detect()，发现设备即 open_port；
        # 由 _scan_daemon 单一线程驱动，避免扫描与重连并发 open_port 竞态

    def try_reconnect(self) -> bool:
        """断线重连：重新自动扫描（支持热插拔换接口/路径），无限重试。"""
        # 注意：热插拔模式下不设 max_reconnect_attempts 上限；
        # 断线后 _scan_daemon 持续 auto_detect()，发现设备即自动 open_port
        return False

    def _read_loop(self):
        """后台线程：读取串口 → 缓冲区 → 帧解析 → 更新结果。"""
        # while self._is_running and self._serial_conn:
        #     try:
        #         if self._serial_conn.in_waiting > 0:
        #             data = self._serial_conn.read(self._serial_conn.in_waiting)
        #             self._buffer.append(data)
        #             while True:
        #                 frame = self._buffer.find_frame()
        #                 if frame is None: break
        #                 result = LaserDataParser.parse(frame)
        #                 with self._data_lock:
        #                     self._latest_result = result
        #                     self._data_timestamp = time.time()
        #     except (serial.SerialException, OSError):
        #         self._is_running = False
        #         break
        #     time.sleep(0.001)
        ...
```

### 3.5 gRPC 客户端模块（`grpc_client/inference_client.py`）

```python
"""grpc_client/inference_client.py"""

import time
from typing import Optional

import grpc
import numpy as np

# protoc 生成的桩代码
from proto import rebar_inference_pb2
from proto import rebar_inference_pb2_grpc
from common.config_loader import NetworkConfig


class InferenceResult:
    """推理响应解析结果。"""
    def __init__(self, mask: np.ndarray, frame_id: int, timestamp_ms: int):
        self.mask = mask                  # (H, W) uint8 类别掩码
        self.frame_id: int = frame_id
        self.timestamp_ms: int = timestamp_ms


class InferenceGrpcClient:
    """
    检测节点 推理服务 gRPC 客户端。
    封装 JPEG 编码 → gRPC 请求 → PNG 解码的全链路。
    """

    def __init__(self, net_cfg: NetworkConfig):
        self._target = f"{net_cfg.remote_ip}:{net_cfg.grpc_port}"
        self._deadline_ms = net_cfg.grpc_deadline_ms
        self._channel: Optional[grpc.Channel] = None
        self._stub: Optional[rebar_inference_pb2_grpc.RebarInferenceStub] = None
        self._connected: bool = False

    def connect(self) -> bool:
        """建立 gRPC 信道。"""
        # 伪代码：
        # self._channel = grpc.insecure_channel(self._target)
        # self._stub = rebar_inference_pb2_grpc.RebarInferenceStub(self._channel)
        # self._connected = True
        # return True
        return False

    def is_connected(self) -> bool:
        return self._connected

    def infer(
        self,
        rgb_frame: np.ndarray,
        frame_id: int,
        camera_distance_mm: float,
    ) -> Optional[InferenceResult]:
        """
        发送单帧推理请求。
        参数: rgb_frame=BGR numpy 数组; frame_id=帧序号; distance=激光距离(mm)
        返回: InferenceResult 或 None（超时/失败）

        伪代码：
        1. jpeg_bytes = cv2.imencode('.jpg', rgb_frame).tobytes()
        2. ts_ms = int(time.time() * 1000)
        3. request = InferRequest(image=jpeg_bytes, frame_id=frame_id,
                                   timestamp_ms=ts_ms,
                                   camera_distance_mm=camera_distance_mm)
        4. try:
               response = self._stub.Infer(request, deadline=self._deadline_ms)
           except grpc.RpcError:
               return None
        5. mask = cv2.imdecode(np.frombuffer(response.label_mask, np.uint8), cv2.IMREAD_GRAYSCALE)
        6. return InferenceResult(mask, response.frame_id, response.timestamp_ms)
        """
        return None

    def probe(self, timeout_ms: int = 2000) -> bool:
        """
        探测检测节点 存活性（用于启动自检与健康检查）。
        返回: True 可达 / False 不可达
        """
        # 伪代码：
        # try:
        #     channel = grpc.insecure_channel(self._target)
        #     grpc.channel_ready_future(channel).result(timeout=timeout_ms / 1000)
        #     return True
        # except grpc.FutureTimeoutError:
        #     return False
        return False

    def close(self) -> None:
        """关闭信道。"""
        # if self._channel: self._channel.close()
        ...

    def send_heartbeat_response(self, node_id: str, status: str,
                                infer_count: int = 0,
                                timeout: float = 2.0) -> bool:
        """
        接收检测节点 心跳调用（反向接口：B 收到 A 的 Heartbeat 后返回确认）。

        注：本方法实际上是供测试和监控用途。检测节点 通过独立 grpc channel
        推送心跳时，调用 stub.Heartbeat() 而非本方法。
        保留此方法作为服务节点 侧 Heartbeat RPC 的桩实现。
        """
        return False

    def request_peer_shutdown(
        self, deadline_s: float = 3.0
    ) -> Optional[bool]:
        """
        S→D 退出通知（关键决策 D1，见 §10.3）。

        服务节点 退出时主动请求检测节点 终止服务并反馈。
        - 方向：B → A（与现有 D→S Shutdown 语义不同，二者并存）
        - 暂记 RPC 名：RequestPeerShutdown
        - deadline：默认 3 秒，超时即放弃等待
        - 失败处理：超时/失败仅记 WARNING 日志，不重试、不阻塞 B 退出

        返回:
            True  — A 反馈 accepted（A 已正常终止）
            False — A 反馈未接受
            None  — 超时/调用失败（调用方按 best-effort 退出）

        > **待 AGENTS.md §5.5 proto 扩展确认**：本 RPC 为本设计新增的 S→D
        > 退出通知，AGENTS.md §5.5 现仅定义 D→S `Shutdown`。落地此设计
        > 需后续 spec 修订 AGENTS.md §5.5 proto 增加 S→D RPC，并修订
        > Design-AI_detect.md 消除 §7.4.1/§9.3/§3.10 三处方向矛盾。
        > 引用 spec 问题清单 #1 / #2。
        """
        # 伪代码：
        # try:
        #     request = RequestPeerShutdownRequest(
        #         node_id="node-server",
        #         timestamp_ms=int(time.time() * 1000),
        #         reason="user_exit",
        #     )
        #     response = self._stub.RequestPeerShutdown(
        #         request, deadline=deadline_s
        #     )
        #     return bool(response.accepted)
        # except grpc.RpcError:
        #     return None
        return None
```

### 3.6 测量模块（`measure/`）

#### 3.6.1 内参管理器

```python
"""measure/intrinsics_manager.py"""

from typing import Dict
from common.config_loader import IntrinsicsConfig


class IntrinsicsManager:
    """
    管理相机内参，提供 mm/px 换算所需参数。
    所有内参值从配置文件读取，禁止硬编码。
    """

    def __init__(self, cfg: IntrinsicsConfig):
        self._fx = cfg.fx
        self._fy = cfg.fy
        self._cx = cfg.cx
        self._cy = cfg.cy

    def get_fx(self) -> float:
        return self._fx

    def get_fy(self) -> float:
        return self._fy

    def get_params(self) -> Dict[str, float]:
        return {"fx": self._fx, "fy": self._fy, "cx": self._cx, "cy": self._cy}

    def calc_mm_per_pixel(self, distance_mm: float) -> float:
        """
        根据工作距离与焦距计算 mm/px。
        公式: mm_per_pixel = distance_mm / fx
        """
        # return distance_mm / self._fx
        return 0.0
```

#### 3.6.2 钢筋测量类（源自 `RebarMeasureV53`）

```python
"""measure/rebar_measure.py"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from common.constants import (
    STANDARD_DIAMETERS, SegClass,
    DEFAULT_FX, DEFAULT_FY,
)

# 从 new-predict.py 迁移时的函数别名
_safe_to_bgr = None


@dataclass
class RebarInfo:
    """单根钢筋测量结果。"""
    center_x: float
    center_y: float
    diameter_mm: float
    standard_diameter_mm: float   # 就近对齐国标后的直径
    width_px: float


@dataclass
class MeasureResult:
    """单次测量完整结果。"""
    rebars: List[RebarInfo]               # 钢筋列表
    spacings_mm: List[float]              # 相邻钢筋间距
    mm_per_pixel: float                   # 像素换算系数
    distance_mm: float                    # 所用工作距离
    annotated_frame: Optional[np.ndarray] # 可视化结果帧


class RebarMeasure:
    """
    钢筋直径与间距测量类。
    源自 RebarMeasureV53，适配新架构的接口形态。
    """

    def __init__(
        self,
        fx: float = DEFAULT_FX,
        fy: float = DEFAULT_FY,
        cx: float = 960.0,
        cy: float = 540.0,
        min_component_area: int = 120,
        min_height_ratio: float = 0.12,
        merge_x_ratio: float = 0.012,
        min_diameter_mm: float = 5.0,
        max_diameter_mm: float = 45.0,
        scanline_ratio: float = 0.50,
        use_standard_diameter: bool = True,
        unify_diameter_per_image: bool = False,
        spacing_scale_factor: float = 1.50,
        small_rebar_nominal_mm: Optional[float] = None,
    ):
        self._fx = fx
        self._fy = fy
        self._cx = cx
        self._cy = cy
        self._min_component_area = min_component_area
        self._min_height_ratio = min_height_ratio
        self._merge_x_ratio = merge_x_ratio
        self._min_diameter_mm = min_diameter_mm
        self._max_diameter_mm = max_diameter_mm
        self._scanline_ratio = scanline_ratio
        self._standard_diameters = np.array(STANDARD_DIAMETERS, dtype=np.float32)
        # V53 新增配置参数
        self._use_standard_diameter = use_standard_diameter
        self._unify_diameter_per_image = unify_diameter_per_image
        self._spacing_scale_factor = spacing_scale_factor
        self._small_rebar_nominal_mm = small_rebar_nominal_mm

    def __post_init_setup(self, distance_mm: float) -> None:
        """每次测量时根据距离计算 mm/px。"""
        # self._mm_per_pixel = distance_mm / self._fx
        # self._spacing_mm_per_pixel = self._mm_per_pixel * 1.50
        # self._min_diameter_px = self._min_diameter_mm / self._mm_per_pixel
        # self._max_diameter_px = self._max_diameter_mm / self._mm_per_pixel
        pass

    def measure(
        self,
        mask: np.ndarray,
        distance_mm: float,
        rgb_frame: Optional[np.ndarray] = None,
    ) -> Optional[MeasureResult]:
        """
        执行一次完整测量流水线（V53 算法）。
        
        参数:
            mask: 单通道 uint8 类别掩码 (0/1/2)，由检测节点 推理返回
            distance_mm: 工作距离（mm），激光测距融合值
            rgb_frame: 可选，用于生成可视化标注画像
            
        返回: MeasureResult 或 None（无有效钢筋）

        完整流水线（源自 new-predict.py RebarMeasureV53.measure_from_label）：
        1. 根据 distance_mm 重新计算 mm/px 换算系数
        2. _label_to_masks: 将 0/1/2 掩码分离为 red_mask(类别1) 和 green_mask(类别2)
        3. _build_regular_rebars:
           a. _regularize_rebar_mask: 形态学操作（开运算 + 闭运算 + 中值滤波）
           b. findContours 提取轮廓（RETR_EXTERNAL）
           c. 逐轮廓几何筛选：面积、长宽比、角度误差、填充率、与横向钢筋重叠率
           d. _robust_component_width: 多行扫描取宽度中位数
           e. 共线分段合并（按 center_x 分组加权合并）
        4. _filter_main_rebars_for_spacing:
           筛选主测量线钢筋（长度足够、位置合理）
        5. _insert_missing_rebars_by_gap:
           基于间距中位数检测并插入漏检钢筋（gap/med_pitch > 1.55 时插筋）
        6. 逐根计算直径：width_px × mm_per_pixel → raw_diameter_mm
           _unify_diameter_per_image: 图像级直径归一化（可选）
           _normalize_rebar_diameter: 就近对齐国标
        7. 按 center_x 排序，计算相邻间距（spacing_scale_factor=1.50）
        8. 若提供 rgb_frame:
           _draw_regular_rebars: 红色半透明掩罩 + 绿色多边形边框
           _draw_diameter_labels: 直径标注（D:XX 或 D:XXmm/SXX）
           _draw_spacing_lines + _draw_spacing_labels: 间距标注（青色虚线 + 间距值）
        9. 返回 MeasureResult
        """
        pass

    @staticmethod
    def _label_to_masks(label_mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        将 0/1/2 类别掩码分离为两个二值掩码。
        
        Returns:
            (red_mask, green_mask) - 类别1(纵向钢筋)和类别2(横向钢筋)的二值掩码
        """
        # label = np.asarray(label_mask)
        # if label.ndim == 3:
        #     label = label[:, :, 0]
        # red_mask = (label == 1).astype(np.uint8) * 255
        # green_mask = (label == 2).astype(np.uint8) * 255
        return np.zeros((1, 1), dtype=np.uint8), np.zeros((1, 1), dtype=np.uint8)

    @staticmethod
    def _regularize_rebar_mask(red_mask: np.ndarray) -> np.ndarray:
        """
        对纵向钢筋掩码进行形态学规整。
        
        步骤:
        1. 开运算（3x3 椭圆核）去除小噪点
        2. 垂直闭运算（5x23 矩形核）修复钢筋断裂
        3. 水平闭运算（7x5 矩形核）连接细小缺失
        4. 中值滤波（3x3）平滑边缘
        
        注意: 闭核尺寸不宜过大，否则会合并相邻钢筋。
        """
        # mask = (red_mask > 0).astype(np.uint8) * 255
        # kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        # kernel_close_v = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 23))
        # kernel_close_h = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 5))
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close_v)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close_h)
        # mask = cv2.medianBlur(mask, 3)
        return red_mask

    def _build_regular_rebars(
        self,
        red_mask: np.ndarray,
        green_mask: np.ndarray,
    ) -> Tuple[List[dict], np.ndarray, List[Tuple[str, float]]]:
        """
        从掩码中提取规整化的钢筋候选集。
        
        完整筛选逻辑（源自 _build_regular_rebars）:
        1. 对 regularized 掩码做 findContours
        2. 逐轮廓筛选:
           - area >= min_component_area (默认 120)
           - 长宽比 >= 2.4（钢筋是细长形）
           - 角度误差 <= 16°（接近竖直）
           - 填充率 >= 0.22（排除稀疏噪声）
           - 与横向钢筋重叠率 <= 28%（排除交叉点）
        3. _robust_component_width: 多行扫描取中位数宽度
        4. 与中位宽度对比，剔除过窄的候选（< 32% 中位宽度）
        5. 按 center_x 分组，间距 < min_center_gap 的共线段加权合并
        6. 计算每个钢筋的显示包围盒（_box_from_center_angle）
        
        Returns:
            (regular, clean_mask, rejected)
            - regular: 筛选后的钢筋字典列表
            - clean_mask: 规整化后的二值掩码
            - rejected: 被拒绝的列表 [(reason, area), ...]
        """
        # h, w = red_mask.shape[:2]
        # clean = self._regularize_rebar_mask(red_mask)
        # contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # ... 筛选逻辑 ...
        return [], red_mask, []

    def _robust_component_width(
        self,
        component_mask: np.ndarray,
        bbox: Tuple[int, int, int, int],
    ) -> Optional[float]:
        """
        多行扫描估算组件的稳定宽度。
        
        在 bbox 范围内逐行扫描，取每行前景像素的最左和最右位置，
        计算宽度。最终取所有行宽度的中位数，并剔除异常行
        （< 0.55 倍或 > 1.35 倍中位宽度）。
        
        仅当有效行数 >= 8 时才返回可靠结果。
        
        Returns:
            宽度中位数（像素），或 None（不稳定/行数不足）
        """
        # x, y, bw, bh = bbox
        # roi = component_mask[y:y + bh, x:x + bw]
        # widths = [float(xs[-1] - xs[0] + 1) for row in roi if len(np.where(row > 0)[0]) >= 3]
        # if len(widths) < 8:
        #     return None
        # med = float(np.median(widths))
        # valid = widths[(widths >= med * 0.55) & (widths <= med * 1.35)]
        # return float(np.median(valid)) if len(valid) >= 6 else float(np.median(widths))
        return None

    def _filter_main_rebars_for_spacing(
        self, rebars: List[dict], img_h: int
    ) -> Tuple[List[dict], int]:
        """
        筛选主测量线钢筋并确定测量 Y 坐标。
        
        筛选条件:
        1. 角度误差 <= 18°
        2. 长度 >= max(img_h * 0.34, median_length * 0.45)
        3. 选择通过 measure_y 位置的钢筋（与 measure_y 有交集的）
        
        measure_y 确定: 取所有长筋 center_y 的中位数，限制在 [0.18h, 0.82h]
        
        Returns:
            (main_rebars, measure_y) - 筛选后的钢筋列表和测量 Y 坐标
        """
        # 返回按 center_x 排序的主钢筋列表
        return rebars, img_h // 2

    def _insert_missing_rebars_by_gap(
        self,
        rebars: List[dict],
        img_h: int,
        measure_y: int,
        candidates: Optional[List[dict]] = None,
        red_mask: Optional[np.ndarray] = None,
    ) -> Tuple[List[dict], int]:
        """
        基于间距分析插入漏检钢筋。
        
        算法:
        1. 计算相邻钢筋 center_x 间距数组
        2. 取中位间距 med_pitch
        3. 若某段间距 > 1.55 * med_pitch 且 < 3.1 * med_pitch，
           判断有漏检，按等分插入 1~2 根钢筋
        4. 优先从 candidates 列表中匹配接近期望位置的候选
        
        仅当钢筋总数 >= 4 时执行（少于 4 根无统计意义）。
        
        Returns:
            (rebars, inserted_count) - 插入后的钢筋列表和插入数量
        """
        # if len(rebars) < 4: return rebars, 0
        # center_gaps[i] = rebars[i].center_x - rebars[i-1].center_x
        # med_pitch = np.median(center_gaps[center_gaps > 0])
        # for i in range(1, len(rebars)):
        #     gap = curr_cx - prev_cx
        #     ratio = gap / med_pitch
        #     if 1.55 <= ratio <= 3.10:
        #         missing_count = int(round(ratio)) - 1
        #         ... 插入 ...
        return rebars, 0

    def _normalize_rebar_diameter(self, d_mm: float) -> float:
        """
        就近对齐国标钢筋规格。
        
        国标规格: [6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40]
        使用 argmin 找到最近的规格值。
        """
        # idx = int(np.argmin(np.abs(self._standard_diameters - d_mm)))
        # return float(self._standard_diameters[idx])
        return 0.0

    def _unify_diameter_per_image(
        self,
        rebars: List[dict],
        use_standard: bool = True,
        small_rebar_nominal: Optional[float] = None,
    ) -> None:
        """
        图像级直径归一化（可选功能）。
        
        逻辑（源自 new-predict.py 第 1783-1820 行）:
        1. 计算所有钢筋 raw_diameter 的中位数 med_d
        2. 保留在 [0.7*med_d, 1.3*med_d] 内的"良好"值
        3. 取良好值的中位数作为 image_raw_diameter_mm
        4. 根据配置策略更新每根钢筋:
           a. unify_diameter_per_image=True: 所有钢筋使用图像级统一直径
           b. small_rebar_nominal_mm 模式: 小直径钢筋（<= nominal*1.15）统一为标称值
           c. standard_diameter <= 12 且直径接近中位: 设为统一标称直径
           d. 其他: 保持原始测量直径
        """
        if not rebars:
            return
        # raw_ds = np.asarray([r["edge"]["width_px"] * self._mm_per_pixel for r in rebars])
        # med_d = float(np.median(raw_ds))
        # good = raw_ds[(raw_ds > med_d * 0.70) & (raw_ds < med_d * 1.30)]
        # if len(good) >= 2:
        #     med_d = float(np.median(good))
        pass

    def _draw_regular_rebars(
        self, vis: np.ndarray, rebars: List[dict], alpha: float = 0.72
    ) -> np.ndarray:
        """
        绘制钢筋规整化结果（红色半透明掩罩 + 绿色边框）。
        
        源自 new-predict.py _draw_regular_rebar.
        """
        # overlay = vis.copy()
        # for r in rebars:
        #     box = np.int32(np.round(r["box"]))
        #     cv2.fillConvexPoly(overlay, box, (0, 0, 255), lineType=cv2.LINE_AA)
        #     cv2.polylines(vis, [box], True, (0, 255, 0), 2, lineType=cv2.LINE_AA)
        # cv2.addWeighted(overlay, alpha, vis, 1 - alpha, 0, dst=vis)
        return vis

    def _draw_diameter_labels(
        self,
        vis: np.ndarray,
        rebars: List[dict],
        measure_y: int,
        font_scale: float,
        text_thick: int,
    ) -> np.ndarray:
        """
        绘制直径标注文本。
        
        标注格式:
        - 标准模式: "D:{standard_mm:.0f}"
        - 非标准模式: "D:{diameter_mm:.1f}mm"
        - 统一模式(记录原始值): "D:{standard:.0f}({raw:.1f})"
        
        文本颜色: 黄色 (0, 255, 255)
        位置: 钢筋顶部上方
        """
        # for i, r in enumerate(rebars):
        #     edge = r["edge"]
        #     if self._use_standard_diameter and r.get("standard_mm") is not None:
        #         label = f"D:{r['standard_mm']:.0f}"
        #     else:
        #         label = f"D:{r['diameter_mm']:.1f}mm"
        #     tx = int(edge["center_x"])
        #     ty = max(24, int(edge["y_top"] + 24))
        #     cv2.putText(vis, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
        #                 font_scale, (0, 255, 255), text_thick + 1, lineType=cv2.LINE_AA)
        return vis

    def _draw_spacing_lines_and_labels(
        self,
        vis: np.ndarray,
        rebars: List[dict],
        spacings: List[Optional[float]],
        measure_y: int,
        font_scale: float,
        text_thick: int,
        line_thick: int,
    ) -> np.ndarray:
        """
        绘制相邻钢筋间距标注。
        
        标注内容:
        - 青色连接线（prev.right_x 到 curr.left_x，在 measure_y 高度）
        - 端点红色圆点
        - 间距标签: "{gap_int}mm"
        - 标签背景: 白色矩形 + 黑色边框
        
        间距计算: (curr.left_x - prev.right_x) * spacing_mm_per_pixel
        注意: spacing_mm_per_pixel = mm_per_pixel * 1.50（修正系数）
        
        仅当间距 > 0 时绘制。
        """
        # for i in range(1, len(rebars)):
        #     prev = rebars[i - 1]["edge"]
        #     curr = rebars[i]["edge"]
        #     if spacings[i] is None or spacings[i] <= 0:
        #         continue
        #     x1 = int(round(prev["right_x"]))
        #     x2 = int(round(curr["left_x"]))
        #     y_line = measure_y
        #     cv2.line(vis, (x1, y_line), (x2, y_line), (0, 255, 255), line_thick)
        #     gap_label = f"{int(round(spacings[i]))}mm"
        #     ... 绘制标签 ...
        return vis

    def _export_csv(
        self,
        rebars: List[dict],
        spacings: List[Optional[float]],
        save_dir: str,
        base_name: str,
        distance_mm: float,
        mm_per_pixel: float,
    ) -> str:
        """
        导出测量结果 CSV 文件（UTF-8 with BOM）。
        
        列结构（源自 new-predict.py 第 1924-1960 行）:
        - 图片名称
        - 钢筋编号（H1, H2, ...）
        - 检测直径/mm（取整为标准规格 或 保留1位小数）
        - 相邻区间（H1~H2 或 -）
        - 检测间距/mm
        
        Returns: CSV 文件路径
        """
        # csv_path = os.path.join(save_dir, f"report_{base_name}.csv")
        # with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        #     writer = csv.writer(f)
        #     writer.writerow(["图片名称", "钢筋编号", "检测直径/mm", "相邻区间", "检测间距/mm"])
        #     writer.writerows(rows)
        return ""
```

### 3.7 UI 模块（`ui/`）

#### 3.7.1 CLI 融合策略菜单

```python
"""ui/cli_fusion_menu.py"""

import sys
from typing import Callable, Optional

from serial.laser_fusion import LaserFusion, FusionStrategy


class CliFusionMenu:
    """
    命令行终端交互式融合策略菜单。
    使用标准 input() / print()，零额外依赖。
    """

    def __init__(
        self,
        fusion: LaserFusion,
        on_change: Optional[Callable[[FusionStrategy], None]] = None,
    ):
        self._fusion = fusion
        self._on_change = on_change

    def show(self) -> None:
        """
        显示菜单并获取用户选择。
        在 CLI 线程中循环运行。
        """
        # 伪代码：
        # while True:
        #     print("\n=== 激光融合策略 ===")
        #     print("1. 取均值（默认）")
        #     print("2. 取中位数")
        #     print("3. 取特定一路（一~四）")
        #     print("4. 手动输入指定值（mm）")
        #     choice = input("请选择 [1-4]: ").strip()
        #     if choice == "1": self._fusion.set_strategy(FusionStrategy.MEAN)
        #     elif choice == "2": self._fusion.set_strategy(FusionStrategy.MEDIAN)
        #     elif choice == "3":
        #         idx = int(input("请输入接口编号（1-4）: ")) - 1
        #         self._fusion.set_strategy(FusionStrategy.SPECIFIC, specific_index=idx)
        #     elif choice == "4":
        #         val = float(input("请输入距离值（mm）: "))
        #         self._fusion.set_strategy(FusionStrategy.MANUAL, manual_value=val)
        #     if self._on_change: self._on_change(self._fusion.get_strategy())
        ...
```

#### 3.7.2 画面渲染器

```python
"""ui/render.py"""

import cv2
import numpy as np
from PIL import Image, ImageTk

from measure.rebar_measure import MeasureResult
from common.constants import SegClass


class UIRenderer:
    """
    叠加渲染器：将掩码、测量结果、激光数据叠加到 RGB 帧。
    返回 ImageTk.PhotoImage 供 tkinter Label 显示。
    """

    # 类别颜色映射（BGR）
    COLOR_BACKGROUND = (0, 0, 0)
    COLOR_VERTICAL = (0, 0, 255)    # 红色 — 纵向钢筋
    COLOR_HORIZONTAL = (0, 255, 0)  # 绿色 — 横向钢筋

    def __init__(self, alpha: float = 0.45):
        self._alpha = alpha

    def compose(
        self,
        rgb_frame: np.ndarray,
        mask: np.ndarray,
        measure_result: Optional[MeasureResult],
        laser_values: tuple,
        distance_mm: float,
        display_size: tuple,
        degraded: bool = False,
    ) -> ImageTk.PhotoImage:
        """
        合成最终显示画面。

        伪代码：
        1. overlay = rgb_frame.copy()
        2. 叠加掩码：类别1区域半透明红色、类别2区域半透明绿色
        3. 若有 measure_result: 绘制绿色边框 + 直径标注（白字）
           相邻钢筋：灰色虚线 + 间距标注（青字）
        4. 绘制激光数据面板：4 路原始值 + 融合值
        5. 左上显示系统状态：fps / 工作距离 / 策略 / 帧同步信息
        6. 若 degraded=True: 顶部显示黄色横幅"推理服务离线 — 降级模式"
        7. resize 到 display_size
        8. BGR → RGB → PIL.Image → ImageTk.PhotoImage
        """
        # h, w = rgb_frame.shape[:2]
        # pil_img = Image.fromarray(cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB))
        # return ImageTk.PhotoImage(pil_img)
        return None  # type: ignore
```

#### 3.7.3 主应用类

```python
"""ui/app.py"""

import sys
import time
import threading
from typing import Optional

import tkinter as tk
from tkinter import ttk, messagebox

from camera.base import BaseCameraInput
from camera.orbbec_336l import Orbbec336LInput
from camera.mock import MockCameraInput
from serial.serial_manager import SerialManager
from serial.laser_fusion import LaserFusion, FusionStrategy
from grpc_client.inference_client import InferenceGrpcClient, InferenceResult
from measure.rebar_measure import RebarMeasure, MeasureResult
from measure.intrinsics_manager import IntrinsicsManager
from ui.cli_fusion_menu import CliFusionMenu
from ui.render import UIRenderer
from common.config_loader import (
    ConfigLoader, CameraConfig, IntrinsicsConfig, NetworkConfig,
    InferenceConfig,
)


class RebarMeasureApp:
    """
    服务节点 主应用类（tkinter GUI 主线程）。
    整合所有子模块，驱动采集 → 推理 → 测量 → 显示全链路。
    """

    def __init__(self, root: tk.Tk):
        self._root = root
        root.title("钢筋直径测量系统 — 服务节点 主控")
        root.geometry("1280x800")

        # ---- 加载配置 ----
        self._camera_cfg = ConfigLoader.load_camera_config(
            "config/camera.json"
        )
        self._intrinsics_cfg = ConfigLoader.load_intrinsics_config(
            "config/intrinsics.json"
        )
        self._network_cfg = ConfigLoader.load_network_config(
            "config/network.json"
        )
        self._inference_cfg: InferenceConfig = ConfigLoader.load_inference_config(
            "config/inference.json"
        )

        # ---- 初始化模块 ----
        self._camera: Optional[BaseCameraInput] = None
        self._serial_mgr = SerialManager(self._network_cfg)
        self._fusion = LaserFusion()
        self._grpc_client = InferenceGrpcClient(self._network_cfg)
        self._intrinsics_mgr = IntrinsicsManager(self._intrinsics_cfg)
        self._rebar_measure = RebarMeasure(
            fx=self._intrinsics_cfg.fx,
            fy=self._intrinsics_cfg.fy,
        )
        self._renderer = UIRenderer()

        # ---- 运行状态 ----
        self._is_running: bool = False
        self._degraded_mode: bool = False       # 推理服务离线降级标志
        self._frame_id: int = 0
        self._last_infer_time: float = 0.0
        self._latest_mask: Optional[np.ndarray] = None
        self._latest_measure: Optional[MeasureResult] = None

        # 工作线程
        self._main_loop_thread: Optional[threading.Thread] = None
        self._cli_thread: Optional[threading.Thread] = None
        self._health_thread: Optional[threading.Thread] = None

        self._setup_ui()

    def _setup_ui(self) -> None:
        """初始化 Tkinter 界面布局。"""
        # 布局结构（自上而下）：
        # row 0: 视频显示区 (LabelFrame "测量画面")
        #   - self._video_label: ttk.Label (anchor=center)
        # row 1: 激光数据面板 (LabelFrame "激光测距数据")
        #   - 4x 传感器值标签 + 融合策略与结果
        # row 2: 钢筋测量结果面板 (LabelFrame "钢筋直径/间距")
        #   - self._rebar_treeview: ttk.Treeview (编号, 直径, 国标直径, 间距)
        # row 3: 操作面板 (LabelFrame "操作")
        #   - self._measure_btn: "测量"按钮
        #   - self._status_label: 系统状态文本
        pass

    def start(self) -> None:
        """启动所有工作线程。"""
        # 伪代码：
        # self._is_running = True
        #
        # 1. 打开相机（异步扫描 → 选择 → 打开）
        # 2. 连接 gRPC（probe）→ 不可达则 self._degraded_mode = True
        # 3. 打开串口（异步扫描选择）
        # 4. 启动主循环线程: self._main_loop_thread
        # 5. 启动 CLI 线程: self._cli_thread
        # 6. 启动健康检查线程: self._health_thread
        pass

    def _request_measurement(self) -> None:
        """用户触发测量：推理 + 测量换算（在工作线程执行）。"""
        # interval_sec = self._inference_cfg.inference_interval_seconds
        # if time.time() - self._last_infer_time < interval_sec:
        #     msg = f"两次测量间隔至少 {interval_sec:.1f} 秒"
        #     root.after(0, lambda: messagebox.showwarning("提示", msg))
        #     return
        # if self._degraded_mode:
        #     root.after(0, lambda: messagebox.showwarning("降级", "推理服务离线，无法测量"))
        #     return
        # threading.Thread(target=self._do_measurement, daemon=True).start()

    def _do_measurement(self) -> None:
        """工作线程中执行推理 + 测量。"""
        # 伪代码：
        # 1. rgb_frame = self._camera.get_rgb_frame()
        # 2. laser_result, _ = self._serial_mgr.get_latest()
        # 3. distance = self._fusion.fuse(laser_result)
        # 4. if distance is None: 提示激光无效 → return
        # 5. self._frame_id += 1
        # 6. infer_result = self._grpc_client.infer(rgb_frame, self._frame_id, distance)
        # 7. if infer_result is None: 提示推理超时 → return
        # 8. 帧同步校验: infer_result.frame_id == self._frame_id
        # 9. measure_result = self._rebar_measure.measure(
        #        infer_result.mask, distance, rgb_frame)
        # 10. self._last_infer_time = time.time()
        # 11. self._latest_measure = measure_result
        # 12. self._root.after(0, self._refresh_display)
        pass

    def _main_loop(self) -> None:
        """主循环线程：仅负责视频预览刷新，不执行推理与测量。"""
        # while self._is_running:
        #     frame = self._camera.get_rgb_frame() if self._camera else None
        #     if frame is None: time.sleep(0.033); continue
        #     laser_result, _ = self._serial_mgr.get_latest()
        #     distance = self._fusion.fuse(laser_result) if laser_result else 0.0
        #     raw_values = laser_result.raw_values if laser_result else (0,0,0,0)
        #     img = self._renderer.compose(frame, self._latest_mask,
        #         self._latest_measure, raw_values, distance,
        #         (display_w, display_h), self._degraded_mode)
        #     self._root.after(0, lambda: self._update_video(img))
        #     time.sleep(0.033)
        pass

    def _refresh_display(self) -> None:
        """测量完成后，用最新结果刷新显示（主线程）。"""
        # if self._latest_measure:
        #     更新 Treeview 钢筋列表与间距
        #     _refresh_video_display()
        pass

    def _refresh_video_display(self, img: ImageTk.PhotoImage) -> None:
        """更新视频标签图像（主线程安全）。"""
        # self._video_label.imgtk = img
        # self._video_label.config(image=img)
        pass

    def _on_closing(self) -> None:
        """窗口关闭时的清理。"""
        # self._is_running = False
        # self._camera.close() if self._camera else None
        # self._serial_mgr.close()
        # self._grpc_client.close()
        # self._root.destroy()
        pass
```

#### 3.7.4 主界面设计

主界面有两种互斥状态，随摄像头连接状态切换。两个状态共用同一主窗口，通过隐藏/显示控件实现切换。

| 状态 | 触发条件 | 界面内容 |
|------|----------|----------|
| **摄像头选择状态** | 程序启动后、摄像头未连接时 | 显示摄像头扫描结果列表 + 手动添加网络摄像头入口 + 选择连接按钮（详见 §3.3.6） |
| **识别工作状态** | 摄像头连接成功后 | 实时 RGB 画面 + 边缘 OSD 关键参数（详见 §3.7.7）+ 底部识别按钮（详见 §3.7.6）+ 边角副页面入口（详见 §3.7.5） |

**切换条件**：
- 摄像头选择状态 → 识别工作状态：用户选择摄像头并连接成功（对应 §0.2 → §0.3）
- 识别工作状态 → 摄像头选择状态：用户主动断开摄像头 / 相机连接异常重连失败

```
┌─────────────────────────────────────────────┐
│ [设置]                              [推理结果]│  ← 边角副页面入口
├─────────────────────────────────────────────┤
│                                              │
│   FPS:60 | NetCamera | TCP    ← OSD 边缘     │
│                                              │
│            (实时 RGB 画面)                    │
│                                              │
│                                              │
├─────────────────────────────────────────────┤
│              [ 识别 ]  ← 识别按钮             │
└─────────────────────────────────────────────┘
```

> **降级模式显示**：当推理服务离线（降级模式）时，主界面顶部叠加黄色横幅"推理服务离线 — 降级模式"，识别按钮禁用并提示。

#### 3.7.5 副页面设计

副页面以**弹出式独立窗口**呈现，从主界面边角入口打开。主界面边角提供两个入口：**"设置"** 与 **"推理结果"**。

**关键约束**：
- **副页面存在时不得发起推理**：任一副页面打开时，主界面识别按钮灰化禁用，并显示提示文字"请先关闭设置/推理结果窗口后再发起识别"。
- **副页面可单独关闭**：关闭副页面不影响主界面状态；关闭所有副页面后识别按钮恢复可用。
- 未在副页面列出的参数（如相机内参 fx/fy、网络端口等）仅由**开发人员修改 JSON 配置文件**，不对最终用户暴露。

**① 设置副页面可配置项**：

| 配置项 | 取值 | 说明 |
|--------|------|------|
| 测距值选取 | 五选一：取指定路 / 均值 / 最大值 / 最小值 / 手动输入 | 4 路激光测距的融合策略（对应 §3.4.3 扩展）；选"取指定路"时附接口编号选择（一~四）；选"手动输入"时附距离值输入框（mm） |
| 主页面显示方式 | 二选一：叠加模式 / 底栏模式 | 见 §3.7.7，用户可运行时切换 |
| 推理结果保存方式 | 多选：CSV 数值表 / 类别掩码图 / 可视化结果图 | 可勾选多项同时保存；`result.jpg` 可视化结果图为强制默认保存项 |

> 设置项修改后立即生效（下一帧/下一次推理使用新配置），并通过共享状态同步至后台调度与 CLI。

**② 推理结果副页面**：

| 元素 | 说明 |
|------|------|
| 历史记录列表 | 按时间倒序列历次推理记录（时间戳、帧序号、钢筋数、所用距离） |
| 可视化回显 | 点击列表项后显示该次推理的可视化结果图（`result.jpg`） |
| 异常提示 | 若该次记录无可视化结果图（异常无图），显示异常提示文字而非图片 |

> 推理结果副页面的历史数据来源于后台调度的"数据接收汇总记录追踪"子模块（见 §3.11），按帧索引可追溯。

#### 3.7.6 识别按钮状态机

识别按钮位于主界面底部，布局参考照相机拍照按钮（圆形大按钮，居中）。按钮有四个状态，状态机如下：

```
        点击(就绪)          推理完成
[准备就绪] ──────→ [加载中] ──────→ [结果展示]
     ↑                                 │
     └───────── 点击(回实时) ──────────┘
```

| 状态 | 显示 | 行为 | 后续触发 |
|------|------|------|----------|
| **准备就绪** | 圆形按钮，常态色 | 可点击；点击后冻结主画面并进入加载中 | 点击 → 加载中 |
| **加载中** | 圆形按钮 + loading 动画/转圈 | 禁用点击；后台执行九步核心循环 ③~⑥（锁定帧→推理→测量→保存） | 推理完成 → 结果展示 |
| **结果展示** | 圆形按钮，结果态色 | 主画面显示可视化结果图（步骤 ⑦）；可点击 | 点击 → 准备就绪（解除冻结，回实时画面，步骤 ⑨） |
| **禁用** | 灰化按钮 | 不可点击；副页面打开 / 降级模式 / 激光无效时进入 | 条件解除 → 准备就绪 |

> **与九步循环的对应**：按钮状态机驱动 §0.4 九步核心循环——"准备就绪→加载中"对应步骤 ①②，"加载中"持续步骤 ③④⑤⑥，"加载中→结果展示"对应步骤 ⑦⑧，"结果展示→准备就绪"对应步骤 ⑨。

#### 3.7.7 显示模式与 OSD

**OSD 关键参数**（主画面边缘显示）：

格式：`FPS:60 | NetCamera | TCP`

| 字段 | 含义 | 取值示例 |
|------|------|----------|
| `FPS` | 当前帧率 | `60` / `30` |
| `数据来源` | 当前 RGB 数据来源 | `NetCamera`（网络摄像头）/ `LocalCamera`（本地 USB） |
| `来源协议` | 数据来源协议 | `TCP`（网络摄像头）/ `USB`/`DirectShow`（本地） |

> OSD 实时反映当前接入方式，便于用户识别数据链路。当从网络摄像头切换到本地摄像头时，OSD 自动更新。

**两种显示模式**（用户可在设置副页面切换，见 §3.7.5）：

| 模式 | 布局 | 说明 |
|------|------|------|
| **叠加模式** | 画面正中心上下左右四方向各一个测距点 + 黄色数字 | 在 RGB 画面中心区域四个方向叠加激光测距值，黄色文字标注；适合单点聚焦观察 |
| **底栏模式** | RGB 下方底部显示栏统一参数与测距 | 在 RGB 画面下方独立参数栏，统一显示 4 路测距值、融合距离、系统参数等；适合完整数据查看 |

```
叠加模式:                        底栏模式:
┌──────────────┐                ┌──────────────┐
│              │                │              │
│   上:812     │                │   (RGB)      │
│ 左:815  右:810│                │              │
│   下:813     │                ├──────────────┤
│              │                │参数栏+4路测距 │
└──────────────┘                └──────────────┘
```

> **默认显示模式**：原型阶段默认使用底栏模式（数据完整、不易遮挡画面）；用户可在设置副页面切换为叠加模式。

### 3.8 系统模块（`system/`）

#### 3.8.1 启动环境自检

```python
"""system/self_check.py"""

import socket
import sys
from typing import Tuple

from common.config_loader import NetworkConfig


class SelfCheckResult:
    """自检结果数据类。"""
    def __init__(self):
        self.admin_privilege_ok: bool = False   # 管理员权限验证
        self.local_ip_ok: bool = False
        self.port_ok: bool = False
        self.network_config_privilege_ok: bool = False  # 网络配置权限
        self.peer_reachable: bool = False
        self.errors: list = []
        self.warnings: list = []


def check_admin_privilege() -> Tuple[bool, str]:
    """
    检查当前进程是否以 root/sudo 启动。
    条件性硬故障：非 root 且需特权操作（注册系统服务、绑定特权端口 < 1024）时为硬故障；
    仅运行无特权操作时降级为 WARNING。
    返回: (是否通过, 消息)
    """
    # 伪代码：
    # if sys.platform == "win32":
    #     return True, "Windows 开发环境：管理员权限校验 bypass"
    # if os.geteuid() == 0:
    #     return True, "以 root 权限启动"
    # # 非 root：判断是否需要特权操作（由调用方传入上下文或读配置）
    # return False, "非 root 启动，注册系统服务/绑定特权端口将失败"
    return False, "not implemented"


def check_local_ip(cfg: NetworkConfig) -> Tuple[bool, str]:
    """
    检查本机 IP 是否与 network.json 中 local_ip 一致。
    Windows 开发环境下可 bypass。
    返回: (是否通过, 消息)
    """
    # 伪代码：
    # if platform == "win32":
    #     return True, "Windows 开发环境：IP 校验 bypass"
    #
    # 获取本机网卡 IP:
    #   s = socket.socket(AF_INET, SOCK_DGRAM)
    #   s.connect(("8.8.8.8", 80))
    #   actual_ip = s.getsockname()[0]
    #
    # if actual_ip != cfg.local_ip:
    #     return False, f"本机 IP {actual_ip} 与配置 {cfg.local_ip} 不一致"
    # return True, "本机 IP 校验通过"
    return False, "not implemented"


def check_port_available(cfg: NetworkConfig) -> Tuple[bool, str]:
    """检查 gRPC 端口是否未被占用（可绑定）。"""
    # 伪代码：
    # s = socket.socket(AF_INET, SOCK_STREAM)
    # result = s.connect_ex(("0.0.0.0", cfg.grpc_port))
    # s.close()
    # if result == 0: return False, f"端口 {cfg.grpc_port} 已被占用"
    # return True, "端口可用"
    return False, "not implemented"


def check_network_config_privilege() -> Tuple[bool, str]:
    """
    检查应用层是否具备读取/校验网络配置的权限（如读取网卡 IP、bind 端口等）。
    软故障：权限缺失时告警但不退出。
    返回: (是否通过, 消息)
    """
    # 伪代码：
    # try:
    #     测试读取网卡 IP / bind 测试端口
    #     return True, "网络配置权限正常"
    # except PermissionError:
    #     return False, "网络配置权限缺失，部分检查可能不可靠"
    return False, "not implemented"


def check_peer_reachable(cfg: NetworkConfig) -> Tuple[bool, str]:
    """TCP 探测对端检测节点 的 IP:端口是否可达。"""
    # 伪代码：
    # try:
    #     s = socket.socket(AF_INET, SOCK_STREAM)
    #     s.settimeout(2.0)
    #     s.connect((cfg.remote_ip, cfg.grpc_port))
    #     s.close()
    #     return True, "对端节点可达"
    # except (socket.timeout, ConnectionRefusedError, OSError):
    #     return False, f"对端 {cfg.remote_ip}:{cfg.grpc_port} 不可达"
    return False, "not implemented"


def run_self_check(cfg: NetworkConfig) -> SelfCheckResult:
    """
    执行完整的启动环境自检。
    返回 SelfCheckResult。
    硬故障（管理员权限条件性/IP 不匹配/端口冲突）需退出；
    软故障（对端不可达/网络配置权限缺失）启用降级模式或告警继续。
    """
    result = SelfCheckResult()

    ok, msg = check_admin_privilege()
    result.admin_privilege_ok = ok
    if not ok:
        result.warnings.append(msg)  # 条件性硬故障由调用方按上下文判断

    ok, msg = check_local_ip(cfg)
    result.local_ip_ok = ok
    if not ok:
        result.errors.append(msg)
    else:
        result.warnings.append(msg)

    ok, msg = check_port_available(cfg)
    result.port_ok = ok
    if not ok:
        result.errors.append(msg)

    ok, msg = check_network_config_privilege()
    result.network_config_privilege_ok = ok
    if not ok:
        result.warnings.append(msg)

    ok, msg = check_peer_reachable(cfg)
    result.peer_reachable = ok
    if not ok:
        result.warnings.append(msg)

    return result
```

> **自检顺序约定**：自检项顺序为 ①管理员权限 → ②本机 IP → ③端口可用性 → ④网络配置权限 → ⑤对端可达性。前四项为本地配置/权限类检查（快速），最后一项为网络 I/O（可能耗时 2 秒）。硬故障（①条件性/②/③）触发 `sys.exit(1)`；软故障（④/⑤）记录 WARNING 并继续。详见 §5.2 自检检查项与失败处理表。

#### 3.8.2 运行时节点状态监测（被动接收为主、主动探测兜底）

```python
"""system/node_monitor.py"""

import logging
import threading
import time
from collections import deque
from typing import Callable, Optional, Deque

from common.config_loader import NetworkConfig


class NodeStatus:
    """检测节点 运行状态枚举。"""
    UNKNOWN = "unknown"
    ONLINE = "online"
    BUSY = "busy"
    OFFLINE = "offline"
    DEGRADED = "degraded"


class HeartbeatEvent:
    """心跳事件记录。"""
    def __init__(self, timestamp_ms: int, status: NodeStatus,
                 infer_count: int = 0, rtt_ms: int = 0):
        self.timestamp_ms = timestamp_ms
        self.status = status
        self.infer_count = infer_count
        self.rtt_ms = rtt_ms  # 本机处理耗时（内部指标）


class NodeMonitor:
    """
    检测节点 状态监测器（服务节点 侧）。

    监测策略：被动接收检测节点 心跳 + 超时判定。
    - 检测节点 每 5 秒主动推送心跳，每次心跳刷新"存活计时器"
    - 服务节点 本身不做频繁 probe；启动阶段一次性 probe 通过后，依赖心跳维持在线状态
    - 连续 3 次心跳未收到（约 15 秒），判定离线，触发降级

    持久化记录：状态变更事件写入 JSONL 日志（供后续审计/大盘展示）。
    """

    def __init__(
        self,
        net_cfg: NetworkConfig,
        on_lost: Callable[[], None],
        on_recovered: Callable[[], None],
        heartbeat_timeout_s: float = 15.0,
        status_log_path: str = "logs/node_detect_status.jsonl",
    ):
        self._net_cfg = net_cfg
        self._on_lost = on_lost
        self._on_recovered = on_recovered
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._status_log_path = status_log_path
        self._is_running: bool = False
        self._current_status: NodeStatus = NodeStatus.UNKNOWN
        self._last_heartbeat_ts: float = 0.0
        self._heartbeat_count: int = 0
        self._watchdog_thread: Optional[threading.Thread] = None
        self._status_lock = threading.Lock()
        # 事件时序缓存（供 UI 实时显示）
        self._event_ring: Deque[HeartbeatEvent] = deque(maxlen=200)

    def start(self) -> None:
        """启动后台 watchdog 线程。"""
        self._is_running = True
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()
        logger.info("节点状态监测线程已启动（被动接收心跳模式）")

    def stop(self) -> None:
        self._is_running = False

    # ---- gRPC Heartbeat Handler 调用此方法上报心跳 ----
    def record_heartbeat(self, status_code: int, infer_count: int = 0) -> None:
        """
        检测节点 心跳处理入口（供 gRPC Heartbeat handler 调用）。

        参数:
            status_code: proto NodeStatus 枚举整数值（来自 HeartbeatRequest.status）
            infer_count: 检测节点 累计推理次数

        开销极小：仅更新时间戳 + 推断状态 + 写环形缓冲。
        """
        now = time.time()
        with self._status_lock:
            self._last_heartbeat_ts = now
            self._heartbeat_count += 1
            # 推断 NodeStatus（从整数值映射）
            new_status = self._infer_status(status_code)
            old_status = self._current_status
            self._current_status = new_status
            event = HeartbeatEvent(
                timestamp_ms=int(now * 1000),
                status=new_status,
                infer_count=infer_count,
            )
            self._event_ring.append(event)

        # 仅在状态翻转时触发回调 + 记录 JSONL
        if old_status != new_status:
            self._record_status_transition(old_status, new_status, now)
            if new_status == NodeStatus.OFFLINE:
                self._on_lost()
            elif old_status == NodeStatus.OFFLINE and new_status != NodeStatus.OFFLINE:
                self._on_recovered()

    def _infer_status(self, status_code: int) -> NodeStatus:
        """将 proto NodeStatus 枚举整数值映射为 NodeStatus。"""
        mapping = {
            0: NodeStatus.UNKNOWN,     # NODE_STATUS_UNKNOWN
            1: NodeStatus.ONLINE,      # NODE_STATUS_IDLE
            2: NodeStatus.BUSY,       # NODE_STATUS_BUSY
            3: NodeStatus.OFFLINE,     # NODE_STATUS_SHUTTING_DOWN
        }
        return mapping.get(status_code, NodeStatus.ONLINE)

    # ---- 后台 watchdog：超时判定 + 持久化 ----
    def _watchdog_loop(self) -> None:
        """
        低开销 watchdog：每秒检查一次心跳是否超时。
        不做任何网络 I/O；仅读本地变量 + 时间比较。
        """
        while self._is_running:
            time.sleep(1.0)
            with self._status_lock:
                elapsed = time.time() - self._last_heartbeat_ts
                if (self._heartbeat_count > 0 and
                        elapsed > self._heartbeat_timeout_s and
                        self._current_status != NodeStatus.OFFLINE):
                    old = self._current_status
                    self._current_status = NodeStatus.OFFLINE
                    self._record_status_transition(old, NodeStatus.OFFLINE, time.time())
                    # 锁外触发回调（避免死锁）
                    self._on_lost()

    # ---- 持久化记录 ----
    def _record_status_transition(self, old: NodeStatus, new: NodeStatus,
                                  ts: float) -> None:
        """
        将状态变更事件写入 JSONL 持久化日志。

        写入格式（每行一个 JSON 对象，便于后续 ELK / Pandas 解析）：
        {"ts": 1711234567.890, "level": "INFO", "node": "A",
         "event": "status_change", "from": "online", "to": "offline",
         "heartbeat_count": 42, "reason": "timeout"}
        """
        import json
        record = {
            "ts": round(ts, 3),
            "level": "INFO",
            "node": "A",
            "module": "node_monitor",
            "event": "status_change",
            "from": old.value if hasattr(old, 'value') else str(old),
            "to": new.value if hasattr(new, 'value') else str(new),
            "heartbeat_count": self._heartbeat_count,
        }
        try:
            with open(self._status_log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError as e:
            logger.warning(f"状态记录写入失败: {e}")

    def is_online(self) -> bool:
        """当前是否在线。"""
        with self._status_lock:
            return self._current_status in (NodeStatus.ONLINE, NodeStatus.BUSY)

    def get_current_status(self) -> NodeStatus:
        with self._status_lock:
            return self._current_status

    def get_event_ring(self) -> list:
        """返回最近的心跳事件环形缓冲（供 UI 实时状态面板）。"""
        with self._status_lock:
            return list(self._event_ring)
```

> **被动接收 vs 主动 probe 的架构变化说明**：
>
> | 对比项 | 旧架构 (probe) | 新架构 (心跳) |
> |--------|----------------|---------------|
> | 探测方向 | Node B 周期性主动连接 Node A | Node A 周期性主动向 Node B 推送 |
> | 网络开销 | 每 N 秒 1 次 TCP connect + TLS 握手 | 每 N 秒 1 次 gRPC unary call（已有 channel 复用，零握手开销） |
> | 云端扩展 | 随节点数线性增长（每个节点需独立探测） | 自然解耦（节点主动上报即可） |
> | 控制平面 | 反向：B 管 A 的生死 | 正向：A 自报家门、B 被动判定 |
> | 兜底 | 仅靠 probe | 超时兜底（连续 3 次心跳丢失 → 降级） |
>
> **例外场景**：启动阶段 Node B 仍调用 `probe()` 一次，确认 Node A 初始化成功；后续进入长期监听模式由心跳接管。

#### 3.8.3 启动引导

```python
"""system/bootstrap.py"""

import sys
import logging

import tkinter as tk

from common.config_loader import ConfigLoader, NetworkConfig
from system.self_check import run_self_check
from ui.app import RebarMeasureApp


def bootstrap() -> None:
    """
    启动引导：环境自检 → Tkinter 初始化 → 应用启动。
    任何硬故障直接 sys.exit(1)。
    """
    # 伪代码：
    # 1. 初始化日志系统（logging.basicConfig → logs/node_server.log）
    #
    # net_cfg = ConfigLoader.load_network_config("config/network.json")
    #
    # result = run_self_check(net_cfg)
    # if result.errors:
    #     for e in result.errors: logging.error(f"自检失败: {e}")
    #     sys.exit(1)
    #
    # if not result.peer_reachable:
    #     for w in result.warnings: logging.warning(f"自检警告: {w}")
    #     degraded = True  # 降级模式
    # else:
    #     degraded = False
    #
    # root = tk.Tk()
    # app = RebarMeasureApp(root)
    # if degraded: app.enable_degraded_mode()
    # app.start()
    # root.mainloop()
    pass
```

#### 3.8.4 系统服务管理脚本调用入口

`system/` 模块提供 `service-manager.sh` 的程序内部调用入口（脚本本身见 §9.1），供 GUI 设置页（§3.7.5）与 CLI 命令（§3.10 `service install/uninstall`）统一调用：

```python
"""system/service_manager.py"""

import subprocess
from typing import Tuple


class ServiceManager:
    """
    系统服务管理器：封装 scripts/service-manager.sh 的程序内部调用。
    所有方法均需 root/sudo 权限（见 §9.3）。
    """

    SCRIPT_PATH = "scripts/service-manager.sh"

    def install(self) -> Tuple[bool, str]:
        """注册系统服务并设置开机自启（等价 ./service-manager.sh install）。"""
        # 伪代码：
        # result = subprocess.run(
        #     ["sudo", "bash", self.SCRIPT_PATH, "install"],
        #     capture_output=True, text=True, timeout=30,
        # )
        # return result.returncode == 0, result.stdout
        return False, "not implemented"

    def uninstall(self) -> Tuple[bool, str]:
        """卸载系统服务并取消开机自启（等价 ./service-manager.sh uninstall）。"""
        # 伪代码：
        # result = subprocess.run(
        #     ["sudo", "bash", self.SCRIPT_PATH, "uninstall"],
        #     capture_output=True, text=True, timeout=30,
        # )
        # return result.returncode == 0, result.stdout
        return False, "not implemented"
```

> **入口等价性**：`ServiceManager.install()` / `uninstall()` 与直接执行 `./service-manager.sh install/uninstall` 语义完全等价；GUI 设置页与 CLI 命令均通过本类调用，避免直接 `subprocess` 散落各处。详见 §9.2。

### 3.9 程序入口（`main.py`）

```python
"""服务节点 程序入口。"""

from system.bootstrap import bootstrap


if __name__ == "__main__":
    bootstrap()
```

### 3.10 CLI 完整命令集

#### 3.10.1 定位与功能范围

CLI（命令行接口）作为 GUI 之外的**备用交互手段**，主要面向**紧急情况**（如 GUI 桌面环境不可用、远程 SSH 运维、自动化脚本调用）与 Linux 终端运维场景。

| 维度 | 说明 |
|------|------|
| 定位 | GUI 的备用补充；正常运维优先使用 GUI |
| 运行环境 | Linux 终端（openEuler），通过 SSH 或本地 tty 访问 |
| 功能范围 | **除图像强相关功能外**的所有功能均可 CLI 操作（图像显示、画面渲染、可视化结果查看等不在 CLI 范围） |
| 实现库 | `argparse` 标准库（零额外依赖），与现有 `CliFusionMenu`（`input()`/`print()` 风格）一致 |
| 风格 | 命令格式遵循 `argparse` 子命令风格；交互菜单沿用 `CliFusionMenu` 的 `input()`/`print()` 模式 |

> **与 §3.7.1 的关系**：现有 `CliFusionMenu`（§3.7.1）仅覆盖激光融合策略切换；本节 CLI 命令集是其超集，统一以 `argparse` 组织子命令。`CliFusionMenu` 作为融合策略切换的一种实现，仍可独立运行。

#### 3.10.2 命令清单

| 子命令 | 功能 | 说明 |
|--------|------|------|
| `infer` | 启动识别 | 触发单帧推理（等价 GUI 点击识别按钮）；受 `inference_interval_seconds` 门控 |
| `config` | 切换配置 | 在线切换运行时配置项（测距值选取、显示方式、保存方式等，等价设置副页面） |
| `status` | 查看状态 | 查询当前系统状态（推理端在线/离线、降级模式、激光有效性、摄像头接入方式等） |
| `exit` | 退出程序 | 触发 CLI 紧急退出路径（见 §0.5），best-effort 通知检测节点 后立即退出 |
| `service install` | 注册系统服务 | 调用 `scripts/service-manager.sh` 注册开机自启（需 root） |
| `service uninstall` | 卸载系统服务 | 调用 `scripts/service-manager.sh` 卸载开机自启（需 root） |
| `camera list` | 列出摄像头 | 列出当前扫描到的本地/网络摄像头 |
| `camera connect` | 连接摄像头 | 按索引或 IP:端口连接指定摄像头 |
| `fusion` | 切换融合策略 | 切换 4 路激光融合策略（等价 `CliFusionMenu`） |

#### 3.10.3 命令格式规范与帮助示例

命令格式：`python main.py <subcommand> [options]`

```
$ python main.py --help
usage: main.py [-h] {infer,config,status,exit,service,camera,fusion} ...

服务节点 钢筋测量系统 CLI

positional arguments:
  {infer,config,status,exit,service,camera,fusion}
    infer               触发单帧识别
    config              切换运行时配置
    status              查看系统状态
    exit                退出程序
    service             注册/卸载系统服务
    camera              摄像头管理
    fusion              切换激光融合策略

options:
  -h, --help            show this help message and exit

$ python main.py infer --help
usage: main.py infer [-h]

触发单帧识别（受 inference_interval_seconds 门控）。

options:
  -h, --help  show this help message and exit

$ python main.py config --help
usage: main.py config [-h] {fusion,display,save} ...

切换运行时配置：
  config fusion --strategy {mean,median,max,min,specific,manual} [--index N] [--value V]
  config display --mode {overlay,panel}
  config save --csv {on,off} --mask {on,off} --vis {on,off}

$ python main.py service install
[INFO] 注册系统服务... [OK] 已注册开机自启
```

#### 3.10.4 CLI 与 GUI 状态同步机制

CLI 与 GUI 共享同一份运行时状态（`threading.Lock` 保护），CLI 操作后 GUI 须实时响应：

```
CLI 线程 ──修改共享状态──→ (Lock 保护)
                              │
                              ▼
                  root.after(0, ...) 调度
                              │
                              ▼
                    GUI 主线程刷新显示
```

| 同步场景 | 共享状态变更 | GUI 响应 |
|----------|--------------|----------|
| `config fusion` 切换融合策略 | `LaserFusion._strategy` | 下一帧画面测距值更新 |
| `config display` 切换显示模式 | 显示模式标志 | 主画面布局切换（叠加↔底栏） |
| `camera connect` 连接摄像头 | 当前 `BaseCameraInput` 实例 | 主界面切到识别工作状态 |
| `exit` 退出 | `_is_running=False` | 触发 §0.5 退出流程 |

> **线程安全**：CLI 在独立线程运行，所有共享状态变更经 `threading.Lock` 保护；GUI 刷新一律经 `root.after(0, ...)` 调度到主线程，子线程禁止直接操作 widget（沿用 §4.2 约定）。

### 3.11 后台处理调度中枢

本章节显式定义服务节点 后台处理的五个子模块职责。这些子模块是对现有 §3.5（gRPC 客户端）、§3.6（测量模块）、§3.8（系统模块/NodeMonitor）职责的**集中表述与补充**，并非新引入的代码模块；其实现散落于上述既有模块中，此处统一编排以确保职责无遗漏、无重叠。

| 子模块 | 职责 | 对应现有模块 | 关键行为 |
|--------|------|--------------|----------|
| ① 状态记录维护 | 实时记录推理端（检测节点）在线/离线/异常状态 | §3.8 `NodeMonitor` | 接收心跳 + 捎带状态 + Shutdown 通知 → 维护状态缓存；超时判定降级；状态变更写日志 |
| ② 逻辑判定 | 系统级决策中枢 | §3.7 / §3.8（散落） | 是否允许推理（间隔门控 + 降级 + 副页面占用 + 激光有效性）；测距值选取计算（融合策略）；退出流程判定（S→D 退出指令触发条件） |
| ③ 基于推理端的测量计算 | 综合 RGB + 拍摄距离 + 类别掩码计算最终结果 | §3.6 `RebarMeasure` | 接收推理返回的类别掩码（PNG 编码、单通道 0/1/2）→ mm/px 换算 → 逐根钢筋直径提取 + 国标对齐 + 间距计算 → 输出 `MeasureResult` |
| ④ RGB 画面数据显示转发 | 接收相机 RGB → 转发 GUI 显示；推理时截帧发推理端 | §3.3 + §3.7 主循环 | 实时预览转发（本地/网络摄像头 RGB → GUI）；用户发起推理时截取当前帧编码为 JPEG 发检测节点（S→D Infer） |
| ⑤ 数据接收汇总记录追踪 | 对每推理帧建立完整数据关联索引 | 新增（补充） | 维护 `RGB原图 ↔ 拍摄距离 ↔ 类别掩码 ↔ 可视化结果图 ↔ CSV数值表` 一一对应可追溯索引 |

#### 3.11.1 状态记录维护

实时记录推理端（检测节点）的在线/离线/异常状态，作为逻辑判定与 UI 提示的依据。实现对应 §3.8 `NodeMonitor`：

- 接收检测节点 心跳（D→S Heartbeat RPC）+ 推理响应捎带状态（InferResponse.status）+ 退出通知（D→S Shutdown RPC）
- 维护节点状态缓存：`NODE_STATUS_IDLE` / `NODE_STATUS_BUSY` / `NODE_STATUS_SHUTTING_DOWN`
- 超时判定：连续丢失心跳达阈值（默认 3 次 / 15 秒）→ 标记离线 → 触发降级
- 状态变更写日志：`时间戳 | 级别 | S | grpc_server | 检测节点状态变更: IDLE→OFFLINE`

#### 3.11.2 逻辑判定

系统级决策中枢，判定各类操作的执行条件：

| 判定项 | 判定条件 | 不满足时行为 |
|--------|----------|--------------|
| 是否允许推理 | 推理服务在线（非降级）+ 间隔门控通过 + 无副页面占用 + 激光有效 | 弹提示并阻止请求发出 |
| 测距值选取计算 | 4 路激光有效性检查通过 + 融合策略有效 | 全无效则 `None`，当前帧测量不可执行 |
| 退出流程判定 | B 退出触发 + 检测节点 在线状态检查 | 在线则发 S→D 退出指令；离线则直接退出 |

#### 3.11.3 基于推理端的测量计算

综合三要素（RGB + 拍摄距离 + 类别掩码）计算最终测量结果，对应 §3.6 `RebarMeasure`：

```
类别掩码(0/1/2, PNG编码) + 拍摄距离(mm) + 内参(fx)
        │
        ▼
mm/px = 拍摄距离 ÷ fx
        │
        ▼
RebarMeasure.measure(mask, distance_mm, rgb_frame)
        │
        ▼
MeasureResult(钢筋列表 + 直径 + 间距 + 可视化帧)
```

> **类别掩码约束**：label_mask 为 PNG 编码、单通道、像素值 0/1/2（0=背景 / 1=纵向钢筋 / 2=横向钢筋），与检测节点 推理输出一致。`frame_id` 与 `timestamp_ms` 由服务节点 生成、检测节点 原样回传、服务节点 校验一致性（帧同步见 §4.4）。

#### 3.11.4 RGB 画面数据显示转发

负责 RGB 数据的接收、显示转发与推理截帧，贯穿主循环（§3.7 `_main_loop`）与推理触发（§3.7 `_request_measurement`）：

| 模式 | 行为 |
|------|------|
| 实时预览转发 | 接收本地/网络摄像头 RGB 帧 → 经渲染器叠加 OSD/测距 → 转发 GUI 主线程显示 |
| 推理截帧 | 用户发起推理时，截取当前 RGB 帧编码为 JPEG → 经 gRPC 发送检测节点（S→D Infer） |

> 该子模块是相机输入（§3.3）与 GUI 显示（§3.7）之间的数据桥接，确保实时预览与推理截帧共用同一帧源。

#### 3.11.5 数据接收汇总记录追踪

对每一次推理帧建立完整的数据关联索引，确保 `RGB原图 ↔ 拍摄距离 ↔ 类别掩码 ↔ 可视化结果图 ↔ CSV数值表` 五者一一对应可追溯：

| 关联项 | 来源 | 存储形态 |
|--------|------|----------|
| RGB 原图 | ④ 截帧（推理时锁定帧） | 内存帧 + `result.jpg` 落盘 |
| 拍摄距离 | ② 测距值选取计算（融合结果） | `report.csv` 字段 + 内存索引 |
| 类别掩码 | 检测节点 推理返回（PNG 解码后 0/1/2） | `ClassMask.png` 落盘 + 内存索引 |
| 可视化结果图 | ③ 测量计算生成（叠加标注帧） | `result.jpg` 落盘 |
| CSV 数值表 | ③ 测量结果结构化导出 | `report.csv` 落盘 |

```
帧索引(frame_id)
   ├── RGB 原图 ──────→ result.jpg
   ├── 拍摄距离 ──────→ report.csv[distance]
   ├── 类别掩码 ──────→ ClassMask.png
   ├── 可视化结果图 ──→ result.jpg (含标注)
   └── CSV 数值表 ───→ report.csv (直径/间距)
```

> **可追溯性**：通过 `frame_id` 主键可从任一关联项回溯到该次推理的全部数据，便于异常排查与结果复核。该索引同时供推理结果副页面（§3.7.5）的历史记录列表查询。

---

## 4. 并发与线程模型

### 4.1 线程总览

服务节点 共使用 **5 类线程**（不含 Tkinter 主线程，不含推理临时工作线程）：

| 线程 | 数量 | 驻留周期 | 职责 | 守护线程 |
|------|------|----------|------|----------|
| 主线程 (Main) | 1 | 全程 | Tkinter 事件循环、UI 渲染、测量绘制 | 否 |
| 相机读取线程 | 1 | 从 `open()` 到 `close()` | 后台读帧 + 断线重连 | 是 |
| 串口读取线程 | 1 | 从 `open_port()` 到 `close()` | 后台读串口 + 帧解析 | 是 |
| 主循环线程 | 1 | 从 `start()` 到 `_on_closing()` | 视频预览刷新 30fps | 是 |
| CLI 菜单线程 | 1 | 全程 | 终端交互式融合策略选择 | 是 |
| 节点状态监测线程 | 1 | 全程（启动心跳后） | 监听心跳超时，维护节点状态缓存，写状态日志 | 是 |

注：用户点击"测量"按钮触发的推理 + 测量任务为**临时工作线程**（daemon），完成后销毁。

```
Main Thread (tkinter)
  ├── UI 事件处理
  ├── root.after(0, ...) 调度回调  ←── 子线程投递
  ├── 画面渲染与绘制
  └── 钢筋结果列表刷新

Camera Read Thread                     Serial Read Thread
  ├── 持续 read() 帧                     ├── 持续 read() 串口
  ├── 更新 _latest_frame (lock)          ├── SerialBuffer.append()
  └── 断线自动 try_reconnect()           ├── LaserDataParser.parse()
                                          └── 更新 _latest_result (lock)

Measure Worker Thread (temporary)       Main Loop Thread
  ├── gRPC Infer()                       ├── 取最新帧 (lock)
  ├── 帧同步校验                         ├── 取最新激光值 (lock)
  ├── RebarMeasure.measure()             ├── 融合距离
  └── root.after(0, refresh)             ├── UIRenderer.compose()
                                         └── root.after(0, update_label)

Node Status Monitor Thread             CLI Menu Thread
  ├── 接收 Node A 心跳 (被动)           ├── input() / print()
  ├── 心跳超时判定                      └── LaserFusion.set_strategy()
  ├── 维护 NodeState 缓存
  └── 触发 on_lost / on_recovered 回调
```

### 4.2 共享状态与锁策略

| 共享变量 | 类型 | 保护锁 | 读方 | 写方 |
|----------|------|--------|------|------|
| `_latest_frame` | `np.ndarray` | `_frame_lock` | 主循环线程、测量线程 | 相机读取线程 |
| `_latest_result` | `LaserParseResult` | `_data_lock` | 主循环线程、测量线程 | 串口读取线程 |
| `_degraded_mode` | `bool` | GIL（Python bool 原子性） | 主循环线程、测量线程 | 健康检查线程回调（via after） |
| `_latest_mask` | `np.ndarray` | GIL | 主循环线程 | 测量回调线程 |
| `_latest_measure` | `MeasureResult` | GIL | 主循环线程 | 测量回调线程 |

**锁使用原则**：
- 仅相机帧与激光数据因持续高频更新需独立 Lock
- 测量结果与掩码由单次触发更新、`root.after(0, ...)` 调度读取，利用 GIL 保证 Python 字节码级安全
- Lock 持有时间严格最小化，禁止在 Lock 内执行 IO 或耗时计算

### 4.3 线程安全规则

1. **tkinter 主线程唯一性**：所有 widget 操作（`Label.config()`、`Treeview.insert()`、`messagebox.show*()`）必须在主线程执行。子线程通过 `root.after(0, callback)` 调度
2. **禁止跨模块直接状态读写**：`FusionDisplayApp` 过去直接访问 `camera_manager.latest_frame`、`serial_manager.latest_data` — 重构后必须通过公共接口（`get_frame()`、`get_latest()`）且内部加锁返回拷贝
3. **daemon 线程**：除主线程外均为 daemon，确保主线程退出时所有子线程自动终止
4. **资源释放顺序**：停止标志 → join 工作线程 → 释放设备资源。关闭窗口时先设 `_is_running = False`，再 join，最后 `root.destroy()`

### 4.4 帧同步实现

```
用户触发测量
  ↓
生成 frame_id = ++self._frame_id
获取 rgb_frame = camera.get_rgb_frame()
获取 laser_result = serial_mgr.get_latest()
融合 distance = fusion.fuse(laser_result)    ← 使用 laser_result 的时间戳与 rgb_frame 的时间戳对齐
                                   （原型阶段：认为同一 cycle 内采到的帧与激光值"近似同步"）
  ↓
gRPC Infer(frame_id, distance)
  ↓
收到 InferResponse:
  if response.frame_id != self._frame_id: 丢弃（迟到响应）
  if response.timestamp_ms != self._request_timestamp_ms: 丢弃
  else: 执行测量
```

**超时控制**：gRPC 客户端设置 `deadline`（默认 5000ms）。超时后跳过当前帧，不阻塞采集主循环。

---

## 5. 配置加载与启动自检流程

### 5.1 启动流程总览

> **顺序约束（强制要求）**：**第一步读取加载 JSON 配置 → 第二步环境权限检查**，二者顺序不可颠倒。理由：环境权限检查项（本机 IP 校验、端口占用、对端可达性等）依赖配置文件中的参数（`local_ip`、`grpc_port` 等），配置未加载则检查无依据。本顺序与 §0.1 启动阶段约定一致。

```
main.py
  ↓
bootstrap.bootstrap()
  ↓
初始化日志系统 (→ logs/node_server.log)
  ↓
【第一步】读取加载 JSON 配置
  ├── 加载 network.json（网络参数：local_ip / remote_ip / grpc_port / 心跳参数）
  ├── 加载 camera.json（相机参数 + 兼容白名单）
  ├── 加载 intrinsics.json（相机内参 fx/fy/cx/cy）
  ├── 加载 inference.json（推理节奏 inference_interval_seconds）
  └── 加载其他配置（如 logging.json 轮转参数、service.json 服务参数）
  ↓
【第二步】环境权限检查 (run_self_check(net_cfg))
  ├── check_admin_privilege()  ──硬故障─┐
  ├── check_local_ip()         ──硬故障─┤
  ├── check_port_available()   ──硬故障─┼→ sys.exit(1)
  ├── check_network_config_privilege() ──软/硬──→ 告警或退出
  └── check_peer_reachable()   ──软故障─→ degraded = True
  ↓
初始化 Tkinter root
  ↓
RebarMeasureApp(root)
  ├── 初始化全部子模块（配置已在上文加载）
  └── _setup_ui()
  ↓
app.start()
  ├── 异步扫描 → 打开相机
  ├── gRPC probe → 连接（如可达）
  ├── 异步扫描 → 打开串口
  ├── 启动主循环线程
  ├── 启动 CLI 线程
  ├── 启动 NodeMonitor 线程（被动接收心跳 + 超时降级）
  └── 若有 degraded (probe 失败) → UI 显示黄色横幅
  ↓
root.mainloop()
```

### 5.2 自检检查项与失败处理

| 检查项 | 检查逻辑 | 硬/软故障 | 失败处理 |
|--------|----------|-----------|----------|
| 管理员权限验证 | 检查当前进程是否以 `root`/`sudo` 启动（`os.geteuid() == 0`） | **软**（条件性） | 仅配置加载 / 端口绑定等真正需特权时才影响；原型阶段非 root 记 WARNING 不退出 |
| 本机 IP 校验 | 获取网卡实际 IP 与 `network.json.local_ip` 对比（Windows bypass） | **诊断** | 记录 `[诊断]` 日志，**不退出**，交由 NetworkMonitor 持续监测 |
| 端口可用性 | 尝试 bind `0.0.0.0:grpc_port` | **诊断** | 记录 `[诊断]` 日志，**不退出**（本地功能正常，推理可能受限） |
| 网络配置权限 | 检查应用层是否具备读取/校验网络配置的权限（如读取网卡 IP、bind 端口等） | **软** | 记录 WARNING 日志 + 告警提示"网络配置权限缺失，部分检查可能不可靠" + 继续 |
| 对端可达性 | TCP 探测 `remote_ip:grpc_port`（2 秒超时） | **软** | 记录 WARNING 日志 + 进入降级模式 + UI 黄色横幅提示；由 NetworkMonitor 后台周期探测恢复 |

> **说明**：本次迭代将原"硬故障"（本机 IP 不匹配 / 端口冲突 / 非 root）**全部放开为诊断/软故障**，不再 `sys.exit(1)` 退出，由运行时 `NetworkMonitor` 动态监测接管（见 §4.3 与 Design-NodeA 服务端对应）。仅配置文件加载失败等真正致命错误才退出。

### 5.3 降级模式行为

当检测节点 不可达（网络异常 / 启动自检失败 / 运行中健康检查检测到丢失）时，进入降级模式：

| 能力 | 正常模式 | 降级模式 |
|------|----------|----------|
| 视频预览 | 有（30fps） | 有（30fps） |
| 激光数据显示 | 有 | 有 |
| 本地测量/显示/保存 | 有 | **有**（本地功能不受网络影响） |
| AI 推理 | 有（gRPC 到检测节点） | 无（识别按钮禁用） |
| 钢筋直径测量（依赖掩码） | 推理 + 测量 | 无新推理（提示"推理服务离线"） |
| 融合策略选择 | CLI 可用 | CLI 可用 |
| 恢复机制 | — | **NetworkMonitor / 健康检查自动恢复**：对端网络恢复后周期探测命中，自动切回在线模式（无需重启） |

### 5.4 平台兼容处理

| 平台 | IP 校验 | 端口校验 | 相机后端 | 串口 |
|------|---------|----------|----------|------|
| openEuler 生产 | 严格执行 | 严格执行 | CAP_V4L2 | `/dev/ttyUSB*` |
| Windows 开发 | bypass（仅 warning） | 严格执行 | CAP_DSHOW | `COM*` |

---

## 6. 日志与异常处理策略

### 6.1 日志规范

严格遵循 AGENTS.md §7.5 "一机一日志"规范：

```python
"""system/logging_setup.py"""

import logging
import os
from datetime import datetime


def setup_logging(log_path: str = "logs/node_server.log") -> logging.Logger:
    """
    初始化服务节点 全局日志系统。
    输出格式: 时间戳(毫秒) | 级别 | 节点 | 模块 | 消息
    """
    # 伪代码：
    # os.makedirs(os.path.dirname(log_path), exist_ok=True)
    #
    # formatter = logging.Formatter(
    #     fmt="%(asctime)s.%(msecs)03d | %(levelname)-8s | %(node)s | %(module)s | %(message)s",
    #     datefmt="%Y-%m-%d %H:%M:%S"
    # )
    #
    # handler = logging.FileHandler(log_path, encoding="utf-8")
    # handler.setFormatter(formatter)
    #
    # console_handler = logging.StreamHandler()
    # console_handler.setFormatter(formatter)
    #
    # logger = logging.getLogger("node_server")
    # logger.setLevel(logging.INFO)  # 原型阶段默认 INFO
    # logger.addHandler(handler)
    # logger.addHandler(console_handler)
    #
    # return logger
    return logging.getLogger("node_server")
```

### 6.2 日志模块命名空间

按照 §7.5，双节点统一模块标识符：

| 模块标识 | 使用位置 | 示例日志 |
|----------|----------|----------|
| `camera` | `camera/` | "336L 设备打开成功: 1920x1080@30fps" |
| `serial` | `serial/` | "S21C 帧解析成功: 4路距离=[812,815,810,813]mm" |
| `grpc_client` | `grpc_client/` | "推理请求 frame_id=42, deadine=5000ms" |
| `measure` | `measure/` | "测量完成: 3 根钢筋, 直径 = [12,12,12]mm" |
| `ui` | `ui/` | "用户触发测量 / 融合策略切换为 MEAN" |
| `system` | `system/` | "启动自检: 本机 IP 校验通过" |
| `laser_fusion` | `serial/laser_fusion.py` | "4 路有效=[812,815,810,813], 均值=812.5mm" |

### 6.3 各级别日志触发条件

| 级别 | 触发场景 | 示例 |
|------|----------|------|
| DEBUG | 开发调试（帧序列号、时间戳细节） | "frame_id=42 采集时刻=1690700000123ms" |
| INFO | 正常流程节点 | "相机打开成功"、"推理完成: 3根钢筋" |
| WARNING | 可恢复异常 / 数据质量下降 | "激光通道 2 零值剔除"、"对端可达性检查失败: 进入降级模式" |
| ERROR | 不可恢复异常 / 功能无法执行 | "4 路激光全无效"、"gRPC 调用超时" |
| CRITICAL | 程序即将退出 | "本机 IP 不匹配: 配置=192.168.10.1 actual=192.168.0.50" |

### 6.4 异常分类与处理

#### 6.4.1 异常分类

| 异常类别 | 来源 | 是否退出 | 处理策略 |
|----------|------|----------|----------|
| 环境配置异常 | 本机 IP 不匹配、端口被占用 | **是** | `self_check` → `sys.exit(1)` |
| 推理服务离线 | gRPC 超时、对端不可达 | 否（降级） | 日志 WARNING + 降级模式 + UI 提示 |
| 激光数据全无效 | 4 路全无效 | 否 | 日志 ERROR + 界面"激光测距无效" |
| 相机断线 | VideoCapture.read() 持续失败 | 否 | 后台自动重连；UI 提示"摄像头断线" |
| 串口断线 | SerialException | 否 | 后台自动重连；UI"重连串口"按钮启用 |
| 测量无结果 | 无有效连通域 | 否 | 日志 INFO + UI"未检测到钢筋" |
| 帧同步失败 | frame_id 不匹配 | 否 | 日志 WARNING + 跳过当前帧 |

#### 6.4.2 异常处理模板

```python
# 在 camera/orbbec_336l.py _read_loop 中:
#   except (cv2.error, OSError) as e:
#       logger.error("camera", f"读帧异常: {e}, 尝试重连")
#       self._is_opened = False
#       self.try_reconnect()

# 在 grpc_client/inference_client.py infer 中:
#   except grpc.RpcError as e:
#       logger.error("grpc_client", f"gRPC 调用失败: {e.code()}")
#       return None

# 在 serial/serial_manager.py _read_loop 中:
#   except (serial.SerialException, OSError) as e:
#       logger.error("serial", f"串口异常: {e}")
#       self._is_running = False
#       break  # 由主循环线程检测后触发重连

# 在 ui/app.py _do_measurement 中:
#   except Exception as e:
#       logger.error("ui", f"测量执行异常: {e}")
#       self._root.after(0, lambda: messagebox.showerror("错误", str(e)))
```

### 6.5 关键数据日志格式（双节点对齐示例）

为确保跨节点比对调试时间线可对齐，以下为关键日志的标准格式：

```text
2026-07-30 14:23:01.125 | INFO  | S | system     | 启动自检: 本机 IP 校验通过 [192.168.10.1]
2026-07-30 14:23:01.456 | INFO  | S | grpc_client| gRPC 连接成功: 192.168.10.2:50051
2026-07-30 14:23:02.001 | INFO  | S | camera     | 336L 设备打开成功: 1920x1080@30fps
2026-07-30 14:23:02.345 | INFO  | S | serial     | S21C 帧解析成功: 4路距离=[812,815,810,813]mm
2026-07-30 14:23:02.346 | INFO  | S | laser_fusion| 4路有效=[812,815,810,813], 融合策略=MEAN, 结果=812.5mm
2026-07-30 14:23:05.789 | INFO  | S | ui         | 用户触发测量: frame_id=1
2026-07-30 14:23:05.790 | INFO  | S | grpc_client| 推理请求: frame_id=1, distance=812.5ms, jpeg_size=245678B
2026-07-30 14:23:07.123 | INFO  | S | grpc_client| 推理响应: frame_id=1, mask_size=480x640, 耗时=1333ms
2026-07-30 14:23:07.456 | INFO  | S | measure    | 测量完成: 找到 3 根钢筋, 直径(mm)=[12.0,12.0,12.0], 间距(mm)=[150.2,149.8]
2026-07-30 14:23:08.001 | WARNING| S | laser_fusion| 通道 2 零值剔除, 有效路数=3/4
2026-07-30 14:23:15.002 | ERROR | S | laser_fusion| 4 路激光全无效: raw=[0,0,0,0], 当前帧测量不可执行
2026-07-30 14:23:15.003 | WARNING| S | system     | 健康检查: 对端 192.168.10.2:50051 不可达, 进入降级模式
```

### 6.6 日志文件管理

- `logs/node_server.log` 为服务节点 全局运行日志（一机一日志，对齐 AGENTS.md §7.5）
- 采用最小轮转策略（见 §6.7），避免长时间运行导致日志文件膨胀
- 关闭程序时 flush 并关闭 handler
- 异常退出时 logging.shutdown() 确保落盘

### 6.7 日志轮转策略

> 原型阶段不引入 logrotate 等外部工具，但约定最小轮转策略，避免长时间运行导致 `node_server.log` 无限膨胀。

| 配置项 | 默认值 | 配置位置 | 说明 |
|--------|--------|----------|------|
| 单文件大小上限 | 10 MB | `config/` 下 JSON（如 `config/logging.json`，禁止硬编码） | 达到上限时触发滚动归档 |
| 保留归档份数 | 3 | 同上 | 保留最近 N 个归档文件，超出后删除最旧 |

**滚动归档规则**：

```
达到上限时：
node_server.log        → node_server.log.1
node_server.log.1      → node_server.log.2
node_server.log.2      → node_server.log.3
node_server.log.3      → 删除（超出保留份数）
新建空 node_server.log 继续写入
```

| 实现要素 | 说明 |
|----------|------|
| 实现库 | Python 标准库 `logging.handlers.RotatingFileHandler`（零额外依赖，对齐 AGENTS.md §7.1 配置 JSON 与标准库优先原则） |
| 滚动触发 | 单文件写入字节数达 `maxBytes` 时自动滚动 |
| 归档保留 | `backupCount` 控制保留份数，超出自动删除最旧 |
| 配置化 | `maxBytes` / `backupCount` 从 `config/` 下 JSON 读取，禁止硬编码 |

```python
# 伪代码：logging_setup.py 中初始化 RotatingFileHandler
# from logging.handlers import RotatingFileHandler
# handler = RotatingFileHandler(
#     filename="logs/node_server.log",
#     maxBytes=cfg.max_bytes,        # 默认 10 * 1024 * 1024 = 10MB
#     backupCount=cfg.backup_count,  # 默认 3
#     encoding="utf-8",
# )
# handler.setFormatter(formatter)
```

> **退出告警写入保证**：§10.5 退出时 A 异常告警须在 `logging.shutdown()` 之前写入日志；RotatingFileHandler 在程序退出时由 `logging.shutdown()` 自动 flush 并关闭，确保告警信息不丢失。

### 6.8 异常退出码约定

| 退出码 | 含义 |
|--------|------|
| 0 | 正常退出 |
| 1 | 环境自检硬故障（IP 不匹配 / 端口冲突） |
| 2 | Tkinter 初始化失败（无 DISPLAY / tkinter 未安装） |
| 3 | 配置文件加载失败（JSON 解析错误 / 缺少必需字段） |

---

## 7. 推理端状态监测与记录机制（Design-server.md）

> **全局架构规范**见 `AGENTS.md` §5.7。本章节仅定义**服务节点（服务端）**侧的监测、判定与记录实现。

### 7.1 机制定位与设计理念

检测节点（推理端）的生命体征监护仪。以**最小资源代价**接收检测节点 的实时状态反馈，维护权威的状态缓存，并提供持久化记录用于后续审计/追溯。

### 7.2 核心职责边界（强制解耦 - 服务端视角）

| 服务节点 负责 | 服务节点 不负责 |
|-------------|---------------|
| 被动接收检测节点 的心跳推送 | 不主动轮询检测节点（启动时一次性探测除外） |
| 超时判定：连续丢失 N 次心跳 → 离线 | 不干预检测节点 的生命周期 |
| 状态缓存维护（内存） | 不将状态缓存长期占用（仅在内存） |
| 状态变更持久化记录（JSONL 文件） | 不回放心跳到除 UI 和日志外的其他模块 |
| 触发降级 / 恢复回调 | 直接控制检测节点 的任何行为 |

### 7.3 被动监测流程

```
检测节点 端（HeartbeatSender 每 5 秒推送）
    ↓ (gRPC unary) over existing channel (端口 50051)
    ↓
服务节点 NodeMonitor.record_heartbeat() 入口
    ↓
1. 更新 last_heartbeat_ts + heartbeat_count
2. 推断 NodeStatus (idle→ONLINE / inferencing→BUSY / shutting_down→OFFLINE)
3. 比对新旧状态 → 翻转时触发回调 + 写 JSONL
4. (ONLINE/BUSY 时) 刷新 watchdog 计时器
    ↓
后台 watchdog 线程 (1 秒周期)
    ↓
1. 读取 last_heartbeat_ts → 计算 elapsed
2. elapsed > 15 秒 AND 心跳数 > 0 → 超时判定
3. 状态翻转为 OFFLINE → 写 JSONL → 触发 on_lost() → UI 降级
    ↓
超时兜底：最终兜底手段，保障检测节点 崩溃/断电时可被检测
```

### 7.4 状态持久化记录机制（JSONL）

#### 7.4.1 记录文件与格式

| 持久化项 | 文件路径 | 格式 | 写入时机 |
|----------|----------|------|----------|
| 状态变更日志 | `logs/node_detect_status.jsonl` | JSONL（每行一个 JSON 对象） | 每次状态翻转 |
| 心跳采样（可选） | `logs/node_detect_heartbeat_sample.jsonl` | JSONL | 采样周期（如每 60 秒或每次推理响应后） |

#### 7.4.2 JSONL 记录示例

```jsonl
{"ts":1711234567.890,"level":"INFO","node":"A","module":"node_monitor","event":"status_change","from":"online","to":"busy","heartbeat_count":42}
{"ts":1711234572.901,"level":"INFO","node":"A","module":"node_monitor","event":"status_change","from":"busy","to":"online","heartbeat_count":43}
{"ts":1711234590.123,"level":"WARNING","node":"A","module":"node_monitor","event":"status_change","from":"online","to":"offline","heartbeat_count":46,"reason":"heartbeat_timeout"}
{"ts":1711234605.456,"level":"INFO","node":"A","module":"node_monitor","event":"status_change","from":"offline","to":"online","heartbeat_count":47,"reason":"heartbeat_received"}
```

#### 7.4.3 低开销持久化设计

| 设计要素 | 说明 |
|----------|------|
| **仅状态翻转时写入** | 心跳每 5 秒一次，但状态翻转远少，避免文件膨胀 |
| **JSONL 追加写** | 直接 `O_APPEND`，无需 read-modify-write，原子性由 OS 保证 |
| **单行 ≤ 200 字节** | 一条记录极小，单文件可存数十年心跳 |
| **无锁写入** | 单线程 watchdog 顺序写，无需额外锁机制 |
| **后续消费** | JSONL 天然支持 `pandas.read_json(lines=True)` / `jq` / ELK 摄入 |

### 7.5 UI 状态面板集成

服务节点 Tkinter UI 可新增**节点状态指示器**（显示在状态栏）：

```python
"""ui/components/node_status_indicator.py"""

# 伪代码：
# canv = Canvas(root, width=12, height=12)
# color = "green" if monitor.is_online() else "red"
# canv.create_oval(2, 2, 10, 10, fill=color, outline="")
#
# 每 1 秒通过 root.after(0, refresh_indicator) 刷新颜色 + tooltip 显示:
# "检测节点 在线 | 心跳 42 次 | 最近: 3 秒前"
```

### 7.6 超时降级与兜底策略

| 场景 | 触发条件 | 系统行为 |
|------|----------|----------|
| 检测节点 正常退出 | 收到 status=SHUTTING_DOWN 心跳 | 立即触发降级（不依赖超时） |
| 检测节点 异常退出（崩溃/断电） | 连续 15 秒未收到任何心跳 | 超时降级：UI 显示"推理服务离线" |
| 检测节点 网络闪断后恢复 | 心跳重新收到 | 恢复在线：UI 隐藏警告，恢复可用 |
| 启动阶段检测节点 不可达 | probe() 失败（一次性） | 启用降级模式（沿用 §4.3 自检逻辑） |

降级模式下系统行为：
- 界面叠加提示"推理服务离线 — 仅视频预览可用"
- 屏蔽"测量"按钮 + 显示 tooltip"检测节点 不可用"
- 心跳持续重试，恢复后自动清除降级
- 所有降级事件写入 `logs/node_detect_status.jsonl`（reason=degraded / recovered）

### 7.7 心跳参数配置化

| 参数 | 配置键 | 默认值 | 说明 |
|------|--------|--------|------|
| 心跳接收超时 | `config/network.json` → `heartbeat_timeout_s` | 15.0 秒 | 连续 3 次心跳丢失（3 × 5 秒） |
| 状态日志路径 | `config/network.json` → `node_status_log_path` | `logs/node_detect_status.jsonl` | 持久化路径 |
| 采样（可选） | `config/network.json` → `heartbeat_sample_interval_s` | 60 秒 | 采样周期，用于性能大盘 |

### 7.8 安全审计

持久化状态日志可用于：
- 故障复盘：检测节点 崩溃前最后一次心跳时间
- 性能监测：推理频率 + 周期异常检测
- 统计报表：在线率 Uptime % = 在线时长 / 总时长
- 远程运维接入：后续可对接 Prometheus `node_detect_up` 指标推送

---

## 8. 识别结果存储规范

> 本章节定义服务节点 推理结果的落盘规范，包括路径结构、文件命名、保存约束。存储行为由设置副页面（§3.7.5）的"推理结果保存方式"多选项控制，配置写回 `config/` 下 JSON（禁止硬编码）。

### 8.1 存储路径结构

每次成功推理并完成测量后，按以下路径结构保存结果：

```
./result/{YYYYMMDD-HHMMSS}/
    ├── result.jpg
    ├── report.csv
    └── ClassMask.png
```

| 属性 | 说明 |
|------|------|
| 根目录 | 统一保存于程序所在根目录的 `./result/`（相对路径，由程序启动位置决定） |
| 子目录命名 | 以系统时间戳命名，格式 `YYYYMMDD-HHMMSS`，精确到秒 |
| 命名示例 | `./result/20260731-080130/`（2026 年 7 月 31 日 08 时 01 分 30 秒） |
| 自适应创建 | 路径不存在时由程序自动 `os.makedirs(..., exist_ok=True)` 创建，禁止因目录缺失导致保存失败 |
| 同秒覆盖 | **同一秒内重复触发时强制用新结果覆盖旧结果**：相同时间戳目录已存在则直接覆写其中的同名文件，不报错、不追加序号 |

> **覆盖语义说明**：同秒覆盖策略确保"同一时间点只保留最新结果"，避免因目录命名冲突导致保存失败；同秒内若用户连续多次触发，仅最后一次结果保留完整，前面同秒内的中间态被覆盖。该策略符合原型阶段"按需单帧触发"的设计前提（默认 3 秒间隔下不会出现同秒冲突）。

### 8.2 文件命名与说明

| 文件名 | 格式 | 内容 | 保存策略 |
|--------|------|------|----------|
| `result.jpg` | JPEG | 可视化结果图（RGB 原图叠加掩码 + 直径/间距标注） | ✅ **强制默认保存**（无论用户如何设置保存方式均保存） |
| `report.csv` | CSV（UTF-8 with BOM） | 识别结果明细表（图片名称、钢筋编号、检测直径、相邻区间、检测间距） | 按用户设置（设置副页面"保存方式"勾选 CSV 时保存） |
| `ClassMask.png` | PNG（8-bit 单通道无损） | 类别掩码图（像素值 0/1/2，与检测节点 推理输出一致） | 按用户设置（设置副页面"保存方式"勾选"类别掩码图"时保存） |

**`report.csv` 列结构**（沿用 §3.6.2 `_export_csv`）：

| 列 | 说明 |
|----|------|
| 图片名称 | `result.jpg`（与同目录可视化结果图关联） |
| 钢筋编号 | H1, H2, H3, ... |
| 检测直径/mm | 就近对齐国标后的直径（取整）或原始测量值（1 位小数） |
| 相邻区间 | H1~H2 / H2~H3 / ...（首行无前驱时填 `-`） |
| 检测间距/mm | 相邻钢筋间距（mm） |

### 8.3 存储约束

| 约束 | 说明 |
|------|------|
| **可视化结果图强制保存** | `result.jpg` 无论用户在设置副页面如何勾选保存方式，都**强制默认保存**。理由：可视化结果图是用户复核测量结果最直观的载体，必须保留作为最低输出物。 |
| **类别掩码必须 PNG 无损** | `ClassMask.png` 必须以 PNG（8-bit 单通道无损）格式保存，**禁止使用 JPEG 等有损压缩格式**。理由：类别掩码像素值 0/1/2 具有语义含义（0=背景 / 1=纵向钢筋 / 2=横向钢筋），有损压缩会引入失真像素值导致后续分析错误。 |
| **保存方式配置化** | 保存方式（CSV / 类别掩码图 / 可视化结果图多选）由设置副页面（§3.7.5）控制，配置写回 `config/` 下 JSON 文件（禁止硬编码于代码中）。`result.jpg` 不参与多选配置，恒为保存。 |
| **路径不可硬编码** | `./result/` 根目录路径与子目录命名格式由配置文件管理，禁止在代码中硬编码绝对路径。 |
| **保存失败不阻塞主流程** | 落盘失败时记录 ERROR 日志 + UI 提示"结果保存失败"，但不影响主循环与下一次推理。 |

> **与 §3.11.5 数据接收汇总记录追踪的关系**：本章节定义物理落盘规范；§3.11.5 定义内存中 `RGB原图 ↔ 拍摄距离 ↔ 类别掩码 ↔ 可视化结果图 ↔ CSV数值表` 一一对应可追溯索引，两者共同构成完整的"内存索引 + 物理落盘"结果追溯体系。

---

## 9. 系统服务管理脚本

> 本章节定义服务节点 程序作为 Linux 系统服务（systemd）的管理脚本与程序内部调用能力，用于实现开机自启与运维卸载。

### 9.1 附带脚本文件 `scripts/service-manager.sh`

| 属性 | 说明 |
|------|------|
| 路径 | `scripts/service-manager.sh`（位于 `node_server/` 工程目录下） |
| 平台 | Linux（bash 脚本） |
| 底层机制 | systemd（`.service` unit 文件） |
| 调用方式 | `./service-manager.sh install` / `./service-manager.sh uninstall` |

**子命令清单**：

| 子命令 | 功能 | 说明 |
|--------|------|------|
| `install` | 一键注册为系统服务并设置开机自启 | 生成 `/etc/systemd/system/rebar-node-server.service`（INI 格式 unit 文件，由脚本从项目 JSON 参数源生成，符合 AGENTS.md §7.1 例外约定）→ `systemctl daemon-reload` → `systemctl enable rebar-node-server` → `systemctl start rebar-node-server` |
| `uninstall` | 一键卸载并取消开机自启 | `systemctl stop rebar-node-server`（若运行中）→ `systemctl disable rebar-node-server` → 删除 unit 文件 → `systemctl daemon-reload` |

> **systemd unit 文件格式说明**：systemd `.service` unit 文件因 OS 规范强制要求 INI 格式，是 AGENTS.md §7.1 "配置文件统一 JSON 格式"约定的**唯一例外**。本脚本生成的 unit 文件不直接作为项目参数输入源，而是由脚本从项目维护的 JSON 参数源（如 `config/service.json`，含 `ExecStart`、`WorkingDirectory`、`User` 等字段）转换生成，符合"项目侧 JSON 源 → 部署侧 INI 产物"的转换约定。

### 9.2 程序自身调用脚本能力

服务节点 程序内部可通过 `subprocess` 调用 `scripts/service-manager.sh`，实现与脚本相同的注册/卸载功能，并在以下两个入口暴露给用户：

| 入口 | 操作路径 | 说明 |
|------|----------|------|
| GUI 设置页 | §3.7.5 设置副页面 → "系统服务" 区域 | 提供"注册开机自启"与"卸载开机自启"两个按钮，点击后通过 `subprocess.run(["sudo", "bash", "scripts/service-manager.sh", "install/uninstall"])` 调用脚本；调用结果以弹窗反馈 |
| CLI 命令 | §3.10 `service install` / `service uninstall` | 在终端以 `python main.py service install` / `python main.py service uninstall` 触发，等价 GUI 操作 |

> **能力等价性**：GUI 入口、CLI 入口与直接执行脚本三种方式语义完全等价，均最终调用同一 `service-manager.sh`；区别仅在调用方与权限上下文（GUI 经 `sudo` 提权，CLI 直接以 root 运行或 sudo 调用）。

### 9.3 权限要求

| 操作 | 权限要求 | 说明 |
|------|----------|------|
| 注册系统服务（install） | `root` / `sudo` | 写入 `/etc/systemd/system/`、`systemctl enable`、`systemctl start` 均需 root 权限（与 §0.1 启动权限一致） |
| 卸载系统服务（uninstall） | `root` / `sudo` | `systemctl stop`、`systemctl disable`、删除 unit 文件均需 root 权限 |
| 普通运行（不调用脚本） | 无需 root | 仅当需要注册/卸载系统服务时才提权 |

> **权限校验**：脚本启动时首先校验当前用户是否为 root（`[ "$(id -u)" -ne 0 ]`），非 root 时打印提示并退出；程序内部 `subprocess` 调用时通过 `sudo` 提权，由系统密码策略处理授权。

---

## 10. 双退出流程

> 本章节定义服务节点 的两条退出路径与统一退出行为，包含 S→D 退出通知 RPC 设计、检测节点 退出行为基准、退出告警落盘约定。

### 10.1 退出方式

| 退出方式 | 触发条件 | 路径类型 | 行为特征 |
|----------|----------|----------|----------|
| **GUI 退出（正常）** | 用户点击 GUI 关闭按钮 / 退出菜单 | 正常退出流程 | 保存运行时状态 → 通知检测节点 → 等反馈 → 释放资源 → 退出 |
| **CLI 退出（紧急）** | 用户在终端执行 CLI 退出命令（§3.10 `exit`）/ 收到 `SIGTERM`/`SIGINT` | 紧急退出流程 | 跳过部分状态保存 → best-effort 通知检测节点 → 释放资源 → 立即退出 |

### 10.2 统一退出行为

两条退出路径共用统一退出行为（差异仅在状态保存的完整度）：

```
① 检查推理端（检测节点）服务是否在线
        ↓
② 若在线：先向检测节点 发送退出指令（S→D `RequestPeerShutdown`，见 §10.3）
          并在设定时限（deadline=3s）内等待推理端服务终止退出反馈
        ↓
   ┌──── 时限内收到反馈 ────→ ③ 正常退出（标记 A 已下线，释放资源）
   │
   └──── 时限内未收到反馈 ──→ ④ 仍然退出，但告警检测节点 异常
                                （记录 WARNING 日志 + 终端输出告警，见 §10.5）
        ↓
⑤ 若检测节点 离线：直接释放资源并退出（无需通知 A）
        ↓
⑥ 释放本地资源（关闭相机、串口、gRPC channel、日志 flush）
        ↓
⑦ 进程退出（退出码见 §6.8）
```

> **同步等待语义**：步骤 ② 的"等待反馈"为同步阻塞，但设 deadline=3s 上限，超时后立即转步骤 ④，不会无限阻塞退出流程。GUI 退出可在主线程同步执行；CLI 退出（紧急）若需立即退出可设更短 deadline 或跳过等待（best-effort）。

### 10.3 S→D 退出通知 RPC 设计（关键决策 D1）

服务节点 退出时向检测节点 发送的退出指令 RPC 设计如下：

| 属性 | 说明 |
|------|------|
| RPC 名称 | `RequestPeerShutdown`（暂记，待 AGENTS.md §5.5 proto 扩展确认） |
| 方向 | **B → A**（服务节点 主动请求检测节点 终止） |
| 语义 | 服务节点 退出时命令检测节点 终止服务并反馈；检测节点 收到后终止自身服务并回 `accepted` |
| deadline | 3 秒（B 侧设短 deadline，超时即放弃等待） |
| 失败处理 | 超时/失败仅记 WARNING 日志，不重试、不阻塞 B 退出流程（best-effort） |

> **本 S→D RPC 待 AGENTS.md §5.5 proto 扩展确认；与现有 D→S `Shutdown`（A 自报下线）语义不同，二者并存。**
>
> - **D→S `Shutdown`（已有，AGENTS.md §5.5）**：检测节点 退出前主动通知服务节点，使服务节点 立即降级。语义为"A 自报下线"。
> - **S→D `RequestPeerShutdown`（本设计新增）**：服务节点 退出时主动命令检测节点 终止。语义为"B 请求 A 关机"。
>
> 落地此设计需后续 spec 修订 **AGENTS.md §5.5**（proto 增加 S→D RPC `RequestPeerShutdown`）与 **Design-AI_detect.md**（消除 §7.4.1 S→D / §9.3 D→S / §3.10 proto 无 Shutdown 三处方向矛盾）。引用 spec 问题清单 #1（S→D 退出通知 vs AGENTS.md 冻结 proto）、#2（Design-AI_detect.md 内部 Shutdown 方向三处矛盾）。

### 10.4 检测节点 退出行为基准

| 行为 | 基准来源 | 说明 |
|------|----------|------|
| 检测节点 退出时主动通知服务节点（D→S `Shutdown`） | **AGENTS.md §5.7**（权威） | best-effort，deadline=2s；A 侧退出前经 `grpc_client` 调用 `Shutdown` RPC 通知 B，B 收到后立即标记 A 离线并触发降级 |
| 检测节点 异常退出（崩溃/断电/网络中断） | AGENTS.md §5.7.6 兜底策略 | `Shutdown` 发不出，由 B 侧心跳超时（默认 15 秒）兜底判定离线 |
| S→D `RequestPeerShutdown` 收到后 A 的行为 | 本设计（待 proto 扩展） | A 收到后终止自身服务并回 `accepted`；具体终止流程以检测节点 设计为准 |

> **Design-AI_detect.md 矛盾规避声明**：Design-AI_detect.md 在检测节点 退出行为上存在内部矛盾（§7.4.1 表述为 S→D `Shutdown`、§9.3 表述为 D→S 通知、§3.10 proto 中无 `Shutdown` 定义），三处方向互相冲突。**本设计检测节点 退出行为一律以 AGENTS.md §5.7 为准，不引用 Design-AI_detect.md 的矛盾表述**；待后续 spec 修订 Design-AI_detect.md 消除矛盾后回写对齐。引用 spec 问题清单 #2。

### 10.5 退出告警落盘

当 §10.2 步骤 ④ 触发（时限内未收到检测节点 反馈）时，按以下方式落盘告警：

| 落盘项 | 内容 | 路径 |
|--------|------|------|
| WARNING 日志 | `检测节点 退出反馈超时（deadline=3s），已 best-effort 退出。检测节点 可能异常离线，请人工核查。` | `logs/node_server.log`（对齐 §6 日志规范，格式：`时间戳(毫秒) | WARNING | S | grpc_client | 消息`） |
| 终端输出告警 | 在 CLI 终端 / GUI 退出提示框输出告警信息 | 标准输出 / messagebox |

> **日志落盘保证**：退出告警日志必须在程序释放 gRPC channel 与日志 handler 之前写入，确保异常退出时告警信息不丢失（对齐 §6.6 异常退出 `logging.shutdown()` 约定）。

---

## 附录 A：现有代码到新架构的映射清单

> **归档说明**：`new-predict.py` 已归档至 `temp/`，本表"源文件"列中 `new-predict.py` 的行号引用对应归档版本。`demo.py` 仍在项目根目录。

| 现有类 / 函数 | 源文件 | 对应新模块 | 对应新类 |
|----------------|--------|------------|----------|
| `SerialBuffer` | `demo.py:25-84` | `serial/serial_buffer.py` | `SerialBuffer` |
| `LaserDataParser` | `demo.py:87-104` | `serial/laser_parser.py` | `LaserDataParser` |
| `CameraManager` | `demo.py:107-1215` | `camera/orbbec_336l.py` | `Orbbec336LInput` |
| `SerialManager` | `demo.py:1217-1439` | `serial/serial_manager.py` | `SerialManager` |
| `FusionDisplayApp` | `demo.py:1442-1796` | `ui/app.py` | `RebarMeasureApp` |
| `check_gui_environment` | `demo.py:1799-1843` | `system/self_check.py` | `_check_gui_environment()` |
| `RebarMeasureV53` | `new-predict.py:504+` | `measure/rebar_measure.py` | `RebarMeasure` |
| `Unet` | `new-predict.py:357+` | （移至检测节点） | `grpc_server/inference_service.py` |
| `LaserDataParser.parse()` 有效性检查 | — | `serial/laser_parser.py` | `LaserParseResult` + `_is_valid()` |
| 4 路融合逻辑 | — | `serial/laser_fusion.py` | `LaserFusion` |
| CLI 融合菜单 | — | `ui/cli_fusion_menu.py` | `CliFusionMenu` |
| 画面渲染 + 掩码叠加 | — | `ui/render.py` | `UIRenderer` |
| gRPC 客户端封装 | — | `grpc_client/inference_client.py` | `InferenceGrpcClient` |
| 启动环境自检 | — | `system/self_check.py` | `run_self_check()` |
| 运行时健康检查 | — | `system/health_check.py` | `HealthChecker` |
| 配置加载器 | — | `common/config_loader.py` | `ConfigLoader` + dataclasses |
| `NetworkCameraInput` | 新增 | `camera/network_camera.py` | TCP 网络摄像头实现（S→D 退出通知客户端能力亦在 `grpc_client/inference_client.py` 扩展） |
| `LocalCameraScanner` | 新增 | `camera/scanner.py` | 本地摄像头自动扫描 + 兼容白名单 |
| `service-manager.sh` | 新增 | `scripts/service-manager.sh` | 系统服务管理脚本（install/uninstall 开机自启，见 §9） |
| 识别结果存储规范 | 新增 | 见 §8 | `./result/{YYYYMMDD-HHMMSS}/` 路径 + `result.jpg`/`report.csv`/`ClassMask.png` 文件规范 |
| 双退出流程 | 新增 | 见 §10 | GUI/CLI 双退出路径 + S→D `RequestPeerShutdown` RPC（待 proto 扩展）+ 退出告警落盘 |
| 日志轮转策略 | 新增 | 见 §6.7 | `RotatingFileHandler` 最小轮转策略（10MB / 3 份） |

## 附录 B：接口依赖矩阵

```
[camera]  ←→ [ measure ] : 提供 RGB 帧用于测量结果可视化
[camera]  ←→ [ ui ]      : 提供预览帧
[camera]  ←→ [grpc_client]: 提供 JPEG 编码帧用于推理请求

[serial]  ←→ [ measure ] : 提供圆整距离 mm
[serial]  ←→ [ ui ]      : 提供激光原始值用于面板显示
[laser_fusion] ←→ [grpc_client]: 提供融合距离用于请求携带

[grpc_client] ←→ [ measure ] : 提供类别掩码用于测量
[grpc_client] ←→ [ ui ]  : 提供推理状态（在线/离线）

[measure] ←→ [ ui ]      : 提供 MeasureResult 用于渲染与列表
[intrinsics] ←→ [ measure ] : 提供 fx/fy 用于 mm/px 换算
[system.self_check] ←→ [ system.bootstrap ] : 提供自检结果
[system.health_check] ←→ [ ui ] : 提供降级状态切换回调
```

---

## 附录 C：一致性与已知冲突说明

> 本附录集中记录 `Design-server.md` 与上层规范（`AGENTS.md` / `Design-AI_detect.md`）的一致性确认项与已知冲突点，便于后续 spec 联动修订时追踪。冲突项均已在正文相应章节用 `>` 引用块标注，本附录为汇总索引。

### C.1 S→D 退出通知与 AGENTS.md §5.5 冻结 proto 冲突

| 项 | 说明 |
|----|------|
| 冲突点 | Design-server.md §10.3 定义的 `RequestPeerShutdown` 为 **S→D** 方向；AGENTS.md §5.5 现仅定义 D→S `Shutdown`（A 自报下线），未定义 S→D 退出通知 RPC。 |
| 引用 | spec 问题清单 #1（S→D 退出通知 vs AGENTS.md 冻结 proto） |
| 当前处理 | Design-server.md 已写入 S→D 流程，RPC 暂记 `RequestPeerShutdown` 并标注"待 AGENTS.md proto 扩展确认"；与现有 D→S `Shutdown` 语义不同，二者并存。 |
| 待联动修订 | 后续 spec 修订 AGENTS.md §5.5 proto 增加 S→D RPC `RequestPeerShutdown`（含请求/响应消息体定义）。 |

### C.2 Design-AI_detect.md 内部 Shutdown 方向三处矛盾

| 项 | 说明 |
|----|------|
| 冲突点 | Design-AI_detect.md 在检测节点 退出行为上存在三处互相矛盾的方向表述：§7.4.1（S→D，B 请求 A 下线）、§9.3（D→S，A 通知 B）、§3.10 proto（无 `Shutdown` 定义）。 |
| 引用 | spec 问题清单 #2（Design-AI_detect.md 内部 Shutdown 方向三处矛盾） |
| 当前处理 | **本设计检测节点 退出行为一律以 AGENTS.md §5.7 为准**（D→S `Shutdown`，best-effort，deadline=2s），不引用 Design-AI_detect.md 的矛盾表述（见 §10.4 矛盾规避声明）。 |
| 待联动修订 | 后续 spec 修订 Design-AI_detect.md 消除 §7.4.1/§9.3/§3.10 三处方向矛盾，明确"D→S 自通知 + S→D 远程关机"为两个不同 RPC。 |

### C.3 相机接入方式扩展

| 项 | 说明 |
|----|------|
| 冲突点 | AGENTS.md §5.1 现表述为"336L USB 直连服务节点"；本设计扩展为"USB 为主、TCP 网络摄像头为辅"（§3.3.4 `NetworkCameraInput`）。 |
| 引用 | spec 问题清单 #3（相机接入方式：USB-only vs USB+网络摄像头） |
| 当前处理 | Design-server.md 定义 `NetworkCameraInput`（TCP）与 `Orbbec336LInput`（USB）并列，二者均继承 `BaseCameraInput`；上层逻辑无需感知接入差异。 |
| 待联动修订 | 后续 spec 修订 AGENTS.md §5.1 表述，补充"网络摄像头 TCP 接入为辅"的说明。 |

### C.4 新增无上层规范背书项

| 项 | 说明 |
|----|------|
| 冲突点 | 以下内容为 Design-server.md 新增，AGENTS.md 暂无对应约定： |
| ① `./result/` 识别结果存储规范 | 见 §8（路径 `./result/{YYYYMMDD-HHMMSS}/`、文件命名、保存约束）。 |
| ② `service-manager.sh` 系统服务管理 | 见 §9（脚本路径 `scripts/service-manager.sh`、install/uninstall 子命令、程序内部调用能力）。 |
| ③ 日志轮转策略 | 见 §6.7（`RotatingFileHandler` 最小轮转策略，10MB / 3 份，配置化于 `config/` 下 JSON）。 |
| 当前处理 | **Design-server.md 作为这些细节的权威源**；后续按需将关键约定回写 AGENTS.md。 |
| 引用 | spec 问题清单 #4（AGENTS.md 无结果存储 / 服务管理 / 日志轮转约定） |

### C.5 一致性确认项

以下各项已与上层规范对齐，无冲突：

| 项 | Design-server.md 位置 | 上层规范基准 | 一致性说明 |
|----|----------------------|--------------|------------|
| 配置 JSON 约定 | §3.2 / §3.3 / §5.1 / §6.7 / §8.3 / §9.1 | AGENTS.md §3.3 / §7.1 | 所有项目侧参数配置文件统一 JSON 格式（RFC 8259，Python `json` 模块），禁止 YAML/TOML/INI；唯一例外为 systemd `.service` unit 文件（OS 规范要求 INI，由脚本从 JSON 源生成）。 |
| 日志格式 | §6.1 / §6.5 | AGENTS.md §7.5 | 统一格式 `时间戳(毫秒) \| 级别 \| 节点 \| 模块 \| 消息`，路径 `logs/node_server.log`，一机一日志。 |
| label_mask 编码与类别值 | §3.5 / §3.11.3 / §8.2 | Design-AI_detect.md | PNG 编码、单通道、像素值 0/1/2（0=背景 / 1=纵向钢筋 / 2=横向钢筋）。 |
| 帧同步 frame_id / timestamp_ms | §3.5 / §3.11.5 / §4.4 | Design-AI_detect.md | `frame_id`（单调递增）与 `timestamp_ms` 由服务节点 生成、检测节点 原样回传、服务节点 校验一致性；乱序/迟到响应直接丢弃。 |
| gRPC 三 RPC 方向 | §3.5 / §3.10 / §10.3 / §10.4 | AGENTS.md §5.5 | Infer S→D、Heartbeat D→S、Shutdown D→S；本设计新增 S→D `RequestPeerShutdown` 已在 C.1 标注待扩展。 |
| 检测节点 退出行为基准 | §10.4 | AGENTS.md §5.7 | A 退出前主动通知 B（D→S `Shutdown`，best-effort，deadline=2s）；异常退出由 B 侧心跳超时（15 秒）兜底。 |
| 网络摄像头协议路径 | §3.3.4 | spec 问题清单 #6 | 项目内相对路径 `orbbec-336L/orbbec-336L-win-protocol.md`，无 Windows 绝对路径残留。 |
| CLI 实现库 | §3.10.1 | spec 问题清单 #5 | `argparse` 标准库（零额外依赖），与现有 `CliFusionMenu` 风格一致。 |
| 推理请求间隔控制权 | §1.4 / §0.4 | AGENTS.md §3.3 | 仅在服务节点 侧执行（`config/inference.json` 的 `inference_interval_seconds`），检测节点 侧不节流。 |

### C.6 服务节点 `grpc_server` 模块独立性问题

| 项 | 说明 |
|----|------|
| 冲突点 | AGENTS.md §3.1 模块划分总览要求服务节点 包含 **7 个独立模块**：`camera` / `serial` / `grpc_client` / **`grpc_server`** / `measure` / `ui` / `system`，其中 `grpc_server` 职责为"gRPC 服务端，在 B:50051 接收检测节点 的 Heartbeat + Shutdown RPC，转交 NodeMonitor 处理（D→S）"。Design-server.md §2 工程目录结构仅列出 6 个模块目录（`camera/` / `serial/` / `grpc_client/` / `measure/` / `ui/` / `system/`），`grpc_server` 职责被折叠进 `system/node_monitor.py`，未作为独立目录或章节体现。 |
| 引用 | 校验阶段发现（spec 验收 Checklist 6.7） |
| 当前处理 | `grpc_server` 的功能职责**已完整实现**（接收 D→S Heartbeat + Shutdown RPC，见 §3.8 `NodeMonitor` / §3.11.1 状态记录维护），仅模块组织形式与 AGENTS.md §3.1 的"7 独立模块"约定不一致。`grpc_server` 作为**日志命名空间**（见 §6.5 日志示例 `| S | grpc_server |`）已存在，但代码物理位置在 `system/` 下。 |
| 待联动修订 | 二选一：① 后续 spec 修订 Design-server.md，将 `grpc_server/` 从 `system/` 中拆分为独立模块目录，并新增 §3.x `grpc_server` 模块章节；② 后续 spec 修订 AGENTS.md §3.1，注明"服务节点 的 `grpc_server` 职责由 `system` 模块下的 `node_monitor` 承载，不另立目录"。推荐方案 ②（避免过度拆分，保持 `system` 模块的内聚性）。 |

---

> **文档结束**
> 
> 本文件是服务节点 原型阶段的工程蓝本。所有接口签名与关键逻辑以本方案为准；具体实现可据实测微调，但模块边界与职责划分、锁策略、帧同步机制、自检与降级逻辑为强制约定，未经评审不得破坏。
> 
> 本文档与 `AGENTS.md` / `Design-AI_detect.md` 的已知冲突已在附录 C 集中记录，待后续 spec 联动修订；冲突项在正文相应章节均以 `>` 引用块显式标注，未静默引入与上层规范矛盾的设计。
