"""
启动环境自检 — 程序启动时一次性执行的启动门控。

遵循 AGENTS.md §4.3 自检规范（诊断模式）：
  - 本机 IP 校验：**诊断模式** → 仅记录日志 + 修复指引，不阻塞启动；运行期由 NetworkMonitor 动态跟踪
  - 端口可用性：**硬故障** → 异常 → 安全退出（端口被占则服务无法运行）
  - CUDA 设备：**硬故障** → 异常 → 安全退出（Jetson 上 CUDA 为必需）
  - 模型权重可读：**硬故障** → 异常 → 安全退出
  - 非 Windows 环境才检查 root 权限

自检必须在 UI / gRPC server 启动之前在 main.py 同步执行。
诊断项（IP）不阻塞；硬故障项（端口/CUDA/权重）不通过则**严禁带病运行**。
"""

import logging
import os
import socket
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

logger = logging.getLogger("node_detect.system.self_check")


@dataclass
class SelfCheckResult:
    """自检结果"""
    admin_privilege: bool = False     # root 权限（Linux 上需要）
    local_ip_ok: bool = False         # 本机 IP 校验通过
    port_available: bool = False      # gRPC 端口可用（未被占用）
    cuda_available: bool = False      # CUDA 设备可用
    weight_readable: bool = False     # 权重文件可读
    errors: List[str] = field(default_factory=list)  # 失败项描述

    @property
    def all_ok(self) -> bool:
        """硬故障项全部通过（IP 不纳入，按 AGENTS.md §4.3 诊断模式处理）"""
        return (
            self.port_available
            and self.cuda_available
            and self.weight_readable
        )


class SelfCheckError(RuntimeError):
    """自检失败异常 — 触发安全退出"""
    pass


def _check_admin_privilege() -> bool:
    """
    检查是否具备管理员权限。

    仅在 Linux/Jetson 上有效，Windows 或开发环境返回 True（跳过）。
    """
    if sys.platform == "win32":
        # Windows 开发环境跳过
        return True
    try:
        return os.geteuid() == 0
    except AttributeError:
        # 非 Unix 系统跳过
        return True


def _check_local_ip(expected_ip: str) -> bool:
    """
    校验本机网卡 IP 是否与 network.json 中的 local_ip 一致。

    使用 socket 连接对端 UDP（不发送数据）获取系统出口 IP，适用于各种平台。
    """
    if sys.platform == "win32":
        # Windows 开发环境跳过（AGENTS.md §4.3 开发兼容说明）
        logger.warning("[自检] Windows 开发环境 — 跳过本地 IP 校验")
        return True

    try:
        # 通过 UDP socket 获取本机出口 IP（不需真实送达）
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            # 使用对端的 gRPC 端口，协议无关紧要，只是获取本机 IP
            s.connect(("8.8.8.8", 80))
            actual_ip = s.getsockname()[0]
        ok = actual_ip == expected_ip
        if not ok:
            logger.error(f"[自检] 本机 IP 不匹配: 期望 {expected_ip}, 实际 {actual_ip}")
        return ok
    except Exception as e:
        logger.error(f"[自检] 获取本机 IP 失败: {e}")
        # 不能确定时，保守返回 False
        return False


def _check_port(port: int) -> bool:
    """
    检查端口是否可用（未被占用）。

    通过尝试绑定端口来验证，会立即释放。不依赖外部工具。
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", port))
        return True
    except OSError as e:
        logger.error(f"[自检] 端口 {port} 不可用: {e}")
        return False


def _check_cuda() -> bool:
    """
    检查 CUDA 设备是否可用。

    通过 torch.cuda.is_available() 判断。Windows 开发环境无 CUDA 时跳过。
    """
    try:
        import torch
        available = torch.cuda.is_available()
        if not available:
            if sys.platform == "win32":
                logger.warning("[自检] Windows 无 CUDA — 跳过（部署到 Jetson 后必须具备）")
                return True
            logger.error("[自检] CUDA 不可用 — Jetson 上必须配置 PyTorch CUDA 版本")
        else:
            # 打印 GPU 信息
            gpu_name = torch.cuda.get_device_name(0)
            logger.info(f"[自检] CUDA OK — GPU: {gpu_name}")
        return available or sys.platform == "win32"
    except ImportError:
        logger.error("[自检] torch 未安装 — 请安装 Jetson 专用 PyTorch")
        return False


def _check_weight_readable(model_path: str) -> bool:
    """检查权重文件是否可读"""
    p = Path(model_path)
    if not p.exists():
        logger.error(f"[自检] 权重文件不存在: {model_path}")
        return False
    if not p.is_file():
        logger.error(f"[自检] 权重路径不是文件: {model_path}")
        return False
    # 尝试读取前几个字节验证是否可读
    try:
        with open(p, "rb") as f:
            f.read(8)
        return True
    except PermissionError:
        logger.error(f"[自检] 权重文件无读权限: {model_path}")
        return False
    except Exception as e:
        logger.error(f"[自检] 权重文件读取失败: {e}")
        return False


def run_self_check(
    config,  # AppConfig
    strict_admin: bool = False,
) -> SelfCheckResult:
    """
    执行启动自检。

    Args:
        config: 加载后的 AppConfig 配置对象
        strict_admin: 是否严格要求 root（默认 False，开发环境跳过）

    Returns:
        SelfCheckResult — 自检结果

    Raises:
        SelfCheckError: 硬故障（IP/端口/CUDA/权重异常），触发安全退出
    """
    result = SelfCheckResult()
    logger.info("=" * 60)
    logger.info("[自检] 启动环境自检开始")
    logger.info("=" * 60)

    # ---- 1. 管理员权限 ----
    result.admin_privilege = _check_admin_privilege()
    if strict_admin and not result.admin_privilege:
        result.errors.append("非 root 权限运行")
        logger.error("[自检] 非 root 权限")
    else:
        tag = "OK" if result.admin_privilege else "跳过"
        logger.info(f"[自检] 管理员权限: {tag}")

    # ---- 2. 本机 IP（诊断模式，不阻塞启动；对齐 AGENTS.md §4.3） ----
    result.local_ip_ok = _check_local_ip(config.local_ip)
    if not result.local_ip_ok:
        # IP 不匹配仅记录错误与修复指引；运行期由 NetworkMonitor 动态跟踪
        logger.error(f"[自检] 本机 IP 不匹配（诊断模式，不阻塞启动）: 期望 {config.local_ip}")
        logger.error("[自检] 修复指引：")
        logger.error("  1. sudo bash /opt/node_detect/scripts/setup_network.sh    # 自动配置静态 IP")
        logger.error("  2. 或手工校对 /opt/node_detect/config/network.json 的 local_ip")
        logger.error("  3. 运行期 NetworkMonitor 会持续探测配置正确性")
    else:
        logger.info(f"[自检] 本机 IP: {config.local_ip} OK")

    # ---- 3. 端口可用性 ----
    result.port_available = _check_port(config.grpc_port)
    if not result.port_available:
        result.errors.append(f"端口 {config.grpc_port} 不可用")
        logger.error(f"[自检] 端口 {config.grpc_port} 不可用（可能已占用）")
    else:
        logger.info(f"[自检] 端口 {config.grpc_port} 可用")

    # ---- 4. CUDA ----
    result.cuda_available = _check_cuda()
    if not result.cuda_available:
        result.errors.append("CUDA 不可用")
        # 已在 _check_cuda 中 logger.error
    else:
        logger.info(f"[自检] CUDA: {'可用' if not sys.platform == 'win32' else '跳过（Win开发环境）'}")

    # ---- 5. 权重文件可读 ----
    result.weight_readable = _check_weight_readable(config.model_path)
    if not result.weight_readable:
        result.errors.append(f"权重文件不可读: {config.model_path}")
    else:
        logger.info(f"[自检] 权重文件可读: {config.model_path}")

    # ---- 结果判定 ----
    logger.info("-" * 60)
    if result.all_ok:
        logger.info("[自检] ✓ 全部通过 — 系统可安全启动")
    else:
        logger.error(f"[自检] ✗ 自检失败项: {result.errors}")
        raise SelfCheckError(
            f"启动自检失败: {'; '.join(result.errors)}"
        )
    logger.info("=" * 60)

    return result
