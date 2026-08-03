"""启动环境自检（服务节点）。

对齐 AGENTS.md §4.3（启动环境自检机制）与 Design-server.md §3.8.1 / §5。
自检项顺序：①管理员权限 → ②本机 IP → ③端口可用性 → ④网络配置权限 → ⑤对端可达性。

失败处理（退出策略由 bootstrap 决定，本模块仅返回结果）：
- 硬故障（IP 不匹配 / 端口冲突 / 网络配置权限缺失）→ bootstrap 记录 CRITICAL + sys.exit(1)
- 软故障（对端不可达）→ bootstrap 进入降级模式（界面提示"推理服务离线"）
- 管理员权限：条件性检查，原型阶段应用核心运行无需特权（gRPC 端口 50051 > 1024），
  非管理员记录 WARNING 但不阻塞；仅 ServiceManager 注册系统服务时需特权。
- Windows 开发环境：IP 校验 bypass（记 WARNING 但 ip_ok=True），对齐 §4.3 平台兼容要求。
"""
import ctypes
import logging
import os
import socket
import sys
from dataclasses import dataclass, field
from typing import List, Tuple

from common.config_loader import NetworkConfig

# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 system）
_logger = logging.getLogger("node_server.system")


@dataclass
class SelfCheckResult:
    """自检结果数据类（对齐 Step 4 接口契约）。

    各字段 True 表示该检查项通过，False 表示失败。
    messages 记录每一项检查的详细消息，顺序与检查顺序一致。
    """

    admin_ok: bool = False  # 管理员权限检查
    ip_ok: bool = False  # 本机 IP 校验（与 network.json local_ip 一致）
    port_ok: bool = False  # gRPC 端口可用性（未被占用）
    net_priv_ok: bool = False  # 网络配置权限（能访问网络接口）
    peer_reachable: bool = False  # 对端检测节点 可达性（TCP 探测）
    messages: List[str] = field(default_factory=list)  # 各项检查的详细消息


def _get_local_ip() -> str:
    """获取本机出口 IP（通过 UDP socket 探测，不实际发包）。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # 连接公共 DNS（不实际发包，仅用于路由解析本机出口 IP）
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except OSError:
        # 路由不可达时回退到 hostname 解析
        try:
            ip = socket.gethostbyname(socket.gethostname())
        except OSError:
            ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def _check_admin_privilege() -> Tuple[bool, str]:
    """检查管理员/root 权限。

    Linux/openEuler：``os.geteuid() == 0``
    Windows：``ctypes.windll.shell32.IsUserAnAdmin()``
    """
    if sys.platform == "win32":
        try:
            is_admin = bool(ctypes.windll.shell32.IsUserAnAdmin())
        except Exception as e:
            return False, f"Windows 管理员权限检查异常: {e}"
        if is_admin:
            return True, "Windows：以管理员权限运行"
        return False, "Windows：非管理员权限运行（原型阶段不阻塞，仅记录 WARNING）"
    # Linux / openEuler
    try:
        if os.geteuid() == 0:
            return True, "Linux：以 root 权限运行"
        return False, "Linux：非 root 权限运行（注册系统服务/绑定特权端口将失败）"
    except AttributeError:
        return False, "无法获取进程权限（os.geteuid 不可用）"


def _check_local_ip(cfg: NetworkConfig) -> Tuple[bool, str]:
    """检查本机 IP 是否与 network.json 中 local_ip 一致。

    Windows 开发环境提供 bypass（记 WARNING 但通过），对齐 §4.3 平台兼容要求。
    """
    actual_ip = _get_local_ip()
    if sys.platform == "win32":
        return True, (
            f"Windows 开发环境：IP 校验 bypass（本机 IP={actual_ip}, "
            f"配置 local_ip={cfg.local_ip}）"
        )
    if actual_ip == cfg.local_ip:
        return True, f"本机 IP 校验通过: {actual_ip}"
    return False, f"本机 IP {actual_ip} 与配置 local_ip {cfg.local_ip} 不一致"


def _check_port_available(cfg: NetworkConfig) -> Tuple[bool, str]:
    """检查 gRPC 端口是否未被占用（尝试 bind 0.0.0.0:port）。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("0.0.0.0", cfg.grpc_port))
        return True, f"gRPC 端口 {cfg.grpc_port} 可用（未被占用）"
    except OSError as e:
        return False, f"gRPC 端口 {cfg.grpc_port} 不可用: {e}"
    finally:
        s.close()


def _check_network_config_privilege() -> Tuple[bool, str]:
    """检查网络配置访问权限（能否读取本机网卡 IP）。

    软故障：权限缺失时告警但不阻塞（对齐 Design-server.md §5.2）。
    """
    try:
        _ = _get_local_ip()
        return True, "网络配置访问权限正常"
    except PermissionError as e:
        return False, f"网络配置权限缺失: {e}"
    except OSError as e:
        return False, f"网络配置访问异常: {e}"


def _check_peer_reachable(cfg: NetworkConfig, timeout: float = 2.0) -> Tuple[bool, str]:
    """TCP 探测对端检测节点 的 IP:端口是否可达。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((cfg.remote_ip, cfg.grpc_port))
        return True, f"对端检测节点 {cfg.remote_ip}:{cfg.grpc_port} 可达"
    except socket.timeout:
        return False, f"对端检测节点 {cfg.remote_ip}:{cfg.grpc_port} 探测超时（{timeout}s）"
    except (ConnectionRefusedError, OSError) as e:
        return False, f"对端检测节点 {cfg.remote_ip}:{cfg.grpc_port} 不可达: {e}"
    finally:
        s.close()


def run_self_check(cfg: NetworkConfig) -> SelfCheckResult:
    """执行完整的启动环境自检（5 项）。

    严格顺序：①管理员权限 → ②本机 IP → ③端口可用性 → ④网络配置权限 → ⑤对端可达性。
    前四项为本地配置/权限类检查（快速），最后一项为网络 I/O（可能耗时 2 秒）。

    :param cfg: 网络配置（含 local_ip / remote_ip / grpc_port）
    :return: SelfCheckResult（不主动退出，退出策略由 bootstrap 决定）
    """
    result = SelfCheckResult()
    _logger.info("启动环境自检开始（5 项检查）")

    # ① 管理员权限（条件性，原型阶段不阻塞）
    ok, msg = _check_admin_privilege()
    result.admin_ok = ok
    result.messages.append(f"[管理员权限] {msg}")
    if ok:
        _logger.info("[自检] 管理员权限: %s", msg)
    else:
        _logger.warning("[自检] 管理员权限: %s", msg)

    # ② 本机 IP（Windows bypass）
    ok, msg = _check_local_ip(cfg)
    result.ip_ok = ok
    result.messages.append(f"[本机IP] {msg}")
    if ok:
        _logger.info("[自检] 本机 IP: %s", msg)
    else:
        _logger.error("[自检] 本机 IP: %s", msg)

    # ③ 端口可用性
    ok, msg = _check_port_available(cfg)
    result.port_ok = ok
    result.messages.append(f"[端口可用性] {msg}")
    if ok:
        _logger.info("[自检] 端口可用性: %s", msg)
    else:
        _logger.error("[自检] 端口可用性: %s", msg)

    # ④ 网络配置权限
    ok, msg = _check_network_config_privilege()
    result.net_priv_ok = ok
    result.messages.append(f"[网络配置权限] {msg}")
    if ok:
        _logger.info("[自检] 网络配置权限: %s", msg)
    else:
        _logger.error("[自检] 网络配置权限: %s", msg)

    # ⑤ 对端可达性（软故障）
    ok, msg = _check_peer_reachable(cfg)
    result.peer_reachable = ok
    result.messages.append(f"[对端可达性] {msg}")
    if ok:
        _logger.info("[自检] 对端可达性: %s", msg)
    else:
        _logger.warning("[自检] 对端可达性: %s", msg)

    _logger.info(
        "启动环境自检完成: admin=%s, ip=%s, port=%s, net_priv=%s, peer=%s",
        result.admin_ok, result.ip_ok, result.port_ok,
        result.net_priv_ok, result.peer_reachable,
    )
    return result
