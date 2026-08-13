"""启动引导：配置加载 → 环境自检 → NodeMonitor → GrpcServerB → GUI（Step 5 接入）。

对齐 Design-server.md §0.1 / §5 与 AGENTS.md §4.3。

严格顺序：
1. 第一步：读取加载 JSON 配置（logging → network → camera → intrinsics → inference）
2. 第二步：环境自检（run_self_check）—— 网络属功能受限型依赖，全部改为诊断日志，不阻塞启动
   - 管理员权限：条件性，非管理员 WARNING 不阻塞
   - 网络项（ip/port/net_priv/peer）：诊断日志 + 由 NetworkMonitor 运行时动态监测
3. 初始化 NodeMonitor（被动接收心跳 + 超时降级）
4. 启动 GrpcServerB（注入 NodeMonitor）
4B. 初始化 NetworkMonitor（运行时动态监测网络状态，在线/降级动态翻转）
5. 创建 Tkinter root + RebarMeasureApp（Step 5 实现，此处 try/except 占位）
6. 启动 App
7. 进入 Tkinter mainloop

异常处理：仅配置文件加载失败（真正致命）→ 记录 CRITICAL 日志 + sys.exit(1)。
网络/检测节点 不可用 → 推理功能降级禁用，程序正常启动继续运行，恢复后自动上线。
"""
import logging
import os
import sys

# 显式将项目根目录加入 sys.path，确保 systemd / 不同启动方式下
# `from proto import ...` 等绝对导入均可解析（对齐 AGENTS.md §7.2 启动流程）
# 本文件位于 utils/system/bootstrap.py，需要上溯 3 层才能到达项目根
_PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))  # utils/system/
_PROJECT_ROOT = os.path.dirname(_PROJECT_ROOT)              # utils/
_PROJECT_ROOT = os.path.dirname(_PROJECT_ROOT)              # 项目根目录
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from utils.common.config_loader import (
    CameraConfig,
    ConfigLoader,
    InferenceConfig,
    IntrinsicsConfig,
    LoggingConfig,
    NetworkConfig,
    SerialConfig,
)
from utils.common.logging_setup import setup_logging
from utils.grpc_client.grpc_server import GrpcServerB
from utils.system.network_monitor import NetworkMonitor
from utils.system.node_monitor import NodeMonitor
from utils.system.self_check import run_self_check

# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 system）
_logger = logging.getLogger("node_server.system")

# 配置文件路径（相对 node_server/ 工作目录，对齐 §3.3 config/ 目录约定）
_CONFIG_DIR = "config"
_LOGGING_CFG_PATH = os.path.join(_CONFIG_DIR, "logging.json")
_NETWORK_CFG_PATH = os.path.join(_CONFIG_DIR, "network.json")
_CAMERA_CFG_PATH = os.path.join(_CONFIG_DIR, "camera.json")
_INTRINSICS_CFG_PATH = os.path.join(_CONFIG_DIR, "intrinsics.json")
_INFERENCE_CFG_PATH = os.path.join(_CONFIG_DIR, "inference.json")
_SERIAL_CFG_PATH = os.path.join(_CONFIG_DIR, "serial.json")
_LOG_PATH = "logs/node_server.log"


def bootstrap() -> None:
    """启动引导入口。

    严格按 §7 生命周期顺序执行；仅配置文件加载失败（真正致命）→ CRITICAL + sys.exit(1)。
    网络/检测节点 不可用不阻塞启动，由 NetworkMonitor/NodeMonitor 运行时动态降级。
    GUI 模块（ui.app.RebarMeasureApp）由 Step 5 接入，本 Step 用 try/except 占位。
    """
    # ============================================================
    # 第一步：读取加载 JSON 配置
    # ============================================================
    # 先加载日志配置并初始化日志系统（后续所有步骤的日志都依赖它）
    try:
        logging_cfg: LoggingConfig = ConfigLoader.load_logging_config(_LOGGING_CFG_PATH)
    except Exception as e:
        # 日志配置加载失败时用默认值兜底，确保后续日志可用
        print(f"[bootstrap] 加载 logging.json 失败，使用默认日志配置: {e}")
        logging_cfg = LoggingConfig()
    setup_logging(_LOG_PATH, logging_cfg)

    _logger.info("=" * 60)
    _logger.info("服务节点 启动引导开始")
    _logger.info("=" * 60)

    # 加载其余配置文件
    try:
        net_cfg: NetworkConfig = ConfigLoader.load_network_config(_NETWORK_CFG_PATH)
        camera_cfg: CameraConfig = ConfigLoader.load_camera_config(_CAMERA_CFG_PATH)
        intrinsics_cfg: IntrinsicsConfig = ConfigLoader.load_intrinsics_config(
            _INTRINSICS_CFG_PATH
        )
        inference_cfg: InferenceConfig = ConfigLoader.load_inference_config(
            _INFERENCE_CFG_PATH
        )
        serial_cfg: SerialConfig = ConfigLoader.load_serial_config(_SERIAL_CFG_PATH)
    except Exception as e:
        _logger.critical("加载配置文件失败: %s", e)
        sys.exit(1)

    _logger.info("配置加载完成:")
    _logger.info(
        "  network: local_ip=%s, remote_ip=%s, grpc_port=%d",
        net_cfg.local_ip, net_cfg.remote_ip, net_cfg.grpc_port,
    )
    _logger.info(
        "  camera: rgb=%dx%d@%dfps",
        camera_cfg.rgb_width, camera_cfg.rgb_height, camera_cfg.rgb_fps,
    )
    _logger.info(
        "  intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
        intrinsics_cfg.fx, intrinsics_cfg.fy, intrinsics_cfg.cx, intrinsics_cfg.cy,
    )
    _logger.info(
        "  inference: interval=%.1fs", inference_cfg.inference_interval_seconds,
    )
    _logger.info(
        "  serial: device_path=%s, auto_scan=%s, baudrate=%d",
        serial_cfg.device_path if serial_cfg.device_path else "(空)",
        serial_cfg.auto_scan, serial_cfg.baudrate,
    )

    # ============================================================
    # 第二步：环境自检（网络属功能受限型依赖，不阻塞启动）
    # ============================================================
    # 网络检测全部改为"诊断日志"，不再 sys.exit(1) 阻塞启动（AGENTS.md §4.3 软故障/降级原则）
    # 原因：即便网络不可用（IP 不匹配/端口占用/对端不可达），GUI/相机/串口等本地功能仍可用
    _logger.info("执行启动环境自检（诊断模式，不阻塞启动）...")
    check_result = run_self_check(net_cfg)

    # 自检结果全部记录为诊断日志（INFO/WARNING），不做硬故障退出
    if not check_result.ip_ok:
        _logger.warning("[诊断] 本机 IP 校验未通过: %s", check_result.messages[1])
    else:
        _logger.info("[诊断] 本机 IP 校验通过")
    if not check_result.port_ok:
        _logger.warning("[诊断] gRPC 端口可用性未通过: %s", check_result.messages[2])
        _logger.warning("[诊断]   端口被占用不影响本地功能，推理功能可能受限")
    if not check_result.net_priv_ok:
        _logger.warning("[诊断] 网络配置权限未通过: %s", check_result.messages[3])

    # 管理员权限：条件性，非管理员 WARNING 不阻塞
    # （原型阶段应用核心运行无需特权：gRPC 端口 50051 > 1024，无需特权端口绑定；
    #   仅 ServiceManager 注册系统服务时需特权，由 ServiceManager 自行校验）
    if not check_result.admin_ok:
        _logger.warning("管理员权限检查未通过（原型阶段不阻塞，仅 WARNING）")

    # ============================================================
    # 第三步：初始化 NodeMonitor（被动接收心跳 + 超时降级）
    # ============================================================
    monitor = NodeMonitor(
        heartbeat_interval_s=float(net_cfg.heartbeat_interval_seconds),
        timeout_count=int(net_cfg.heartbeat_timeout_count),
        on_lost=_on_node_detect_lost,
        on_recovered=_on_node_detect_recovered,
    )
    monitor.start()

    # ============================================================
    # 第四步：启动 GrpcServerB（注入 NodeMonitor，A→B 心跳/退出通知）
    # ============================================================
    grpc_server = GrpcServerB(net_cfg, node_monitor=monitor)
    try:
        grpc_server.start()
    except RuntimeError as e:
        # 绑定失败（local_ip 未在本机网卡配置，Windows 开发环境常见）→ 不阻塞，仅诊断
        _logger.warning("GrpcServerB 启动失败: %s", e)
        _logger.warning("gRPC 服务端未启动（无法接收检测节点 心跳），推理功能受限")

    # ============================================================
    # 第四步 B：初始化 NetworkMonitor（运行时动态监测网络状态）
    # ============================================================
    # 网络状态由 NetworkMonitor 实时监测（watchdog 线程），在线/降级动态翻转，
    # 不再由启动时的单次探测决定 degraded 布尔。
    network_monitor = NetworkMonitor(
        net_cfg,
        on_online=_on_network_online,
        on_degraded=_on_network_degraded,
    )
    network_monitor.start()
    if network_monitor.is_online():
        _logger.info("网络初始状态: 在线（推理功能可用）")
    else:
        _logger.warning("网络初始状态: 降级（推理功能禁用，网络恢复后自动上线）")

    # ============================================================
    # 第五步：创建 Tkinter root + RebarMeasureApp（Step 5 接入）
    # ============================================================
    try:
        import tkinter as tk
        from utils.ui.app import RebarMeasureApp  # Step 5 实现
    except ImportError as e:
        # GUI 模块尚未实现（Step 5 待接入），占位终止
        _logger.warning("GUI 模块（utils.ui.app.RebarMeasureApp）尚未实现，待 Step 5 接入: %s", e)
        _logger.warning(
            "bootstrap 在此终止（配置加载 + 自检 + NodeMonitor + GrpcServerB + NetworkMonitor 已跑通）"
        )
        grpc_server.stop()
        monitor.stop()
        network_monitor.stop()
        return

    # ============================================================
    # 第六步 & 第七步：启动 App + 进入 Tkinter mainloop
    # ============================================================
    # GUI 启动前置检查（DISPLAY / XAUTHORITY）
    # systemd 服务进程默认无头环境，需显式注入 DISPLAY 与 XAUTHORITY
    # 详见 service.json 的 environment 字段配置
    _display = os.environ.get("DISPLAY")
    if not _display:
        _logger.error("缺少 DISPLAY 环境变量，无法启动 GUI")
        _logger.error(
            "前台启动: sudo DISPLAY=:1 "
            "XAUTHORITY=/run/user/1200/gdm/Xauthority "
            "/opt/node_server/.venv/bin/python /opt/node_server/main.py"
        )
        _logger.error(
            "systemd 服务: 检查 config/service.json 的 environment 字段"
            "是否配置 DISPLAY 与 XAUTHORITY"
        )
        grpc_server.stop()
        monitor.stop()
        network_monitor.stop()
        return
    _xauth = os.environ.get("XAUTHORITY")
    if _xauth and not os.path.exists(_xauth):
        _logger.warning(
            "XAUTHORITY 文件不存在: %s（桌面会话可能未登录）",
            _xauth,
        )
        _logger.warning("tk.Tk() 可能失败，继续尝试启动...")

    root = tk.Tk()
    # 注：RebarMeasureApp 的构造签名由 Step 5 定义，此处按预期调用
    # degraded 初始值取 NetworkMonitor 实时状态（网络在线=非降级）；
    # 运行时的在线/降级翻转由 NetworkMonitor 回调驱动 app 动态切换。
    app = RebarMeasureApp(
        root,
        net_cfg=net_cfg,
        camera_cfg=camera_cfg,
        intrinsics_cfg=intrinsics_cfg,
        inference_cfg=inference_cfg,
        serial_cfg=serial_cfg,
        node_monitor=monitor,
        grpc_server=grpc_server,
        degraded=not network_monitor.is_online(),
        network_monitor=network_monitor,
    )
    if not network_monitor.is_online():
        app.enable_degraded_mode()

    try:
        app.start()
        _logger.info("服务节点 启动完成，进入 Tkinter mainloop")
        root.mainloop()
    finally:
        _logger.info("服务节点 退出，清理资源...")
        grpc_server.stop()
        monitor.stop()
        network_monitor.stop()
        _logger.info("服务节点 已退出")


def _on_node_detect_lost() -> None:
    """检测节点 离线回调（由 NodeMonitor 触发，标记降级模式）。"""
    _logger.warning("检测节点 离线，触发降级模式")


def _on_node_detect_recovered() -> None:
    """检测节点 恢复在线回调（由 NodeMonitor 触发）。"""
    _logger.info("检测节点 恢复在线，退出降级模式")


def _on_network_online() -> None:
    """网络恢复在线回调（由 NetworkMonitor 触发，供日志/审计）。

    实际的 GUI 降级状态切换由 RebarMeasureApp 内部监听 NetworkMonitor 完成，
    此处仅记录日志（bootstrap 不直接持有 app 实例引用，避免耦合）。
    """
    _logger.info("网络状态变化: 恢复在线（推理功能可用）")


def _on_network_degraded() -> None:
    """网络降级回调（由 NetworkMonitor 触发，供日志/审计）。"""
    _logger.warning("网络状态变化: 降级（推理功能禁用，网络恢复后自动上线）")
