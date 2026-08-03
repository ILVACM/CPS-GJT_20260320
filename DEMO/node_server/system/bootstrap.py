"""启动引导：配置加载 → 环境自检 → NodeMonitor → GrpcServerB → GUI（Step 5 接入）。

对齐 Design-server.md §0.1 / §5 与 AGENTS.md §4.3。

严格顺序：
1. 第一步：读取加载 JSON 配置（logging → network → camera → intrinsics → inference）
2. 第二步：环境权限检查（run_self_check）
   - 硬故障（ip/port/net_priv）→ 记录 CRITICAL 日志 + sys.exit(1)
   - 管理员权限：条件性，非管理员 WARNING 不阻塞（原型阶段核心运行无需特权）
   - 软故障（peer_reachable）→ 进入降级模式
3. 初始化 NodeMonitor（被动接收心跳 + 超时降级）
4. 启动 GrpcServerB（注入 NodeMonitor）
5. 创建 Tkinter root + RebarMeasureApp（Step 5 实现，此处 try/except 占位）
6. 启动 App
7. 进入 Tkinter mainloop

异常处理：任一硬前置失败 → 记录 CRITICAL 日志 + sys.exit(1)。
降级模式：peer_reachable=False 时，App 仍启动但识别功能禁用。
"""
import logging
import os
import sys

from common.config_loader import (
    CameraConfig,
    ConfigLoader,
    InferenceConfig,
    IntrinsicsConfig,
    LoggingConfig,
    NetworkConfig,
)
from common.logging_setup import setup_logging
from grpc_client.grpc_server import GrpcServerB
from system.node_monitor import NodeMonitor
from system.self_check import run_self_check

# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 system）
_logger = logging.getLogger("node_server.system")

# 配置文件路径（相对 node_server/ 工作目录，对齐 §3.3 config/ 目录约定）
_CONFIG_DIR = "config"
_LOGGING_CFG_PATH = os.path.join(_CONFIG_DIR, "logging.json")
_NETWORK_CFG_PATH = os.path.join(_CONFIG_DIR, "network.json")
_CAMERA_CFG_PATH = os.path.join(_CONFIG_DIR, "camera.json")
_INTRINSICS_CFG_PATH = os.path.join(_CONFIG_DIR, "intrinsics.json")
_INFERENCE_CFG_PATH = os.path.join(_CONFIG_DIR, "inference.json")
_LOG_PATH = "logs/node_server.log"


def bootstrap() -> None:
    """启动引导入口。

    严格按 §5 五阶段生命周期顺序执行；任何硬前置失败 → CRITICAL + sys.exit(1)。
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

    # ============================================================
    # 第二步：环境权限检查
    # ============================================================
    _logger.info("执行启动环境自检...")
    check_result = run_self_check(net_cfg)

    # 硬故障：IP 不匹配 / 端口冲突 / 网络配置权限缺失 → 退出
    hard_failures: list = []
    if not check_result.ip_ok:
        hard_failures.append("本机 IP 校验失败")
    if not check_result.port_ok:
        hard_failures.append("gRPC 端口可用性检查失败")
    if not check_result.net_priv_ok:
        hard_failures.append("网络配置权限检查失败")

    if hard_failures:
        for f in hard_failures:
            _logger.critical("[硬故障] %s", f)
        _logger.critical("启动环境自检存在硬故障，服务节点 终止启动")
        sys.exit(1)

    # 管理员权限：条件性，非管理员 WARNING 不阻塞
    # （原型阶段应用核心运行无需特权：gRPC 端口 50051 > 1024，无需特权端口绑定；
    #   仅 ServiceManager 注册系统服务时需特权，由 ServiceManager 自行校验）
    if not check_result.admin_ok:
        _logger.warning("管理员权限检查未通过（原型阶段不阻塞，仅 WARNING）")

    # 软故障：对端不可达 → 降级模式
    degraded: bool = not check_result.peer_reachable
    if degraded:
        _logger.warning("对端检测节点 不可达，进入降级模式（仅视频 + 激光，推理功能禁用）")
    else:
        _logger.info("对端检测节点 可达，正常模式启动")

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
        # 绑定失败（local_ip 未在本机网卡配置，Windows 开发环境常见）→ 降级继续
        _logger.warning("GrpcServerB 启动失败: %s", e)
        _logger.warning("gRPC 服务端未启动（无法接收检测节点 心跳），进入降级模式")
        degraded = True

    # ============================================================
    # 第五步：创建 Tkinter root + RebarMeasureApp（Step 5 接入）
    # ============================================================
    try:
        import tkinter as tk
        from ui.app import RebarMeasureApp  # Step 5 实现
    except ImportError as e:
        # GUI 模块尚未实现（Step 5 待接入），占位终止
        _logger.warning("GUI 模块（ui.app.RebarMeasureApp）尚未实现，待 Step 5 接入: %s", e)
        _logger.warning(
            "bootstrap 在此终止（配置加载 + 自检 + NodeMonitor + GrpcServerB 已跑通）"
        )
        grpc_server.stop()
        monitor.stop()
        return

    # ============================================================
    # 第六步 & 第七步：启动 App + 进入 Tkinter mainloop
    # ============================================================
    root = tk.Tk()
    # 注：RebarMeasureApp 的构造签名由 Step 5 定义，此处按预期调用
    app = RebarMeasureApp(
        root,
        net_cfg=net_cfg,
        camera_cfg=camera_cfg,
        intrinsics_cfg=intrinsics_cfg,
        inference_cfg=inference_cfg,
        node_monitor=monitor,
        grpc_server=grpc_server,
        degraded=degraded,
    )
    if degraded:
        app.enable_degraded_mode()

    try:
        app.start()
        _logger.info("服务节点 启动完成，进入 Tkinter mainloop")
        root.mainloop()
    finally:
        _logger.info("服务节点 退出，清理资源...")
        grpc_server.stop()
        monitor.stop()
        _logger.info("服务节点 已退出")


def _on_node_detect_lost() -> None:
    """检测节点 离线回调（由 NodeMonitor 触发，标记降级模式）。"""
    _logger.warning("检测节点 离线，触发降级模式")


def _on_node_detect_recovered() -> None:
    """检测节点 恢复在线回调（由 NodeMonitor 触发）。"""
    _logger.info("检测节点 恢复在线，退出降级模式")
