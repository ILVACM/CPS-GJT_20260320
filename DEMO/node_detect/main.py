"""
检测节点 入口（main.py）。

遵循 Design-AI_detect.md §4 七段式主循环 + §5 启动序列：
  1. argparse（run / start / status / stop / install 子命令）
  2. self_check — 启动环境自检（硬故障退出）
  3. load_model — 加载推理权重
  4. start_grpc_server — 启动 gRPC server（Infer 服务）
  5. start_heartbeat — 启动心跳推送（daemon 线程）
  6. install_signal_handlers — 注册 SIGINT/SIGTERM 清理钩子
  7. wait — 阻塞主线程直到 shutdown_event 被 set
  8. cleanup_resources — 清理 gRPC server + heartbeat + predictor + channel

接口契约：run 子命令完整启动序列，其余子命令用于 systemd 管理。
"""

import argparse
import logging
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Optional

# ============================================================
# 路径配置
# ============================================================

# 项目根目录
BASE_DIR: Path = Path(__file__).resolve().parent
CONFIG_DIR: Path = BASE_DIR / "config"
LOGS_DIR: Path = BASE_DIR / "logs"
WEIGHTS_DIR: Path = BASE_DIR / "weights"


# ============================================================
# 全局引用（main.py 生命周期内持有）
# ============================================================

_state_machine: Optional[object] = None      # StateMachine 实例
_predictor: Optional[object] = None          # RebarPredictor 实例
_grpc_server: Optional[object] = None        # GRPCServer 实例
_heartbeat_worker: Optional[object] = None   # HeartbeatWorker 实例
_node_server_client: Optional[object] = None      # NodeServerClient 实例

# 退出事件（跨线程共享）
_shutdown_event = threading.Event()

# 日志器
logger: logging.Logger = logging.getLogger("node_detect")


def _get_state_machine():
    """获取 StateMachine 实例（避免循环导入）"""
    from system.lifecycle import StateMachine
    return _state_machine


# ============================================================
# 子命令处理函数
# ============================================================

def cmd_run(args: argparse.Namespace) -> int:
    """
    子命令 run：完整启动序列（前台开发模式）。

    启动流程：
      load_config → init_logger → self_check → load_model → start_grpc → start_heartbeat → wait → cleanup
    """
    global _state_machine, _predictor, _grpc_server
    global _heartbeat_worker, _node_server_client

    # ---- 1. 加载配置 ----
    from system.config_loader import load_config
    try:
        config = load_config(str(CONFIG_DIR))
    except Exception as e:
        print(f"[启动] 配置加载失败: {e}", file=sys.stderr)
        return 1

    # ---- 2. 初始化日志 ----
    from system.logger import init_logger
    init_logger(
        log_dir=str(LOGS_DIR),
        log_level=config.log_level,
        log_file="node_detect.log",
    )

    logger.info("=" * 60)
    logger.info("检测节点 (node-detect) 启动序列开始")
    logger.info(f"版本: 0.1.0 | 配置: {config.to_safe_dict()}")
    logger.info("=" * 60)

    # ---- 3. 启动环境自检 ----
    from system.lifecycle import StateMachine, NodeState
    from system.self_check import run_self_check, SelfCheckError

    _state_machine = StateMachine(NodeState.INIT)
    try:
        run_self_check(config, strict_admin=False)
    except SelfCheckError as e:
        logger.critical(f"[自检] 失败 — 安全退出: {e}")
        return 1

    # ---- 4. 加载推理权重 ----
    from inference.predictor import RebarPredictor
    try:
        # model_path 支持绝对路径和相对路径；相对路径相对于项目根
        model_path = Path(config.model_path)
        if not model_path.is_absolute():
            model_path = BASE_DIR / model_path

        _predictor = RebarPredictor(
            model_path=str(model_path),
            use_cuda=config.use_cuda,
        )
    except FileNotFoundError as e:
        logger.critical(f"[模型] 权重文件不存在: {e}")
        return 1
    except RuntimeError as e:
        logger.critical(f"[模型] PyTorch 加载失败: {e}")
        return 1

    # ---- 5. 状态转换: INIT → IDLE ----
    _state_machine.transition(NodeState.IDLE)
    logger.info("[状态机] INIT → IDLE（推理就绪）")

    # ---- 6. 启动 gRPC server ----
    from grpc_server.server_factory import GRPCServer
    listen_addr = f"0.0.0.0:{config.grpc_port}"
    _grpc_server = GRPCServer(
        predictor=_predictor,
        listen_address=listen_addr,
        state_machine=_state_machine,
        max_workers=4,
    )
    try:
        _grpc_server.start()
    except RuntimeError as e:
        logger.critical(f"[gRPC] 启动失败: {e}")
        return 1

    # ---- 7. 启动心跳推送 ----
    from grpc_client.node_server_client import NodeServerClient
    from grpc_client.heartbeat_worker import HeartbeatWorker

    node_server_addr = f"{config.remote_ip}:{config.grpc_port}"
    _node_server_client = NodeServerClient(
        node_server_address=node_server_addr,
        node_id="node-detect",
    )
    _heartbeat_worker = HeartbeatWorker(
        node_server_client=_node_server_client,
        shutdown_event=_shutdown_event,
        interval_seconds=config.heartbeat_interval_seconds,
        status_provider=_state_machine.current_proto_status,
        infer_count_provider=lambda: 0,  # 待 servoicer 注入后由 servicer 提供
        infer_duration_provider=lambda: 0,
    )
    _heartbeat_worker.start()
    logger.info(
        f"[心跳] 推送线程已启动: {node_server_addr}, "
        f"间隔={config.heartbeat_interval_seconds}s"
    )

    # ---- 8. 注册信号处理器 ----
    _install_signal_handlers()

    # ---- 9. 阻塞等待退出信号 ----
    logger.info("[主循环] 阻塞等待退出信号（Ctrl+C / SIGTERM）...")
    _shutdown_event.wait()

    # ---- 10. 清理资源 ----
    _cleanup(reason="shutdown_signal")
    return 0


def cmd_start(args: argparse.Namespace) -> int:
    """
    子命令 start：后台启动（systemd 调用，委托给 run）。

    实际开发中通过 systemd service 启动；此子命令提供纯 Python 后台化入口。
    """
    print("[start] 请使用 systemd 管理或: python -m node_detect run &")
    return cmd_run(args)


def cmd_status(args: argparse.Namespace) -> int:
    """子命令 status：打印节点状态（查询 PID / 进程存活 / 端口监听）"""
    print("[status] 查询节点状态...")
    # 检查端口
    from system.config_loader import load_config
    import socket
    config = load_config(str(CONFIG_DIR))
    port = config.grpc_port
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(1)
        try:
            s.connect(("127.0.0.1", port))
            print(f"  端口 {port}: 监听中")
        except ConnectionRefusedError:
            print(f"  端口 {port}: 未监听")
        except Exception as e:
            print(f"  端口 {port}: 检测异常 {e}")
    return 0


def cmd_stop(args: argparse.Namespace) -> int:
    """子命令 stop：发送 SIGTERM 信号给运行中的节点"""
    print("[stop] 发送退出信号...")
    pid_file = BASE_DIR / "node_detect.pid"
    if pid_file.exists():
        pid = int(pid_file.read_text())
        try:
            os.kill(pid, signal.SIGTERM)
            print(f"  SIGTERM 已发送 (PID={pid})")
        except ProcessLookupError:
            print(f"  PID={pid} 不存在")
        except PermissionError:
            print(f"  权限不足，请使用 sudo")
    else:
        print("  PID 文件不存在（节点可能未运行）")
    return 0


def cmd_install(args: argparse.Namespace) -> int:
    """子命令 install：安装 systemd unit 文件（需 sudo）"""
    print("[install] 安装 systemd unit 文件...")
    # TODO: 读取 config/service_unit.json，生成 /etc/systemd/system/node-detect-inference.service
    print("  此功能尚未实现（原型阶段跳过）")
    return 0


def cmd_self_check(args: argparse.Namespace) -> int:
    """
    子命令 self-check：仅执行自检并打印结果。

    用于部署前验证环境是否就绪，不启动 gRPC 服务或加载权重。
    """
    from system.config_loader import load_config
    from system.lifecycle import StateMachine, NodeState
    from system.self_check import run_self_check
    from system.logger import init_logger

    # 基本日志输出（非文件，仅控制台）
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s | %(levelname)-7s | %(message)s")

    try:
        config = load_config(str(CONFIG_DIR))
    except Exception as e:
        print(f"[自检] 配置失败: {e}")
        return 1

    sm = StateMachine(NodeState.INIT)
    try:
        result = run_self_check(config, strict_admin=False)
        if result.all_ok:
            print("[自检] ✓ 全部通过")
            return 0
    except Exception as e:
        print(f"[自检] ✗ 失败: {e}")
        return 1
    return 0


# ============================================================
# 信号处理器与清理
# ============================================================

def _install_signal_handlers() -> None:
    """注册 SIGINT/SIGTERM 清理钩子"""

    def _handler(signum, frame):
        sig_name = signal.Signals(signum).name
        logger.info(f"[信号] 收到 {sig_name}，触发清理...")
        _shutdown_event.set()

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    logger.info("[信号] SIGINT/SIGTERM 处理器已注册")


def _cleanup(reason: str = "unknown") -> None:
    """
    清理资源（幂等设计，可安全重复调用）。

    清理顺序：
      1. 状态机 → SHUTDOWN（标记退出中）
      2. 心跳最后推送（Shutdown RPC best-effort）
      3. 停止心跳线程
      4. 停止 gRPC server
      5. 关闭 NodeServerClient channel
      6. 释放 CUDA 缓存
      7. 关闭日志
    """
    logger.info("=" * 60)
    logger.info(f"[清理] 开始（原因: {reason}）")
    logger.info("=" * 60)

    if _state_machine is not None:
        try:
            _state_machine.transition(__import__('system.lifecycle', fromlist=['NodeState']).NodeState.SHUTDOWN)
            logger.info("[清理] 状态机 → SHUTDOWN")
        except Exception as e:
            logger.warning(f"[清理] 状态机转换失败: {e}")

    # 最后心跳：通知服务节点 我们退出
    if _node_server_client is not None:
        try:
            _node_server_client.send_shutdown(reason=reason)
        except Exception as e:
            logger.warning(f"[清理] Shutdown 通知失败: {e}")

    # 停止心跳线程
    if _heartbeat_worker is not None:
        try:
            _heartbeat_worker.join(timeout=5)
            stats = _heartbeat_worker.stats
            logger.info(
                f"[清理] 心跳线程已停止 "
                f"(累计发送={stats.total_sent}, 失败={stats.total_failed})"
            )
        except Exception as e:
            logger.warning(f"[清理] 心跳停止异常: {e}")

    # 停止 gRPC server
    if _grpc_server is not None:
        try:
            _grpc_server.stop(grace=2.0)
            logger.info("[清理] gRPC server 已停止")
        except Exception as e:
            logger.warning(f"[清理] gRPC server 停止异常: {e}")

    # 关闭 NodeServerClient
    if _node_server_client is not None:
        try:
            _node_server_client.close()
            logger.info("[清理] NodeServerClient channel 已关闭")
        except Exception as e:
            logger.warning(f"[清理] NodeServerClient close 异常: {e}")

    # 释放 CUDA 缓存
    try:
        import torch
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            logger.info("[清理] CUDA 缓存已清空")
    except ImportError:
        pass

    # 关闭日志
    logging.shutdown()
    print(f"[清理] 完成（原因: {reason}）")


# ============================================================
# CLI 入口
# ============================================================

def build_parser() -> argparse.ArgumentParser:
    """构建 argparse 解析器"""
    parser = argparse.ArgumentParser(
        prog="node_detect",
        description="检测节点 — 钢筋直径测量推理服务",
    )
    subparsers = parser.add_subparsers(dest="command", help="子命令")

    # run：前台开发模式
    sub_run = subparsers.add_parser("run", help="前台启动（开发模式）")

    # start：后台启动
    sub_start = subparsers.add_parser("start", help="后台启动")

    # status：查询状态
    sub_status = subparsers.add_parser("status", help="查询节点状态")

    # stop：停止节点
    sub_stop = subparsers.add_parser("stop", help="停止节点")

    # install：安装 systemd unit
    sub_install = subparsers.add_parser("install", help="安装 systemd unit")

    # self-check：仅做自检
    sub_self_check = subparsers.add_parser(
        "self-check", help="执行启动环境自检（不启动服务）"
    )

    return parser


def main(argv: Optional[list] = None) -> int:
    """
    主入口函数。

    根据子命令调度相应处理函数。
    """
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command is None:
        parser.print_help()
        return 0

    dispatch = {
        "run": cmd_run,
        "start": cmd_start,
        "status": cmd_status,
        "stop": cmd_stop,
        "install": cmd_install,
        "self-check": cmd_self_check,
    }

    handler = dispatch.get(args.command)
    if handler is None:
        parser.print_help()
        return 1

    return handler(args)


# ============================================================
# 程序入口
# ============================================================

if __name__ == "__main__":
    sys.exit(main())
