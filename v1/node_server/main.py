"""服务节点 程序入口。

启动引导由 system.bootstrap 模块承担（Step 4 实现），
本入口支持以下子命令：
- run / 无参数：启动引导（默认行为，兼容旧调用）
- setup-network：配置静态 IP（独立工具，对齐 AGENTS.md §4.2）
- self-check：仅执行环境自检，不启动服务
- status：查询节点状态（端口监听）
"""
import argparse
import sys

from utils.system.bootstrap import bootstrap


def build_parser() -> argparse.ArgumentParser:
    """构建 argparse 解析器"""
    parser = argparse.ArgumentParser(
        prog="node_server",
        description="服务节点 — 钢筋直径测量主控服务",
    )
    subparsers = parser.add_subparsers(dest="command", help="子命令")

    # run（默认）：启动引导
    subparsers.add_parser("run", help="启动引导（默认）")

    # setup-network：一键式网络配置工具（独立工具，需 sudo）
    sp_network = subparsers.add_parser(
        "setup-network",
        help="一键式网络配置：备份+设置静态IP+启动接口+重启服务（需 sudo，详见 QUICKSTART.md §5.4）",
    )
    sp_network.add_argument(
        "--ip", help="临时指定 IP 地址（仅本次生效，不修改配置文件；默认取 network.json local_ip）"
    )
    sp_network.add_argument(
        "--netmask", help="临时指定子网掩码（仅本次生效；默认取 network.json netmask）"
    )
    sp_network.add_argument(
        "--gateway", help="临时指定网关地址（仅本次生效；默认取 network.json gateway，未配置则推导为 IP 网段 .1）"
    )
    sp_network.add_argument(
        "--restore", action="store_true", help="复原模式：从备份恢复原配置（与设置参数互斥）"
    )
    sp_network.add_argument(
        "--no-start-interface", action="store_true", help="配置后不自动启动网络接口"
    )
    sp_network.add_argument(
        "--no-restart-service", action="store_true", help="配置后不自动重启网络服务"
    )

    # self-check：仅自检
    subparsers.add_parser("self-check", help="执行启动环境自检（不启动服务）")

    # status：查询状态
    subparsers.add_parser("status", help="查询节点状态（端口监听）")

    return parser


def cmd_run() -> int:
    """子命令 run：启动引导（默认行为）。"""
    bootstrap()
    return 0


def cmd_setup_network(args: argparse.Namespace) -> int:
    """子命令 setup-network：一键式网络配置（备份+设置+启动+重启），支持复原与临时参数覆盖。"""
    from utils.system.network_setup import SetupOptions, run_setup_network

    opts = SetupOptions(
        ip=args.ip,
        netmask=args.netmask,
        gateway=args.gateway,
        restore=args.restore,
        start_interface=not args.no_start_interface,
        restart_service=not args.no_restart_service,
    )
    return run_setup_network(opts=opts)


def cmd_self_check() -> int:
    """子命令 self-check：仅执行环境自检（不启动服务）。

    用于部署前验证环境是否就绪。退出码：0 全部通过，1 失败。
    """
    import logging
    from utils.common.config_loader import ConfigLoader
    from utils.system.self_check import run_self_check

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)-7s | %(message)s",
    )

    try:
        net_cfg = ConfigLoader.load_network_config("config/network.json")
    except Exception as e:
        print(f"[self-check] 配置加载失败: {e}", file=sys.stderr)
        return 1

    result = run_self_check(net_cfg)

    # 打印详细结果
    print("\n" + "=" * 60)
    print("自检结果汇总")
    print("=" * 60)
    for msg in result.messages:
        print(f"  {msg}")

    hard_ok = result.ip_ok and result.port_ok and result.net_priv_ok
    if hard_ok:
        print("\n[自检] ✓ 硬前置全部通过")
        if not result.peer_reachable:
            print("[自检] ⚠ 对端不可达（软故障，允许降级运行）")
        return 0
    else:
        print("\n[自检] ✗ 存在硬故障，禁止启动服务", file=sys.stderr)
        if not result.ip_ok:
            print(
                "[自检] 修复指引：本机 IP 与配置 local_ip 不一致，"
                "请执行 `sudo /opt/node_server/.venv/bin/python "
                "/opt/node_server/main.py setup-network` 自动配置静态 IP",
                file=sys.stderr,
            )
        return 1


def cmd_status() -> int:
    """子命令 status：查询端口监听状态。"""
    import socket
    from utils.common.config_loader import ConfigLoader
    try:
        net_cfg = ConfigLoader.load_network_config("config/network.json")
    except Exception as e:
        print(f"[status] 配置加载失败: {e}", file=sys.stderr)
        return 1
    port = net_cfg.grpc_port
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


def main(argv=None) -> int:
    """主入口函数。根据子命令调度。"""
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command in (None, "run"):
        return cmd_run()
    elif args.command == "setup-network":
        return cmd_setup_network(args)
    elif args.command == "self-check":
        return cmd_self_check()
    elif args.command == "status":
        return cmd_status()
    else:
        parser.print_help()
        return 1


if __name__ == "__main__":
    sys.exit(main())
