"""argparse CLI 命令集（服务节点 备用交互手段）。

对齐 Design-server.md §3.10。

设计要点：
- 实现库：``argparse`` 标准库（零额外依赖），与 CliFusionMenu 的 input/print 风格一致
- 命令清单：``infer`` / ``config`` / ``status`` / ``exit`` / ``service`` / ``camera`` / ``fusion``
- CLI 在独立线程运行，共享状态变更经 ``root.after`` 调度 GUI 刷新（线程安全，§3.10.4）
- 支持除图像强相关外的所有功能（图像显示、可视化结果查看等仅 GUI 可用）

交互模式：
- CliRunner.run() 启动交互式 REPL（``node_server>`` 提示符），逐行解析命令
- 每行命令经 argparse 子命令解析后调用 App 对应公共方法
- 输入 ``exit`` 退出程序；``Ctrl+D`` / ``Ctrl+C`` 退出 REPL
"""
import argparse
import logging
import shlex
import sys
from typing import List, Optional

# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 ui）
_logger = logging.getLogger("node_server.ui")


class CliRunner:
    """CLI 命令交互式运行器。

    生命周期：
        runner = CliRunner(app)
        runner.run()  # 阻塞 REPL 循环
    """

    def __init__(self, app) -> None:
        """初始化 CLI 运行器。

        :param app: RebarMeasureApp 实例（提供公共 API 供 CLI 调用）
        """
        self._app = app
        self._parser: argparse.ArgumentParser = self._build_parser()
        # 退出请求标志：exit 命令触发后置 True，REPL 循环检测后退出
        self._exit_requested: bool = False

    # ------------------------------------------------------------------
    # 公共入口
    # ------------------------------------------------------------------

    def run(self) -> None:
        """启动交互式 REPL 循环。

        在 CLI 线程中阻塞运行，直到用户输入 ``exit`` 或 stdin 关闭。
        每行命令经 ``shlex.split`` 拆分后交 argparse 解析。
        ``exit`` 命令触发后置 ``_exit_requested``，REPL 循环检测后退出。
        """
        print("\n=== 服务节点 CLI 命令行接口 ===")
        print("输入 'help' 查看命令列表，'exit' 退出程序\n")

        while True:
            try:
                line: str = input("node_server> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nCLI 退出")
                return
            if not line:
                continue
            try:
                self.execute(line)
            except SystemExit:
                # argparse 在参数错误时调用 sys.exit；捕获后继续 REPL
                continue
            except Exception as e:
                print(f"[错误] 命令执行失败: {e}")
            # exit 命令触发后退出 REPL（cli_exit 已调度 root.destroy）
            if self._exit_requested:
                return

    def execute(self, line: str) -> str:
        """执行单行命令（供测试或外部调用）。

        :param line: 命令行字符串（如 ``"status"`` / ``"config display --mode overlay"``）
        :return: 执行结果摘要字符串
        """
        args: List[str] = shlex.split(line)
        if not args:
            return ""
        parsed = self._parser.parse_args(args)
        return self._dispatch(parsed)

    # ------------------------------------------------------------------
    # 内部：argparse 构建
    # ------------------------------------------------------------------

    def _build_parser(self) -> argparse.ArgumentParser:
        """构建 argparse 子命令解析器。"""
        parser = argparse.ArgumentParser(
            prog="node_server",
            description="服务节点 钢筋测量系统 CLI",
            add_help=True,
        )
        subparsers = parser.add_subparsers(
            dest="command", help="子命令",
        )

        # infer
        subparsers.add_parser(
            "infer", help="触发单帧识别（受 inference_interval_seconds 门控）",
        )

        # config
        config_parser = subparsers.add_parser("config", help="切换运行时配置")
        config_sub = config_parser.add_subparsers(dest="config_subcommand")
        # config fusion
        cf = config_sub.add_parser("fusion", help="切换激光融合策略")
        cf.add_argument(
            "--strategy", required=True,
            choices=["mean", "median", "specific", "manual"],
            help="融合策略",
        )
        cf.add_argument("--index", type=int, default=1, help="SPECIFIC 策略接口编号 1~4")
        cf.add_argument("--value", type=float, help="MANUAL 策略距离值(mm)")
        # config display
        cd = config_sub.add_parser("display", help="切换显示模式")
        cd.add_argument(
            "--mode", required=True,
            choices=["overlay", "panel"],
            help="显示模式",
        )
        # config save
        cs = config_sub.add_parser("save", help="设置保存方式")
        cs.add_argument("--csv", choices=["on", "off"], default="off", help="CSV 数值表")
        cs.add_argument("--mask", choices=["on", "off"], default="off", help="类别掩码图")
        cs.add_argument("--vis", choices=["on", "off"], default="on", help="可视化结果图(恒强制)")

        # status
        subparsers.add_parser("status", help="查看系统状态")

        # exit
        subparsers.add_parser("exit", help="退出程序")

        # service
        svc_parser = subparsers.add_parser("service", help="注册/卸载系统服务")
        svc_sub = svc_parser.add_subparsers(dest="service_subcommand")
        svc_sub.add_parser("install", help="注册开机自启")
        svc_sub.add_parser("uninstall", help="卸载开机自启")

        # camera
        cam_parser = subparsers.add_parser("camera", help="摄像头管理")
        cam_sub = cam_parser.add_subparsers(dest="camera_subcommand")
        cam_sub.add_parser("list", help="列出摄像头")
        cc = cam_sub.add_parser("connect", help="连接摄像头")
        cc.add_argument("--index", type=int, help="按索引连接（来自 camera list）")
        cc.add_argument("--host", help="网络摄像头 IP")
        cc.add_argument("--port", type=int, help="网络摄像头端口")

        # fusion
        subparsers.add_parser("fusion", help="进入激光融合策略交互菜单")

        return parser

    # ------------------------------------------------------------------
    # 内部：命令分发
    # ------------------------------------------------------------------

    def _dispatch(self, args: argparse.Namespace) -> str:
        """根据解析结果分发到对应处理函数。"""
        cmd: Optional[str] = getattr(args, "command", None)
        if cmd is None:
            self._parser.print_help()
            return ""

        handler = {
            "infer": self._cmd_infer,
            "config": self._cmd_config,
            "status": self._cmd_status,
            "exit": self._cmd_exit,
            "service": self._cmd_service,
            "camera": self._cmd_camera,
            "fusion": self._cmd_fusion,
        }.get(cmd)
        if handler is None:
            print(f"[错误] 未知命令: {cmd}")
            self._parser.print_help()
            return ""
        return handler(args)

    # ------------------------------------------------------------------
    # 各命令处理
    # ------------------------------------------------------------------

    def _cmd_infer(self, _args: argparse.Namespace) -> str:
        """infer 命令：触发单帧识别。"""
        result: str = self._app.request_measurement()
        print(f"[INFO] {result}")
        return result

    def _cmd_config(self, args: argparse.Namespace) -> str:
        """config 命令：切换运行时配置。"""
        sub: Optional[str] = getattr(args, "config_subcommand", None)
        if sub == "fusion":
            strategy: str = args.strategy.upper()
            specific_index: int = max(0, args.index - 1)
            manual_value: Optional[float] = args.value
            result: str = self._app.set_fusion_strategy(
                strategy=strategy,
                specific_index=specific_index,
                manual_value=manual_value,
            )
            print(f"[INFO] {result}")
            return result
        elif sub == "display":
            mode: str = args.mode
            result = self._app.set_display_mode(mode)
            print(f"[INFO] {result}")
            return result
        elif sub == "save":
            csv_on: bool = args.csv == "on"
            mask_on: bool = args.mask == "on"
            vis_on: bool = args.vis == "on"
            result = self._app.set_save_options(csv=csv_on, mask=mask_on, vis=vis_on)
            print(f"[INFO] {result}")
            return result
        else:
            print("[错误] 请指定 config 子命令: fusion / display / save")
            return ""

    def _cmd_status(self, _args: argparse.Namespace) -> str:
        """status 命令：输出当前系统状态。"""
        status: dict = self._app.get_status()
        lines: List[str] = [
            "=== 系统状态 ===",
            f"  检测节点 在线:    {'是' if status['node_detect_online'] else '否'}",
            f"  检测节点 状态码:  {status['node_detect_status']}",
            f"  降级模式:       {'是' if status['degraded_mode'] else '否'}",
            f"  摄像头已连接:   {'是' if status['camera_connected'] else '否'}",
            f"  摄像头来源:     {status['camera_source']}",
            f"  摄像头协议:     {status['camera_protocol']}",
            f"  融合策略:       {status['fusion_strategy']}",
            f"  显示模式:       {status['display_mode']}",
            f"  保存选项:       csv={status['save_options'].get('csv')}, "
            f"mask={status['save_options'].get('mask')}",
            f"  推理间隔(s):    {status['inference_interval_s']}",
            f"  当前 FPS:       {status['fps']}",
            f"  历史记录数:     {status['history_count']}",
            f"  串口:           {status['serial_port']}",
            "================",
        ]
        text: str = "\n".join(lines)
        print(text)
        return text

    def _cmd_exit(self, _args: argparse.Namespace) -> str:
        """exit 命令：触发 CLI 紧急退出路径（§10.1 CLI 退出）。

        与 GUI 关闭按钮（``_on_closing``）的区别：
        - 跳过运行时状态保存（CLI 为紧急退出，不保存配置）
        - best-effort 通知检测节点（``cli_exit`` 内 deadline=3s，失败不阻塞）
        - 立即退出：``cli_exit`` 完成清理后调度 ``root.destroy``，本方法置
          ``_exit_requested`` 标志，REPL 循环检测后退出 CLI 线程

        ``cli_exit`` 在 CLI 线程同步执行清理（含 B→A 退出通知与资源释放），
        完成后回主线程 ``root.destroy`` 销毁窗口，mainloop 返回后进程退出。
        """
        result: str = self._app.exit_app()
        print(f"[INFO] {result}")
        # 标记退出请求，REPL 循环检测后退出 CLI 线程
        self._exit_requested = True
        return result

    def _cmd_service(self, args: argparse.Namespace) -> str:
        """service 命令：注册/卸载系统服务。"""
        sub: Optional[str] = getattr(args, "service_subcommand", None)
        mgr = self._app.get_service_manager()
        if sub == "install":
            print("[INFO] 注册系统服务...")
            ok, msg = mgr.install()
            tag = "[OK]" if ok else "[失败]"
            print(f"{tag} {msg}")
            return f"install: {'ok' if ok else 'fail'}"
        elif sub == "uninstall":
            print("[INFO] 卸载系统服务...")
            ok, msg = mgr.uninstall()
            tag = "[OK]" if ok else "[失败]"
            print(f"{tag} {msg}")
            return f"uninstall: {'ok' if ok else 'fail'}"
        else:
            print("[错误] 请指定 service 子命令: install / uninstall")
            return ""

    def _cmd_camera(self, args: argparse.Namespace) -> str:
        """camera 命令：摄像头管理。"""
        sub: Optional[str] = getattr(args, "camera_subcommand", None)
        if sub == "list":
            entries: List[dict] = self._app.get_camera_list()
            if not entries:
                print("[INFO] 未发现摄像头（请先在 GUI 中刷新扫描）")
                return "empty"
            print("=== 摄像头列表 ===")
            for i, entry in enumerate(entries):
                print(f"  [{i}] {entry['display']}")
            print("==================")
            return f"{len(entries)} 个摄像头"
        elif sub == "connect":
            if args.host is not None and args.port is not None:
                result: str = self._app.connect_network_camera(args.host, args.port)
                print(f"[INFO] {result}")
                return result
            elif args.index is not None:
                result = self._app.connect_camera_by_index(args.index)
                print(f"[INFO] {result}")
                return result
            else:
                print("[错误] 请指定 --index 或 --host/--port")
                return ""
        else:
            print("[错误] 请指定 camera 子命令: list / connect")
            return ""

    def _cmd_fusion(self, _args: argparse.Namespace) -> str:
        """fusion 命令：进入激光融合策略交互菜单。"""
        result: str = self._app.enter_fusion_menu()
        print(f"[INFO] {result}")
        return result


# ==================================================================
# 单次命令执行入口（供外部脚本或 ``python -m ui.cli`` 调用）
# ==================================================================

def run_one_shot(app, command_line: str) -> str:
    """单次执行一条 CLI 命令（不进入 REPL）。

    :param app: RebarMeasureApp 实例
    :param command_line: 命令行字符串（如 ``"status"``）
    :return: 执行结果摘要
    """
    runner: CliRunner = CliRunner(app)
    return runner.execute(command_line)
