"""RebarMeasureApp — 服务节点 主应用（tkinter GUI）。

对齐 Design-server.md §3.7.3 / §0.4（九步循环）/ §3.7.4-3.7.7 与 AGENTS.md §7.2（线程安全）。

构造签名严格匹配 system/bootstrap.py 预期：
    app = RebarMeasureApp(root, net_cfg=..., camera_cfg=..., intrinsics_cfg=...,
                          inference_cfg=..., node_monitor=..., grpc_server=...,
                          degraded=...)
    if degraded: app.enable_degraded_mode()
    app.start()

线程模型（AGENTS.md §7.2）：
- 主线程：tkinter 事件循环、所有 widget 操作、绘制
- 主循环线程（daemon）：取帧 + 渲染计算，经 root.after(0, ...) 投递到主线程
- 测量工作线程（daemon 临时）：推理 + 测量 + 保存，结果经 root.after 投递
- CLI 线程（daemon）：交互式命令，共享状态变更经 root.after 刷新 GUI
"""
import logging
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np
import tkinter as tk
from tkinter import messagebox, ttk
from PIL import Image, ImageTk

from camera.base import BaseCameraInput
from camera.mock import MockCameraInput
from camera.network_camera import NetworkCameraInput
from camera.orbbec_336l import Orbbec336LInput
from camera.scanner import CameraDeviceInfo, LocalCameraScanner
from common.config_loader import (
    CameraConfig,
    InferenceConfig,
    IntrinsicsConfig,
    NetworkConfig,
)
from grpc_client.grpc_server import GrpcServerB
from grpc_client.inference_client import InferenceGrpcClient, InferenceResult
from measure.intrinsics_manager import IntrinsicsManager
from measure.rebar_measure import MeasureResult, RebarMeasure
from serial.laser_fusion import FusionStrategy, LaserFusion
from serial.laser_parser import LaserParseResult
from serial.serial_manager import SerialManager
from storage.result_store import ResultStore
from system.exit_coordinator import ExitCoordinator
from system.node_monitor import NodeMonitor
from system.service_manager import ServiceManager
from ui.components.node_status_indicator import NodeStatusIndicator
from ui.render import MODE_OVERLAY, MODE_PANEL, UIRenderer

# 模块级 logger（对齐 AGENTS.md §7.5 模块命名空间 ui）
_logger = logging.getLogger("node_server.ui")

# 识别按钮四态
BTN_READY: str = "ready"
BTN_LOADING: str = "loading"
BTN_RESULT: str = "result"
BTN_DISABLED: str = "disabled"

# Loading 动画帧
_LOADING_FRAMES: Tuple[str, ...] = ("●○○", "○●○", "○○●")

# 显示尺寸（16:9）
_DISPLAY_W: int = 960
_DISPLAY_H: int = 540

# 按钮画布尺寸
_BTN_SIZE: int = 80

# 主循环目标帧率
_TARGET_FPS: int = 30


class RebarMeasureApp:
    """服务节点 主应用类（tkinter GUI 主线程）。

    整合所有子模块，驱动采集 → 推理 → 测量 → 显示全链路。
    所有 widget 操作在主线程执行；子线程仅取帧/计算，经 ``root.after`` 投递。
    """

    def __init__(
        self,
        root: tk.Tk,
        net_cfg: NetworkConfig,
        camera_cfg: CameraConfig,
        intrinsics_cfg: IntrinsicsConfig,
        inference_cfg: InferenceConfig,
        node_monitor: NodeMonitor,
        grpc_server: GrpcServerB,
        degraded: bool = False,
        **kwargs: Any,
    ) -> None:
        """初始化主应用。

        :param root: tkinter Tk 实例
        :param net_cfg: 网络配置
        :param camera_cfg: 相机配置
        :param intrinsics_cfg: 内参配置
        :param inference_cfg: 推理节奏配置
        :param node_monitor: 检测节点 状态监测器
        :param grpc_server: 服务节点 gRPC 服务端
        :param degraded: 是否降级模式启动
        :param kwargs: 可选注入项（inference_client 用于测试注入 mock 客户端）
        """
        self._root: tk.Tk = root
        self._net_cfg: NetworkConfig = net_cfg
        self._camera_cfg: CameraConfig = camera_cfg
        self._intrinsics_cfg: IntrinsicsConfig = intrinsics_cfg
        self._inference_cfg: InferenceConfig = inference_cfg
        self._node_monitor: NodeMonitor = node_monitor
        self._grpc_server: GrpcServerB = grpc_server
        self._degraded_mode: bool = bool(degraded)

        # 可选注入：推理客户端（测试时传 mock）
        injected_client: Optional[InferenceGrpcClient] = kwargs.get("inference_client")
        self._inference_client: InferenceGrpcClient = (
            injected_client if injected_client is not None else InferenceGrpcClient(net_cfg)
        )

        # ---- 子模块 ----
        self._serial_mgr: SerialManager = SerialManager()
        self._fusion: LaserFusion = LaserFusion()
        self._intrinsics_mgr: IntrinsicsManager = IntrinsicsManager(intrinsics_cfg)
        self._rebar_measure: RebarMeasure = RebarMeasure(self._intrinsics_mgr)
        self._renderer: UIRenderer = UIRenderer()
        self._result_store: ResultStore = ResultStore()
        self._service_mgr: ServiceManager = ServiceManager()
        self._scanner: LocalCameraScanner = LocalCameraScanner(
            camera_cfg.compatible_models
        )

        # ---- 共享状态（Lock 保护） ----
        self._lock: threading.Lock = threading.Lock()
        self._is_running: bool = False
        self._frame_id: int = 0
        self._last_infer_time: float = 0.0
        self._latest_mask: Optional[np.ndarray] = None
        self._latest_measure: Optional[MeasureResult] = None

        # 冻结状态（推理时冻结画面）
        self._frozen: bool = False
        self._frozen_frame: Optional[np.ndarray] = None
        self._frozen_distance: Optional[float] = None
        self._frozen_frame_id: int = 0

        # 按钮状态
        self._button_state: str = BTN_DISABLED

        # 显示与保存设置
        self._display_mode: str = MODE_PANEL
        self._save_options: Dict[str, bool] = {"csv": True, "mask": True}

        # 副页面状态
        self._settings_open: bool = False
        self._results_open: bool = False
        self._settings_window: Optional[tk.Toplevel] = None
        self._results_window: Optional[tk.Toplevel] = None

        # 相机状态
        self._camera: Optional[BaseCameraInput] = None
        self._camera_source: str = "None"
        self._camera_protocol: str = "None"
        self._camera_entries: List[dict] = []
        self._camera_listbox: Optional[tk.Listbox] = None

        # 历史记录（推理结果副页面数据源）
        self._history_records: List[dict] = []

        # FPS 统计
        self._fps: int = 0
        self._fps_frame_count: int = 0
        self._fps_last_time: float = time.time()

        # Loading 动画索引
        self._loading_idx: int = 0

        # 线程引用
        self._main_loop_thread: Optional[threading.Thread] = None
        self._cli_thread: Optional[threading.Thread] = None

        # 停止标志（§7.4 资源释放顺序 step 1；与 _is_running 并行使用）
        self._stop_event: threading.Event = threading.Event()

        # 退出协调器（统一 GUI / CLI 退出行为，§10.2 / §10.5 / §7.4）
        self._exit_coordinator: ExitCoordinator = ExitCoordinator(
            node_monitor=node_monitor,
            inference_client=self._inference_client,
        )
        # 退出进行中标志（防止 GUI 关闭与 CLI exit 并发触发重复清理）
        self._exit_lock: threading.Lock = threading.Lock()
        self._exiting: bool = False

        # 当前显示的 PhotoImage 引用（防止 GC 回收）
        self._current_photo: Optional[ImageTk.PhotoImage] = None

        # ---- UI 构建 ----
        root.title("钢筋直径测量系统 — 服务节点 主控")
        root.geometry("1280x800")
        self._setup_ui()
        root.protocol("WM_DELETE_WINDOW", self._on_closing)

        # 初始按钮状态
        self._recompute_button_state()

    # ==================================================================
    # UI 布局
    # ==================================================================

    def _setup_ui(self) -> None:
        """初始化主界面布局：顶栏 + 主区域（两状态） + 底栏。"""
        # 顶栏：节点状态指示器 + 副页面入口 + 降级提示
        self._top_bar: tk.Frame = tk.Frame(self._root, height=44, bg="#2c3e50")
        self._top_bar.pack(side=tk.TOP, fill=tk.X, padx=0, pady=0)

        # 节点状态指示器（左）
        self._node_indicator: NodeStatusIndicator = NodeStatusIndicator(
            self._top_bar,
            status_provider=self._get_node_status_info,
        )
        self._node_indicator.pack(side=tk.LEFT, padx=8, pady=10)

        tk.Label(
            self._top_bar, text="检测节点",
            bg="#2c3e50", fg="white", font=("Arial", 10),
        ).pack(side=tk.LEFT, padx=2)

        # 副页面入口（右）
        self._settings_btn: ttk.Button = ttk.Button(
            self._top_bar, text="设置", command=self._open_settings,
        )
        self._settings_btn.pack(side=tk.RIGHT, padx=8, pady=8)

        self._results_btn: ttk.Button = ttk.Button(
            self._top_bar, text="推理结果", command=self._open_results,
        )
        self._results_btn.pack(side=tk.RIGHT, padx=4, pady=8)

        # 降级横幅标签（中间，默认隐藏）
        self._degraded_label: tk.Label = tk.Label(
            self._top_bar,
            text=" ⚠ 推理服务离线 — 降级模式 ",
            bg="#f59e0b", fg="black",
            font=("Arial", 10, "bold"),
        )
        if self._degraded_mode:
            self._degraded_label.pack(side=tk.LEFT, padx=12, pady=8)

        # ---- 主区域容器 ----
        self._main_container: tk.Frame = tk.Frame(self._root, bg="#1a1a2e")
        self._main_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # 状态 1：摄像头选择
        self._camera_selection_frame: tk.Frame = tk.Frame(
            self._main_container, bg="#1a1a2e"
        )
        self._setup_camera_selection_ui(self._camera_selection_frame)

        # 状态 2：识别工作
        self._working_frame: tk.Frame = tk.Frame(
            self._main_container, bg="#1a1a2e"
        )
        self._setup_working_ui(self._working_frame)

        # 默认显示摄像头选择状态
        self._show_camera_selection()

        # ---- 底栏：状态栏 ----
        self._status_bar: tk.Label = tk.Label(
            self._root,
            text="就绪",
            anchor=tk.W,
            bg="#34495e", fg="white",
            font=("Arial", 9),
            padx=8, pady=4,
        )
        self._status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def _setup_camera_selection_ui(self, parent: tk.Frame) -> None:
        """摄像头选择状态界面。"""
        tk.Label(
            parent, text="摄像头选择",
            bg="#1a1a2e", fg="white",
            font=("Arial", 16, "bold"),
        ).pack(pady=16)

        # 刷新扫描按钮
        ttk.Button(parent, text="刷新扫描", command=self._refresh_camera_list).pack(pady=4)

        # 摄像头列表
        list_frame: tk.Frame = tk.Frame(parent, bg="#1a1a2e")
        list_frame.pack(pady=8, padx=40, fill=tk.BOTH, expand=True)
        self._camera_listbox = tk.Listbox(
            list_frame, width=80, height=12,
            font=("Consolas", 11),
            selectbackground="#3b82f6",
        )
        self._camera_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar: ttk.Scrollbar = ttk.Scrollbar(
            list_frame, orient=tk.VERTICAL, command=self._camera_listbox.yview
        )
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self._camera_listbox.config(yscrollcommand=scrollbar.set)

        # 手动添加网络摄像头
        net_frame: tk.Frame = tk.Frame(parent, bg="#1a1a2e")
        net_frame.pack(pady=8)
        tk.Label(
            net_frame, text="手动添加网络摄像头: IP",
            bg="#1a1a2e", fg="white",
        ).pack(side=tk.LEFT, padx=4)
        self._net_ip_entry: tk.Entry = tk.Entry(
            net_frame, width=16, font=("Arial", 10)
        )
        self._net_ip_entry.insert(0, "192.168.1.100")
        self._net_ip_entry.pack(side=tk.LEFT, padx=4)
        tk.Label(
            net_frame, text="端口", bg="#1a1a2e", fg="white",
        ).pack(side=tk.LEFT, padx=4)
        self._net_port_entry: tk.Entry = tk.Entry(
            net_frame, width=6, font=("Arial", 10)
        )
        self._net_port_entry.insert(0, "8080")
        self._net_port_entry.pack(side=tk.LEFT, padx=4)
        ttk.Button(
            net_frame, text="添加", command=self._add_network_camera
        ).pack(side=tk.LEFT, padx=8)

        # 连接按钮
        ttk.Button(
            parent, text="选择并连接", command=self._connect_selected_camera
        ).pack(pady=12)

    def _setup_working_ui(self, parent: tk.Frame) -> None:
        """识别工作状态界面。"""
        # 视频显示区
        self._video_label: tk.Label = tk.Label(
            parent, bg="black",
            width=_DISPLAY_W, height=_DISPLAY_H,
        )
        self._video_label.pack(pady=8)

        # 识别按钮（tk.Canvas 圆形按钮）
        self._button_canvas: tk.Canvas = tk.Canvas(
            parent, width=_BTN_SIZE, height=_BTN_SIZE,
            highlightthickness=0, bg="#1a1a2e",
        )
        self._button_canvas.pack(pady=8)
        self._button_canvas.bind("<Button-1>", lambda _e: self._on_button_click())
        self._draw_button()

    # ==================================================================
    # 状态切换
    # ==================================================================

    def _show_camera_selection(self) -> None:
        """切换到摄像头选择状态。"""
        self._working_frame.pack_forget()
        self._camera_selection_frame.pack(fill=tk.BOTH, expand=True)
        self._refresh_camera_list()

    def _show_working_state(self) -> None:
        """切换到识别工作状态。"""
        self._camera_selection_frame.pack_forget()
        self._working_frame.pack(fill=tk.BOTH, expand=True)

    # ==================================================================
    # 摄像头操作
    # ==================================================================

    def _refresh_camera_list(self) -> None:
        """刷新摄像头扫描列表。"""
        if self._camera_listbox is None:
            return
        self._camera_listbox.delete(0, tk.END)
        self._camera_entries.clear()

        # Mock 相机（始终可用，调试用）
        self._camera_entries.append({
            "display": "Mock: 调试用黑帧 (1920x1080)",
            "type": "mock",
            "source": "MockCamera",
            "protocol": "Internal",
        })

        # 本地摄像头扫描（在工作线程执行避免阻塞 UI）
        def _scan_worker() -> None:
            try:
                devices: List[CameraDeviceInfo] = self._scanner.scan(max_index=10)
            except Exception as e:
                _logger.warning("摄像头扫描异常: %s", e)
                devices = []
            # 投递到主线程更新列表
            self._root.after(0, lambda: self._on_scan_complete(devices))

        threading.Thread(target=_scan_worker, daemon=True).start()

    def _on_scan_complete(self, devices: List[CameraDeviceInfo]) -> None:
        """扫描完成回调（主线程）。"""
        for info in devices:
            tag = "已验证" if info.is_compatible else "未验证"
            display = f"{info.index}: {info.name} [{tag}] ({info.backend})"
            self._camera_entries.append({
                "display": display,
                "type": "local",
                "info": info,
                "source": "LocalCamera",
                "protocol": info.backend,
            })
        # 更新 Listbox
        if self._camera_listbox is not None:
            self._camera_listbox.delete(0, tk.END)
            for entry in self._camera_entries:
                self._camera_listbox.insert(tk.END, entry["display"])

    def _add_network_camera(self) -> None:
        """手动添加网络摄像头到列表。"""
        host: str = self._net_ip_entry.get().strip()
        port_str: str = self._net_port_entry.get().strip()
        try:
            port: int = int(port_str)
        except ValueError:
            messagebox.showwarning("提示", "端口必须为整数")
            return
        if not host:
            messagebox.showwarning("提示", "请输入 IP 地址")
            return
        display = f"Network: {host}:{port}"
        self._camera_entries.append({
            "display": display,
            "type": "network",
            "host": host,
            "port": port,
            "source": "NetCamera",
            "protocol": "TCP",
        })
        if self._camera_listbox is not None:
            self._camera_listbox.insert(tk.END, display)

    def _connect_selected_camera(self) -> None:
        """连接选中的摄像头。"""
        if self._camera_listbox is None:
            return
        selection = self._camera_listbox.curselection()
        if not selection:
            messagebox.showwarning("提示", "请先选择一个摄像头")
            return
        idx: int = selection[0]
        if idx >= len(self._camera_entries):
            return
        entry: dict = self._camera_entries[idx]
        cam: Optional[BaseCameraInput] = None

        if entry["type"] == "mock":
            cam = MockCameraInput()
        elif entry["type"] == "local":
            cam = Orbbec336LInput(self._camera_cfg, self._intrinsics_cfg)
        elif entry["type"] == "network":
            cam = NetworkCameraInput(
                self._intrinsics_cfg,
                host=entry["host"], port=entry["port"],
            )

        if cam is None:
            return

        # 在工作线程打开（避免阻塞 UI）
        def _open_worker() -> None:
            ok: bool = False
            try:
                ok = cam.open()
            except Exception as e:
                _logger.error("摄像头打开异常: %s", e)
                ok = False
            self._root.after(0, lambda: self._on_camera_opened(ok, cam, entry))

        threading.Thread(target=_open_worker, daemon=True).start()
        self._status_bar.config(text=f"正在连接 {entry['display']} ...")

    def _on_camera_opened(
        self, ok: bool, cam: BaseCameraInput, entry: dict,
    ) -> None:
        """摄像头打开完成回调（主线程）。"""
        if not ok:
            messagebox.showerror("错误", f"摄像头连接失败: {entry['display']}")
            self._status_bar.config(text="摄像头连接失败")
            return
        self._camera = cam
        self._camera_source = entry["source"]
        self._camera_protocol = entry["protocol"]
        _logger.info("摄像头已连接: %s (%s/%s)",
                     entry["display"], self._camera_source, self._camera_protocol)
        self._status_bar.config(text=f"已连接: {entry['display']}")
        self._show_working_state()
        self._recompute_button_state()

    # ==================================================================
    # 主循环
    # ==================================================================

    def start(self) -> None:
        """启动主循环线程与 CLI 线程。"""
        self._is_running = True

        # 连接推理 gRPC 客户端（非降级时）
        if not self._degraded_mode:
            try:
                self._inference_client.connect()
            except Exception as e:
                _logger.warning("推理客户端连接失败: %s", e)

        # 启动主循环线程
        self._main_loop_thread = threading.Thread(
            target=self._main_loop, name="ui-main-loop", daemon=True,
        )
        self._main_loop_thread.start()

        # 启动节点状态周期刷新
        self._root.after(1000, self._poll_node_status)

        # 启动 CLI 线程（可选，stdin 不可用时安全跳过）
        self._cli_thread = threading.Thread(
            target=self._run_cli_safe, name="ui-cli", daemon=True,
        )
        self._cli_thread.start()

        _logger.info("RebarMeasureApp 已启动")

    def _main_loop(self) -> None:
        """主循环线程：30fps 取帧 + 渲染 + 投递主线程显示。

        严格线程安全：本线程不直接操作 widget，所有 widget 操作经
        ``root.after(0, ...)`` 投递到主线程。
        """
        frame_interval: float = 1.0 / _TARGET_FPS
        while self._is_running:
            loop_start: float = time.time()
            try:
                # 冻结状态下跳过取帧（保持画面静止）
                with self._lock:
                    frozen = self._frozen

                if not frozen and self._camera is not None and self._camera.is_opened():
                    frame: Optional[np.ndarray] = self._camera.get_rgb_frame()
                    if frame is not None:
                        self._render_and_post(frame)
            except Exception as e:
                _logger.warning("主循环异常: %s", e)

            # FPS 统计
            self._fps_frame_count += 1
            now: float = time.time()
            if now - self._fps_last_time >= 1.0:
                self._fps = self._fps_frame_count
                self._fps_frame_count = 0
                self._fps_last_time = now

            # 节流到目标帧率
            elapsed: float = time.time() - loop_start
            sleep_time: float = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _render_and_post(self, frame: np.ndarray) -> None:
        """渲染帧并投递到主线程显示。"""
        # 获取激光数据
        laser_result, _ = self._serial_mgr.get_latest()
        laser_values: Tuple[int, ...] = (0, 0, 0, 0)
        distance_mm: Optional[float] = None
        if laser_result is not None:
            laser_values = tuple(laser_result.distances_mm[:4])
            distance_mm = self._fusion.fuse(laser_result)
        else:
            # 无串口数据时，MANUAL 策略仍可提供距离
            if self._fusion.get_strategy() == FusionStrategy.MANUAL:
                dummy = LaserParseResult(
                    distances_mm=[0, 0, 0, 0],
                    valid_mask=[False, False, False, False],
                    timestamp=0.0, valid_count=0,
                )
                distance_mm = self._fusion.fuse(dummy)

        with self._lock:
            display_mode = self._display_mode
            degraded = self._degraded_mode
            latest_measure = self._latest_measure
            latest_mask = self._latest_mask

        osd_info: dict = {
            "fps": self._fps,
            "source": self._camera_source,
            "protocol": self._camera_protocol,
            "laser_values": laser_values,
            "distance_mm": distance_mm,
            "strategy": self._fusion.get_strategy().name,
        }

        # 渲染（在子线程执行 OpenCV 计算，不阻塞主线程）
        canvas: np.ndarray = self._renderer.render_overlay(
            frame=frame,
            measure_result=latest_measure,
            osd_info=osd_info,
            display_mode=display_mode,
            mask=latest_mask,
            degraded=degraded,
        )

        # 缩放到显示尺寸
        canvas = cv2.resize(canvas, (_DISPLAY_W, _DISPLAY_H))

        # 投递到主线程更新 widget
        self._root.after(0, lambda c=canvas, d=distance_mm: self._update_video_display(c, d))

    def _update_video_display(
        self, canvas_bgr: np.ndarray, distance_mm: Optional[float]
    ) -> None:
        """更新视频标签图像（主线程安全）。"""
        try:
            rgb: np.ndarray = cv2.cvtColor(canvas_bgr, cv2.COLOR_BGR2RGB)
            pil_img: Image.Image = Image.fromarray(rgb)
            photo: ImageTk.PhotoImage = ImageTk.PhotoImage(pil_img)
            self._current_photo = photo  # 防 GC
            self._video_label.config(image=photo, width=_DISPLAY_W, height=_DISPLAY_H)
        except Exception as e:
            _logger.warning("视频显示更新失败: %s", e)

        # 更新底栏状态
        dist_text = f"{int(distance_mm)}mm" if distance_mm is not None else "无效"
        strat = self._fusion.get_strategy().name
        save_opts = []
        if self._save_options.get("csv"):
            save_opts.append("CSV")
        if self._save_options.get("mask"):
            save_opts.append("Mask")
        save_str = ",".join(save_opts) if save_opts else "仅result.jpg"
        self._status_bar.config(
            text=f"FPS:{self._fps} | 距离:{dist_text} | 策略:{strat} | "
                 f"来源:{self._camera_source}/{self._camera_protocol} | "
                 f"保存:{save_str}"
        )

    # ==================================================================
    # 识别按钮状态机
    # ==================================================================

    def _on_button_click(self) -> None:
        """识别按钮点击事件（主线程）。"""
        state: str = self._button_state
        if state == BTN_READY:
            self._request_measurement()
        elif state == BTN_RESULT:
            # 再次点击：解除冻结，回实时画面（九步循环第 ⑨ 步）
            self._unfreeze()
        elif state == BTN_LOADING:
            # 加载中不可点击
            pass
        elif state == BTN_DISABLED:
            # 禁用态不可点击；给出原因提示
            if self._degraded_mode:
                messagebox.showwarning("提示", "推理服务离线，无法识别")
            elif self._settings_open or self._results_open:
                messagebox.showwarning(
                    "提示", "请先关闭设置/推理结果窗口后再发起识别"
                )

    def _set_button_state(self, state: str) -> None:
        """设置按钮状态并重绘（主线程）。"""
        self._button_state = state
        self._draw_button()
        # 进入 LOADING 时启动动画
        if state == BTN_LOADING:
            self._loading_idx = 0
            self._root.after(200, self._loading_animation_step)

    def _recompute_button_state(self) -> None:
        """根据当前条件重新计算按钮状态（主线程）。

        禁用条件：降级模式 / 副页面打开 / 相机未连接。
        LOADING 状态下不重算（推理进行中不可打断）。
        """
        if self._button_state == BTN_LOADING:
            return
        if self._button_state == BTN_RESULT:
            return  # 结果展示态保持，不自动回退
        if (self._degraded_mode or self._settings_open or self._results_open
                or self._camera is None):
            self._set_button_state(BTN_DISABLED)
        else:
            self._set_button_state(BTN_READY)

    def _draw_button(self, text: Optional[str] = None) -> None:
        """绘制圆形按钮（tk.Canvas）。"""
        self._button_canvas.delete("all")
        size: int = _BTN_SIZE
        pad: int = 6

        state: str = self._button_state
        # 状态颜色
        colors: Dict[str, Tuple[str, str]] = {
            BTN_READY: ("#22c55e", "识别"),       # 绿
            BTN_LOADING: ("#f59e0b", "···"),      # 黄
            BTN_RESULT: ("#3b82f6", "结果"),       # 蓝
            BTN_DISABLED: ("#6b7280", "禁用"),     # 灰
        }
        fill, default_text = colors.get(state, ("#6b7280", "--"))
        display_text: str = text if text is not None else default_text

        # 外圆
        self._button_canvas.create_oval(
            pad, pad, size - pad, size - pad,
            fill=fill, outline="#1a1a2e", width=2,
        )
        # 文字
        self._button_canvas.create_text(
            size // 2, size // 2,
            text=display_text, fill="white",
            font=("Arial", 11, "bold"),
        )

    def _loading_animation_step(self) -> None:
        """Loading 动画轮切文字（主线程，root.after 周期调度）。"""
        if self._button_state != BTN_LOADING:
            return
        text: str = _LOADING_FRAMES[self._loading_idx % len(_LOADING_FRAMES)]
        self._loading_idx += 1
        self._draw_button(text=text)
        self._root.after(200, self._loading_animation_step)

    # ==================================================================
    # 九步循环：测量编排
    # ==================================================================

    def _request_measurement(self) -> None:
        """识别按钮点击触发测量（主线程 → 工作线程）。

        门控逻辑：
        - 推理间隔门控：``inference_interval_seconds`` 内的重复请求被拒绝
        - 降级模式拦截：降级模式下识别按钮禁用（此处双保险）
        - 副页面占用拦截：副页面打开时识别按钮禁用（此处双保险）
        - 激光无效拦截：distance 为 None 时提示"激光测距无效"
        """
        # 间隔门控
        interval: float = self._inference_cfg.inference_interval_seconds
        now: float = time.time()
        if now - self._last_infer_time < interval:
            remaining: float = interval - (now - self._last_infer_time)
            messagebox.showwarning(
                "提示", f"两次识别间隔至少 {interval:.1f} 秒，请等待 {remaining:.1f} 秒"
            )
            return

        # 降级/副页面双保险
        if self._degraded_mode:
            messagebox.showwarning("提示", "推理服务离线，无法识别")
            return
        if self._settings_open or self._results_open:
            messagebox.showwarning("提示", "请先关闭设置/推理结果窗口后再发起识别")
            return

        # 获取当前帧与距离
        if self._camera is None:
            messagebox.showwarning("提示", "摄像头未连接")
            return
        frame: Optional[np.ndarray] = self._camera.get_rgb_frame()
        if frame is None:
            messagebox.showwarning("提示", "当前无可用画面，无法识别")
            return

        # 激光距离
        laser_result, _ = self._serial_mgr.get_latest()
        distance_mm: Optional[float] = None
        if laser_result is not None:
            distance_mm = self._fusion.fuse(laser_result)
        elif self._fusion.get_strategy() == FusionStrategy.MANUAL:
            dummy = LaserParseResult(
                distances_mm=[0, 0, 0, 0],
                valid_mask=[False, False, False, False],
                timestamp=0.0, valid_count=0,
            )
            distance_mm = self._fusion.fuse(dummy)

        if distance_mm is None or distance_mm <= 0:
            messagebox.showwarning("提示", "激光测距无效，无法识别")
            return

        # 九步循环 ②③：冻结画面 + 锁定帧
        with self._lock:
            self._frozen = True
            self._frozen_frame = frame.copy()
            self._frozen_distance = float(distance_mm)
            self._frame_id += 1
            self._frozen_frame_id = self._frame_id

        # 按钮 → LOADING（步骤 ②）
        self._set_button_state(BTN_LOADING)
        self._status_bar.config(text="识别中...")

        # 步骤 ④：工作线程执行推理 + 测量
        threading.Thread(
            target=self._do_measurement, daemon=True,
            name="measurement-worker",
        ).start()

    def _do_measurement(self) -> None:
        """工作线程：执行九步循环 ④~⑥（推理 → 测量 → 保存）。"""
        with self._lock:
            frame: Optional[np.ndarray] = (
                self._frozen_frame.copy() if self._frozen_frame is not None else None
            )
            distance_mm: float = float(self._frozen_distance or 0.0)
            frame_id: int = self._frozen_frame_id
            save_options: Dict[str, bool] = dict(self._save_options)

        if frame is None or distance_mm <= 0:
            self._root.after(0, lambda: self._on_measurement_failed("锁定帧无效"))
            return

        # 步骤 ④：gRPC 推理
        infer_result: Optional[InferenceResult] = self._inference_client.infer(
            rgb=frame, frame_id=frame_id, camera_distance_mm=distance_mm,
        )
        if infer_result is None:
            self._root.after(0, lambda: self._on_measurement_failed("推理超时或失败"))
            return

        # 帧同步校验（§5.6 强制要求）
        if infer_result.frame_id != frame_id:
            _logger.warning(
                "帧同步不一致: 请求=%d, 响应=%d", frame_id, infer_result.frame_id
            )
            self._root.after(0, lambda: self._on_measurement_failed("帧同步不一致"))
            return

        # 步骤 ⑤：测量换算
        measure_result: Optional[MeasureResult] = self._rebar_measure.measure(
            mask=infer_result.mask,
            distance_mm=distance_mm,
            rgb_frame=frame,
        )
        if measure_result is None:
            self._root.after(0, lambda: self._on_measurement_failed("测量换算失败"))
            return

        # 步骤 ⑥：保存结果
        result_dir: str = self._result_store.save(
            frame_id=frame_id,
            rgb_frame=frame,
            mask=infer_result.mask,
            measure_result=measure_result,
            save_options=save_options,
        )

        # 记录历史
        record: dict = {
            "frame_id": frame_id,
            "timestamp": time.time(),
            "distance_mm": distance_mm,
            "rebar_count": len(measure_result.rebars),
            "result_dir": result_dir,
            "annotated_frame": measure_result.annotated_frame.copy(),
        }
        with self._lock:
            self._history_records.append(record)
            self._latest_measure = measure_result
            self._latest_mask = infer_result.mask
            self._last_infer_time = time.time()

        # 步骤 ⑦⑧：主线程显示结果 + 按钮转 RESULT
        self._root.after(
            0, lambda r=measure_result: self._on_measurement_done(r)
        )

    def _on_measurement_done(self, result: MeasureResult) -> None:
        """测量完成回调（主线程）：显示结果图 + 按钮转 RESULT。"""
        with self._lock:
            self._latest_measure = result
        # 视频区显示标注图（保持冻结）
        canvas: np.ndarray = self._renderer.render_overlay(
            frame=result.annotated_frame,
            measure_result=result,
            osd_info={
                "fps": self._fps,
                "source": self._camera_source,
                "protocol": self._camera_protocol,
                "laser_values": (0, 0, 0, 0),
                "distance_mm": result.distance_mm,
                "strategy": self._fusion.get_strategy().name,
            },
            display_mode=self._display_mode,
            degraded=self._degraded_mode,
        )
        canvas = cv2.resize(canvas, (_DISPLAY_W, _DISPLAY_H))
        self._update_video_display(canvas, result.distance_mm)

        self._set_button_state(BTN_RESULT)
        self._status_bar.config(
            text=f"识别完成: 钢筋数={len(result.rebars)}, 距离={result.distance_mm:.0f}mm"
        )

    def _on_measurement_failed(self, reason: str) -> None:
        """测量失败回调（主线程）。"""
        _logger.warning("测量失败: %s", reason)
        messagebox.showwarning("识别失败", reason)
        self._unfreeze()
        self._status_bar.config(text=f"识别失败: {reason}")

    def _unfreeze(self) -> None:
        """解除冻结，回到实时画面（九步循环第 ⑨ 步）。"""
        with self._lock:
            self._frozen = False
            self._frozen_frame = None
            self._frozen_distance = None
            self._latest_measure = None
            self._latest_mask = None
        self._recompute_button_state()
        self._status_bar.config(text="实时画面")

    # ==================================================================
    # 副页面
    # ==================================================================

    def _open_settings(self) -> None:
        """打开设置副页面（弹出 Toplevel）。"""
        if self._settings_open:
            # 已打开则前置
            if self._settings_window is not None and self._settings_window.winfo_exists():
                self._settings_window.lift()
            return
        self._settings_open = True
        self._recompute_button_state()

        win: tk.Toplevel = tk.Toplevel(self._root)
        win.title("设置")
        win.geometry("420x480")
        win.protocol("WM_DELETE_WINDOW", self._close_settings)
        self._settings_window = win

        # ---- 测距值选取 ----
        frm_fusion: tk.LabelFrame = tk.LabelFrame(win, text="测距值选取", padx=10, pady=8)
        frm_fusion.pack(fill=tk.X, padx=10, pady=6)

        self._set_fusion_var: tk.IntVar = tk.IntVar(value=self._fusion.get_strategy().value)
        strategies: List[Tuple[int, str]] = [
            (FusionStrategy.MEAN.value, "取均值（默认）"),
            (FusionStrategy.MEDIAN.value, "取中位数"),
            (FusionStrategy.SPECIFIC.value, "取特定一路"),
            (FusionStrategy.MANUAL.value, "手动输入指定值（mm）"),
        ]
        for val, label in strategies:
            ttk.Radiobutton(
                frm_fusion, text=label, variable=self._set_fusion_var, value=val,
            ).pack(anchor=tk.W)

        # 指定路编号
        spec_frame: tk.Frame = tk.Frame(frm_fusion)
        spec_frame.pack(anchor=tk.W, padx=20, pady=2)
        tk.Label(spec_frame, text="接口编号:").pack(side=tk.LEFT)
        self._set_specific_var: tk.IntVar = tk.IntVar(value=1)
        tk.Spinbox(
            spec_frame, from_=1, to=4, width=4,
            textvariable=self._set_specific_var,
        ).pack(side=tk.LEFT, padx=4)

        # 手动距离值
        manual_frame: tk.Frame = tk.Frame(frm_fusion)
        manual_frame.pack(anchor=tk.W, padx=20, pady=2)
        tk.Label(manual_frame, text="距离值(mm):").pack(side=tk.LEFT)
        self._set_manual_var: tk.StringVar = tk.StringVar(value="1000")
        tk.Entry(manual_frame, width=10, textvariable=self._set_manual_var).pack(side=tk.LEFT, padx=4)

        # ---- 显示方式 ----
        frm_display: tk.LabelFrame = tk.LabelFrame(win, text="主页面显示方式", padx=10, pady=8)
        frm_display.pack(fill=tk.X, padx=10, pady=6)
        self._set_display_var: tk.StringVar = tk.StringVar(value=self._display_mode)
        ttk.Radiobutton(
            frm_display, text="底栏模式（默认，数据完整）",
            variable=self._set_display_var, value=MODE_PANEL,
        ).pack(anchor=tk.W)
        ttk.Radiobutton(
            frm_display, text="叠加模式（中心四方向数字）",
            variable=self._set_display_var, value=MODE_OVERLAY,
        ).pack(anchor=tk.W)

        # ---- 保存方式 ----
        frm_save: tk.LabelFrame = tk.LabelFrame(
            win, text="推理结果保存方式（result.jpg 恒强制保存）", padx=10, pady=8,
        )
        frm_save.pack(fill=tk.X, padx=10, pady=6)
        self._set_csv_var: tk.BooleanVar = tk.BooleanVar(value=self._save_options.get("csv", False))
        self._set_mask_var: tk.BooleanVar = tk.BooleanVar(value=self._save_options.get("mask", False))
        tk.Checkbutton(
            frm_save, text="report.csv（CSV 数值表）",
            variable=self._set_csv_var,
        ).pack(anchor=tk.W)
        tk.Checkbutton(
            frm_save, text="ClassMask.png（类别掩码图）",
            variable=self._set_mask_var,
        ).pack(anchor=tk.W)

        # ---- 应用按钮 ----
        ttk.Button(win, text="应用", command=self._apply_settings).pack(pady=10)

    def _apply_settings(self) -> None:
        """应用设置副页面的配置项。"""
        # 融合策略
        strategy_val: int = self._set_fusion_var.get()
        strategy: Optional[FusionStrategy] = {
            FusionStrategy.MEAN.value: FusionStrategy.MEAN,
            FusionStrategy.MEDIAN.value: FusionStrategy.MEDIAN,
            FusionStrategy.SPECIFIC.value: FusionStrategy.SPECIFIC,
            FusionStrategy.MANUAL.value: FusionStrategy.MANUAL,
        }.get(strategy_val)
        if strategy is not None:
            if strategy == FusionStrategy.SPECIFIC:
                idx: int = max(1, min(4, int(self._set_specific_var.get())))
                self._fusion.set_strategy(
                    "SPECIFIC", specific_index=idx - 1,
                )
            elif strategy == FusionStrategy.MANUAL:
                try:
                    val: float = float(self._set_manual_var.get())
                    if val > 0:
                        self._fusion.set_strategy("MANUAL", manual_value=val)
                    else:
                        messagebox.showwarning("提示", "距离值必须为正数")
                        return
                except ValueError:
                    messagebox.showwarning("提示", "距离值必须为数字")
                    return
            else:
                self._fusion.set_strategy(strategy.name)

        # 显示模式
        mode: str = self._set_display_var.get()
        with self._lock:
            self._display_mode = mode

        # 保存方式
        with self._lock:
            self._save_options = {
                "csv": bool(self._set_csv_var.get()),
                "mask": bool(self._set_mask_var.get()),
            }

        _logger.info(
            "设置已应用: 策略=%s, 显示=%s, 保存=%s",
            self._fusion.get_strategy().name, self._display_mode, self._save_options,
        )
        messagebox.showinfo("设置", "设置已应用")

    def _close_settings(self) -> None:
        """关闭设置副页面。"""
        if self._settings_window is not None:
            try:
                self._settings_window.destroy()
            except Exception:
                pass
            self._settings_window = None
        self._settings_open = False
        self._recompute_button_state()

    def _open_results(self) -> None:
        """打开推理结果副页面（弹出 Toplevel）。"""
        if self._results_open:
            if self._results_window is not None and self._results_window.winfo_exists():
                self._results_window.lift()
            return
        self._results_open = True
        self._recompute_button_state()

        win: tk.Toplevel = tk.Toplevel(self._root)
        win.title("推理结果历史")
        win.geometry("900x600")
        win.protocol("WM_DELETE_WINDOW", self._close_results)
        self._results_window = win

        # 上方：历史记录列表
        list_frame: tk.Frame = tk.Frame(win)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        columns: List[str] = ["时间", "帧号", "钢筋数", "距离(mm)"]
        self._results_tree: ttk.Treeview = ttk.Treeview(
            list_frame, columns=columns, show="headings", height=10,
        )
        for col in columns:
            self._results_tree.heading(col, text=col)
            self._results_tree.column(col, width=120, anchor=tk.CENTER)
        self._results_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb: ttk.Scrollbar = ttk.Scrollbar(
            list_frame, orient=tk.VERTICAL, command=self._results_tree.yview
        )
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._results_tree.config(yscrollcommand=sb.set)

        self._results_tree.bind("<<TreeviewSelect>>", self._on_result_selected)

        # 下方：可视化结果图
        img_frame: tk.LabelFrame = tk.LabelFrame(win, text="可视化结果", padx=4, pady=4)
        img_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        self._result_image_label: tk.Label = tk.Label(img_frame, bg="black")
        self._result_image_label.pack(fill=tk.BOTH, expand=True)
        self._result_photo: Optional[ImageTk.PhotoImage] = None

        # 填充历史记录
        self._refresh_results_list()

    def _refresh_results_list(self) -> None:
        """刷新推理结果历史列表。"""
        if not hasattr(self, "_results_tree") or self._results_tree is None:
            return
        self._results_tree.delete(*self._results_tree.get_children())
        with self._lock:
            records: List[dict] = list(self._history_records)
        # 按时间倒序
        records.sort(key=lambda r: r["timestamp"], reverse=True)
        from datetime import datetime
        for r in records:
            ts_str: str = datetime.fromtimestamp(r["timestamp"]).strftime("%Y-%m-%d %H:%M:%S")
            self._results_tree.insert(
                "", tk.END,
                values=(ts_str, r["frame_id"], r["rebar_count"], f"{r['distance_mm']:.0f}"),
                tags=(str(r["frame_id"]),),
            )

    def _on_result_selected(self, _event) -> None:
        """选中历史记录项时显示对应可视化结果图。"""
        sel = self._results_tree.selection()
        if not sel:
            return
        values = self._results_tree.item(sel[0], "values")
        if not values:
            return
        try:
            frame_id: int = int(values[1])
        except (ValueError, IndexError):
            return
        with self._lock:
            record: Optional[dict] = next(
                (r for r in self._history_records if r["frame_id"] == frame_id), None
            )
            annotated: Optional[np.ndarray] = (
                record["annotated_frame"].copy() if record else None
            )
        if annotated is None:
            self._result_image_label.config(image="", text="无可视化结果")
            return
        try:
            rgb: np.ndarray = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            pil_img: Image.Image = Image.fromarray(rgb)
            # 缩放适配
            pil_img.thumbnail((860, 380))
            photo: ImageTk.PhotoImage = ImageTk.PhotoImage(pil_img)
            self._result_photo = photo
            self._result_image_label.config(image=photo, text="")
        except Exception as e:
            self._result_image_label.config(image="", text=f"显示失败: {e}")

    def _close_results(self) -> None:
        """关闭推理结果副页面。"""
        if self._results_window is not None:
            try:
                self._results_window.destroy()
            except Exception:
                pass
            self._results_window = None
        self._results_open = False
        self._recompute_button_state()

    # ==================================================================
    # 降级模式
    # ==================================================================

    def enable_degraded_mode(self) -> None:
        """启用降级模式（bootstrap 调用）。"""
        self._degraded_mode = True
        self._update_degraded_ui()
        self._recompute_button_state()
        _logger.warning("降级模式已启用")

    def _update_degraded_ui(self) -> None:
        """更新降级横幅显示（主线程）。"""
        if self._degraded_mode:
            self._degraded_label.pack(side=tk.LEFT, padx=12, pady=8)
        else:
            self._degraded_label.pack_forget()

    # ==================================================================
    # 节点状态轮询
    # ==================================================================

    def _poll_node_status(self) -> None:
        """周期轮询检测节点 状态（1 秒间隔，主线程 root.after 调度）。"""
        if not self._is_running:
            return
        try:
            online: bool = self._node_monitor.is_online()
            # 状态翻转检测
            if online and self._degraded_mode:
                _logger.info("检测节点 恢复在线，退出降级模式")
                self._degraded_mode = False
                self._update_degraded_ui()
                self._recompute_button_state()
                # 连接推理客户端
                try:
                    self._inference_client.connect()
                except Exception as e:
                    _logger.warning("推理客户端重连失败: %s", e)
            elif not online and not self._degraded_mode:
                _logger.warning("检测节点 离线，进入降级模式")
                self._degraded_mode = True
                self._update_degraded_ui()
                self._recompute_button_state()

            # 刷新指示器
            self._node_indicator.refresh()
        except Exception as e:
            _logger.warning("节点状态轮询异常: %s", e)

        self._root.after(1000, self._poll_node_status)

    def _get_node_status_info(self) -> Tuple[bool, str, str]:
        """获取节点状态信息（供 NodeStatusIndicator 回调）。"""
        online: bool = self._node_monitor.is_online()
        status_code: int = self._node_monitor.get_current_status()
        status_map: Dict[int, str] = {
            0: "UNKNOWN", 1: "IDLE", 2: "BUSY", 3: "SHUTTING_DOWN", -1: "OFFLINE",
        }
        summary: str = status_map.get(status_code, "UNKNOWN")
        detail: str = f"状态: {summary}\n在线: {'是' if online else '否'}"
        return online, summary, detail

    # ==================================================================
    # CLI 集成（公共 API 供 CliRunner 调用）
    # ==================================================================

    def _run_cli_safe(self) -> None:
        """安全启动 CLI 线程（stdin 不可用时静默退出）。"""
        try:
            from ui.cli import CliRunner
            runner: CliRunner = CliRunner(self)
            runner.run()
        except Exception as e:
            _logger.debug("CLI 线程退出: %s", e)

    def request_measurement(self) -> str:
        """CLI infer 命令入口：触发单帧识别（经 root.after 投递主线程）。"""
        self._root.after(0, self._request_measurement)
        return "已触发识别请求"

    def get_status(self) -> dict:
        """CLI status 命令入口：返回当前系统状态。"""
        online: bool = self._node_monitor.is_online()
        with self._lock:
            return {
                "node_detect_online": online,
                "node_detect_status": self._node_monitor.get_current_status(),
                "degraded_mode": self._degraded_mode,
                "camera_connected": self._camera is not None,
                "camera_source": self._camera_source,
                "camera_protocol": self._camera_protocol,
                "fusion_strategy": self._fusion.get_strategy().name,
                "display_mode": self._display_mode,
                "save_options": dict(self._save_options),
                "inference_interval_s": self._inference_cfg.inference_interval_seconds,
                "fps": self._fps,
                "history_count": len(self._history_records),
                "serial_port": self._serial_mgr.get_port_name() or "未连接",
            }

    def exit_app(self) -> str:
        """CLI exit 命令入口：触发 CLI 紧急退出路径（§10.1 CLI 退出）。

        与 GUI 关闭按钮（``_on_closing``）走不同退出路径：
        - GUI 退出：保存状态 → 通知 A → 释放资源 → root.destroy（工作线程）
        - CLI 退出：跳过状态保存 → best-effort 通知 A → 释放资源 → root.destroy（CLI 线程）
        """
        return self.cli_exit()

    def set_display_mode(self, mode: str) -> str:
        """CLI config display 命令入口：切换显示模式。"""
        if mode not in (MODE_OVERLAY, MODE_PANEL):
            return f"无效模式: {mode}（支持: overlay/panel）"
        with self._lock:
            self._display_mode = mode
        _logger.info("显示模式已切换: %s", mode)
        return f"显示模式已切换: {mode}"

    def set_save_options(self, csv: bool, mask: bool, vis: bool = True) -> str:
        """CLI config save 命令入口：设置保存方式。"""
        with self._lock:
            self._save_options = {"csv": bool(csv), "mask": bool(mask)}
        # vis 恒为 True（result.jpg 强制保存），参数仅用于 CLI 兼容
        return f"保存方式已设置: csv={csv}, mask={mask}, vis=True(强制)"

    def set_fusion_strategy(
        self,
        strategy: str,
        specific_index: int = 0,
        manual_value: Optional[float] = None,
    ) -> str:
        """CLI config fusion 命令入口：切换融合策略。"""
        try:
            self._fusion.set_strategy(
                strategy=strategy,
                specific_index=specific_index,
                manual_value=manual_value,
            )
            return f"融合策略已切换: {strategy}"
        except Exception as e:
            return f"策略切换失败: {e}"

    def get_camera_list(self) -> List[dict]:
        """CLI camera list 命令入口：返回摄像头列表。"""
        return list(self._camera_entries)

    def connect_camera_by_index(self, idx: int) -> str:
        """CLI camera connect 命令入口：按索引连接摄像头。"""
        if idx < 0 or idx >= len(self._camera_entries):
            return f"无效索引: {idx}（范围 0~{len(self._camera_entries) - 1}）"
        entry: dict = self._camera_entries[idx]
        cam: Optional[BaseCameraInput] = None
        if entry["type"] == "mock":
            cam = MockCameraInput()
        elif entry["type"] == "local":
            cam = Orbbec336LInput(self._camera_cfg, self._intrinsics_cfg)
        elif entry["type"] == "network":
            cam = NetworkCameraInput(
                self._intrinsics_cfg, host=entry["host"], port=entry["port"],
            )
        if cam is None:
            return "不支持的摄像头类型"

        def _open_worker() -> None:
            ok: bool = False
            try:
                ok = cam.open()
            except Exception as e:
                _logger.error("CLI 摄像头打开异常: %s", e)
            self._root.after(0, lambda: self._on_camera_opened(ok, cam, entry))

        threading.Thread(target=_open_worker, daemon=True).start()
        return f"正在连接: {entry['display']}"

    def connect_network_camera(self, host: str, port: int) -> str:
        """CLI camera connect 命令入口：连接网络摄像头。"""
        entry: dict = {
            "display": f"Network: {host}:{port}",
            "type": "network",
            "host": host,
            "port": port,
            "source": "NetCamera",
            "protocol": "TCP",
        }
        cam: NetworkCameraInput = NetworkCameraInput(
            self._intrinsics_cfg, host=host, port=port,
        )

        def _open_worker() -> None:
            ok: bool = False
            try:
                ok = cam.open()
            except Exception as e:
                _logger.error("CLI 网络摄像头打开异常: %s", e)
            self._root.after(0, lambda: self._on_camera_opened(ok, cam, entry))

        threading.Thread(target=_open_worker, daemon=True).start()
        return f"正在连接网络摄像头: {host}:{port}"

    def get_service_manager(self) -> ServiceManager:
        """CLI service 命令入口：返回 ServiceManager 实例。"""
        return self._service_mgr

    def get_fusion(self) -> LaserFusion:
        """返回 LaserFusion 实例（供 CliFusionMenu 使用）。"""
        return self._fusion

    def enter_fusion_menu(self) -> str:
        """CLI fusion 命令入口：进入 CliFusionMenu 交互菜单。"""
        from ui.cli_fusion_menu import CliFusionMenu
        menu: CliFusionMenu = CliFusionMenu(self._fusion)
        menu.show()
        return "已退出融合策略菜单"

    # ==================================================================
    # 退出（§10 双退出流程：GUI 正常退出 / CLI 紧急退出，统一协调器 §10.2）
    # ==================================================================

    def _on_closing(self) -> None:
        """GUI 关闭按钮触发（主线程，§10.1 GUI 退出路径）。

        退出流程在工作线程执行避免阻塞 GUI；最终 ``root.after(0, root.destroy)``
        回主线程销毁窗口。

        流程（§10.1 GUI 退出）：
        1. 保存运行时状态（``_save_runtime_state``）
        2. 检查检测节点 在线 → 在线则发 B→A 退出通知并等反馈（deadline 3s）
        3. 释放资源（§7.4 严格顺序）
        4. ``root.destroy()`` 退出
        """
        # 防止重复触发（GUI 关闭按钮 + CLI exit 并发时仅执行一次）
        if not self._begin_exit():
            return
        # 主线程显示"正在退出..."提示（widget 操作须在主线程）
        try:
            self._status_bar.config(text="正在退出...（保存状态 → 通知检测节点 → 释放资源）")
        except Exception:
            pass
        # 退出流程在工作线程执行，避免阻塞 GUI 事件循环
        threading.Thread(
            target=self._do_gui_exit, name="gui-exit-worker", daemon=True,
        ).start()

    def _do_gui_exit(self) -> None:
        """GUI 退出工作线程：完整退出流程（§10.1 GUI 正常退出路径）。"""
        try:
            _logger.info("GUI 退出流程开始（正常退出路径）")
            # 1. 保存运行时状态
            self._save_runtime_state()
            # 2. 通知检测节点（best-effort，deadline=3s，§10.2 ①~⑤）
            self._exit_coordinator.notify_peer_shutdown(deadline_s=3.0)
            # 3. 释放资源（§7.4 严格顺序）
            self._cleanup_resources()
            _logger.info("GUI 退出流程完成，销毁窗口")
        except Exception as e:
            _logger.error("GUI 退出流程异常: %s", e)
        finally:
            # 4. 回主线程销毁窗口（widget 操作须在主线程）
            try:
                self._root.after(0, self._root.destroy)
            except Exception:
                pass

    def cli_exit(self) -> str:
        """CLI exit 命令入口：CLI 紧急退出路径（§10.1 CLI 退出）。

        与 GUI 退出的区别：
        - 跳过运行时状态保存（CLI 为紧急退出）
        - best-effort 通知检测节点（失败不阻塞）
        - 在 CLI 线程同步执行清理，最终回主线程 ``root.destroy``

        :return: 退出结果摘要
        """
        if not self._begin_exit():
            return "退出流程已在进行中"
        print("[INFO] CLI 紧急退出路径触发...")
        try:
            _logger.info("CLI 退出流程开始（紧急退出路径，跳过状态保存）")
            # 1. best-effort 通知检测节点（deadline=3s，§10.2 ①~⑤）
            self._exit_coordinator.notify_peer_shutdown(deadline_s=3.0)
            # 2. 释放资源（§7.4 严格顺序）
            self._cleanup_resources()
            _logger.info("CLI 退出流程完成，销毁窗口")
        except Exception as e:
            _logger.error("CLI 退出流程异常: %s", e)
        finally:
            try:
                self._root.after(0, self._root.destroy)
            except Exception:
                pass
        return "退出完成"

    def _begin_exit(self) -> bool:
        """标记退出流程开始（线程安全，防止重复进入）。

        :return: True 表示首次进入退出流程；False 表示已在退出中
        """
        with self._exit_lock:
            if self._exiting:
                return False
            self._exiting = True
            return True

    def _set_stop_flag(self) -> None:
        """设置停止标志（§7.4 step 1，供 ExitCoordinator.release_resources 回调）。"""
        self._is_running = False
        self._stop_event.set()

    def _save_runtime_state(self) -> None:
        """保存运行时状态（§10.1 GUI 退出 step 1）。

        原型阶段仅记录当前运行时配置到日志（配置文件为只读启动输入，不回写；
        对齐 AGENTS.md §3.3 配置文件管理）。
        """
        try:
            with self._lock:
                state: dict = {
                    "display_mode": self._display_mode,
                    "save_options": dict(self._save_options),
                    "fusion_strategy": self._fusion.get_strategy().name,
                    "camera_source": self._camera_source,
                    "frame_id": self._frame_id,
                    "history_count": len(self._history_records),
                }
            _logger.info("运行时状态已保存（日志记录）: %s", state)
        except Exception as e:
            _logger.warning("保存运行时状态异常: %s", e)

    def _cleanup_resources(self) -> None:
        """释放本地资源（§7.4 严格顺序，委托 ExitCoordinator）。"""
        self._exit_coordinator.release_resources(
            stop_callback=self._set_stop_flag,
            threads=[self._main_loop_thread, self._cli_thread],
            camera=self._camera,
            serial_manager=self._serial_mgr,
            grpc_server=self._grpc_server,
        )
