"""
demo/3/viewer.py  —  Gemini 336L RGB 实时显示 Demo（技术验证版）

目标：
  1. 验证 Orbbec SDK v2.1.1 aarch64 基础调用链可用
  2. 为后续集成到其他程序提供可复用的 API 范式

参考示例：
  - 设备枚举：pyorbbecsdk-v2.0.18/examples/enumerate.py
  - Color 流：pyorbbecsdk-v2.0.18/examples/color.py
  - 帧转换  ：demo/1/utils.py frame_to_bgr_image()

运行环境：ARM64 / openEuler 22 desktop / Python 3.9+ / uv venv
部署资源  ：demo/3/source/ 目录下的 udev 规则与安装脚本
"""

import sys
import time
import queue
import threading
import os                           # 拍照功能：目录创建
from datetime import datetime       # 拍照功能：时间戳生成

from PIL import Image, ImageTk  # tkinter 显示需要 PIL + ImageTk 桥接
import tkinter as tk
import cv2
import numpy as np

from pyorbbecsdk import *
from utils import frame_to_bgr_image


# =========================== 常量定义 ===========================

ESC_KEY = 27                    # ESC 键码


# =========================== ① 设备扫描 ===========================

def scan_device():
    """
    扫描并枚举已连接的 Orbbec 设备，打印关键参数到控制台。
    验证点 V1~V3：
      V1: Context().query_devices()         → DeviceList
      V2: DeviceInfo.get_pid()              → int，可格式化 hex
      V3: DeviceInfo.get_connection_type()  → str（非枚举对象）

    返回：
        Device：第一台可用设备；若无设备则返回 None。
    """
    try:
        ctx = Context()
        dev_list = ctx.query_devices()
        if dev_list.get_count() == 0:
            print("[扫描] 未检测到 Orbbec 设备，请检查 USB 连接后重试。")
            return None

        dev = dev_list[0]
        info = dev.get_device_info()

        name   = info.get_name()
        pid    = info.get_pid()
        serial = info.get_serial_number()
        conn   = info.get_connection_type()

        # 控制台打印设备信息
        print("[扫描] 已识别设备:")
        print(f"  型号  : {name}")
        print(f"  PID   : 0x{pid:04X}")   # V2：验证 int → hex 格式化
        print(f"  序列号: {serial}")
        print(f"  连接  : {conn}")          # V3：验证返回 str 而非枚举

        return dev

    except OBError as e:
        print(f"[扫描] 枚举设备时发生 OBError: {e}")
        return None
    except Exception as e:
        print(f"[扫描] 枚举设备时发生未知异常: {e}")
        return None


# =========================== ② 后台采集线程 ===========================

class FrameCapture(threading.Thread):
    """
    后台采集线程：负责 Pipeline 生命周期管理 + 帧解码入队。
    线程模型（AGENTS.md §4.1 强制）：
      - wait_for_frames() 是阻塞调用，严禁放在 tkinter 主线程内
      - 使用有界队列 maxsize=1，只保留最新帧，旧帧自动丢弃
      - daemon=True，主线程退出时自动回收

    验证点 V4~V9：
      V4: get_default_video_stream_profile() 成功获取默认 profile
      V5: Config.enable_stream(profile) 无 OBError
      V6: FULL_FRAME_REQUIRE 模式生效
      V7: wait_for_frames(1000) 超时返回 None 正确处理
      V8: get_color_frame() 返回 ColorFrame（不为 None）
      V9: frame_to_bgr_image() 对 MJPG/RGB 等格式处理正确
    """

    # 支持的 Color 帧格式集合
    _SUPPORTED_FORMATS = {
        OBFormat.RGB,
        OBFormat.BGR,
        OBFormat.MJPG,
        OBFormat.YUYV,
        OBFormat.I420,
        OBFormat.NV12,
        OBFormat.NV21,
        OBFormat.UYVY,
    }

    def __init__(self, device):
        super().__init__(daemon=True)
        self._device    = device       # SDK Device 对象，保持引用
        self._stopped   = False
        self._queue     = queue.Queue(maxsize=1)   # 有界队列，只保最新帧
        self._rgb_w     = 0
        self._rgb_h     = 0
        self._pipeline  = None         # 保存引用，便于调试

    def run(self):
        pipeline = Pipeline()
        cfg = Config()
        self._pipeline = pipeline

        # V4：获取 ColorSensor 默认 profile
        try:
            profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            color_profile = profiles.get_default_video_stream_profile()
        except OBError as e:
            print(f"[采集] 获取 Color profile 列表失败 (OBError): {e}")
            return
        except Exception as e:
            print(f"[采集] 获取 Color profile 列表时发生异常: {e}")
            return

        self._rgb_w = color_profile.get_width()
        self._rgb_h = color_profile.get_height()
        print(f"[采集] Color 默认 profile: {self._rgb_w}x{self._rgb_h} @ "
              f"{color_profile.get_fps()}fps")

        # V5：启用 Color 流（预期无异常）
        try:
            cfg.enable_stream(color_profile)
        except OBError as e:
            print(f"[采集] enable_stream 失败 (OBError): {e}")
            return
        except Exception as e:
            print(f"[采集] enable_stream 异常: {e}")
            return

        # V6：设置 FULL_FRAME_REQUIRE 帧聚合模式
        cfg.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)

        # 启动 Pipeline
        try:
            pipeline.start(cfg)
            print("[采集] Pipeline 已启动，开始取帧 ...")
        except OBError as e:
            print(f"[采集] Pipeline 启动失败 (OBError): {e}")
            return
        except Exception as e:
            print(f"[采集] Pipeline 启动异常: {e}")
            return

        # 主采集循环
        frame_count = 0
        while not self._stopped:
            # V7：超时 1000ms，返回 None 时跳过（不卡死）
            try:
                frames = pipeline.wait_for_frames(1000)
            except OBError as e:
                print(f"[采集] wait_for_frames OBError: {e}")
                continue
            except Exception as e:
                print(f"[采集] wait_for_frames 异常: {e}")
                continue

            if frames is None:
                continue    # 超时，继续下一轮

            # V8：获取 Color 帧
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue

            # 格式校验（V9 前提）
            fmt = color_frame.get_format()
            if fmt not in self._SUPPORTED_FORMATS:
                print(f"[采集] 不支持的 Color 帧格式: {fmt}，跳过该帧")
                continue

            try:
                # V9：帧格式转换（MJPG/YUYV/I420 等多格式兼容）
                bgr = frame_to_bgr_image(color_frame)
                if bgr is None:
                    print("[采集] frame_to_bgr_image 返回 None，跳过该帧")
                    continue

                # BGR → RGB（tkinter PhotoImage 需要 RGB）
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

                # 非阻塞入队；队列满时丢弃当前帧（已有更新的在等待）
                try:
                    self._queue.put_nowait(rgb)
                    frame_count += 1
                except queue.Full:
                    pass    # 队列满，丢弃本帧（保持最新帧优先）

            except Exception as e:
                # 单帧处理异常不崩溃，继续采集
                print(f"[采集] 帧处理异常（跳过）: {e}")
                continue

        print(f"[采集] 线程退出，共处理 {frame_count} 帧")

    def stop(self):
        """设置停止标志，供入口清理时调用。"""
        self._stopped = True

    def get_latest_frame(self):
        """非阻塞取帧，供 tkinter 主线程调用。满则立即取最新一帧。"""
        try:
            return self._queue.get_nowait()
        except queue.Empty:
            return None

    @property
    def size(self):
        """返回 (width, height) 元组，供窗口尺寸初始化使用。"""
        return (self._rgb_w, self._rgb_h)


# =========================== ③ tkinter GUI 主窗口 ===========================

class ViewerApp:
    """
    基于 tkinter 的实时 RGB 画面展示窗口。
    验证点 V10~V12：
      V10: 窗口以原始分辨率显示，无拉伸变形
      V11: 按 q / ESC 正常退出，线程安全清理
      V12: 关闭窗口正常退出（同 V11）

    架构要点（AGENTS.md §4.1）：
      - 渲染循环通过 root.after(33) 驱动，~30fps，不阻塞 mainloop
      - PhotoImage 必须保存为实例变量，否则被 GC 回收导致黑屏
    """

    # 支持退出的按键集合
    _EXIT_KEYS = {ord('q'), ord('Q'), ESC_KEY}

    def __init__(self, capture: FrameCapture, dev_info):
        self.capture = capture       # FrameCapture 线程实例
        self.info    = dev_info      # DeviceInfo 对象

        # ── 窗口基础设置 ────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title("Orbbec Test Display")

        w, h = capture.size
        # V10：窗口尺寸 = RGB 原始分辨率 + 底部信息栏(40px) + 按钮栏(35px)
        self.root.geometry(f"{w}x{h + 75}")
        self.root.minsize(w, h + 75)

        # ── 内容容器 Frame（包裹 image + info，占据上方全部空间）──
        content_frame = tk.Frame(self.root)
        content_frame.pack(expand=True, fill="both")

        # ── 图像显示区（Label）───────────────────────────────────
        self.label = tk.Label(content_frame)
        self.label.pack(expand=True, fill="both")

        # ── 设备信息文本栏（画面下方）────────────────────────────
        self.info_label = tk.Label(
            content_frame,
            text=self._build_info_text(),
            font=("Consolas", 10),
            fg="#333333",
            anchor="w",
            justify="left",
            height=2,
        )
        self.info_label.pack(fill="x")

        # ── 拍照按钮（新增）───────────────────────────────────
        self.save_dir = os.path.join(os.path.dirname(__file__), "img")
        os.makedirs(self.save_dir, exist_ok=True)

        btn_frame = tk.Frame(self.root, bg="#f0f0f0", height=35)
        btn_frame.pack(fill="x", side="bottom")   # 固定在底部，独立于 content_frame

        self.snapshot_btn = tk.Button(
            btn_frame,
            text="📷 拍照",
            font=("Microsoft YaHei", 12),
            width=10,
            command=self._on_snapshot,
            relief="flat",
            bg="#4a90e2",
            fg="white",
            activebackground="#357abd",
        )
        self.snapshot_btn.pack(expand=True, pady=5)

        # ── 按键绑定：q / ESC 退出（V11）───────────────────────
        self.root.bind("<KeyPress>", self._on_key_press)

        # ── 窗口关闭事件：点击 × 按钮退出（V12）────────────────
        self.root.protocol("WM_DELETE_WINDOW", self._on_window_close)

    # ── 按键处理 ────────────────────────────────────────────────

    def _on_key_press(self, event):
        key = event.keycode
        if key in self._EXIT_KEYS:
            self.root.quit()

    # ── 窗口关闭处理 ────────────────────────────────────────────

    def _on_window_close(self):
        """响应 WM_DELETE_WINDOW，等价于 root.quit()。"""
        self.root.quit()

    # ── 设备信息文本构造 ───────────────────────────────────────

    def _build_info_text(self) -> str:
        """
        构造底部状态栏文本，展示设备关键参数。
        字段格式规范与 scan_device() 输出保持一致。
        """
        return (
            f"型号: {self.info.get_name()}  |  "
            f"PID: 0x{self.info.get_pid():04X}  |  "
            f"SN: {self.info.get_serial_number()}  |  "
            f"RGB: {self.capture.size[0]}×{self.capture.size[1]}"
        )

    # ── 渲染循环（每 33ms 触发一次）────────────────────────────

    def _on_snapshot(self):
        """
        捕获当前显示帧并保存为 JPEG 文件。
        运行在主线程，不阻塞采集线程。
        """
        try:
            img = self.capture.get_latest_frame()
            if img is None:
                print("[拍照] 警告：当前无可用帧")
                return

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"{timestamp}.jpg"
            filepath = os.path.join(self.save_dir, filename)

            # BGR → RGB 转换后保存（cv2.imwrite 接受 BGR）
            bgr_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            success = cv2.imwrite(filepath, bgr_img, [cv2.IMWRITE_JPEG_QUALITY, 95])

            if success:
                print(f"[拍照] 已保存: {filepath}")
                # 更新状态栏显示保存结果
                self.info_label.config(text=f"✅ 已保存: {filename}")
                # 2秒后恢复原状态栏文本
                self.root.after(2000, lambda: self.info_label.config(text=self._build_info_text()))
            else:
                print(f"[拍照] 失败: 无法写入 {filepath}")
                self.info_label.config(text="❌ 保存失败")
                self.root.after(2000, lambda: self.info_label.config(text=self._build_info_text()))

        except Exception as e:
            print(f"[拍照] 异常: {e}")
            self.info_label.config(text="❌ 拍照失败")
            self.root.after(2000, lambda: self.info_label.config(text=self._build_info_text()))

    def _refresh(self):
        """
        从帧队列非阻塞取最新帧，转换为 PhotoImage 并更新 Label。
        通过 root.after() 递归调用实现 ~30fps 非阻塞刷新。
        ⚠️ 必须保留 self.tk_photo 引用，否则 PIL Image 和 PhotoImage
           均会被 Python GC 回收，Label 显示空白（黑屏）。
        """
        img = self.capture.get_latest_frame()
        if img is not None:
            pil_img = Image.fromarray(img)
            self.tk_photo = ImageTk.PhotoImage(image=pil_img)  # PIL→Tkinter 桥接
            self.label.configure(image=self.tk_photo)
        self.root.after(33, self._refresh)

    # ── 启动入口 ────────────────────────────────────────────────

    def run(self):
        self._refresh()
        self.root.mainloop()


# =========================== 主入口 ===========================

def main():
    print("=" * 50)
    print("  Orbbec Gemini 336L — RGB Viewer (Demo 3)")
    print("  平台: ARM64 / openEuler 22 desktop")
    print("=" * 50)

    # ① 扫描设备（同时做 V1~V3 验证）
    device = scan_device()
    if device is None:
        print("[主程序] 未找到可用设备，退出。")
        sys.exit(1)

    dev_info = device.get_device_info()

    # ② 启动后台采集线程
    capture = FrameCapture(device)
    capture.start()
    print("[主程序] 采集线程已启动")

    # ③ 启动 tkinter GUI 主循环
    app = ViewerApp(capture, dev_info)
    app.run()
    print("[主程序] GUI 已退出")

    # ④ 优雅清理（顺序不可颠倒：先停采集，再等线程退出）
    capture.stop()
    capture.join(timeout=2.0)
    if capture.is_alive():
        print("[主程序] 警告：采集线程未在 2s 内退出")
    else:
        print("[主程序] 采集线程已安全停止")

    cv2.destroyAllWindows()
    print("[主程序] 所有资源已释放，程序正常结束。")


if __name__ == "__main__":
    main()
