"""检测节点 状态指示器（tk.Canvas 圆点 + tooltip）。

对齐 Design-server.md §7.5（UI 状态面板集成）与 AGENTS.md §7.2（线程安全）。

设计要点：
- 用 ``tk.Canvas`` 绘制绿/红圆点表示检测节点 在线/离线
- 鼠标悬停弹出 tooltip（``tk.Label`` Toplevel 实现，零外部依赖）
- 状态查询通过回调函数获取（避免直接持有 NodeMonitor 引用造成耦合）
- 周期刷新由调用方经 ``root.after`` 调度（保持主线程唯一性）
"""
import tkinter as tk
from typing import Callable, Optional


class NodeStatusIndicator:
    """检测节点 状态指示器。

    生命周期：
        indicator = NodeStatusIndicator(parent, status_provider=lambda: (True, "在线", "心跳 42 次"))
        indicator.pack(side=tk.LEFT, padx=4)
        indicator.refresh()  # 由调用方周期性 root.after 调度
    """

    # 圆点尺寸
    _CANVAS_SIZE: int = 16
    _CIRCLE_PAD: int = 2

    def __init__(
        self,
        parent: tk.Widget,
        status_provider: Optional[Callable[[], tuple]] = None,
    ) -> None:
        """初始化节点状态指示器。

        :param parent: 父容器
        :param status_provider: 状态查询回调，返回元组 ``(is_online: bool, summary: str, detail: str)``；
                                None 时显示未知状态
        """
        self._parent: tk.Widget = parent
        self._status_provider: Optional[Callable[[], tuple]] = status_provider
        self._is_online: bool = False
        self._summary: str = "未知"
        self._detail: str = "尚未收到心跳"

        # Canvas 绘制圆点
        self._canvas: tk.Canvas = tk.Canvas(
            parent,
            width=self._CANVAS_SIZE,
            height=self._CANVAS_SIZE,
            highlightthickness=0,
            bg=parent.cget("bg") if hasattr(parent, "cget") else "#f0f0f0",
        )
        # 初始绘制（灰色 — 未知）
        self._oval_id = self._canvas.create_oval(
            self._CIRCLE_PAD, self._CIRCLE_PAD,
            self._CANVAS_SIZE - self._CIRCLE_PAD, self._CANVAS_SIZE - self._CIRCLE_PAD,
            fill="#888888", outline="",
        )

        # tooltip（悬停弹出，Toplevel + Label）
        self._tooltip: Optional[tk.Toplevel] = None
        self._canvas.bind("<Enter>", self._on_enter)
        self._canvas.bind("<Leave>", self._on_leave)

    # ------------------------------------------------------------------
    # 公共 API
    # ------------------------------------------------------------------

    def pack(self, **kwargs) -> None:
        """pack 布局（透传到 Canvas）。"""
        self._canvas.pack(**kwargs)

    def grid(self, **kwargs) -> None:
        """grid 布局（透传到 Canvas）。"""
        self._canvas.grid(**kwargs)

    def refresh(self) -> None:
        """刷新状态颜色与文字（必须在主线程调用）。

        通过 ``status_provider`` 回调获取最新状态，更新圆点颜色与 tooltip 文本。
        """
        if self._status_provider is None:
            return
        try:
            result: tuple = self._status_provider()
            is_online, summary, detail = result
        except Exception:
            return
        self._is_online = bool(is_online)
        self._summary = str(summary)
        self._detail = str(detail)

        color: str = "#22c55e" if self._is_online else "#ef4444"  # 绿 / 红
        self._canvas.itemconfig(self._oval_id, fill=color)

        # 若 tooltip 当前可见，同步刷新文本
        if self._tooltip is not None and self._tooltip.winfo_exists():
            self._update_tooltip_text()

    @property
    def widget(self) -> tk.Canvas:
        """返回底层 Canvas widget（供外部布局调整）。"""
        return self._canvas

    # ------------------------------------------------------------------
    # 内部：tooltip
    # ------------------------------------------------------------------

    def _on_enter(self, _event) -> None:
        """鼠标进入：弹出 tooltip。"""
        self._show_tooltip()

    def _on_leave(self, _event) -> None:
        """鼠标离开：隐藏 tooltip。"""
        self._hide_tooltip()

    def _show_tooltip(self) -> None:
        """显示 tooltip Toplevel。"""
        if self._tooltip is not None and self._tooltip.winfo_exists():
            return
        self._tooltip = tk.Toplevel(self._parent)
        self._tooltip.wm_overrideredirect(True)  # 无边框
        # 定位到鼠标附近
        try:
            x = self._canvas.winfo_rootx() + self._CANVAS_SIZE + 6
            y = self._canvas.winfo_rooty() + self._CANVAS_SIZE + 6
            self._tooltip.wm_geometry(f"+{x}+{y}")
        except Exception:
            pass
        self._update_tooltip_text()

    def _update_tooltip_text(self) -> None:
        """更新 tooltip 文本。"""
        if self._tooltip is None or not self._tooltip.winfo_exists():
            return
        # 清空旧子控件
        for child in self._tooltip.winfo_children():
            child.destroy()
        text = f"检测节点: {self._summary}\n{self._detail}"
        label = tk.Label(
            self._tooltip,
            text=text,
            background="#ffffe0",
            foreground="#000000",
            relief=tk.SOLID,
            borderwidth=1,
            justify=tk.LEFT,
            padx=6,
            pady=4,
        )
        label.pack()

    def _hide_tooltip(self) -> None:
        """隐藏 tooltip。"""
        if self._tooltip is not None:
            try:
                self._tooltip.destroy()
            except Exception:
                pass
            self._tooltip = None
