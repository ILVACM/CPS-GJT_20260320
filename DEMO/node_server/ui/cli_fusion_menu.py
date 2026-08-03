"""激光融合策略 CLI 交互式菜单（input/print 风格）。

对齐 Design-server.md §3.7.1 与 AGENTS.md §6.2（CLI 菜单）。

设计要点：
- 标准库 ``input()`` / ``print()``，零额外依赖
- 四策略切换：1.均值 2.中位数 3.指定路 4.手动输入
- 与 GUI 设置副页面功能等价（CLI 备用通道）
- 在独立 CLI 线程中循环运行；策略变更经 ``on_change`` 回调通知调用方
- 退出指令 ``q`` 退出菜单循环（不退出程序）
"""
import logging
from typing import Callable, Optional

from serial.laser_fusion import FusionStrategy, LaserFusion

# 模块级 logger
_logger = logging.getLogger("node_server.ui")


class CliFusionMenu:
    """命令行终端交互式融合策略菜单。

    使用标准 input() / print()，零额外依赖。

    生命周期：
        menu = CliFusionMenu(fusion, on_change=lambda s: print(f"已切换: {s}"))
        menu.show()  # 阻塞循环，直到用户输入 q
    """

    def __init__(
        self,
        fusion: LaserFusion,
        on_change: Optional[Callable[[FusionStrategy], None]] = None,
    ) -> None:
        """初始化 CLI 融合菜单。

        :param fusion: LaserFusion 实例（策略状态由本菜单切换）
        :param on_change: 策略变更回调（可选）；回调在 CLI 线程触发，
                          调用方负责用 ``root.after`` 调度到主线程刷新 GUI
        """
        self._fusion: LaserFusion = fusion
        self._on_change: Optional[Callable[[FusionStrategy], None]] = on_change

    def show(self) -> None:
        """显示菜单并循环获取用户选择。

        在 CLI 线程中阻塞运行，直到用户输入 ``q`` 退出菜单。
        策略切换立即生效（``LaserFusion.set_strategy`` 线程安全）。
        """
        print("\n=== 激光融合策略菜单 ===")
        while True:
            current: FusionStrategy = self._fusion.get_strategy()
            print(f"\n当前策略: {self._strategy_label(current)}")
            print("1. 取均值（默认）")
            print("2. 取中位数")
            print("3. 取特定一路（一~四）")
            print("4. 手动输入指定值（mm）")
            print("q. 返回上级")
            choice: str = input("请选择 [1-4 / q]: ").strip()

            if choice == "q":
                print("退出融合策略菜单")
                return

            if choice == "1":
                self._apply_strategy(FusionStrategy.MEAN)
            elif choice == "2":
                self._apply_strategy(FusionStrategy.MEDIAN)
            elif choice == "3":
                self._handle_specific()
            elif choice == "4":
                self._handle_manual()
            else:
                print("[警告] 无效选择，请输入 1/2/3/4/q")

    # ------------------------------------------------------------------
    # 内部：策略应用与回调
    # ------------------------------------------------------------------

    def _apply_strategy(
        self,
        strategy: FusionStrategy,
        specific_index: int = 0,
        manual_value: Optional[float] = None,
    ) -> None:
        """应用策略并触发回调。"""
        strategy_name = strategy.name
        try:
            self._fusion.set_strategy(
                strategy=strategy_name,
                specific_index=specific_index,
                manual_value=manual_value,
            )
            print(f"[OK] 已切换至: {self._strategy_label(strategy)}")
        except Exception as e:
            print(f"[错误] 策略切换失败: {e}")
            return

        if self._on_change is not None:
            try:
                self._on_change(strategy)
            except Exception as e:
                _logger.warning("融合策略 on_change 回调异常: %s", e)

    def _handle_specific(self) -> None:
        """处理"取特定一路"策略：要求用户输入接口编号 1~4。"""
        raw = input("请输入接口编号（1-4）: ").strip()
        try:
            idx1 = int(raw)
            if not 1 <= idx1 <= 4:
                print("[警告] 编号必须为 1~4")
                return
            self._apply_strategy(FusionStrategy.SPECIFIC, specific_index=idx1 - 1)
        except ValueError:
            print("[警告] 无效编号，请输入 1~4 的整数")

    def _handle_manual(self) -> None:
        """处理"手动输入指定值"策略：要求用户输入距离值（mm）。"""
        raw = input("请输入距离值（mm，70~7500）: ").strip()
        try:
            val = float(raw)
            if val <= 0:
                print("[警告] 距离值必须为正数")
                return
            self._apply_strategy(FusionStrategy.MANUAL, manual_value=val)
        except ValueError:
            print("[警告] 无效数值，请输入数字")

    @staticmethod
    def _strategy_label(strategy: FusionStrategy) -> str:
        """策略枚举转中文标签。"""
        return {
            FusionStrategy.MEAN: "取均值",
            FusionStrategy.MEDIAN: "取中位数",
            FusionStrategy.SPECIFIC: "取特定一路",
            FusionStrategy.MANUAL: "手动输入指定值",
        }.get(strategy, "未知")
