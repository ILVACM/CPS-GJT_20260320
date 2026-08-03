"""
system 模块 — 检测节点 基础设施层。

子模块：
  - config_loader: JSON 配置校验与 dataclass 加载
  - self_check: 启动环境自检（IP/CUDA/端口/权重）
  - logger: 一机一日志 + 轮转
  - lifecycle: StateMachine 状态机（Step 4）
"""

from system.config_loader import AppConfig, load_config
from system.logger import init_logger
from system.self_check import run_self_check, SelfCheckResult, SelfCheckError

__all__ = [
    "AppConfig",
    "load_config",
    "init_logger",
    "run_self_check",
    "SelfCheckResult",
    "SelfCheckError",
]
