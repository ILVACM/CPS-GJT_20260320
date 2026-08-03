"""共享常量集中定义，双节点禁止各自硬编码副本。

对齐 AGENTS.md §5.1（串口帧协议）、§5.3（分割类别）、§6.3（激光量程）、
§8.3（内参默认值）与 Design-server.md §3.1。
"""
from typing import List


# ---- 串口帧协议（与 S21C 主控使用说明 §3.1 一致） ----
SERIAL_FRAME_HEADER: int = 0x7B  # 帧头
SERIAL_FRAME_TAIL: int = 0x7D  # 帧尾
SERIAL_FRAME_LENGTH: int = 11  # 11 字节帧：1 帧头 + 4×2 距离 + 1 校验 + 1 帧尾
SERIAL_BAUDRATE: int = 115200  # CH9102 串口波特率
SERIAL_CHANNELS: int = 4  # 4 路 STP23L 测距

# ---- 激光测距有效性范围（STP-23L Datasheet §4.1：0.07~7.5m） ----
LASER_MIN_MM: int = 70
LASER_MAX_MM: int = 7500

# ---- 分割类别定义（与检测节点 推理输出一致） ----
class SegClass:
    """语义分割类别枚举（值与检测节点 UNet 输出通道严格一致）。"""

    BACKGROUND: int = 0  # 背景
    REBAR_VERTICAL: int = 1  # 纵向钢筋
    REBAR_HORIZONTAL: int = 2  # 横向钢筋


# 语义分割类别总数
NUM_CLASSES: int = 3

# ---- 国标规格表（mm） ----
STANDARD_DIAMETERS: List[int] = [6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40]

# ---- 推理节奏 ----
# 已迁移至 config/inference.json 的 inference_interval_seconds 字段，此处不作为编译期常量。

# ---- 内参默认值（沿用现有默认值，正式值由 config/intrinsics.json 覆盖） ----
DEFAULT_FX: float = 950.0
DEFAULT_FY: float = 950.0
