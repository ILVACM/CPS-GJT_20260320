"""Orbbec 设备识别与兼容性匹配模块。

对应方案：
- N2：identify_orbbec_device() — 设备型号识别函数
- N3：match_compatible() — 设备型号匹配函数
- F1：build_info_text() — 设备信息状态栏文本格式
- N1/Q3：CompatibleDevice dataclass — 兼容设备结构化定义

设计理由（D2）：将识别/匹配/文本格式集中于独立模块，scanner 专注枚举，
识别/匹配可独立单测，新增品牌仅改 JSON 零代码改动。
"""
import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

logger = logging.getLogger("node_server.camera")


@dataclass
class CompatibleDevice:
    """兼容设备结构化定义（N1/Q3：重构 camera.json 的 compatible_devices）。

    JSON 中 pid/vid 用 hex 字符串（如 "0x0807"），便于人工对照 udev 规则/datasheet；
    ConfigLoader 解析时转换为 int 供代码内部比对（D5 决策）。
    """

    brand: str = ""                        # 品牌，如 "Orbbec"
    model: str = ""                        # 型号，如 "Gemini 336L"
    pid: int = 0                           # USB Product ID（十进制 int，如 0x0807=2055）
    vid: int = 0                           # USB Vendor ID（十进制 int，如 0x2bc5=11189）
    connection_type: str = "USB"           # 连接类型，如 "USB"


def identify_orbbec_device(device: Any) -> Optional[Dict[str, Any]]:
    """N2：识别 Orbbec 设备，返回结构化设备信息。

    基于 demo3 viewer.py::scan_device() 的 DeviceInfo 读取逻辑（F5），
    适配为返回结构化 dict 供 N3 匹配与 N4 显示。

    :param device: pyorbbecsdk Device 对象（由 scanner 通过 Context.query_devices() 枚举获得）
    :return: 结构化设备信息 dict，含：
        - name: str          设备完整名称（如 "Orbbec Gemini 336L"）
        - brand: str         品牌（name 首个空格前部分，如 "Orbbec"）
        - model: str         型号（name 首个空格后部分，如 "Gemini 336L"）
        - pid: int           USB Product ID（如 0x0807=2055）
        - vid: int           USB Vendor ID（Orbbec 固定 0x2bc5=11189）
        - serial: str        序列号
        - connection_type: str  连接类型（如 "USB"）
        - device: Any        原始 Device 对象引用（供 Orbbec336LInput 打开使用）
        失败返回 None。
    """
    if device is None:
        return None
    try:
        info = device.get_device_info()
        name: str = info.get_name()
        pid: int = info.get_pid()
        serial: str = info.get_serial_number()
        conn: str = info.get_connection_type()

        # A18：brand/model 拆分规则
        # name 形如 "Orbbec Gemini 336L" → brand="Orbbec", model="Gemini 336L"
        parts = name.split(" ", 1)
        brand: str = parts[0] if len(parts) > 0 else ""
        model: str = parts[1] if len(parts) > 1 else ""

        # Orbbec 设备 VID 固定为 0x2bc5（源自 udev 规则，已核实）
        vid: int = 0x2bc5

        result: Dict[str, Any] = {
            "name": name,
            "brand": brand,
            "model": model,
            "pid": pid,
            "vid": vid,
            "serial": serial,
            "connection_type": conn,
            "device": device,
        }
        logger.info(
            "设备识别成功: 型号=%s | PID=0x%04X | SN=%s | 连接=%s",
            name, pid, serial, conn,
        )
        return result
    except Exception as e:
        logger.error("设备识别失败: %s", e, exc_info=True)
        return None


def match_compatible(
    device_info: Optional[Dict[str, Any]],
    compatible_devices: List[CompatibleDevice],
) -> bool:
    """N3：结构化匹配设备是否在兼容列表内。

    基于 brand + model + pid 三元结构化比对，替代旧的 name 字符串模糊匹配。
    匹配规则（任一条件满足即视为兼容）：
    1. brand + model 完全匹配（大小写不敏感）且 pid 匹配
    2. pid 精确匹配（pid 非零时优先用 pid 比对，最可靠）

    :param device_info: identify_orbbec_device() 返回的结构化 dict
    :param compatible_devices: 从 camera.json 加载的兼容设备列表
    :return: 在兼容列表内返回 True，否则 False
    """
    if device_info is None:
        return False
    if not compatible_devices:
        return False

    dev_pid: int = device_info.get("pid", 0)
    dev_brand: str = device_info.get("brand", "").lower()
    dev_model: str = device_info.get("model", "").lower()

    for compat in compatible_devices:
        # 规则 1：brand + model + pid 三元匹配（大小写不敏感）
        if (dev_brand == compat.brand.lower()
                and dev_model == compat.model.lower()
                and dev_pid == compat.pid):
            return True
        # 规则 2：pid 精确匹配（pid 非零时最可靠）
        if dev_pid != 0 and compat.pid != 0 and dev_pid == compat.pid:
            return True

    return False


def build_info_text(
    device_info: Optional[Dict[str, Any]],
    width: int = 0,
    height: int = 0,
) -> str:
    """F1：构建设备信息状态栏文本。

    基于思路重写为纯函数（demo3 viewer.py::_build_info_text() 用实例变量拼接）。
    格式：型号: {name}  |  PID: 0x{pid:04X}  |  SN: {serial}  |  RGB: {w}x{h}

    :param device_info: identify_orbbec_device() 返回的结构化 dict；None 时返回占位文本
    :param width: RGB 分辨率宽（0 时省略分辨率段）
    :param height: RGB 分辨率高
    :return: 格式化文本字符串
    """
    if device_info is None:
        return "设备未知"
    name: str = device_info.get("name", "未知")
    pid: int = device_info.get("pid", 0)
    serial: str = device_info.get("serial", "未知")
    parts: List[str] = [
        f"型号: {name}",
        f"PID: 0x{pid:04X}",
        f"SN: {serial}",
    ]
    if width > 0 and height > 0:
        parts.append(f"RGB: {width}x{height}")
    return "  |  ".join(parts)
