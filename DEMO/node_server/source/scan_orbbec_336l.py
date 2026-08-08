#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scan_orbbec_336l.py — 奥比中光 Gemini 336L 设备信息扫描工具

功能：
  1. 枚举所有已连接的 Orbbec 设备
  2. 自动识别型号中包含 "336L" 的目标设备
  3. 获取并打印关键参数（品牌、型号、PID、SN、分辨率等）
  4. 每个参数同时显示原始值与解析结果，采用对齐表格格式

运行环境：ARM64 / openEuler 22 desktop / Python 3.9+ / pyorbbecsdk v2.1.1
脚本位置：node_server/source/scan_orbbec_336l.py（来源：OrbbecSDK-Linux-ARM64-python39/source）

依赖安装（首次使用）：
  # 使用阿里云镜像加速（推荐国内网络）
  uv pip install --default-index https://mirrors.aliyun.com/pypi/simple pyorbbecsdk
  # 或直接安装本地 wheel（推荐用于 ARM64 生产环境）
  uv pip install source/pyorbbecsdk2-2.1.1-cp39-cp39-linux_aarch64.whl

运行命令：
  cd node_server
  uv run python source/scan_orbbec_336l.py
"""

import sys
from typing import List, Optional, Tuple, Any

# ---------------------------------------------------------------------------
# SDK 导入（与 node_server/camera/orbbec_336l.py 保持一致）
# ---------------------------------------------------------------------------
try:
    from pyorbbecsdk import (
        Context,
        OBError,
        OBSensorType,
        Pipeline,
    )
except ImportError as e:
    print(f"[错误] 无法导入 pyorbbecsdk，请确认已安装：{e}")
    print("       安装命令：uv pip install source/pyorbbecsdk2-2.1.1-cp39-cp39-linux_aarch64.whl")
    sys.exit(1)


# =========================================================================
# 常量定义
# =========================================================================
TARGET_KEYWORD = "336L"   # 目标设备型号关键词

# 表格列宽配置
COL_HEADER_WIDTH = 32
COL_RAW_WIDTH = 48
COL_PARSED_WIDTH = 52

# 计算总分隔线长度
_LINE_LEN = COL_HEADER_WIDTH + COL_RAW_WIDTH + COL_PARSED_WIDTH + 8


def _sep_line(char: str = "=") -> str:
    """生成分隔线。"""
    return char * _LINE_LEN


# =========================================================================
# 表格辅助函数
# =========================================================================

def _pad_right(text: str, width: int) -> str:
    """将文本右补齐至指定宽度。"""
    if text is None:
        text = "(None)"
    return str(text).ljust(width)


def _trunc_string(text: Any, max_len: int = 40) -> str:
    """截断过长字符串，避免破坏表格对齐。"""
    s = str(text) if text is not None else "(None)"
    return s[:max_len] + "..." if len(s) > max_len else s


def _print_table_header() -> None:
    """打印表格头部行。"""
    print(_sep_line())
    print(
        f"  {_pad_right('参数名称', COL_HEADER_WIDTH)} |"
        f" {_pad_right('原始字符串', COL_RAW_WIDTH)} |"
        f" {_pad_right('解析结果', COL_PARSED_WIDTH)}"
    )
    print(_sep_line())


def _print_table_row(param: str, raw: Any, parsed: str) -> None:
    """打印表格数据行。"""
    print(
        f"  {_pad_right(param, COL_HEADER_WIDTH)} |"
        f" {_pad_right(_trunc_string(raw), COL_RAW_WIDTH)} |"
        f" {_pad_right(parsed, COL_PARSED_WIDTH)}"
    )


def _print_table_sep() -> None:
    """打印表格分隔线。"""
    print("-" * _LINE_LEN)


# =========================================================================
# 核心逻辑
# =========================================================================

def enumerate_devices() -> List[Tuple[Any, Any]]:
    """
    枚举所有已连接的 Orbbec 设备。

    Returns:
        列表，每项为 (Device, DeviceInfo) 元组；
        若无设备或发生异常则返回空列表。
    """
    try:
        ctx = Context()
        dev_list = ctx.query_devices()
        count = dev_list.get_count()
        if count == 0:
            print("[提示] 未检测到任何 Orbbec 设备，请检查 USB 连接。")
            return []
        print(f"[信息] 共检测到 {count} 台 Orbbec 设备。")
    except OBError as e:
        print(f"[错误] SDK 初始化失败 (OBError): {e}")
        print("       可能原因：SDK 库未正确加载，或 USB 权限不足。")
        print("       解决：执行 sudo bash source/install_udev_rules.sh")
        return []
    except Exception as e:
        print(f"[错误] SDK 初始化发生未知异常: {e}")
        return []

    devices = []
    for i in range(count):
        try:
            dev = dev_list[i]
            info = dev.get_device_info()
            devices.append((dev, info))
        except OBError as e:
            print(f"[警告] 读取第 {i} 台设备信息时发生 OBError: {e}")
        except Exception as e:
            print(f"[警告] 读取第 {i} 台设备信息时发生异常: {e}")

    return devices


def find_target_device(
    devices: List[Tuple[Any, Any]],
    keyword: str = TARGET_KEYWORD,
) -> Optional[Tuple[Any, Any]]:
    """
    从设备列表中查找型号包含指定关键词的目标设备。

    Args:
        devices: enumerate_devices() 返回的设备列表
        keyword: 型号匹配关键词，默认 "336L"

    Returns:
        匹配到的 (Device, DeviceInfo) 元组；未找到返回 None。
    """
    for dev, info in devices:
        name = info.get_name()
        if isinstance(name, str) and keyword in name:
            return (dev, info)
    return None


def get_supported_profiles(device: Any, sensor_type: OBSensorType) -> List[dict]:
    """
    获取指定传感器类型的所有支持分辨率配置。

    Args:
        device: Orbbec SDK Device 对象
        sensor_type: OBSensorType 枚举值（COLOR_SENSOR / DEPTH_SENSOR）

    Returns:
        列表，每项为字典 {"width": int, "height": int, "fps": int, "format": str}
    """
    profiles = []
    try:
        pipeline = Pipeline()
        profile_list = pipeline.get_stream_profile_list(sensor_type)
        count = profile_list.get_count()
        for i in range(count):
            try:
                sp = profile_list.get_video_stream_profile(i)
                profiles.append({
                    "width": sp.get_width(),
                    "height": sp.get_height(),
                    "fps": sp.get_fps(),
                    "format": str(sp.get_format()),
                })
            except Exception as e:
                pass  # 单条 profile 读取失败不影响整体
    except Exception as e:
        print(f"[警告] 查询 {sensor_type} profile 时发生异常: {e}")
    return profiles


def scan_device_info(dev: Any, info: Any) -> None:
    """
    扫描并打印单台设备的详细信息。

    Args:
        dev: Orbbec SDK Device 对象
        info: DeviceInfo 对象
    """
    print()
    print(_sep_line())
    serial_short = info.get_serial_number()[:8] if info.get_serial_number() else "UNKNOWN"
    print(f"  设备标识: SN={serial_short}...")
    print(_sep_line())
    _print_table_header()

    # ── 1. 品牌（vendor）─────────────────────────────────────
    # 原始值：SDK 无直接 vendor 字段，需从 PID 反推
    # 336L 的 USB Vendor ID 固定为 0x2BC5（Orbbec）
    pid_raw = info.get_pid()
    vendor_parsed = "Orbbec (USB VID: 0x2BC5)"
    _print_table_row("品牌/Vendor", f"PID=0x{pid_raw:04X}", vendor_parsed)

    # ── 2. 设备型号 ──────────────────────────────────────────
    name_raw = info.get_name()
    name_parsed = name_raw.strip() if isinstance(name_raw, str) else str(name_raw)
    _print_table_row("设备型号", repr(name_raw), name_parsed)

    # ── 3. PID（Product ID）──────────────────────────────────
    # get_pid() 返回 int，来源为 SDK DeviceInfo（USB 描述符中的 Product ID）
    pid_int = info.get_pid()
    pid_hex = f"0x{pid_int:04X}"
    _print_table_row("PID", pid_int, pid_hex)

    # ── 4. 序列号 ────────────────────────────────────────────
    sn_raw = info.get_serial_number()
    sn_parsed = sn_raw.strip() if isinstance(sn_raw, str) else str(sn_raw)
    _print_table_row("序列号/SN", repr(sn_raw), sn_parsed)

    # ── 5. 固件版本 ──────────────────────────────────────────
    try:
        fw_raw = info.get_firmware_version()
        fw_parsed = fw_raw.strip() if isinstance(fw_raw, str) else str(fw_raw)
    except Exception:
        fw_raw = None
        fw_parsed = "(SDK 未提供)"
    _print_table_row("固件版本", fw_raw, fw_parsed)

    # ── 6. 连接类型 ──────────────────────────────────────────
    conn_raw = info.get_connection_type()
    conn_parsed = conn_raw.strip() if isinstance(conn_raw, str) else str(conn_raw)
    _print_table_row("连接类型", repr(conn_raw), conn_parsed)

    _print_table_sep()

    # ── 7. 深度流分辨率 ──────────────────────────────────────
    print()
    print("  [深度流 Depth Stream]")
    depth_profiles = get_supported_profiles(dev, OBSensorType.DEPTH_SENSOR)
    if depth_profiles:
        _print_table_header()
        for p in depth_profiles:
            res_raw = (
                f"w={p['width']}, h={p['height']}, "
                f"fps={p['fps']}, fmt={p['format']}"
            )
            res_parsed = (
                f"{p['width']}x{p['height']}@{p['fps']}fps "
                f"({p['format']})"
            )
            _print_table_row("分辨率配置", res_raw, res_parsed)
        _print_table_sep()
    else:
        print("  (无法获取深度流分辨率，设备可能不支持或需要额外权限)")

    # ── 8. 彩色流分辨率 ──────────────────────────────────────
    print()
    print("  [彩色流 Color Stream]")
    color_profiles = get_supported_profiles(dev, OBSensorType.COLOR_SENSOR)
    if color_profiles:
        _print_table_header()
        for p in color_profiles:
            res_raw = (
                f"w={p['width']}, h={p['height']}, "
                f"fps={p['fps']}, fmt={p['format']}"
            )
            res_parsed = (
                f"{p['width']}x{p['height']}@{p['fps']}fps "
                f"({p['format']})"
            )
            _print_table_row("分辨率配置", res_raw, res_parsed)
        _print_table_sep()
    else:
        print("  (无法获取彩色流分辨率，设备可能不支持或需要额外权限)")

    print()


# =========================================================================
# 主入口
# =========================================================================

def main() -> int:
    """
    主函数：枚举设备 → 筛选目标 → 打印信息。

    Returns:
        0 表示成功，非 0 表示有错误。
    """
    print(_sep_line("-"))
    print("  Orbbec Gemini 336L 设备信息扫描工具")
    print("  位置: node_server/source/scan_orbbec_336l.py")
    print("  平台: ARM64 / openEuler 22 desktop")
    print("  SDK : pyorbbecsdk (与 node_server 一致)")
    print(_sep_line("-"))

    # Step 1: 枚举所有设备
    all_devices = enumerate_devices()
    if not all_devices:
        print()
        print("[退出] 无可用设备，程序结束。")
        return 1

    # Step 2: 查找目标设备（型号含 "336L"）
    target = find_target_device(all_devices, TARGET_KEYWORD)
    if target is None:
        print()
        print(f"[提示] 未找到型号包含 '{TARGET_KEYWORD}' 的设备。")
        print("       当前检测到的设备列表：")
        for _, info in all_devices:
            print(
                f"         - {info.get_name()}  "
                f"(PID: 0x{info.get_pid():04X})"
            )
        return 0

    dev, info = target
    print()
    print(f"[确认] 已定位目标设备: {info.get_name()}")

    # Step 3: 打印详细信息
    scan_device_info(dev, info)

    print(_sep_line("="))
    print("  扫描完成。")
    print(_sep_line("="))
    return 0


if __name__ == "__main__":
    sys.exit(main())
