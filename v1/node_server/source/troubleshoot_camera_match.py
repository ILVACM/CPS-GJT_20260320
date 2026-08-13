#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
troubleshoot_camera_match.py — 一键排查 "设备扫描成功但不兼容" 问题
位置: node_server/source/troubleshoot_camera_match.py

用途：在 OrangePi 目标硬件上直接运行，无需修改任何业务代码。
覆盖 "扫描成功但不兼容" 关键验证点，矩阵式展示每台设备
与 camera.json 每条匹配规则比对结果。

决策树引导用户在开发机断网 + 弱性能 OrangePi 上逐项排查根因。

用法（OrangePi）：
    cd /opt/node_server
    uv run python source/troubleshoot_camera_match.py
    # 或直接用系统 Python：
    python3 source/troubleshoot_camera_match.py

输出：结构化 6 段报告 + 自动推断结论（PASS / SKIP / FAIL + 建议）。
"""
import importlib.util
import json
import logging
import os
import platform
import sys
import traceback
from pathlib import Path
from typing import Any, Dict, List, Optional

# ─────────────────────────────────────────────────────────────
# 颜色 helpers（仅在 tty 生效，避免污染日志文件）
# ─────────────────────────────────────────────────────────────
_USE_COLOR: bool = hasattr(sys.stdout, "isatty") and sys.stdout.isatty()


def _c(color_code: str, text: str) -> str:
    return f"\033[{color_code}m{text}\033[0m" if _USE_COLOR else text


def _ok(text: str) -> str:
    return _c("32", text)   # green


def _fail(text: str) -> str:
    return _c("31", text)   # red


def _warn(text: str) -> str:
    return _c("33", text)   # yellow


def _bold(text: str) -> str:
    return _c("1", text)    # bold


def _yn(b: bool) -> str:
    """布尔值 → 彩色 ✓/✗ 标记。"""
    return _ok("✓") if b else _fail("✗")


def section(title: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


# ─────────────────────────────────────────────────────────────
# 路径自举：确保脚本无论从哪里执行都能 import 项目模块
# ─────────────────────────────────────────────────────────────
SCRIPT_DIR: Path = Path(__file__).resolve().parent
_PROJECT_ROOT: Path = SCRIPT_DIR.parent  # node_server/
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

# 模块缓存（避免重复导入）
_MOD_CONFIG = None
_MOD_DEVICE_ID = None
_MOD_SCANNER = None


def _load_mod(name: str):
    """懒加载项目模块，失败时返回 None。"""
    global _MOD_CONFIG, _MOD_DEVICE_ID, _MOD_SCANNER
    if name == "config_loader" and _MOD_CONFIG is None:
        try:
            from utils.common.config_loader import ConfigLoader, CameraConfig

            _MOD_CONFIG = (ConfigLoader, CameraConfig)
        except Exception as e:
            print(_fail(f"[错误] 导入 utils.common.config_loader 失败: {e}"))
            _MOD_CONFIG = False
    if name == "device_identifier" and _MOD_DEVICE_ID is None:
        try:
            from utils.camera.device_identifier import (
                build_info_text,
                CompatibleDevice,
                identify_orbbec_device,
                match_compatible,
            )

            _MOD_DEVICE_ID = (
                build_info_text,
                CompatibleDevice,
                identify_orbbec_device,
                match_compatible,
            )
        except Exception as e:
            print(_fail(f"[错误] 导入 utils.camera.device_identifier 失败: {e}"))
            _MOD_DEVICE_ID = False
    if name == "scanner" and _MOD_SCANNER is None:
        try:
            from utils.camera.scanner import CameraDeviceInfo, LocalCameraScanner

            _MOD_SCANNER = (CameraDeviceInfo, LocalCameraScanner)
        except Exception as e:
            print(_fail(f"[错误] 导入 utils.camera.scanner 失败: {e}"))
            _MOD_SCANNER = False
    return {
        "config_loader": _MOD_CONFIG,
        "device_identifier": _MOD_DEVICE_ID,
        "scanner": _MOD_SCANNER,
    }[name]


# =================================================================
section("A. 环境信息")
# =================================================================
print(f"  Python 版本 : {platform.python_version()}  ({sys.executable})")
print(f"  操作系统     : {platform.platform()}")
print(f"  工作目录     : {os.getcwd()}")
print(f"  脚本位置     : {SCRIPT_DIR}")
print(f"  DISPLAY      : {os.environ.get('DISPLAY', '(未设置)')}")
print(f"  UID          : {os.getuid() if hasattr(os, 'getuid') else 'N/A'}")


# =================================================================
section("B. camera.json 原始内容与语法校验")
# =================================================================
CAMERA_JSON_PATH: Path = _PROJECT_ROOT / "config" / "camera.json"
B_pass: bool = True

if not CAMERA_JSON_PATH.exists():
    print(_fail(f"[FAIL] camera.json 不存在: {CAMERA_JSON_PATH}"))
    print(_warn("[建议] 请确认文件是否已同步到目标设备"))
    sys.exit(1)

raw_text: str = CAMERA_JSON_PATH.read_text(encoding="utf-8")
try:
    raw_data: dict = json.loads(raw_text)
    print(_ok("[PASS] JSON 语法合法"))
except json.JSONDecodeError as e:
    print(_fail(f"[FAIL] JSON 语法错误: {e}"))
    print(_warn(f"[建议] 检查第 {e.lineno} 列附近是否有非法逗号/注释"))
    sys.exit(1)

top_keys: List[str] = list(raw_data.keys())
print(f"  顶层 keys: {top_keys}")

if "compatible_devices" in raw_data:
    raw_compat: list = raw_data["compatible_devices"]
    print(_ok(f"[PASS] compatible_devices 字段存在，共 {len(raw_compat)} 条规则"))
    for i, item in enumerate(raw_compat):
        print(f"    [{i}] {item}")
else:
    print(_fail("[FAIL] 顶层缺少 compatible_devices 字段"))
    print(_warn("[建议] camera.json 可能是旧版本 schema，请重写为 compatible_devices 数组"))
    B_pass = False

if "compatible_models" in raw_data:
    print(_warn(f"[WARN] 检测到旧字段 compatible_models: {raw_data['compatible_models']}"))
    print(_warn("       ConfigLoader 会忽略此字段并输出 WARNING。"))


# =================================================================
section("C. ConfigLoader 解析验证（C2：hex→int 转换结果）")
# =================================================================
config_loader_result = _load_mod("config_loader")
if config_loader_result is False:
    print(_fail("[FAIL] ConfigLoader 不可用，跳过本节"))
else:
    ConfigLoader, CameraConfig = config_loader_result
    try:
        cfg: CameraConfig = ConfigLoader.load_camera_config(str(CAMERA_JSON_PATH))
        print(_ok("[PASS] ConfigLoader.load_camera_config() 执行成功"))
        print(f"  cfg.rgb_width       = {cfg.rgb_width}")
        print(f"  cfg.rgb_height      = {cfg.rgb_height}")
        print(f"  cfg.rgb_fps         = {cfg.rgb_fps}")
        print(f"  cfg.device_index    = {cfg.device_index}")
        print(f"  cfg.compatible_devices 数量 = {len(cfg.compatible_devices)}")
        for d in cfg.compatible_devices:
            print(
                f"    rule: brand={d.brand!r:10} model={d.model!r:15} "
                f"pid={d.pid} (0x{d.pid:04X})  vid={d.vid} (0x{d.vid:04X})"
            )
        if len(cfg.compatible_devices) == 0:
            print(_fail("[FAIL] compatible_devices 解析后为空"))
            print(_warn("[根因推断] camera.json 中的 compatible_devices 数组被完全清空 —"))
            print(_warn("         原因是 JSON 中缺少该 key、或所有条目解析失败"))
            B_pass = False
    except Exception as e:
        print(_fail(f"[FAIL] ConfigLoader 加载时抛出异常: {e}"))
        traceback.print_exc()
        B_pass = False


# =================================================================
section("D. pyorbbecsdk 设备枚举（C4：原始 name / pid / serial）")
# =================================================================
SDK_AVAILABLE: bool = False
try:
    from pyorbbecsdk import Context, OBError

    SDK_AVAILABLE = True
    print(_ok("[PASS] pyorbbecsdk 可导入"))
except ImportError as e:
    print(_warn(f"[SKIP] pyorbbecsdk 不可用 ({e})"))
    print(_warn("       本节跳过。SDK 仅在安装于 OrangePi Linux 环境"))

DEVICES_LIST: List[Dict[str, Any]] = []  # 供后续比对的枚举结果

if SDK_AVAILABLE:
    try:
        ctx = Context()
        dev_list = ctx.query_devices()
        count: int = dev_list.get_count()
        print(f"  Context.query_devices() 枚举到 {count} 台 Orbbec 设备")

        di_result = _load_mod("device_identifier")
        di_available: bool = di_result is not False and di_result is not None
        if di_available:
            (
                build_info_text,
                CompatibleDevice,
                identify_orbbec_device,
                match_compatible,
            ) = di_result
        else:
            identify_orbbec_device = None
            match_compatible = None

        for i in range(count):
            dev = dev_list[i]
            info = dev.get_device_info()

            raw_name = info.get_name()
            raw_pid = info.get_pid()
            raw_serial = info.get_serial_number()
            raw_conn = info.get_connection_type()

            print(f"\n  --- Device {i} ---")
            print(f"    name (raw)        = {raw_name!r}  (type: {type(raw_name).__name__})")
            print(f"    pid (raw)         = {raw_pid!r}  (type: {type(raw_pid).__name__})")
            print(
                f"    pid (hex)         = "
                f"0x{int(raw_pid):04X}" if isinstance(raw_pid, int) else "N/A"
            )
            print(f"    serial (raw)      = {raw_serial!r}")
            print(f"    connection_type   = {raw_conn!r}")

            ident: Optional[Dict[str, Any]] = None
            if di_available and identify_orbbec_device is not None:
                try:
                    ident = identify_orbbec_device(dev)
                except Exception as e:
                    print(_fail(f"    identify_orbbec_device() 抛出异常: {e}"))

            if ident is None:
                print(_fail("    identify_orbbec_device() 返回 None → 设备会消失（非"不兼容"）"))
                DEVICES_LIST.append(
                    {
                        "index": i,
                        "name": raw_name,
                        "pid": raw_pid,
                        "serial": raw_serial,
                        "ident": None,
                        "final": "VANISHED",
                    }
                )
            else:
                print(f"    identify 返回:")
                for k, v in ident.items():
                    if k == "device":
                        continue
                    print(f"      {k:16} = {v!r}")

                # 立即试跑 match_compatible
                matched: bool = False
                if match_compatible is not None and B_pass:
                    try:
                        matched = match_compatible(
                            ident, cfg.compatible_devices
                        )
                    except Exception as e:
                        print(_fail(f"    match_compatible() 抛出异常: {e}"))

                mark: str = _ok("[兼容]") if matched else _fail("[不兼容]")
                DEVICES_LIST.append(
                    {
                        "index": i,
                        "name": raw_name,
                        "pid": raw_pid,
                        "serial": raw_serial,
                        "ident": ident,
                        "final": "MATCHED" if matched else "MISMATCH",
                    }
                )
                print(f"    → 匹配结果: {mark}")

    except Exception as e:
        print(_fail(f"[FAIL] 设备枚举异常: {e}"))
        traceback.print_exc()


# =================================================================
section("E. match_compatible 全量比对矩阵（规则 x 设备）")
# =================================================================
di_available = di_available if 'di_available' in dir() else False
if SDK_AVAILABLE and B_pass and len(DEVICES_LIST) > 0 and di_available:
    device_identifier = _load_mod("device_identifier")
    if device_identifier:
        (
            build_info_text,
            CompatibleDevice,
            identify_orbbec_device,
            match_compatible,
        ) = device_identifier

        print(f"\n  规则数: {len(cfg.compatible_devices)}  ×  设备数: {len(DEVICES_LIST)}\n")

        for dev_info in DEVICES_LIST:
            ident = dev_info["ident"]
            if ident is None:
                print(f"  Device {dev_info['index']}=({dev_info['name']!r}) 无 identify 结果, 跳过")
                continue

            dev_brand: str = ident.get("brand", "")
            dev_model: str = ident.get("model", "")
            dev_pid: int = ident.get("pid", 0)

            print(f"  Device {dev_info['index']}: name={ident.get('name', '')!r}  "
                  f"brand={dev_brand!r}  model={dev_model!r}  pid=0x{dev_pid:04X}")

            for rule_idx, rule in enumerate(cfg.compatible_devices):
                ok_brand = dev_brand.lower() == rule.brand.lower()
                ok_model = dev_model.lower() == rule.model.lower()
                ok_pid = dev_pid == rule.pid and dev_pid != 0

                r_brand = rule.brand or ""
                r_model = rule.model or ""
                r_pid = rule.pid or 0

                rule1 = ok_brand and ok_model and ok_pid
                rule2 = dev_pid != 0 and r_pid != 0 and dev_pid == r_pid

                hit = rule1 or rule2
                status = _ok("MATCH ") if hit else _fail("MISS  ")
                rpid_str = f"0x{r_pid:04X}" if r_pid > 0 else f"{r_pid}(!)"
                print(
                    f"    Rule {rule_idx}: [{status}] "
                    f"brand={r_brand!r:8} model={r_model!r:14} pid={rpid_str:8}  "
                    f"→ R1[brand:{_yn(ok_brand)} model:{_yn(ok_model)} pid:{_yn(ok_pid)}] "
                    f"R2[pid_eq:{_yn(rule2)}]"
                )

            final = "MATCHED" if dev_info["final"] == "MATCHED" else dev_info["final"]
            final_mark = _ok("[兼容]") if final == "MATCHED" else _fail(f"[{final}]")
            print(f"    ⇒ 最终: {final_mark}\n")
else:
    print(_warn("[SKIP] 需要同时满足：SDK 可用 + compatible_devices 非空 + 枚举到设备"))


# =================================================================
section("F. Scanner 端到端验证（检测 scan() 全链路）")
# =================================================================
scanner_result = _load_mod("scanner")
if scanner_result and SDK_AVAILABLE and B_pass:
    CameraDeviceInfo, LocalCameraScanner = scanner_result
    try:
        test_scanner = LocalCameraScanner(cfg.compatible_devices)
        scan_results = test_scanner.scan(max_index=5)
        print(f"  scan() 返回 {len(scan_results)} 个设备")
        for info in scan_results:
            mark = _ok("[兼容]") if info.is_compatible else _fail("[不兼容]")
            pid_hex = f"0x{info.pid:04X}" if info.pid else "None"
            print(
                f"    [{mark}] idx={info.index} name={info.name!r} "
                f"pid={pid_hex} backend={info.backend}"
            )
    except Exception as e:
        print(_fail(f"[FAIL] scan() 抛出异常: {e}"))
        traceback.print_exc()
else:
    print(_warn("[SKIP] scanner 模块不可用 或 前置条件不满足"))


# =================================================================
section("G. 结论文与根因推断（自动诊断）")
# =================================================================
causes: List[str] = []

if not CAMERA_JSON_PATH.exists():
    causes.append("camera.json 文件不存在 (B1 FAIL)")
if "compatible_devices" not in raw_data:
    causes.append("camera.json 缺少 compatible_devices 字段 (B1 FAIL)")
elif len(raw_data.get("compatible_devices", [])) == 0:
    causes.append("compatible_devices 数组为空 (B1 FAIL)")

if B_pass is False or len(getattr(cfg, "compatible_devices", [])) == 0:
    causes.append("ConfigLoader 解析后 compatible_devices 为空 (B2 FAIL → 根因 #1 或 #5)")

if SDK_AVAILABLE:
    if len(DEVICES_LIST) == 0:
        causes.append("pyorbbecsdk 未枚举到任何设备 (SDK 或 USB 问题)")
    else:
        for d in DEVICES_LIST:
            if d["ident"] is None:
                causes.append(f"Device {d['index']}: identify 返回 None (根因 #4 / SDK 异常)")
            elif d["final"] == "MISMATCH":
                ident = d["ident"]
                dev_pid = ident.get("pid", 0)
                dev_brand = ident.get("brand", "")
                dev_model = ident.get("model", "")
                pid_in_rules = any(
                    rule.pid != 0 and dev_pid == rule.pid
                    for rule in (
                        cfg.compatible_devices if B_pass else []
                    )
                )
                if not pid_in_rules and B_pass:
                    causes.append(
                        f"Device {d['index']}: PID=0x{dev_pid:04X} 不在任何规则中 (根因 #3)"
                    )
                brand_ok = any(
                    dev_brand.lower() == rule.brand.lower()
                    for rule in cfg.compatible_devices
                )
                model_ok = any(
                    dev_model.lower() == rule.model.lower()
                    for rule in cfg.compatible_devices
                )
                if not brand_ok:
                    causes.append(
                        f"Device {d['index']}: brand={dev_brand!r} 不在 JSON (根因 #4)"
                    )
                if not model_ok:
                    causes.append(
                        f"Device {d['index']}: model={dev_model!r} 不在 JSON (根因 #4)"
                    )

if not causes:
    print(_ok("[结论] 所有检查项通过，未发现已知异常根因"))
    print(_warn("      若问题仍存在，需手动检查："))
    print(_warn("        1. OrangePi 端 camera.json 文件修改时间"))
    print(_warn("        2. 开发机 vs OrangePi camera.json diff"))
    print(_warn("        3. 系统日志 journalctl -u rebar-node-server --since '5 min ago'"))
else:
    print(_fail(f"[结论] 发现 {len(causes)} 个异常:"))
    for c in causes:
        print(f"  {_fail('●')} {c}")

print()
print(_bold("=" * 60))
print(_bold("  诊断输出结束。请将本输出贴回定位根因。"))
print(_bold("=" * 60))
print()
