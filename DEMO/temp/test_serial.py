#!/usr/bin/env python3
"""S21C 串口诊断脚本 — 扫描、连接、原始数据显示。

用途：验证 S21C 主控板经 CH9102 USB 转串口接入服务节点的物理链路与数据传输。
不做帧解析、不做有效性检查，仅将接收到的原始字节流以十六进制 + ASCII 形式实时显示。

对齐 AGENTS.md §5.1（11 字节帧协议：0x7B ... 0x7D，115200 波特率）。
本脚本为硬件诊断工具，不依赖项目其他模块，可独立运行。

用法（Linux）：
    python3 test_serial.py                  # 自动扫描 CH9102 设备并连接
    python3 test_serial.py /dev/ttyUSB0     # 指定设备路径
    python3 test_serial.py -l               # 仅列出可用串口
    python3 test_serial.py --baud 115200 /dev/ttyUSB0
    sudo python3 test_serial.py             # 权限不足时用 root 运行

环境要求（Linux）：
    - 用户需在 dialout 组：sudo usermod -aG dialout $USER（重新登录生效）
    - CH9102 驱动：ch341 或 cdc_acm 内核模块（一般自动加载）
    - 确认设备节点：ls /dev/ttyUSB* /dev/ttyACM*
    - 兼容 USB 直连与扩展坞接入（设备节点可为 /dev/ttyUSB* 或 /dev/ttyACM*）
"""
import argparse
import getpass
import grp
import os
import signal
import sys
import time
from collections import namedtuple
from typing import Optional


# ============================================================
# 加载 pyserial（绕过本地 serial/ 包同名遮蔽）
# ============================================================
# node_server/serial/ 包与 pyserial 库（安装名 serial）同名，
# 从 node_server/ 目录运行时 import serial 会命中本地包而非 pyserial。
# 以下代码临时调整 sys.path，导入 pyserial 后恢复原始状态。


def _load_pyserial():
    """加载 pyserial 模块，绕过本地 serial/ 包的同名遮蔽。"""
    cwd_abs = os.path.abspath(os.getcwd())
    script_dir = os.path.dirname(os.path.abspath(__file__))
    saved_path = sys.path[:]
    # 移除当前目录、空路径及脚本所在目录，确保 Python 从 site-packages 找到 pyserial
    sys.path = [
        p for p in sys.path
        if p not in ('', '.')
        and os.path.abspath(p) != cwd_abs
        and os.path.abspath(p) != script_dir
    ]
    saved_modules = {}
    for key in list(sys.modules.keys()):
        if key == 'serial' or key.startswith('serial.'):
            saved_modules[key] = sys.modules.pop(key)
    try:
        import serial as _pyserial
        import serial.tools.list_ports  # noqa: F401
        return _pyserial
    finally:
        sys.path = saved_path
        sys.modules.update(saved_modules)


_pyserial = _load_pyserial()
Serial = _pyserial.Serial
SerialException = _pyserial.SerialException
list_ports = _pyserial.tools.list_ports


# ============================================================
# 常量
# ============================================================

# CH9102 USB 转串口芯片标识（WCH / QinHeng Electronics）
# PID 含 0x55D4（CDC ACM 模式，节点 /dev/ttyACM*）与 0x55D5（UART 模式，节点 /dev/ttyUSB*）
CH9102_VID: int = 0x1A86
CH9102_PIDS: tuple = (0x55D4, 0x55D5)

# 帧头帧尾（仅用于显示高亮，不做解析）
FRAME_HEADER: int = 0x7B
FRAME_TAIL: int = 0x7D

# 串口默认参数
DEFAULT_BAUDRATE: int = 115200

# 统计信息输出间隔（秒）
STATS_INTERVAL: float = 5.0

# 轮询间隔（秒）
POLL_INTERVAL: float = 0.01


# ============================================================
# ANSI 颜色
# ============================================================
class C:
    """ANSI 终端颜色码。"""
    HEADER = '\033[95m'
    OK = '\033[92m'
    WARN = '\033[93m'
    ERR = '\033[91m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    DIM = '\033[2m'
    RESET = '\033[0m'


# ============================================================
# 串口扫描与选择
# ============================================================

PortInfo = namedtuple(
    'PortInfo',
    ['device', 'description', 'vid', 'pid', 'manufacturer',
     'product', 'serial_number'],
)


def normalize_port(p) -> PortInfo:
    """将 pyserial ListPortInfo 转为统一结构。"""
    return PortInfo(
        device=p.device,
        description=p.description or '',
        vid=p.vid,
        pid=p.pid,
        manufacturer=p.manufacturer,
        product=p.product,
        serial_number=p.serial_number,
    )


def check_permissions() -> bool:
    """检查当前用户是否在 dialout 组（Linux 串口访问权限）。"""
    username: str = getpass.getuser()
    try:
        groups: list = [g.gr_name for g in grp.getgrall() if username in g.gr_mem]
    except Exception:
        groups = []
    if 'dialout' not in groups:
        if os.geteuid() == 0:
            print(f"{C.WARN}[权限] 以 root 运行，串口访问无限制{C.RESET}")
            return True
        print(f"{C.WARN}[权限警告] 用户 '{username}' 不在 dialout 组{C.RESET}")
        print(f"  修复方法：sudo usermod -aG dialout $USER，然后重新登录")
        print(f"  或直接用 sudo 运行：sudo python3 test_serial.py")
        return False
    print(f"{C.OK}[权限] 用户 '{username}' 在 dialout 组，串口访问权限正常{C.RESET}")
    return True


def scan_ports() -> list:
    """扫描所有可用串口，返回 PortInfo 列表。"""
    return [normalize_port(p) for p in list_ports.comports()]


def display_ports(ports: list) -> None:
    """展示扫描到的串口信息。"""
    if not ports:
        print(f"{C.WARN}未发现任何串口设备{C.RESET}")
        print("排查步骤：")
        print("  1. 确认 S21C 主控板 Type-C 数据线已连接（数据线，非纯充电线）")
        print("  2. lsusb | grep 1a86  — 查看 CH9102 是否被系统识别")
        print("  3. ls /dev/ttyUSB* /dev/ttyACM*  — 查看设备节点是否生成")
        print("  4. lsmod | grep -E 'ch341|cdc_acm' — 查看驱动是否加载")
        print("  5. sudo dmesg | tail  — 查看内核最近的 USB 设备日志")
        return

    print(f"{C.BOLD}发现 {len(ports)} 个串口设备：{C.RESET}")
    print("-" * 80)
    for i, p in enumerate(ports):
        vid_str = f"VID=0x{p.vid:04X}" if p.vid is not None else "VID=N/A"
        pid_str = f"PID=0x{p.pid:04X}" if p.pid is not None else "PID=N/A"
        is_ch9102 = (p.vid == CH9102_VID and p.pid in CH9102_PIDS)
        marker = f"  {C.OK}← CH9102（S21C 主控芯片）{C.RESET}" if is_ch9102 else ""
        print(f"  [{i}] {C.BOLD}{p.device}{C.RESET}")
        print(f"      描述:   {p.description}")
        print(f"      厂商:   {p.manufacturer or 'N/A'}")
        print(f"      产品:   {p.product or 'N/A'}")
        print(f"      标识:   {vid_str}  {pid_str}")
        print(f"      序列号: {p.serial_number or 'N/A'}")
        if marker:
            print(marker)
        print()


def is_ch9102(p: PortInfo) -> bool:
    """判断是否为 CH9102 芯片设备（兼容 0x55D4 CDC ACM 与 0x55D5 UART 两种 PID）。"""
    return p.vid == CH9102_VID and p.pid in CH9102_PIDS


def auto_detect(ports: list) -> Optional[PortInfo]:
    """自动检测 CH9102 设备，多个时让用户选择。"""
    candidates = [p for p in ports if is_ch9102(p)]
    if len(candidates) == 1:
        print(f"  自动识别 CH9102 设备: {candidates[0].device}")
        return candidates[0]
    if len(candidates) > 1:
        print(f"{C.WARN}发现 {len(candidates)} 个 CH9102 设备，请手动选择：{C.RESET}")
        for i, p in enumerate(candidates):
            print(f"  [{i}] {p.device}  ({p.description})")
        return select_interactive(candidates)
    return None


def select_interactive(ports: list) -> Optional[PortInfo]:
    """交互式选择串口。"""
    if not ports:
        return None
    while True:
        try:
            choice = input("请输入序号选择串口（回车退出）: ").strip()
            if not choice:
                return None
            idx = int(choice)
            if 0 <= idx < len(ports):
                return ports[idx]
        except (ValueError, EOFError):
            pass
        print("输入无效，请重试")


# ============================================================
# 串口打开与数据接收
# ============================================================

def open_port(port_name: str, baudrate: int) -> Optional[Serial]:
    """打开串口，返回 Serial 对象或 None。"""
    try:
        ser = Serial(
            port=port_name,
            baudrate=baudrate,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=0.1,
        )
        print(f"{C.OK}[连接成功] {port_name} @ {baudrate} baud (8N1){C.RESET}")
        return ser
    except PermissionError:
        print(f"{C.ERR}[权限拒绝] 无法打开 {port_name}{C.RESET}")
        print(f"  请用 sudo 运行或将用户加入 dialout 组")
        return None
    except (SerialException, OSError) as e:
        print(f"{C.ERR}[打开失败] {port_name}: {e}{C.RESET}")
        return None


def hex_dump(data: bytes, base_offset: int = 0) -> str:
    """将字节数据格式化为 hex dump（高亮帧头帧尾）。"""
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        # 十六进制部分（高亮 0x7B / 0x7D）
        hex_parts = []
        for b in chunk:
            if b == FRAME_HEADER:
                hex_parts.append(f"{C.CYAN}{b:02X}{C.RESET}")
            elif b == FRAME_TAIL:
                hex_parts.append(f"{C.HEADER}{b:02X}{C.RESET}")
            else:
                hex_parts.append(f"{b:02X}")
        hex_str = ' '.join(hex_parts)
        # 对齐填充（考虑颜色码占位，用空格补到固定宽度）
        plain_len = len(' '.join(f'{b:02X}' for b in chunk))
        hex_str += ' ' * max(0, 47 - plain_len)
        # ASCII 部分
        ascii_part = ''.join(
            chr(b) if 32 <= b < 127 else '.'
            for b in chunk
        )
        offset = base_offset + i
        lines.append(f"  {offset:06X}  {hex_str}  |{ascii_part}|")
    return '\n'.join(lines)


def receive_loop(ser: Serial, port_name: str) -> None:
    """持续接收并显示原始数据，Ctrl+C 停止。"""
    print()
    print(f"{C.BOLD}{'=' * 80}{C.RESET}")
    print(f"{C.BOLD}  开始接收数据 — Ctrl+C 停止{C.RESET}")
    print(f"  端口: {port_name}  波特率: {ser.baudrate}  数据格式: 8N1")
    print(f"  说明: {C.CYAN}7B{C.RESET}=帧头(高亮)  "
          f"{C.HEADER}7D{C.RESET}=帧尾(高亮)  其余为原始字节")
    print(f"  预期: S21C 以 20Hz 上报，每帧 11 字节（0x7B + 4×2距离 + 校验 + 0x7D）")
    print(f"{C.BOLD}{'=' * 80}{C.RESET}")
    print()

    total_bytes: int = 0
    frame_header_count: int = 0
    frame_tail_count: int = 0
    start_time: float = time.time()
    last_stats_time: float = start_time

    # 恢复默认 SIGINT 处理，确保 Ctrl+C 正常触发
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        while True:
            # 读取可用数据
            waiting = ser.in_waiting
            if waiting > 0:
                data: bytes = ser.read(waiting)
                if data:
                    now = time.time()
                    total_bytes += len(data)
                    frame_header_count += data.count(FRAME_HEADER)
                    frame_tail_count += data.count(FRAME_TAIL)

                    # 实时 hex dump 输出
                    ts_str = time.strftime('%H:%M:%S', time.localtime(now))
                    ms = int((now % 1) * 1000)
                    print(f"{C.DIM}[{ts_str}.{ms:03d}] "
                          f"收到 {len(data)} 字节:{C.RESET}")
                    print(hex_dump(data, total_bytes - len(data)))
                    print()

            # 周期性统计
            now = time.time()
            if now - last_stats_time >= STATS_INTERVAL:
                elapsed = now - start_time
                rate = total_bytes / elapsed if elapsed > 0 else 0
                print(f"{C.DIM}--- 统计: {total_bytes} 字节 | "
                      f"{rate:.1f} B/s | "
                      f"0x7B×{frame_header_count} | 0x7D×{frame_tail_count} | "
                      f"耗时 {elapsed:.1f}s ---{C.RESET}")
                print()
                last_stats_time = now

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print()
        print(f"{C.WARN}用户中断，停止接收{C.RESET}")

    finally:
        _print_summary(ser, port_name, total_bytes,
                       frame_header_count, frame_tail_count, start_time)
        ser.close()
        print(f"\n{C.OK}串口已关闭{C.RESET}")


def _print_summary(ser: Serial, port_name: str, total_bytes: int,
                   header_count: int, tail_count: int,
                   start_time: float) -> None:
    """输出接收统计与诊断提示。"""
    elapsed: float = time.time() - start_time
    rate = total_bytes / elapsed if elapsed > 0 else 0

    print()
    print(f"{C.BOLD}{'=' * 80}{C.RESET}")
    print(f"{C.BOLD}  接收统计{C.RESET}")
    print(f"{C.BOLD}{'=' * 80}{C.RESET}")
    print(f"  端口:          {port_name}")
    print(f"  波特率:        {ser.baudrate}")
    print(f"  总字节:        {total_bytes}")
    print(f"  总耗时:        {elapsed:.2f}s")
    print(f"  平均速率:      {rate:.1f} B/s")
    print(f"  帧头 0x7B:     {header_count} 个")
    print(f"  帧尾 0x7D:     {tail_count} 个")
    if header_count > 0:
        match = "匹配" if tail_count == header_count else "不匹配"
        print(f"  帧头尾配对:    {tail_count}/{header_count}（{match}）")

    # 诊断提示
    print()
    if total_bytes == 0:
        print(f"{C.WARN}[诊断] 未收到任何数据，可能原因：{C.RESET}")
        print(f"  1. S21C 主控板未上电或未启动测距")
        print(f"  2. STP23L 探头未连接或全部无回波")
        print(f"  3. Type-C 数据线仅供电不传数据（换数据线测试）")
        print(f"  4. 波特率不匹配（当前 {ser.baudrate}，尝试其他值）")
        print(f"  5. S21C 固件未配置 Type-C 串口输出模式")
        print(f"  6. 串口被其他程序占用（fuser {port_name}）")
    elif header_count == 0:
        print(f"{C.WARN}[诊断] 收到数据但未发现帧头 0x7B，可能原因：{C.RESET}")
        print(f"  1. 波特率不匹配导致乱码（当前 {ser.baudrate}）")
        print(f"  2. S21C 串口输出格式与预期不符（查看 S21C 手册）")
        print(f"  3. 连接到非 S21C 的其他串口设备")
    elif header_count != tail_count:
        print(f"{C.WARN}[诊断] 帧头帧尾数量不匹配，可能丢包或数据错位{C.RESET}")
        print(f"  这在高速传输时属正常现象，若频繁出现请检查：")
        print(f"  1. USB 数据线质量与接触")
        print(f"  2. S21C 供电稳定性")
    else:
        print(f"{C.OK}[诊断] 数据接收正常，帧头帧尾配对完整{C.RESET}")


# ============================================================
# 主入口
# ============================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description='S21C 串口诊断脚本 — 扫描、连接、原始数据显示',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 test_serial.py                  # 自动扫描 CH9102 设备并连接
  python3 test_serial.py -l               # 仅列出可用串口
  python3 test_serial.py /dev/ttyUSB0     # 指定设备路径
  sudo python3 test_serial.py             # 权限不足时用 root 运行
        """,
    )
    parser.add_argument('port', nargs='?', default=None,
                        help='指定串口设备路径（如 /dev/ttyUSB0）')
    parser.add_argument('-l', '--list', action='store_true',
                        help='仅列出可用串口，不连接')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUDRATE,
                        help=f'波特率（默认 {DEFAULT_BAUDRATE}）')
    args = parser.parse_args()

    print(f"{C.BOLD}")
    print("=" * 80)
    print("  S21C 串口诊断脚本")
    print("  扫描 CH9102 USB 串口 → 连接 S21C 主控 → 显示原始数据")
    print("=" * 80)
    print(f"{C.RESET}")

    # ---- 步骤 1：权限检查 ----
    print(f"{C.BOLD}[1/3] 权限检查...{C.RESET}")
    check_permissions()
    print()

    # ---- 步骤 2：扫描串口 ----
    print(f"{C.BOLD}[2/3] 扫描串口设备...{C.RESET}")
    ports = scan_ports()
    display_ports(ports)

    if args.list:
        return

    # ---- 步骤 3：选择串口 ----
    print(f"{C.BOLD}[3/3] 选择串口...{C.RESET}")
    selected: Optional[PortInfo] = None

    if args.port:
        # 命令行指定设备路径
        matched = [p for p in ports if p.device == args.port]
        if matched:
            selected = matched[0]
            print(f"  用户指定: {selected.device}  ({selected.description})")
        else:
            # 不在扫描列表，但用户明确指定，直接尝试打开
            print(f"  用户指定: {args.port}（不在扫描列表中，尝试直接打开）")
            selected = PortInfo(
                device=args.port, description='用户指定',
                vid=None, pid=None, manufacturer=None,
                product=None, serial_number=None,
            )
    else:
        # 自动检测 CH9102
        selected = auto_detect(ports)
        if selected is None:
            if len(ports) == 1:
                selected = ports[0]
                print(f"  仅一个串口，自动选择: {selected.device}")
            elif len(ports) > 1:
                print("  未自动识别 CH9102，请手动选择：")
                selected = select_interactive(ports)
            else:
                print(f"{C.ERR}无可用串口，退出{C.RESET}")
                sys.exit(1)

    if selected is None:
        print(f"{C.ERR}未选择串口，退出{C.RESET}")
        sys.exit(1)

    print(f"  → 选中: {C.BOLD}{selected.device}{C.RESET}")
    print()

    # ---- 步骤 4：打开串口并接收 ----
    print(f"{C.BOLD}[4/4] 打开串口并接收数据...{C.RESET}")
    ser = open_port(selected.device, args.baud)
    if ser is None:
        sys.exit(1)

    # 清空输入缓冲区（丢弃历史残留数据）
    ser.reset_input_buffer()
    print(f"  输入缓冲区已清空")
    print()

    receive_loop(ser, selected.device)


if __name__ == '__main__':
    main()
