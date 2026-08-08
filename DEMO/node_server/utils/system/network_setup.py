"""网络配置模块：一键式设置 / 复原网络配置。

对齐 AGENTS.md §4.1 网络拓扑（双节点网线直连 + 静态 IP）与 §4.2 网络参数配置化。
本模块由 `main.py setup-network` 子命令驱动（人工触发，不在 systemd 启动流程内自动执行），
仅面向 Linux 目标环境，无需 Windows 兼容层。

默认流程（无显式参数）：
  1. 备份当前系统网络配置（仅保留最新一份，原子写入）
  2. 设置静态 IP（IP/掩码取自 network.json，网关取 gateway 或自动推导 `.1`）
  3. 启动网络接口
  4. 重启网络服务

显式参数（--ip/--netmask/--gateway）仅本次生效，不修改配置文件；
--restore 从备份复原原配置，与设置参数互斥。
"""
import ipaddress
import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LOGS_DIR = PROJECT_ROOT / "logs"
BACKUP_ROOT = LOGS_DIR / "network_backup"  # 固定目录：仅保留最新一份备份
BACKUP_TMP = LOGS_DIR / "network_backup.tmp"  # 原子交换用临时目录


@dataclass
class SetupOptions:
    """setup-network 子命令的一次性执行参数（均不落盘到配置文件）。"""

    ip: Optional[str] = None            # --ip：临时 IP
    netmask: Optional[str] = None       # --netmask：临时掩码
    gateway: Optional[str] = None       # --gateway：临时网关
    restore: bool = False               # --restore：复原模式
    start_interface: bool = True        # --no-start-interface 置 False
    restart_service: bool = True        # --no-restart-service 置 False


# ============================================================
# 辅助函数
# ============================================================

def _log(msg: str) -> None:
    """统一步骤日志输出。"""
    print(msg)


def _is_valid_ipv4(text: str) -> bool:
    """校验字符串是否为合法 IPv4 地址。"""
    try:
        ipaddress.ip_address(text.strip())
        return True
    except ValueError:
        return False


def derive_gateway(local_ip: str, configured_gateway: Optional[str]) -> str:
    """解析出要使用的网关地址。

    优先使用显式网关；未指定则取 local_ip 所在网段的 `.1` 地址。
    对 local_ip 与推导出的网关做合法性校验，非法则报错退出。
    """
    if configured_gateway:
        if not _is_valid_ipv4(configured_gateway):
            print(
                f"[ERROR] 网关地址非法: {configured_gateway!r}，"
                "必须为合法的 IPv4 地址",
                file=sys.stderr,
            )
            sys.exit(1)
        return configured_gateway.strip()

    if not _is_valid_ipv4(local_ip):
        print(
            f"[ERROR] IP 地址非法: {local_ip!r}，无法推导网关，"
            "请检查 config/network.json 的 local_ip 或使用 --ip 指定",
            file=sys.stderr,
        )
        sys.exit(1)

    parts = local_ip.strip().split(".")
    derived = f"{parts[0]}.{parts[1]}.{parts[2]}.1"
    if not _is_valid_ipv4(derived):
        print(
            f"[ERROR] 推导出的网关 {derived!r} 非法（源 IP {local_ip!r}）",
            file=sys.stderr,
        )
        sys.exit(1)
    return derived


def netmask_to_prefix(netmask: str) -> int:
    """将 netmask 转为 prefix length（如 255.255.255.252 → 30）。"""
    parts = netmask.split(".")
    if len(parts) != 4:
        return 24
    try:
        binary = "".join(bin(int(p))[2:].zfill(8) for p in parts)
    except ValueError:
        return 24
    return binary.count("1")


def detect_distro_id() -> str:
    """从 /etc/os-release 读取发行版 ID。"""
    if os.path.exists("/etc/os-release"):
        try:
            with open("/etc/os-release", "r", encoding="utf-8") as f:
                for line in f:
                    if line.startswith("ID="):
                        return line.split("=", 1)[1].strip().strip('"').lower()
        except OSError:
            pass
    return ""


def detect_network_interface() -> Optional[str]:
    """自动检测用于直连的物理网卡名。

    遍历 /sys/class/net/ 排除 lo / docker / virbr / 无线网卡，优先选状态 UP 的物理网卡。
    """
    net_dir = Path("/sys/class/net")
    if not net_dir.exists():
        return None

    candidates = []
    for iface_path in net_dir.iterdir():
        iface = iface_path.name
        if iface in ("lo",) or iface.startswith(("docker", "virbr", "br-")):
            continue
        # 检查是否为无线网卡
        if (iface_path / "wireless").exists():
            continue
        # 检查是否为物理网卡（有 device 符号链接）
        if (iface_path / "device").is_symlink():
            candidates.append(iface)

    if not candidates:
        all_ifaces = [p.name for p in net_dir.iterdir() if p.name != "lo"]
        return all_ifaces[0] if all_ifaces else None

    # 优先选状态 UP 的
    for iface in candidates:
        operstate_path = net_dir / iface / "operstate"
        if operstate_path.exists():
            try:
                if operstate_path.read_text().strip() == "up":
                    return iface
            except OSError:
                pass
    return candidates[0]


def _run_cmd(cmd: List[str], timeout: int = 60) -> Tuple[int, str, str]:
    """执行命令并返回 (returncode, stdout, stderr)。"""
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout
        )
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return -1, "", f"命令超时({timeout}s): {' '.join(cmd)}"


def _run_cmd_checked(cmd: List[str], step: str, timeout: int = 60) -> bool:
    """执行命令；失败时打印日志并返回 False，由调用方决定是否继续。"""
    _log(f"  · 执行: {' '.join(cmd)}")
    rc, out, err = _run_cmd(cmd, timeout)
    if rc != 0:
        print(f"  ✗ {step} 失败 (exit={rc}): {err or out}", file=sys.stderr)
        return False
    if out:
        _log(f"  ✓ {out.splitlines()[0] if out.splitlines() else ''}")
    return True


# ============================================================
# 备份（唯一化 + 原子写）
# ============================================================

def _backup_runtime_snapshot(backup_dir: Path) -> None:
    """抓取运行时网络状态快照（ip addr/route、nmcli 等），写入备份目录。"""
    commands = {
        "ip_addr.txt": ["ip", "addr"],
        "ip_route.txt": ["ip", "route"],
        "nmcli_dev_show.txt": ["nmcli", "dev", "show"],
        "nmcli_connection_show.txt": ["nmcli", "connection", "show"],
    }
    for filename, cmd in commands.items():
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            (backup_dir / filename).write_text(
                f"$ {' '.join(cmd)}\n{result.stdout}\n[stderr]\n{result.stderr}",
                encoding="utf-8",
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as e:
            (backup_dir / filename).write_text(f"命令不可用: {e}", encoding="utf-8")


def _backup_config_files(backup_dir: Path) -> None:
    """备份网络相关的配置文件与配置目录。"""
    config_files = [
        "/etc/resolv.conf",
        "/etc/network/interfaces",
    ]
    for cf in config_files:
        if os.path.exists(cf):
            try:
                shutil.copy2(cf, backup_dir / (Path(cf).name + ".bak"))
            except OSError as e:
                (backup_dir / (Path(cf).name + ".bak.error")).write_text(
                    str(e), encoding="utf-8"
                )

    for src_dir, dst_name in [
        ("/etc/sysconfig/network-scripts", "ifcfg-backup"),
        ("/etc/netplan", "netplan-backup"),
    ]:
        if os.path.isdir(src_dir):
            try:
                shutil.copytree(src_dir, backup_dir / dst_name)
            except OSError as e:
                (backup_dir / f"{dst_name}.error").write_text(
                    str(e), encoding="utf-8"
                )


def backup_network_config() -> dict:
    """备份当前网络配置，仅保留最新一份，采用原子写入。

    先写入 BACKUP_TMP 临时目录，成功后删除旧备份并 rename 为 BACKUP_ROOT，
    避免中途失败损坏既有备份。
    """
    # 清理历史临时目录
    if BACKUP_TMP.exists():
        shutil.rmtree(BACKUP_TMP, ignore_errors=True)
    BACKUP_TMP.mkdir(parents=True, exist_ok=True)

    _backup_runtime_snapshot(BACKUP_TMP)
    _backup_config_files(BACKUP_TMP)

    hostname, _, _ = _run_cmd(["hostname"], timeout=5)
    manifest = {
        "timestamp": datetime.now().strftime("%Y%m%d-%H%M%S"),
        "hostname": hostname or "",
        "files": sorted(
            str(p.relative_to(BACKUP_TMP))
            for p in BACKUP_TMP.rglob("*")
            if p.is_file()
        ),
    }
    (BACKUP_TMP / "backup_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    # 原子交换：删除旧备份，rename 临时目录为新备份
    if BACKUP_ROOT.exists():
        shutil.rmtree(BACKUP_ROOT, ignore_errors=True)
    BACKUP_TMP.replace(BACKUP_ROOT)

    manifest["backup_dir"] = str(BACKUP_ROOT)
    return manifest


def check_backup_valid() -> Tuple[bool, str]:
    """校验备份是否完整可用。返回 (是否可用, 提示信息)。"""
    if not BACKUP_ROOT.exists():
        return False, f"备份目录不存在: {BACKUP_ROOT}，请先执行一次设置模式"
    manifest_file = BACKUP_ROOT / "backup_manifest.json"
    if not manifest_file.is_file():
        return False, f"备份清单缺失: {manifest_file}，备份可能损坏"

    # 至少应包含可复原的持久配置文件或运行时快照
    recoverable = (
        list((BACKUP_ROOT / "ifcfg-backup").glob("ifcfg-*"))
        if (BACKUP_ROOT / "ifcfg-backup").is_dir()
        else []
    )
    netplan = (
        list((BACKUP_ROOT / "netplan-backup").glob("*.yaml"))
        if (BACKUP_ROOT / "netplan-backup").is_dir()
        else []
    )
    has_interfaces = (BACKUP_ROOT / "interfaces.bak").is_file()
    if not (recoverable or netplan or has_interfaces):
        return False, "备份中未发现可复原的持久配置文件（ifcfg/netplan/interfaces）"
    return True, f"备份可用: {BACKUP_ROOT}"


# ============================================================
# 写入静态 IP 配置（仅写配置文件，不含启动/重启）
# ============================================================

def _write_ifcfg_rhel(interface: str, ip: str, netmask: str, gateway: str) -> None:
    """写入 RHEL 系 ifcfg 配置（openEuler/CentOS）。"""
    ifcfg_path = Path(f"/etc/sysconfig/network-scripts/ifcfg-{interface}")
    content = (
        f"TYPE=Ethernet\n"
        f"BOOTPROTO=static\n"
        f"DEFROUTE=yes\n"
        f"NAME={interface}\n"
        f"DEVICE={interface}\n"
        f"ONBOOT=yes\n"
        f"IPADDR={ip}\n"
        f"NETMASK={netmask}\n"
        f"GATEWAY={gateway}\n"
    )
    ifcfg_path.write_text(content, encoding="utf-8")


def _write_netplan(interface: str, ip: str, netmask: str, gateway: str) -> None:
    """写入 Ubuntu netplan 配置（01-rebar-static.yaml，覆盖先前生成文件）。"""
    netplan_dir = Path("/etc/netplan")
    netplan_dir.mkdir(parents=True, exist_ok=True)
    config = f"""network:
  version: 2
  ethernets:
    {interface}:
      addresses:
        - {ip}/{netmask_to_prefix(netmask)}
      routes:
        - to: default
          via: {gateway}
      dhcp4: no
"""
    (netplan_dir / "01-rebar-static.yaml").write_text(
        config, encoding="utf-8"
    )


def _write_interfaces_debian(
    interface: str, ip: str, netmask: str, gateway: str
) -> None:
    """写入 Debian 系 /etc/network/interfaces（回退方案）。"""
    interfaces_path = Path("/etc/network/interfaces")
    with open(interfaces_path, "a", encoding="utf-8") as f:
        f.write(
            f"\nauto {interface}\n"
            f"iface {interface} inet static\n"
            f"    address {ip}\n"
            f"    netmask {netmask}\n"
            f"    gateway {gateway}\n"
        )


def write_static_ip(
    interface: str, ip: str, netmask: str, gateway: str
) -> Tuple[bool, str, List[str]]:
    """写入静态 IP 配置（不启动/不重启）。

    返回 (是否成功, 写入方式, 恢复所需的重启命令)。
    """
    distro_id = detect_distro_id()
    rhel_distro = {
        "openeuler", "centos", "rhel", "fedora", "rocky",
        "almalinux", "kylin", "euleros", "anolis",
    }
    debian_distro = {"ubuntu", "debian", "raspbian", "linuxmint", "kali"}

    if distro_id in rhel_distro:
        try:
            _write_ifcfg_rhel(interface, ip, netmask, gateway)
        except OSError as e:
            return False, "ifcfg", [f"写入 ifcfg 失败: {e}"]
        return True, "ifcfg", ["systemctl", "restart", "NetworkManager"]
    elif distro_id in debian_distro:
        if Path("/etc/netplan").is_dir():
            try:
                _write_netplan(interface, ip, netmask, gateway)
            except OSError as e:
                return False, "netplan", [f"写入 netplan 失败: {e}"]
            return True, "netplan", ["netplan", "apply"]
        else:
            try:
                _write_interfaces_debian(interface, ip, netmask, gateway)
            except OSError as e:
                return False, "interfaces", [f"写入 interfaces 失败: {e}"]
            return True, "interfaces", ["systemctl", "restart", "networking"]
    else:
        return False, f"unsupported:{distro_id}", ["", "unsupported"]


# ============================================================
# 启动接口 / 重启服务
# ============================================================

def start_interface(interface: str) -> bool:
    """启动/拉起网络接口（ip link set <iface> up）。"""
    _log(f"\n[启动接口] {interface}")
    return _run_cmd_checked(["ip", "link", "set", interface, "up"], "接口启动")


def restart_network_service_fallback(restart_cmd: List[str]) -> bool:
    """重启网络服务，NetworkManager 失败时回退到 network/networking。"""
    _log(("\n[重启网络服务] ") + " ".join(restart_cmd))
    if not restart_cmd:
        return False
    if _run_cmd_checked(restart_cmd, "网络服务重启"):
        return True
    # 回退尝试
    for fallback in (["systemctl", "restart", "network"],
                     ["systemctl", "restart", "networking"]):
        if _run_cmd_checked(fallback, "网络服务重启(回退)"):
            return True
    print("[ERROR] 网络服务重启失败（含回退）", file=sys.stderr)
    return False


# ============================================================
# 复原（--restore）
# ============================================================

def _restore_netplan() -> bool:
    """从备份恢复 netplan 配置并 apply。"""
    src = BACKUP_ROOT / "netplan-backup"
    if not src.is_dir():
        print("[ERROR] 备份中无 netplan-backup 目录，无法复原 netplan 配置", file=sys.stderr)
        return False
    dst = Path("/etc/netplan")
    dst.mkdir(parents=True, exist_ok=True)
    try:
        # 先清理自动生成的静态配置，再恢复备份
        auto = dst / "01-rebar-static.yaml"
        if auto.exists():
            auto.unlink()
        for f in src.glob("*.yaml"):
            shutil.copy2(f, dst / f.name)
    except OSError as e:
        print(f"[ERROR] 复制 netplan 配置失败: {e}", file=sys.stderr)
        return False
    _log("  ✓ 已从备份恢复 netplan 配置")
    return _run_cmd_checked(["netplan", "apply"], "netplan apply")


def _restore_ifcfg() -> bool:
    """从备份恢复 RHEL 系 ifcfg 配置。"""
    src = BACKUP_ROOT / "ifcfg-backup"
    if not src.is_dir():
        print("[ERROR] 备份中无 ifcfg-backup 目录，无法复原 ifcfg 配置", file=sys.stderr)
        return False
    dst = Path("/etc/sysconfig/network-scripts")
    dst.mkdir(parents=True, exist_ok=True)
    try:
        for f in src.rglob("ifcfg-*"):
            shutil.copy2(f, dst / f.name)
    except OSError as e:
        print(f"[ERROR] 复制 ifcfg 配置失败: {e}", file=sys.stderr)
        return False
    _log("  ✓ 已从备份恢复 ifcfg 配置")
    return _run_cmd_checked(
        ["systemctl", "restart", "NetworkManager"], "NetworkManager"
    ) or _run_cmd_checked(["systemctl", "restart", "network"], "network 服务")


def _restore_interfaces() -> bool:
    """从备份恢复 /etc/network/interfaces。"""
    src = BACKUP_ROOT / "interfaces.bak"
    if not src.is_file():
        print("[ERROR] 备份中无 interfaces.bak，无法复原 interfaces 配置", file=sys.stderr)
        return False
    dst = Path("/etc/network/interfaces")
    try:
        shutil.copy2(src, dst)
    except OSError as e:
        print(f"[ERROR] 复制 interfaces 失败: {e}", file=sys.stderr)
        return False
    _log("  ✓ 已从备份恢复 /etc/network/interfaces")
    return _run_cmd_checked(["systemctl", "restart", "networking"], "networking 服务")


def run_restore(opts: SetupOptions) -> int:
    """从备份复原网络配置（--restore 模式）。"""
    print("=" * 60)
    print("服务节点 网络配置复原（--restore）")
    print("=" * 60)

    if os.geteuid() != 0:
        print(
            "[ERROR] 必须以 root 权限运行：sudo venv-python main.py setup-network --restore",
            file=sys.stderr,
        )
        return 1

    # 校验备份完整性
    ok, msg = check_backup_valid()
    if not ok:
        print(f"[ERROR] {msg}", file=sys.stderr)
        return 1
    _log(f"  ✓ {msg}")

    # 依可用内容选择复原方式
    distro_id = detect_distro_id()
    restored = False
    if Path("/etc/netplan").is_dir() and (BACKUP_ROOT / "netplan-backup").is_dir():
        restored = _restore_netplan()
    elif (BACKUP_ROOT / "ifcfg-backup").is_dir():
        restored = _restore_ifcfg()
    elif (BACKUP_ROOT / "interfaces.bak").is_file():
        restored = _restore_interfaces()
    else:
        print(
            "[ERROR] 备份内容无法匹配可复原的配置类型（当前系统与备份来源体系可能不同）",
            file=sys.stderr,
        )
        return 1

    if not restored:
        print("[ERROR] 复原失败，请检查上方日志", file=sys.stderr)
        return 1

    # 启动接口（默认，可禁用）
    interface = detect_network_interface()
    if opts.start_interface and interface:
        start_interface(interface)

    print("\n" + "=" * 60)
    print("复原完成，网络配置已从备份恢复")
    print("=" * 60)
    return 0


# ============================================================
# 主入口
# ============================================================

def run_setup_network(
    network_cfg_path: str = "config/network.json",
    opts: Optional[SetupOptions] = None,
) -> int:
    """setup-network 主入口：备份 + 设置静态 IP + 启动接口 + 重启服务。

    默认无参执行设置模式；--restore 进入复原模式。
    显式参数（--ip/--netmask/--gateway）仅本次生效，不修改配置文件。

    任一步骤失败即停止后续步骤（不回滚），输出已完成步骤与失败原因。
    """
    if opts is None:
        opts = SetupOptions()

    # ---------- 复原模式 ----------
    if opts.restore:
        return run_restore(opts)

    # ---------- 设置模式 ----------
    print("=" * 60)
    print("服务节点 一键式静态 IP 配置工具")
    print("=" * 60)

    if os.geteuid() != 0:
        print(
            "[ERROR] 必须以 root 权限运行："
            "sudo venv-python main.py setup-network",
            file=sys.stderr,
        )
        return 1

    # 读取配置
    try:
        with open(network_cfg_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        cfg_local_ip = cfg.get("local_ip")
        cfg_netmask = cfg.get("netmask", "255.255.255.252")
        cfg_gateway = cfg.get("gateway") or None
        cfg_iface = cfg.get("network_interface", "")
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"[ERROR] 读取 {network_cfg_path} 失败: {e}", file=sys.stderr)
        return 1

    # 参数优先级：显式参数 > 配置文件
    ip = opts.ip or cfg_local_ip
    netmask = opts.netmask or cfg_netmask
    interface = cfg_iface or detect_network_interface()

    if not ip:
        print(
            "[ERROR] 未获取到 IP：config/network.json 缺 local_ip 且未指定 --ip",
            file=sys.stderr,
        )
        return 1
    if not interface:
        print(
            "[ERROR] 无法检测到物理网卡，请在 config/network.json 的 "
            "network_interface 字段指定或接入网卡",
            file=sys.stderr,
        )
        return 1

    # 网关：显式 > 配置 > 自动推导 `.1`（带合法性校验）
    gateway = derive_gateway(ip, opts.gateway or cfg_gateway)

    # 打印本次执行计划（含来源标注）
    def _src(val_is_explicit: bool) -> str:
        return "(命令行)" if val_is_explicit else "(配置文件/自动推导)"

    _log(f"\n配置文件: {network_cfg_path}")
    _log(f"  网卡: {interface}")
    _log(f"  静态 IP: {ip} {_src(opts.ip is not None)}")
    _log(f"  子网掩码: {netmask} {_src(opts.netmask is not None)}")
    _log(f"  网关: {gateway}")
    _log(f"  启动接口: {'是' if opts.start_interface else '否'}")
    _log(f"  重启服务: {'是' if opts.restart_service else '否'}")
    _log("\n⚠️  警告：配置后若 IP 变化，当前 SSH 会话可能断开；请确保已通过本地终端或带外方式连接")

    # ---------- 步骤 1/4 备份 ----------
    _log("\n[步骤 1/4] 备份当前网络配置...")
    try:
        manifest = backup_network_config()
    except OSError as e:
        print(f"[ERROR] 备份失败: {e}", file=sys.stderr)
        return 1
    _log(f"  ✓ 备份完成: {manifest['backup_dir']}（文件 {len(manifest['files'])} 个）")

    # ---------- 步骤 2/4 写入静态 IP ----------
    _log("\n[步骤 2/4] 写入静态 IP 配置...")
    ok, method, restart_cmd = write_static_ip(interface, ip, netmask, gateway)
    if not ok:
        print(
            f"[ERROR] 写入静态 IP 失败（方式: {method}）: {restart_cmd}",
            file=sys.stderr,
        )
        print(
            "[提示] 已完成的变更不会自动回滚，备份已保存，可手动执行复原或人工恢复",
            file=sys.stderr,
        )
        return 1
    _log(f"  ✓ 静态 IP 已写入（方式: {method}）")

    # ---------- 步骤 3/4 启动接口 ----------
    if opts.start_interface:
        if not start_interface(interface):
            print("[ERROR] 接口启动失败，停止后续步骤", file=sys.stderr)
            return 1
    else:
        _log("\n[启动接口] 已跳过（--no-start-interface）")

    # ---------- 步骤 4/4 重启网络服务 ----------
    if opts.restart_service:
        if not restart_network_service_fallback(restart_cmd):
            print("[ERROR] 网络服务重启失败，停止后续步骤", file=sys.stderr)
            print(
                "[提示] 配置已写入但服务未重启，可稍后手动执行上述重启命令使配置生效",
                file=sys.stderr,
            )
            return 1
    else:
        _log("\n[重启网络服务] 已跳过（--no-restart-service）")

    # ---------- 验证 ----------
    _log("\n[验证] 检查新 IP 是否生效...")
    import time
    time.sleep(2)
    rc, out, _ = _run_cmd(["ip", "addr", "show", interface], timeout=5)
    if rc == 0 and ip in out:
        _log(f"  ✓ 新 IP {ip} 已生效（网卡 {interface}）")
    else:
        _log(f"  ⚠️  新 IP {ip} 未在 ip addr 中立即确认，可能需要稍等或重启；"
             f"可执行: ip addr show {interface}")

    print("\n" + "=" * 60)
    print("配置完成 | 网卡: %s | IP: %s | 掩码: %s | 网关: %s" % (interface, ip, netmask, gateway))
    print("备份目录: %s" % manifest["backup_dir"])
    print("=" * 60)
    return 0
