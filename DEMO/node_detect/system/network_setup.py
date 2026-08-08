"""网络配置模块：备份 + 静态 IP 设置。

对齐 AGENTS.md §4.1 网络拓扑（双节点网线直连 + 静态 IP）。
独立子命令由人工触发，不在 systemd 启动流程内自动执行。

注：本文件与 node_server/system/network_setup.py 对称实现，
仅"节点标题"与"systemd 服务名"两处差异。
"""
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LOGS_DIR = PROJECT_ROOT / "logs"


def backup_network_config() -> dict:
    """备份当前网络配置到 logs/network_backup_YYYYMMDD-HHMMSS/ 目录。

    备份内容：
    - ip addr / ip route 输出
    - nmcli dev show / nmcli connection show（若 nmcli 可用）
    - /etc/resolv.conf
    - /etc/sysconfig/network-scripts/ifcfg-*（RHEL系）
    - /etc/network/interfaces（Debian系）
    - /etc/netplan/*.yaml（Ubuntu netplan）

    返回：备份信息字典（backup_dir / files / timestamp）
    """
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    backup_dir = LOGS_DIR / f"network_backup_{timestamp}"
    backup_dir.mkdir(parents=True, exist_ok=True)

    # 命令输出保存
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

    # 配置文件备份
    config_files = [
        "/etc/resolv.conf",
        "/etc/network/interfaces",
    ]
    for cf in config_files:
        if os.path.exists(cf):
            dst = backup_dir / (Path(cf).name + ".bak")
            try:
                shutil.copy2(cf, dst)
            except OSError as e:
                (backup_dir / (Path(cf).name + ".bak.error")).write_text(str(e))

    # 目录批量备份
    for src_dir, dst_name in [
        ("/etc/sysconfig/network-scripts", "ifcfg-backup"),
        ("/etc/netplan", "netplan-backup"),
    ]:
        if os.path.isdir(src_dir):
            dst = backup_dir / dst_name
            try:
                shutil.copytree(src_dir, dst)
            except OSError as e:
                (backup_dir / f"{dst_name}.error").write_text(str(e))

    # 写入备份清单
    manifest = {
        "timestamp": timestamp,
        "backup_dir": str(backup_dir),
        "hostname": subprocess.run(
            ["hostname"], capture_output=True, text=True
        ).stdout.strip(),
        "files": sorted(
            [
                str(p.relative_to(backup_dir))
                for p in backup_dir.rglob("*")
                if p.is_file()
            ]
        ),
    }
    (backup_dir / "backup_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return manifest


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
        wireless_path = iface_path / "wireless"
        if wireless_path.exists():
            continue
        # 检查是否为物理网卡（有 device 符号链接）
        if (iface_path / "device").is_symlink():
            candidates.append(iface)

    if not candidates:
        # 回退：返回所有非 lo 网卡
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


def setup_static_ip_rhel(interface: str, local_ip: str, netmask: str) -> bool:
    """RHEL系（openEuler/CentOS）：写入 /etc/sysconfig/network-scripts/ifcfg-<iface>。"""
    ifcfg_path = Path(f"/etc/sysconfig/network-scripts/ifcfg-{interface}")

    # 备份原文件
    if ifcfg_path.exists():
        backup = ifcfg_path.with_suffix(".bak")
        shutil.copy2(ifcfg_path, backup)

    # 构造 ifcfg 内容
    content = f"""TYPE=Ethernet
BOOTPROTO=static
DEFROUTE=yes
NAME={interface}
DEVICE={interface}
ONBOOT=yes
IPADDR={local_ip}
NETMASK={netmask}
"""
    ifcfg_path.write_text(content, encoding="utf-8")

    # 重启网络服务
    try:
        subprocess.run(
            ["systemctl", "restart", "NetworkManager"], check=True, timeout=15
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        print(f"[WARN] NetworkManager 重启失败: {e}，尝试 network 服务", file=sys.stderr)
        try:
            subprocess.run(["systemctl", "restart", "network"], check=True, timeout=15)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e2:
            print(f"[ERROR] network 服务重启也失败: {e2}", file=sys.stderr)
            return False
    return True


def netmask_to_prefix(netmask: str) -> int:
    """将 netmask 转为 prefix length（如 255.255.255.252 → 30）。"""
    parts = netmask.split(".")
    if len(parts) != 4:
        return 24
    binary = "".join(bin(int(p))[2:].zfill(8) for p in parts)
    return binary.count("1")


def setup_static_ip_debian(interface: str, local_ip: str, netmask: str) -> bool:
    """Debian系（Ubuntu）：写入 /etc/network/interfaces 或 netplan 配置。

    优先使用 netplan（Ubuntu 18.04+），回退到 interfaces。
    """
    # 检测 netplan
    netplan_dir = Path("/etc/netplan")
    if netplan_dir.exists() and list(netplan_dir.glob("*.yaml")):
        # 备份现有 netplan 配置
        backup_dir = netplan_dir / "backup"
        backup_dir.mkdir(exist_ok=True)
        for f in netplan_dir.glob("*.yaml"):
            shutil.copy2(f, backup_dir / f.name)

        # 写入新配置
        netplan_config = f"""network:
  version: 2
  ethernets:
    {interface}:
      addresses:
        - {local_ip}/{netmask_to_prefix(netmask)}
"""
        (netplan_dir / "01-rebar-static.yaml").write_text(
            netplan_config, encoding="utf-8"
        )

        try:
            subprocess.run(["netplan", "apply"], check=True, timeout=30)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            print(f"[ERROR] netplan apply 失败: {e}", file=sys.stderr)
            return False
        return True
    else:
        # 回退到 /etc/network/interfaces
        interfaces_path = Path("/etc/network/interfaces")
        if interfaces_path.exists():
            backup = interfaces_path.with_suffix(".bak")
            shutil.copy2(interfaces_path, backup)

        content = f"""
auto {interface}
iface {interface} inet static
    address {local_ip}
    netmask {netmask}
"""
        with open(interfaces_path, "a", encoding="utf-8") as f:
            f.write(content)

        try:
            subprocess.run(
                ["systemctl", "restart", "networking"], check=True, timeout=15
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            print(f"[ERROR] networking 重启失败: {e}", file=sys.stderr)
            return False
        return True


def run_setup_network(network_cfg_path: str = "config/network.json") -> int:
    """主入口：备份 + 配置静态 IP。

    步骤：
    1. 读取 config/network.json
    2. 备份当前网络配置
    3. 检测发行版与网卡名
    4. 提示用户确认
    5. 调用对应 setup_static_ip_* 函数
    6. 验证新 IP 生效
    7. 输出结果与下一步指引
    """
    print("=" * 60)
    print("检测节点 静态 IP 配置工具")
    print("=" * 60)

    # root 权限检查
    if os.geteuid() != 0:
        print(
            "[ERROR] 必须以 root 权限运行："
            "sudo /opt/node_detect/.venv/bin/python /opt/node_detect/main.py setup-network",
            file=sys.stderr,
        )
        return 1

    # 读取配置
    try:
        with open(network_cfg_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        local_ip = cfg["local_ip"]
        remote_ip = cfg["remote_ip"]
    except (FileNotFoundError, KeyError, json.JSONDecodeError) as e:
        print(f"[ERROR] 读取 {network_cfg_path} 失败: {e}", file=sys.stderr)
        return 1

    netmask = cfg.get("netmask", "255.255.255.252")
    configured_iface = cfg.get("network_interface", "")

    print(f"\n配置文件: {network_cfg_path}")
    print(f"  目标本机 IP: {local_ip}")
    print(f"  对端 IP: {remote_ip}")
    print(f"  子网掩码: {netmask}")

    # 第一步：备份
    print("\n[步骤 1/4] 备份当前网络配置...")
    manifest = backup_network_config()
    print(f"  ✓ 备份完成: {manifest['backup_dir']}")
    print(f"  ✓ 备份文件数: {len(manifest['files'])}")

    # 第二步：检测网卡
    print("\n[步骤 2/4] 检测物理网卡...")
    interface = configured_iface or detect_network_interface()
    if not interface:
        print(
            "[ERROR] 无法检测到物理网卡，请在 config/network.json 的 network_interface 字段指定",
            file=sys.stderr,
        )
        return 1
    print(f"  ✓ 检测到网卡: {interface}")

    # 第三步：用户确认
    print(f"\n[步骤 3/4] 即将修改网络配置：")
    print(f"  网卡: {interface}")
    print(f"  静态 IP: {local_ip}")
    print(f"  子网掩码: {netmask}")
    print(f"\n⚠️  警告：修改 IP 后当前 SSH 会话可能断开！")
    print(f"⚠️  请确保已通过本地终端或带外方式连接设备")
    print(f"⚠️  备份已保存至: {manifest['backup_dir']}")
    confirm = input("\n输入 'yes' 确认继续，其他任意键取消: ").strip().lower()
    if confirm != "yes":
        print("[取消] 用户取消，未做任何修改")
        return 0

    # 第四步：配置静态 IP
    print("\n[步骤 4/4] 配置静态 IP...")
    # 识别发行版
    distro_id = ""
    if os.path.exists("/etc/os-release"):
        with open("/etc/os-release", "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("ID="):
                    distro_id = line.split("=", 1)[1].strip().strip('"').lower()
                    break

    rhel_distro = {
        "openeuler", "centos", "rhel", "fedora", "rocky",
        "almalinux", "kylin", "euleros", "anolis",
    }
    debian_distro = {"ubuntu", "debian", "raspbian", "linuxmint", "kali"}

    if distro_id in rhel_distro:
        ok = setup_static_ip_rhel(interface, local_ip, netmask)
    elif distro_id in debian_distro:
        ok = setup_static_ip_debian(interface, local_ip, netmask)
    else:
        print(f"[ERROR] 不支持的发行版: {distro_id}", file=sys.stderr)
        return 1

    if not ok:
        print("[ERROR] 配置失败，请检查上方日志", file=sys.stderr)
        return 1

    # 验证
    print("\n[验证] 等待 3 秒后检查新 IP...")
    import time
    time.sleep(3)

    try:
        result = subprocess.run(
            ["ip", "addr", "show", interface],
            capture_output=True, text=True, timeout=5,
        )
        if local_ip in result.stdout:
            print(f"  ✓ 新 IP {local_ip} 已生效")
        else:
            print(f"  ⚠️  新 IP 未在 ip addr 中找到，可能需要重启网络或重新登录")
            print(f"  请手动执行: ip addr show {interface}")
    except subprocess.TimeoutExpired:
        print("  ⚠️  ip addr 查询超时")

    print("\n" + "=" * 60)
    print("配置完成")
    print("=" * 60)
    print(f"备份目录: {manifest['backup_dir']}")
    print(f"网卡: {interface}")
    print(f"静态 IP: {local_ip}")
    print("\n下一步操作：")
    print(f"  1. 若 SSH 断开，请用新 IP {local_ip} 重新连接")
    print(f"  2. 重启 systemd 服务：sudo systemctl restart node-detect-inference")
    print(f"  3. 检查状态：sudo systemctl status node-detect-inference")
    return 0
