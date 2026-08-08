#!/bin/bash
# ==============================================================================
# diagnose_display_env.sh — node_server systemd GUI 启动环境诊断脚本
#
# 用途：采集 openEuler 设备的 X11/Display/授权环境信息，用于决策 systemd 服务
#       启动 GUI 所需的 DISPLAY / XAUTHORITY 配置
#
# 用法：
#   sudo bash diagnose_display_env.sh           # 采集到终端
#   sudo bash diagnose_display_env.sh > diag.log 2>&1   # 采集到文件
#
# 执行后将输出发回开发机分析
# ==============================================================================

set -u

echo "=============================================================="
echo "node_server systemd GUI 启动环境诊断"
echo "执行时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "执行主机: $(hostname)"
echo "=============================================================="
echo ""

# ---------- 1. 系统与桌面环境基本信息 ----------
echo "========== 1. 系统与桌面环境基本信息 =========="
echo "[1.1] 操作系统:"
cat /etc/os-release | grep -E "^(NAME|VERSION|ID)="
echo ""

echo "[1.2] 内核与架构:"
uname -a
echo ""

echo "[1.3] 当前 systemd 默认启动目标:"
systemctl get-default
echo "  → graphical.target = 图形登录（X Server 自启）"
echo "  → multi-user.target = 无头（需手动启 X 或改配置）"
echo ""

echo "[1.4] display-manager 服务状态（X Server 守护进程）:"
systemctl status display-manager.service --no-pager 2>&1 | head -10 || echo "  (display-manager.service 不存在)"
echo ""

echo "[1.5] gdm / sddm / lightdm 服务状态（常见桌面管理器）:"
for svc in gdm sddm lightdm; do
    if systemctl list-unit-files | grep -q "^${svc}.service"; then
        echo "  ${svc}: $(systemctl is-enabled ${svc}.service 2>/dev/null) / $(systemctl is-active ${svc}.service 2>/dev/null)"
    fi
done
echo ""

# ---------- 2. 当前活跃的 X 会话与 DISPLAY 值 ----------
echo "========== 2. 当前活跃的 X 会话与 DISPLAY 值 =========="
echo "[2.1] who 命令（登录用户与显示）:"
who
echo ""

echo "[2.2] w 命令（详细登录会话）:"
w
echo ""

echo "[2.3] 登录用户的 DISPLAY 值（从 w 提取）:"
# w -hs 输出: USER TTY FROM LOGIN@ IDLE JCPU PCPU WHAT
# 第二列是 TTY，可能为 :0、:1、tty1 等
echo "  活跃 TTY 列表: $(w -hs | awk '{print $2}' | tr '\n' ' ')"
echo ""

echo "[2.4] 检查 X Server 进程:"
ps aux | grep -E "Xorg|Xwayland|gnome-shell|kwin" | grep -v grep | head -5 || echo "  (未发现 X Server 进程)"
echo ""

echo "[2.5] /tmp/.X11-unix 目录下的 X 套接字（每个文件代表一个 DISPLAY）:"
ls -la /tmp/.X11-unix/ 2>&1
echo "  → 文件 X0 对应 DISPLAY=:0，X1 对应 DISPLAY=:1，以此类推"
echo ""

# ---------- 3. X 授权（XAUTHORITY）相关信息 ----------
echo "========== 3. X 授权（XAUTHORITY）相关信息 =========="

# 找出当前登录的桌面用户（非 root 的普通用户，或 root 本身）
DESKTOP_USER=$(w -hs | awk '$2 ~ /^:/ {print $1; exit}')
if [[ -z "${DESKTOP_USER:-}" ]]; then
    DESKTOP_USER=$(w -hs | awk '{print $1; exit}')
fi
echo "[3.1] 检测到的桌面用户: ${DESKTOP_USER:-（未检测到）}"
echo ""

echo "[3.2] 该用户的家目录与 .Xauthority 文件:"
if [[ -n "${DESKTOP_USER:-}" ]]; then
    USER_HOME=$(getent passwd "${DESKTOP_USER}" | cut -d: -f6)
    echo "  用户家目录: ${USER_HOME}"
    echo "  ~/.Xauthority 存在性: $([[ -f "${USER_HOME}/.Xauthority" ]] && echo "是 ($(ls -l ${USER_HOME}/.Xauthority | awk '{print $5}') bytes)" || echo "否")"
    echo "  ~/.Xauthority 权限: $(ls -l ${USER_HOME}/.Xauthority 2>/dev/null || echo "N/A")"
else
    echo "  (未检测到桌面用户，跳过)"
fi
echo ""

echo "[3.3] /run/user 目录下的 Xauthority 相关文件:"
ls -la /run/user/*/gdm/Xauthority 2>/dev/null || echo "  (无 gdm/Xauthority)"
ls -la /run/user/*/.mutter-Xwaylandauth.* 2>/dev/null || echo "  (无 mutter-Xwaylandauth)"
find /run/user -name "*Xauthority*" -o -name "*xauth*" 2>/dev/null | head -5
echo ""

echo "[3.4] root 用户的 .Xauthority（若 sudo 启动时用过）:"
ls -la /root/.Xauthority 2>/dev/null || echo "  /root/.Xauthority 不存在"
echo ""

echo "[3.5] xhost 授权策略（关键）:"
which xhost >/dev/null 2>&1 && {
    echo "  当前 xhost 列表:"
    DISPLAY=:0 xhost 2>&1 | head -5 || echo "  (DISPLAY=:0 不可访问)"
    echo "  ---"
    DISPLAY=:1 xhost 2>&1 | head -5 || echo "  (DISPLAY=:1 不可访问)"
} || echo "  (xhost 命令不存在，需安装 xorg-x11-server-utils)"
echo "  → 'SI:localuser:root' 表示 root 本地可连"
echo "  → 'LOCAL:' 或 'LOCAL:0' 表示本地任意用户可连（宽松）"
echo ""

# ---------- 4. 模拟 systemd 服务环境测试 ----------
echo "========== 4. 模拟 systemd 服务环境测试 =========="
echo "[4.1] systemd 服务环境变量（systemctl show 环境）:"
systemctl show-environment | grep -E "DISPLAY|XAUTH" || echo "  (systemd 环境无 DISPLAY/XAUTHORITY)"
echo ""

echo "[4.2] 模拟 systemd 最小环境测试 tkinter（仅 DISPLAY=:1）:"
echo "  执行: env -i DISPLAY=:1 /opt/node_server/.venv/bin/python -c 'import tkinter; r=tkinter.Tk(); print(\"DISPLAY=:1 OK\"); r.destroy()'"
env -i DISPLAY=:1 /opt/node_server/.venv/bin/python -c 'import tkinter; r=tkinter.Tk(); print("  → DISPLAY=:1 OK"); r.destroy()' 2>&1 | head -5 || echo "  → 失败"
echo ""

echo "[4.3] 模拟 systemd 最小环境测试 tkinter（DISPLAY=:0）:"
echo "  执行: env -i DISPLAY=:0 /opt/node_server/.venv/bin/python -c 'import tkinter; r=tkinter.Tk(); print(\"DISPLAY=:0 OK\"); r.destroy()'"
env -i DISPLAY=:0 /opt/node_server/.venv/bin/python -c 'import tkinter; r=tkinter.Tk(); print("  → DISPLAY=:0 OK"); r.destroy()' 2>&1 | head -5 || echo "  → 失败"
echo ""

echo "[4.4] 模拟 systemd 环境 + XAUTHORITY 测试:"
if [[ -n "${DESKTOP_USER:-}" ]]; then
    USER_HOME=$(getent passwd "${DESKTOP_USER}" | cut -d: -f6)
    XAUTH_FILE="${USER_HOME}/.Xauthority"
    if [[ -f "${XAUTH_FILE}" ]]; then
        echo "  执行: env -i DISPLAY=:1 XAUTHORITY=${XAUTH_FILE} /opt/node_server/.venv/bin/python -c '...'"
        env -i DISPLAY=:1 XAUTHORITY="${XAUTH_FILE}" /opt/node_server/.venv/bin/python -c 'import tkinter; r=tkinter.Tk(); print("  → DISPLAY=:1 + XAUTH OK"); r.destroy()' 2>&1 | head -5 || echo "  → 失败"
    else
        echo "  ${XAUTH_FILE} 不存在，跳过"
    fi
else
    echo "  (未检测到桌面用户，跳过)"
fi
echo ""

# ---------- 5. 网络与 gRPC 相关（次要问题排查） ----------
echo "========== 5. 网络与 gRPC 相关 =========="
echo "[5.1] 本机所有网卡 IP:"
ip -4 addr show | grep -E "inet " | awk '{print $NF": "$2}'
echo ""

echo "[5.2] 期望的设计 IP（来自 network.json）:"
cat /opt/node_server/config/network.json 2>/dev/null | grep -E "local_ip|remote_ip" || echo "  (network.json 不存在)"
echo ""

echo "[5.3] 50051 端口监听情况:"
ss -tlnp | grep 50051 || echo "  (50051 端口无监听)"
echo ""

echo "[5.4] 对端检测节点 192.168.10.2 可达性:"
ping -c 2 -W 2 192.168.10.2 2>&1 | tail -3 || echo "  (不可达)"
echo ""

# ---------- 6. Python 与 tkinter 依赖确认 ----------
echo "========== 6. Python 与 tkinter 依赖确认 =========="
echo "[6.1] venv Python 版本:"
/opt/node_server/.venv/bin/python --version
echo ""

echo "[6.2] _tkinter 依赖库（应为 Tcl/Tk 8.6）:"
ldd $(/opt/node_server/.venv/bin/python -c "import _tkinter; print(_tkinter.__file__)") 2>&1 | grep -iE "tcl|tk" || echo "  (无 tcl/tk 依赖)"
echo ""

echo "[6.3] Tcl/Tk 版本:"
/opt/node_server/.venv/bin/python -c "import tkinter; print('Tcl:', tkinter.TclVersion); print('Tk:', tkinter.TkVersion)"
echo ""

echo "=============================================================="
echo "诊断完成。请将以上完整输出发回开发机分析。"
echo "=============================================================="
