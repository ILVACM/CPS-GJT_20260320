#!/bin/bash
# ==============================================================================
# verify_xauth.sh — 验证 root 通过 XAUTHORITY cookie 能否连 X Server
#
# 用途：确认 systemd 服务（root 运行）能否通过 DISPLAY+XAUTHORITY 启动 tkinter
#       若验证通过，则 service.json 方案 A 可行
#
# 用法：sudo bash verify_xauth.sh
# ==============================================================================

set -u

echo "=============================================================="
echo "X Server 授权验证测试"
echo "执行时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "=============================================================="
echo ""

# ---------- 前置检查 ----------
echo "========== 1. 前置检查 =========="

echo "[1.1] 当前用户:"
id
echo ""

echo "[1.2] X Server 套接字:"
ls -la /tmp/.X11-unix/ 2>&1
echo ""

echo "[1.3] GDM Xauthority 文件（关键）:"
XAUTH_FILE="/run/user/1200/gdm/Xauthority"
if [[ -f "${XAUTH_FILE}" ]]; then
    echo "  路径: ${XAUTH_FILE}"
    echo "  权限: $(ls -l ${XAUTH_FILE})"
    echo "  属主: $(stat -c '%U:%G' ${XAUTH_FILE})"
    echo "  root 可读: $([[ -r "${XAUTH_FILE}" ]] && echo "是" || echo "否")"
else
    echo "  ✗ ${XAUTH_FILE} 不存在"
    echo "  → openEuler 用户可能未登录桌面，需先登录图形会话"
    exit 1
fi
echo ""

echo "[1.4] Python venv 路径:"
PYTHON_BIN="/opt/node_server/.venv/bin/python"
if [[ -x "${PYTHON_BIN}" ]]; then
    echo "  ${PYTHON_BIN} → $(${PYTHON_BIN} --version)"
else
    echo "  ✗ ${PYTHON_BIN} 不存在或不可执行"
    exit 1
fi
echo ""

# ---------- 核心验证测试 ----------
echo "========== 2. 核心验证测试 =========="

echo "[2.1] 测试1: 仅 DISPLAY=:1（预期失败）"
echo "  命令: env -i DISPLAY=:1 ${PYTHON_BIN} -c 'import tkinter; ...'"
env -i DISPLAY=:1 "${PYTHON_BIN}" -c 'import tkinter; r=tkinter.Tk(); print("  → OK"); r.destroy()' 2>&1 | head -5
echo "  预期: No protocol specified（root 被 xhost 拒绝）"
echo ""

echo "[2.2] 测试2: DISPLAY=:1 + XAUTHORITY（关键测试）"
echo "  命令: env -i DISPLAY=:1 XAUTHORITY=${XAUTH_FILE} ${PYTHON_BIN} -c 'import tkinter; ...'"
env -i DISPLAY=:1 XAUTHORITY="${XAUTH_FILE}" "${PYTHON_BIN}" -c 'import tkinter; r=tkinter.Tk(); print("  → OK：root 通过 cookie 授权成功连接 X Server"); r.destroy()' 2>&1 | head -10
RESULT=$?
echo ""
if [[ ${RESULT} -eq 0 ]]; then
    echo "  ✓✓✓ 验证通过 ✓✓✓"
    echo "  → 方案 A 可行：service.json 加 DISPLAY=:1 XAUTHORITY=${XAUTH_FILE}"
    echo ""
    echo "  建议的 service.json environment 字段:"
    echo "    \"environment\": \"PYTHONPATH=/opt/node_server DISPLAY=:1 XAUTHORITY=${XAUTH_FILE}\""
else
    echo "  ✗ 验证失败"
    echo ""
    echo "  可能原因:"
    echo "    1. Xauthority 文件已被 GDM 轮换（openEuler 重登后路径变化）"
    echo "    2. X Server 配置了额外授权限制"
    echo ""
    echo "  Fallback 方案（xhost 放宽）:"
    echo "    在 openEuler 桌面执行: xhost +SI:localuser:root"
    echo "    或加入 autostart: ~/.config/autostart/xhost-root.desktop"
fi
echo ""

# ---------- 进阶测试（可选） ----------
echo "========== 3. 进阶测试（完整 GUI 窗口） =========="
echo "[3.1] 尝试启动完整 tk 窗口（3 秒后自动关闭）"
echo "  命令: env -i DISPLAY=:1 XAUTHORITY=${XAUTH_FILE} ${PYTHON_BIN} -c '...mainloop()...'"
env -i DISPLAY=:1 XAUTHORITY="${XAUTH_FILE}" "${PYTHON_BIN}" -c '
import tkinter
r = tkinter.Tk()
r.title("X Server 授权验证")
r.geometry("300x100")
lbl = tkinter.Label(r, text="如果看到此窗口说明授权成功", font=("Arial", 12))
lbl.pack(expand=True)
r.after(3000, r.destroy)  # 3 秒后自动关闭
r.mainloop()
print("  → 窗口正常显示并关闭")
' 2>&1 | head -5
echo ""

echo "=============================================================="
echo "验证完成。"
echo "若 [2.2] 显示 OK，请将结果发回，据此修改 service.json"
echo "=============================================================="
