#!/usr/bin/env bash
# ==============================================================================
# setup_network.sh — 检测节点静态 IP 配置脚本（包装）
#
# 用途：备份当前网络配置 + 设置静态 IP，对齐 AGENTS.md §4.1/§4.2
# 执行：sudo bash scripts/setup_network.sh
#
# 详见 QUICKSTART.md §5.3 网络配置（IP 不匹配时使用）
# ==============================================================================
set -euo pipefail

INSTALL_DIR="/opt/node_detect"
VENV_PYTHON="$INSTALL_DIR/.venv/bin/python"

# root 权限检查
if [[ $EUID -ne 0 ]]; then
    echo "[ERROR] 必须以 root 权限运行：sudo bash scripts/setup_network.sh" >&2
    exit 1
fi

# 验证 Python 解释器存在
if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "[ERROR] uv 虚拟环境 Python 不存在: $VENV_PYTHON" >&2
    echo "[ERROR] 请先完成部署（QUICKSTART.md §二 或 §四）" >&2
    exit 1
fi

# 切换到安装目录
cd "$INSTALL_DIR"

# 调用 Python 子命令
exec "$VENV_PYTHON" main.py setup-network
