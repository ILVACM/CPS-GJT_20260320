#!/bin/bash
# 服务节点 系统服务管理脚本（systemd）
# 对齐 Design-server.md §9.1 与 AGENTS.md §7.1（systemd unit 文件为 INI 格式例外）
#
# 用法：
#   ./service-manager.sh install     # 注册为系统服务并设置开机自启
#   ./service-manager.sh uninstall   # 卸载系统服务并取消开机自启
#
# 权限：必须以 root 运行（脚本启动时校验）
# 参数源：../config/service.json（项目侧 JSON 源 → 部署侧 INI 产物）

set -e

# ---------- 启动校验 root ----------
if [ "$(id -u)" -ne 0 ]; then
    echo "错误：本脚本需要 root 权限运行，请使用 sudo 或以 root 用户执行。" >&2
    exit 1
fi

# ---------- 路径与参数源 ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="${SCRIPT_DIR}/../config/service.json"

if [ ! -f "$CONFIG_PATH" ]; then
    echo "错误：未找到服务配置源文件: $CONFIG_PATH" >&2
    exit 1
fi

# 从 service.json 读取参数（用 python3 解析标准 JSON）
SERVICE_NAME="$(python3 -c "import json;print(json.load(open('$CONFIG_PATH'))['service_name'])")"
EXEC_START="$(python3 -c "import json;print(json.load(open('$CONFIG_PATH'))['exec_start'])")"
WORKING_DIRECTORY="$(python3 -c "import json;print(json.load(open('$CONFIG_PATH'))['working_directory'])")"
SERVICE_USER="$(python3 -c "import json;print(json.load(open('$CONFIG_PATH'))['user'])")"
RESTART_POLICY="$(python3 -c "import json;print(json.load(open('$CONFIG_PATH'))['restart'])")"
SERVICE_ENVIRONMENT="$(python3 -c "import json;print(json.load(open('$CONFIG_PATH')).get('environment', ''))")"

UNIT_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

# ---------- install 子命令 ----------
do_install() {
    echo "安装系统服务: ${SERVICE_NAME}"
    echo "  ExecStart       = ${EXEC_START}"
    echo "  WorkingDirectory= ${WORKING_DIRECTORY}"
    echo "  User            = ${SERVICE_USER}"
    echo "  Restart         = ${RESTART_POLICY}"

    # 可选 Environment 行（仅当 service.json 配置了 environment 字段时生成）
    ENV_LINE=""
    if [[ -n "$SERVICE_ENVIRONMENT" ]]; then
        ENV_LINE="Environment=${SERVICE_ENVIRONMENT}"
    fi

    # 生成 systemd unit 文件（INI 格式，AGENTS.md §7.1 例外）
    cat > "$UNIT_FILE" <<EOF
[Unit]
Description=Rebar Node Server Service (钢筋直径测量设备 - 服务节点)
After=network.target display-manager.service

[Service]
Type=simple
ExecStart=${EXEC_START}
WorkingDirectory=${WORKING_DIRECTORY}
User=${SERVICE_USER}
Restart=${RESTART_POLICY}
RestartSec=5
StandardOutput=journal
StandardError=journal
${ENV_LINE}

[Install]
WantedBy=graphical.target
EOF

    echo "unit 文件已生成: ${UNIT_FILE}"
    systemctl daemon-reload
    systemctl enable "${SERVICE_NAME}"
    systemctl start "${SERVICE_NAME}"
    echo "服务 ${SERVICE_NAME} 已启动并设置为开机自启"
    systemctl status "${SERVICE_NAME}" --no-pager || true
}

# ---------- uninstall 子命令 ----------
do_uninstall() {
    echo "卸载系统服务: ${SERVICE_NAME}"
    # 若服务正在运行则先停止
    if systemctl is-active --quiet "${SERVICE_NAME}"; then
        systemctl stop "${SERVICE_NAME}"
        echo "服务已停止"
    else
        echo "服务未运行，跳过 stop"
    fi
    systemctl disable "${SERVICE_NAME}" || true
    echo "已取消开机自启"
    if [ -f "$UNIT_FILE" ]; then
        rm -f "$UNIT_FILE"
        echo "unit 文件已删除: ${UNIT_FILE}"
    fi
    systemctl daemon-reload
    echo "服务 ${SERVICE_NAME} 卸载完成"
}

# ---------- 命令分发 ----------
case "$1" in
    install)
        do_install
        ;;
    uninstall)
        do_uninstall
        ;;
    *)
        echo "用法：$0 {install|uninstall}" >&2
        exit 1
        ;;
esac
