#!/usr/bin/env bash
# ==============================================================================
# deploy_node_server.sh — 服务节点（OrangePi KunPeng）一键部署脚本
#
# 用途：在裸机 OrangePi KunPeng（默认 openEuler 22 LTS）上完成从空系统到
#       可启动 systemd 服务的全流程部署。脚本自适应识别 Linux 发行版：
#         - Debian 系（Ubuntu/Debian/Raspbian）→ 使用 apt + DEB 包名
#         - RHEL 系（openEuler/CentOS/RHEL/Fedora/Rocky/AlmaLinux）→ 使用 dnf + RPM 包名
#
# 执行：sudo bash scripts/deploy_node_server.sh [--aliyun]
#   默认          使用系统当前软件源（dnf/apt + uv），不修改任何 /etc 配置
#   --aliyun      本次运行临时使用阿里云镜像源（dnf/apt + uv），单次生效，不写 /etc
#
# 设计原则：
#   - set -e 任一步骤失败立即停止，不产生半安装状态
#   - 所有外部命令均显式校验返回值
#   - 关键节点输出日志，便于排错
#   - 外设（相机、S21C 串口）由人工插入，脚本不自动检测
#
# 对应文档：node_server/QUICKSTART.md
# ==============================================================================

set -euo pipefail

# ------------------------------------------------------------------------------
# 全局变量
# ------------------------------------------------------------------------------
INSTALL_DIR="/opt/node_server"
SERVICE_NAME="rebar-node-server"
PYTHON_VERSION="3.10"
LOG_PREFIX="[deploy_node_server]"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info()    { echo -e "${GREEN}${LOG_PREFIX}${NC} $1"; }
log_warn()    { echo -e "${YELLOW}${LOG_PREFIX}${NC} $1"; }
log_error()   { echo -e "${RED}${LOG_PREFIX}${NC} $1" >&2; }
log_step()    { echo -e "${BLUE}${LOG_PREFIX}====> $1${NC}"; }

# ------------------------------------------------------------------------------
# 镜像源参数控制（全局变量，由命令行参数初始化）
# ------------------------------------------------------------------------------
ALIYUN_MIRROR=0          # 默认使用系统当前软件源；--aliyun 时置 1
TEMP_REPO_DIR=""         # --aliyun 临时 repo 目录（退出时由 cleanup 清理）
DNF_OPTS=""              # dnf 额外选项（--aliyun 时填充 --setopt=reposdir ...）
APT_OPTS=""              # apt 额外选项（--aliyun 时填充 -o Dir::Etc::sourcelist ...）

# ------------------------------------------------------------------------------
# 发行版识别与包名映射（跨发行版自适应）
# ------------------------------------------------------------------------------
# 逻辑包名 → 实际包名映射表
# Debian 系（Ubuntu/Debian）使用 DEB 包名；RHEL 系（openEuler/CentOS/RHEL）使用 RPM 包名
# 注：libgthread 在 RHEL 系无独立包，libgthread-2.0.so.0 由 glib2 主包提供（映射为空字符串=跳过）
declare -A DEB_PKG_MAP=(
    [opengl]="libgl1"
    [opengl-glu]="libglu1-mesa"
    [opengl-egl]="libegl1"
    [glib2]="libglib2.0-0"
    [glib2-dev]="libglib2.0-dev"
    [libsm]="libsm6"
    [libxext]="libxext6"
    [libxrender]="libxrender1"
    [libxtst]="libxtst6"
    [libxi]="libxi6"
    [libgthread]="libgthread2.0-0"
    [gtk2]="libgtk2.0-0"
    [gtk3]="libgtk-3-0"
    [gtk2-dev]="libgtk2.0-dev"
    [gtk3-dev]="libgtk-3-dev"
    [build-essential]="build-essential"
    [pkg-config]="pkg-config"
    [python-dev]="python3-dev"
    [kernel-dev]="linux-headers-$(uname -r)"
    [tkinter]="python3-tk"
    [x11-server]="xorg"
    [xwayland]="xwayland"
    [x11-utils]="x11-utils"
    [xauth]="xauth"
    [usbutils]="usbutils"
    [util-linux]="util-linux"
    [shadow-utils]="passwd"
    [sudo]="sudo"
)

declare -A RHEL_PKG_MAP=(
    [opengl]="mesa-libGL"
    [opengl-glu]="mesa-libGLU"
    [opengl-egl]="mesa-libEGL"
    [glib2]="glib2"
    [glib2-dev]="glib2-devel"
    [libsm]="libSM"
    [libxext]="libXext"
    [libxrender]="libXrender"
    [libxtst]="libXtst"
    [libxi]="libXi"
    [libgthread]=""                       # glib2 已含 libgthread-2.0.so.0，无需独立安装
    [gtk2]="gtk2"
    [gtk3]="gtk3"
    [gtk2-dev]="gtk2-devel"
    [gtk3-dev]="gtk3-devel"
    [build-essential]="gcc gcc-c++ make cmake"
    [pkg-config]="pkgconfig"
    [python-dev]="python3-devel"
    [kernel-dev]="kernel-devel"
    [tkinter]="python3-tkinter"
    [x11-server]="xorg-x11-server-Xorg"
    [xwayland]="xorg-x11-server-Xwayland"
    [x11-utils]="xorg-x11-utils"
    [xauth]="xorg-x11-xauth"
    [usbutils]="usbutils"
    [util-linux]="util-linux"
    [shadow-utils]="shadow-utils"
    [sudo]="sudo"
)

# 识别 Linux 发行版，设置 DISTRO_FAMILY 与 PKG_MANAGER 全局变量
# - DISTRO_FAMILY: "debian" 或 "rhel"
# - PKG_MANAGER:   "apt-get" / "dnf" / "yum"
detect_distro() {
    if [[ ! -f /etc/os-release ]]; then
        log_error "无法识别发行版（/etc/os-release 不存在），脚本终止"
        exit 1
    fi
    . /etc/os-release
    DISTRO_ID="${ID:-unknown}"
    DISTRO_ID="${DISTRO_ID,,}"              # 归一化为小写，消除大小写差异（如 openEuler → openeuler）
    DISTRO_NAME="${NAME:-unknown}"          # 保留原大小写用于日志显示
    DISTRO_FAMILY=""
    PKG_MANAGER=""

    case "$DISTRO_ID" in
        ubuntu|debian|raspbian|linuxmint|kali)
            DISTRO_FAMILY="debian"
            PKG_MANAGER="apt-get"
            ;;
        centos|rhel|fedora|openeuler|rocky|almalinux|kylin|euleros|anolis)
            DISTRO_FAMILY="rhel"
            if command -v dnf &>/dev/null; then
                PKG_MANAGER="dnf"
            elif command -v yum &>/dev/null; then
                PKG_MANAGER="yum"
            else
                log_error "RHEL 系发行版（$DISTRO_NAME）但未找到 dnf 或 yum，脚本终止"
                exit 1
            fi
            ;;
        *)
            log_error "不支持的发行版: $DISTRO_NAME (ID=$DISTRO_ID)"
            log_error "支持: Ubuntu/Debian, openEuler/CentOS/RHEL/Fedora/Rocky/AlmaLinux/Kylin/EulerOS/Anolis"
            exit 1
            ;;
    esac

    log_info "✓ 检测到发行版: $DISTRO_NAME ($DISTRO_FAMILY 系), 包管理器: $PKG_MANAGER"
}

# 通用包安装函数：参数为逻辑包名列表，根据 DISTRO_FAMILY 自动映射为实际包名
# 用法：install_pkgs "opengl" "glib2" "gtk3"
install_pkgs() {
    local pkgs=()
    for logical in "$@"; do
        local actual=""
        if [[ "$DISTRO_FAMILY" == "debian" ]]; then
            actual="${DEB_PKG_MAP[$logical]:-}"
        else
            actual="${RHEL_PKG_MAP[$logical]:-}"
        fi
        # actual 可能为多词（如 "gcc gcc-c++ make cmake"），用未加引号扩展拆分
        if [[ -n "$actual" ]]; then
            pkgs+=($actual)
        fi
    done

    if [[ ${#pkgs[@]} -eq 0 ]]; then
        log_warn "无可安装包（可能因发行版差异全部被映射为空）"
        return 0
    fi

    log_info "通过 $PKG_MANAGER 安装: ${pkgs[*]}"
    if [[ "$PKG_MANAGER" == "apt-get" ]]; then
        apt-get install -y $APT_OPTS "${pkgs[@]}"
    else
        $PKG_MANAGER install -y $DNF_OPTS "${pkgs[@]}"
    fi
}

# ------------------------------------------------------------------------------
# 阿里云镜像源临时启用（仅 ALIYUN_MIRROR=1 时生效，单次运行，不写 /etc）
# - uv：export UV_INDEX_URL/UV_HTTP_TIMEOUT 到当前会话
# - dnf：复制系统 repo 到临时目录，host 替换为阿里云，--setopt=reposdir 强制只用阿里云
# - apt：复制 sources.list(+sources.list.d) 到临时目录，host 替换，-o Dir::Etc::sourcelist 强制
# 复用系统现有 repo 结构仅替换 host，自动继承版本/SP/codename 配置，最稳健
setup_aliyun_mirror() {
    if [[ "$ALIYUN_MIRROR" -ne 1 ]]; then
        log_info "使用系统当前软件源（未启用 --aliyun）"
        return 0
    fi

    log_info "启用阿里云镜像源（本次运行临时生效，不修改系统配置）..."

    # uv/PyPI：会话级 export
    export UV_HTTP_TIMEOUT=600
    export UV_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/
    log_info "✓ uv 镜像源已临时设置为阿里云（仅当前会话）"

    # dnf/apt：临时 repo 目录 + host 替换
    TEMP_REPO_DIR=$(mktemp -d /tmp/aliyun-repo.XXXXXX)

    if [[ "$DISTRO_FAMILY" == "debian" ]]; then
        # apt：复制 sources.list 与 sources.list.d，host 替换为阿里云
        if [[ -f /etc/apt/sources.list ]]; then
            sed 's|ports.ubuntu.com|mirrors.aliyun.com|g; s|archive.ubuntu.com|mirrors.aliyun.com|g' \
                /etc/apt/sources.list > "$TEMP_REPO_DIR/sources.list"
        else
            touch "$TEMP_REPO_DIR/sources.list"
        fi
        mkdir -p "$TEMP_REPO_DIR/sources.list.d"
        cp /etc/apt/sources.list.d/* "$TEMP_REPO_DIR/sources.list.d/" 2>/dev/null || true
        find "$TEMP_REPO_DIR/sources.list.d" -type f \
            \( -name '*.list' -o -name '*.sources' \) \
            -exec sed -i 's|ports.ubuntu.com|mirrors.aliyun.com|g; s|archive.ubuntu.com|mirrors.aliyun.com|g' {} +
        APT_OPTS="-o Dir::Etc::sourcelist=$TEMP_REPO_DIR/sources.list -o Dir::Etc::sourceparts=$TEMP_REPO_DIR/sources.list.d"
        log_info "✓ apt 临时源已就绪（$TEMP_REPO_DIR）"
    else
        # dnf：复制 repo 文件，host 替换为阿里云
        cp /etc/yum.repos.d/*.repo "$TEMP_REPO_DIR/" 2>/dev/null || true
        sed -i 's|repo.openeuler.org|mirrors.aliyun.com/openeuler|g' "$TEMP_REPO_DIR"/*.repo 2>/dev/null || true
        DNF_OPTS="--setopt=reposdir=$TEMP_REPO_DIR --nogpgcheck"
        log_info "✓ dnf 临时源已就绪（$TEMP_REPO_DIR）"
    fi
}

# ------------------------------------------------------------------------------
# 错误处理
# ------------------------------------------------------------------------------
cleanup() {
    local exit_code=$?
    # 清理 --aliyun 临时 repo 目录（含正常退出与异常退出）
    if [[ -n "$TEMP_REPO_DIR" && -d "$TEMP_REPO_DIR" ]]; then
        rm -rf "$TEMP_REPO_DIR"
    fi
    if [ $exit_code -ne 0 ]; then
        log_error "部署失败（退出码 $exit_code），请检查上方日志"
        log_error "排错建议："
        log_error "  1. 确认以 root 权限运行（sudo bash scripts/deploy_node_server.sh）"
        log_error "  2. 确认网络可达镜像源（默认系统源；--aliyun 启用阿里云临时源）"
        log_error "  3. 确认已安装 GUI 桌面环境（project_memory 硬约束）"
        log_error "  4. 相机与 S21C 串口需人工插入，脚本不自动检测外设"
        log_error "  5. 若包名不存在，用 dnf provides <libxxx.so> 或 apt-file search <libxxx.so> 查找"
    fi
    exit $exit_code
}
trap cleanup EXIT

# ------------------------------------------------------------------------------
# 命令行参数解析
# ------------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --aliyun|--aliyun-mirror)
            ALIYUN_MIRROR=1
            shift
            ;;
        --help|-h)
            echo "用法: sudo bash scripts/deploy_node_server.sh [--aliyun]"
            echo "  默认          使用系统当前软件源（dnf/apt + uv），不修改 /etc 配置"
            echo "  --aliyun      本次运行临时使用阿里云镜像源（dnf/apt + uv），单次生效，不写 /etc"
            exit 0
            ;;
        *)
            log_warn "未知参数: $1，忽略"
            shift
            ;;
    esac
done

# ==============================================================================
# 步骤 1：环境前置检查
# ==============================================================================
log_step "步骤 1/10：环境前置检查"

# 1.1 root 权限检查
if [[ $EUID -ne 0 ]]; then
    log_error "必须以 root 权限运行（请使用 sudo）"
    exit 1
fi
log_info "✓ root 权限检查通过"

# 1.2 架构检查（必须为 ARM64）
ARCH=$(uname -m)
if [[ "$ARCH" != "aarch64" ]]; then
    log_error "架构不匹配：当前 $ARCH，期望 aarch64（ARM64）"
    log_error "本脚本仅适用于 OrangePi KunPeng（ARM64）"
    exit 1
fi
log_info "✓ 架构检查通过：$ARCH"

# 1.3 发行版识别（自适应包管理器与包名映射）
# detect_distro() 设置 DISTRO_FAMILY（debian/rhel）与 PKG_MANAGER（apt-get/dnf/yum）
# 后续步骤 2/3 据此选择 apt 或 dnf/yum，并经 install_pkgs 自动映射 DEB/RPM 包名
# 支持：Ubuntu/Debian/Raspbian、openEuler/CentOS/RHEL/Fedora/Rocky/AlmaLinux/Kylin/EulerOS/Anolis
detect_distro

# 1.4 GUI 桌面环境检查（project_memory 硬约束）
if [[ -z "${DISPLAY:-}" ]] && [[ -z "${XDG_SESSION_TYPE:-}" ]]; then
    log_warn "未检测到 DISPLAY 或 XDG_SESSION_TYPE 环境变量"
    log_warn "GUI 桌面环境可能未安装（project_memory 硬约束）"
    log_warn "请确认 openEuler 已安装桌面环境，或通过 SSH -X 转发"
fi
log_info "✓ GUI 环境检查完成（警告不阻塞）"

# 1.5 磁盘空间检查（至少 3GB 可用）
AVAILABLE_GB=$(df -BG /opt | awk 'NR==2 {print $4}' | sed 's/G//')
if [[ "$AVAILABLE_GB" -lt 3 ]]; then
    log_error "磁盘空间不足：/opt 可用 ${AVAILABLE_GB}GB，至少需要 3GB"
    exit 1
fi
log_info "✓ 磁盘空间检查通过：/opt 可用 ${AVAILABLE_GB}GB"

# ==============================================================================
# 步骤 2：配置软件源（默认系统源；--aliyun 启用阿里云临时源）
# ==============================================================================
log_step "步骤 2/10：配置软件源（默认系统源；--aliyun 启用阿里云临时源）"

# 默认：使用系统当前软件源，不修改任何 /etc 配置
# --aliyun：临时复制系统 repo 并替换为阿里云 host，仅本次运行生效（setup_aliyun_mirror 内处理）
setup_aliyun_mirror

log_info "✓ 软件源就绪"

# ==============================================================================
# 步骤 3：安装系统级依赖（自适应发行版，经 install_pkgs 自动映射 DEB/RPM 包名）
# ==============================================================================
log_step "步骤 3/10：安装系统级依赖（自适应发行版）"

log_info "刷新 $PKG_MANAGER 元数据索引..."
if [[ "$PKG_MANAGER" == "apt-get" ]]; then
    apt-get $APT_OPTS update -y
else
    # 仅刷新元数据（等价 apt-get update），不执行全系统升级
    # 注：dnf update 会触发全系统升级，可能命中仓库依赖冲突（如 hdf5/gdal），
    # 且弱性能 ARM 盒子耗时与风险不可控，故用 makecache
    $PKG_MANAGER $DNF_OPTS makecache
fi

log_info "安装 tkinter（GUI 必须，AGENTS.md §8.2）..."
install_pkgs "tkinter"

log_info "安装 X11 / Wayland 兼容层..."
install_pkgs "x11-server" "xwayland" "x11-utils" "xauth"

log_info "安装 OpenCV 运行时依赖（libGL / libgthread 等）..."
# 注：libgthread 在 RHEL 系由 glib2 主包提供（映射为空=跳过）；gtk2/gtk3 在 RHEL 系为 gtk2/gtk3
install_pkgs "opengl" "opengl-glu" "opengl-egl" \
             "glib2" "glib2-dev" \
             "libsm" "libxext" "libxrender" "libxtst" "libxi" \
             "libgthread" "gtk2" "gtk3"

log_info "安装构建工具链（gcc / g++ / make / cmake）..."
install_pkgs "build-essential" "pkg-config" "python-dev" "kernel-dev"

log_info "安装串口与 USB 工具..."
install_pkgs "usbutils" "util-linux"
# kernel-modules-extra：RHEL 系专用，含 ch341 等额外内核模块（可选，缺失不阻塞）
if [[ "$DISTRO_FAMILY" == "rhel" ]]; then
    $PKG_MANAGER install -y $DNF_OPTS kernel-modules-extra 2>/dev/null || log_warn "kernel-modules-extra 不可用，跳过"
fi

log_info "安装基础工具（若精简镜像缺失）..."
install_pkgs "sudo" "shadow-utils"

log_info "✓ 系统级依赖安装完成"

# ==============================================================================
# 步骤 4：配置用户组与权限
# ==============================================================================
log_step "步骤 4/10：配置用户组与权限"

# 获取调用 sudo 的真实用户（非 root）
REAL_USER="${SUDO_USER:-root}"
log_info "为用户 $REAL_USER 配置组权限..."

# 4.1 加入 video 组（摄像头权限）
if ! id -nG "$REAL_USER" | grep -qw "video"; then
    usermod -aG video "$REAL_USER"
    log_info "✓ 已将 $REAL_USER 加入 video 组"
else
    log_info "✓ $REAL_USER 已在 video 组"
fi

# 4.2 加入 dialout 组（串口权限）
if ! id -nG "$REAL_USER" | grep -qw "dialout"; then
    usermod -aG dialout "$REAL_USER"
    log_info "✓ 已将 $REAL_USER 加入 dialout 组"
else
    log_info "✓ $REAL_USER 已在 dialout 组"
fi

log_warn "⚠️ 用户组改动需重新登录后生效"
log_warn "   部署完成后请退出当前会话并重新登录"

# 4.3 加载 ch341 内核驱动（CH9102 串口芯片）
if ! lsmod | grep -q "ch341"; then
    log_info "加载 ch341 内核驱动..."
    modprobe ch341 2>/dev/null || log_warn "ch341 驱动加载失败，首次插入 S21C 时再确认"
fi

# 4.4 配置 GDK_BACKEND（Wayland 兼容 tkinter）
if ! grep -q "GDK_BACKEND=x11" /etc/environment; then
    echo 'GDK_BACKEND=x11' >> /etc/environment
    log_info "✓ 已配置 GDK_BACKEND=x11 到 /etc/environment"
fi

# ==============================================================================
# 步骤 5：安装 uv 工具链
# ==============================================================================
log_step "步骤 5/10：安装 uv 工具链"

# 检查是否已安装
if command -v uv &> /dev/null; then
    UV_VERSION=$(uv --version | awk '{print $2}')
    log_info "✓ uv 已安装：$UV_VERSION"
else
    log_info "通过官方脚本安装 uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
    if ! command -v uv &> /dev/null; then
        log_error "uv 安装失败"
        exit 1
    fi
    log_info "✓ uv 安装完成：$(uv --version)"
fi

# uv 镜像源：默认尊重系统/uv 现有配置；--aliyun 时已在 setup_aliyun_mirror() 中临时 export
if [[ "$ALIYUN_MIRROR" -eq 1 ]]; then
    log_info "✓ uv 已使用阿里云源（临时，见步骤 2）"
else
    log_info "uv 镜像源沿用系统当前配置（如需阿里云加速，加 --aliyun 参数重跑）"
fi

# ==============================================================================
# 步骤 6：部署项目代码到 /opt/node_server
# ==============================================================================
log_step "步骤 6/10：部署项目代码到 $INSTALL_DIR"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

mkdir -p "$INSTALL_DIR"
log_info "复制项目文件..."
cp -r "$PROJECT_DIR"/* "$INSTALL_DIR/"
# 清理可能的 __pycache__ 与 logs
find "$INSTALL_DIR" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
rm -rf "$INSTALL_DIR/logs" 2>/dev/null || true

log_info "✓ 代码已部署到 $INSTALL_DIR"

cd "$INSTALL_DIR"

# ==============================================================================
# 步骤 7：创建 uv 虚拟环境并安装依赖
# ==============================================================================
log_step "步骤 7/10：创建 uv 虚拟环境并安装依赖"

log_info "创建 Python $PYTHON_VERSION 虚拟环境..."
uv venv --python "$PYTHON_VERSION"
log_info "✓ 虚拟环境已创建：$INSTALL_DIR/.venv"

log_info "安装 Python 依赖（uv sync）..."
log_warn "⚠️ 如卡死在 opencv 编译，请确认 §1.3 系统依赖已装"
log_warn "   可临时改用 opencv-python-headless（但 GUI 显示会受影响）"

uv sync
log_info "✓ Python 依赖安装完成"

# ==============================================================================
# 步骤 8：编译 proto 文件
# ==============================================================================
log_step "步骤 8/10：编译 proto 文件"

cd "$INSTALL_DIR"

log_info "使用 uv run 调用 grpcio-tools 编译 proto..."
uv run python -m grpc_tools.protoc \
    --proto_path=proto \
    --python_out=proto \
    --grpc_python_out=proto \
    proto/rebar_inference.proto

if [[ ! -f proto/rebar_inference_pb2.py || ! -f proto/rebar_inference_pb2_grpc.py ]]; then
    log_error "proto 编译失败：未生成 _pb2.py 文件"
    exit 1
fi
log_info "✓ proto 编译完成"

# ==============================================================================
# 步骤 9：提示人工插入外设
# ==============================================================================
log_step "步骤 9/10：外设检查提示"

log_warn "⚠️ 相机与 S21C 串口需人工插入，脚本不自动检测外设"
log_warn ""
log_warn "部署完成后请："
log_warn "  1. 插入 Orbbec Gemini 336L 相机（USB）"
log_warn "  2. 插入 S21C 主控板（Type-C 串口，CH9102 芯片）"
log_warn "  3. 确认设备节点："
log_warn "     ls /dev/video*    # 相机"
log_warn "     ls /dev/ttyUSB*   # 串口"
log_warn "  4. 确认 ch341 驱动加载："
log_warn "     lsmod | grep ch341"

# ==============================================================================
# 步骤 10：注册 systemd 服务并启动
# ==============================================================================
log_step "步骤 10/10：注册 systemd 服务并启动"

cd "$INSTALL_DIR"

if [[ ! -f scripts/service-manager.sh ]]; then
    log_error "未找到 scripts/service-manager.sh"
    log_error "请确认项目完整性"
    exit 1
fi

chmod +x scripts/service-manager.sh

log_info "注册 systemd 服务..."
bash scripts/service-manager.sh install

log_info "启动服务..."
systemctl start "$SERVICE_NAME" || log_warn "服务启动失败，请检查日志：journalctl -u $SERVICE_NAME -f"

# 等待服务启动
sleep 2
if systemctl is-active --quiet "$SERVICE_NAME"; then
    log_info "✓ 服务已启动：$SERVICE_NAME"
else
    log_warn "服务未进入 active 状态"
    log_warn "查看日志：sudo journalctl -u $SERVICE_NAME -f"
    log_warn "或查看应用日志：sudo tail -f $INSTALL_DIR/logs/node_server.log"
fi

# ==============================================================================
# 部署完成
# ==============================================================================
echo ""
echo "==============================================================="
log_info "🎉 服务节点 部署完成！"
echo "==============================================================="
echo ""
echo "下一步操作："
echo ""
echo "1. ⚠️ 退出当前会话并重新登录，使用户组（video/dialout）生效"
echo ""
echo "2. 插入外设："
echo "   - Orbbec Gemini 336L 相机（USB）"
echo "   - S21C 主控板（Type-C 串口）"
echo ""
echo "3. 确认外设识别："
echo "   ls /dev/video*      # 相机"
echo "   ls /dev/ttyUSB*      # 串口"
echo "   lsmod | grep ch341   # ch341 驱动"
echo ""
echo "4. 查看服务状态："
echo "   sudo systemctl status $SERVICE_NAME"
echo ""
echo "5. 查看实时日志："
echo "   sudo journalctl -u $SERVICE_NAME -f"
echo "   sudo tail -f $INSTALL_DIR/logs/node_server.log"
echo ""
echo "6. 环境自检（独立验证）："
echo "   cd $INSTALL_DIR && sudo uv run main.py self-check"
echo ""
echo "7. 前台调试模式（可选）："
echo "   sudo systemctl stop $SERVICE_NAME"
echo "   cd $INSTALL_DIR && sudo uv run main.py"
echo ""
echo "8. 五项自检脚本（相机/串口/GUI/通信/推理测量）："
echo "   cd $INSTALL_DIR && sudo uv run python test_node_server.py"
echo ""
