#!/usr/bin/env bash
# ==============================================================================
# deploy_node_detect.sh — 检测节点（Jetson Nano）一键部署脚本
#
# 用途：在裸机 Jetson Nano（Ubuntu 22.04 LTS / JetPack）上完成从空系统到
#       可启动 systemd 服务的全流程部署。脚本自适应识别 Linux 发行版：
#         - Debian 系（Ubuntu/Debian/Raspbian）→ 使用 apt + DEB 包名
#         - RHEL 系（openEuler/CentOS/RHEL/Fedora/Rocky/AlmaLinux）→ 使用 dnf + RPM 包名
#       注：Jetson Nano 实际仅 Ubuntu 系，自适应逻辑为容错与未来扩展预留
#
# 执行：sudo bash scripts/deploy_node_detect.sh [--aliyun]
#   默认          使用系统当前软件源（dnf/apt + uv），不修改任何 /etc 配置
#   --aliyun      本次运行临时使用阿里云镜像源（dnf/apt + uv），单次生效，不写 /etc
#
# 设计原则：
#   - set -e 任一步骤失败立即停止，不产生半安装状态
#   - 所有外部命令均显式校验返回值
#   - 关键节点输出日志，便于排错
#   - 模型权重不自动下载，由人工放置（AGENTS.md §8.4 约定）
#
# 对应文档：node_detect/QUICKSTART.md
# ==============================================================================

set -euo pipefail

# 兜底：nounset(-u) 模式下引用未定义变量会报 unbound variable 并退出。
# Jetson 默认 shell 环境可能不含 LD_LIBRARY_PATH，第 311 行拼接 CUDA 库路径时
# 会被 set -u 拦截。此处用 ${var:-} 展开为空字符串避免报错。
LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"

# ------------------------------------------------------------------------------
# 全局变量
# ------------------------------------------------------------------------------
INSTALL_DIR="/opt/node_detect"
SERVICE_NAME="node-detect-inference"
PYTHON_VERSION="3.10"
UV_VERSION_REQUIRED="0.4.0"
LOG_PREFIX="[deploy_node_detect]"

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
# 注：libgthread-2.0.so.0 在 Debian/RHEL 均由 glib2 主包提供（无独立包，映射为空字符串=跳过）
# 注：nvidia-jetpack 仅 Ubuntu/Jetson 有，RHEL 系映射为空（Jetson 实际仅 Ubuntu）
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
    [libgthread]=""                       # glib2 已含 libgthread-2.0.so.0，无需独立安装
    [gtk2]="libgtk2.0-0"
    [gtk3]="libgtk-3-0"
    [gtk2-dev]="libgtk2.0-dev"
    [gtk3-dev]="libgtk-3-dev"
    [build-essential]="build-essential"
    [pkg-config]="pkg-config"
    [python-dev]="python3-dev"
    [kernel-dev]=""                       # 原型阶段无需编译内核模块；Jetson 定制内核无独立 headers 包
    [nvidia-jetpack]=""                   # nvidia-jetpack 通常由 JetPack 镜像预装，apt 无独立包
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
    [nvidia-jetpack]=""                   # nvidia-jetpack 仅 Ubuntu/Jetson 有，RHEL 系跳过
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
        log_error "  1. 确认以 root 权限运行（sudo bash scripts/deploy_node_detect.sh）"
        log_error "  2. 确认网络可达镜像源（默认系统源；--aliyun 启用阿里云临时源）"
        log_error "  3. 确认 Jetson Nano 内存充足（建议关闭 GUI 桌面后部署）"
        log_error "  4. PyTorch wheel 不在自动安装范围，需人工按 QUICKSTART §2.2 安装"
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
            echo "用法: sudo bash scripts/deploy_node_detect.sh [--aliyun]"
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
log_step "步骤 1/9：环境前置检查"

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
    log_error "本脚本仅适用于 Jetson Nano（ARM64）"
    exit 1
fi
log_info "✓ 架构检查通过：$ARCH"

# 1.3 Jetson 特性检查（硬性要求：Jetson Nano 专用镜像）
if [[ ! -f /etc/nv_tegra_release ]]; then
    log_error "未检测到 Jetson 特性文件 /etc/nv_tegra_release"
    log_error "请确认当前为 Jetson 官方镜像（非普通 Ubuntu）"
    exit 1
fi
log_info "✓ Jetson 特性检查通过"

# 1.4 发行版识别（自适应包管理器与包名映射）
# detect_distro() 设置 DISTRO_FAMILY（debian/rhel）与 PKG_MANAGER（apt-get/dnf/yum）
# 后续步骤 2/3 据此选择 apt 或 dnf/yum，并经 install_pkgs 自动映射 DEB/RPM 包名
# 注：Jetson Nano 实际仅 Ubuntu 系，detect_distro 必返回 debian/apt-get，此处为容错与扩展预留
detect_distro

# 1.5 CUDA 环境检查
if ! command -v nvcc &> /dev/null; then
    log_warn "nvcc 未在 PATH 中，尝试通过 CUDA_HOME 推断"
    if [[ -d /usr/local/cuda ]]; then
        export CUDA_HOME=/usr/local/cuda
        export PATH="$CUDA_HOME/bin:$PATH"
        export LD_LIBRARY_PATH="$CUDA_HOME/lib64:$LD_LIBRARY_PATH"
        log_info "✓ CUDA 路径已配置：$CUDA_HOME"
    else
        log_error "未找到 CUDA 安装（/usr/local/cuda 不存在）"
        log_error "请确认 JetPack 已正确安装，参考 NVIDIA 官方文档"
        exit 1
    fi
else
    export CUDA_HOME=${CUDA_HOME:-/usr/local/cuda}
    log_info "✓ CUDA 环境检查通过：$(nvcc --version | grep release)"
fi

# 1.6 磁盘空间检查（至少 5GB 可用）
AVAILABLE_GB=$(df -BG /opt | awk 'NR==2 {print $4}' | sed 's/G//')
if [[ "$AVAILABLE_GB" -lt 5 ]]; then
    log_error "磁盘空间不足：/opt 可用 ${AVAILABLE_GB}GB，至少需要 5GB"
    log_error "PyTorch wheel + 模型权重占用空间大，请扩容后重试"
    exit 1
fi
log_info "✓ 磁盘空间检查通过：/opt 可用 ${AVAILABLE_GB}GB"

# ==============================================================================
# 步骤 2：配置软件源（默认系统源；--aliyun 启用阿里云临时源）
# ==============================================================================
log_step "步骤 2/9：配置软件源（默认系统源；--aliyun 启用阿里云临时源）"

# 默认：使用系统当前软件源，不修改任何 /etc 配置
# --aliyun：临时复制系统 repo 并替换为阿里云 host，仅本次运行生效（setup_aliyun_mirror 内处理）
setup_aliyun_mirror

log_info "✓ 软件源就绪"

# ==============================================================================
# 步骤 3：安装系统级依赖（自适应发行版，经 install_pkgs 自动映射 DEB/RPM 包名）
# ==============================================================================
log_step "步骤 3/9：安装系统级依赖（自适应发行版）"

log_info "刷新 $PKG_MANAGER 元数据索引..."
if [[ "$PKG_MANAGER" == "apt-get" ]]; then
    apt-get $APT_OPTS update -y
else
    # 仅刷新元数据（等价 apt-get update），不执行全系统升级
    # 注：dnf update 会触发全系统升级，可能命中仓库依赖冲突（如 hdf5/gdal），
    # 且弱性能 ARM 盒子耗时与风险不可控，故用 makecache
    $PKG_MANAGER $DNF_OPTS makecache
fi

log_info "安装 OpenCV 运行时依赖（libGL / libgthread 等）..."
# 注：libgthread 在 RHEL 系由 glib2 主包提供（映射为空=跳过）；gtk2-dev 在 RHEL 系为 gtk2-devel
install_pkgs "opengl" "glib2" \
             "libsm" "libxext" "libxrender" \
             "libgthread" "gtk2-dev"

log_info "安装构建工具链（gcc / g++ / make / cmake）..."
install_pkgs "build-essential" "pkg-config" "python-dev" "kernel-dev"

log_info "安装 Jetson 专用工具与 CUDA 工具链（仅 Debian/Jetson 系）..."
# nvidia-jetpack 仅 Ubuntu/Jetson 有；JetPack 通常已预装，此处为容错
# 注：--aliyun 在 Jetson 上会排除 NVIDIA 私有 repo，nvidia-jetpack 可能不可用，容错跳过
if [[ "$DISTRO_FAMILY" == "debian" ]]; then
    apt-get install -y $APT_OPTS nvidia-jetpack 2>/dev/null || log_warn "nvidia-jetpack 包不可用，跳过（可能已预装或 --aliyun 排除 NVIDIA 私有源）"
else
    log_warn "RHEL 系无 nvidia-jetpack，CUDA 工具链需人工确认（Jetson 实际仅 Ubuntu）"
fi

log_info "✓ 系统级依赖安装完成"

# ==============================================================================
# 步骤 4：安装 uv 工具链
# ==============================================================================
log_step "步骤 4/9：安装 uv 工具链"

# QUICKSTART.md §二：uv 未安装时使用官方脚本安装；安装后需 PATH 生效且通过 uv --version 验证
# 注：sudo bash 会触发 env_reset 重置 PATH，官方脚本默认装到 ~/.local/bin，所以
#     先主动探测常见位置（而非仅依赖 command -v，避免因 PATH 重置误判为未安装）
_uv_path=""
if command -v uv &>/dev/null; then
    _uv_path="$(command -v uv)"
elif [[ -x "${HOME}/.local/bin/uv" ]]; then
    _uv_path="${HOME}/.local/bin/uv"
elif [[ -x "/root/.local/bin/uv" ]]; then
    _uv_path="/root/.local/bin/uv"
fi

# 已安装 → 验证可执行后跳过
if [[ -n "${_uv_path}" ]] && "${_uv_path}" --version &>/dev/null; then
    UV_VERSION="$("${_uv_path}" --version | awk '{print $2}')"
    log_info "✓ uv 已安装：${UV_VERSION}（${_uv_path}）"
else
    # 未安装 → 使用 QUICKSTART 官方脚本安装（已安装的 uv 会被脚本自动识别，幂等）
    log_info "通过官方脚本安装 uv（curl -LsSf https://astral.sh/uv/install.sh | sh）..."
    curl -LsSf https://astral.sh/uv/install.sh | sh || {
        log_error "uv 安装脚本非正常退出"
        exit 1
    }
    # 官方安装脚本将 uv 写入 $HOME/.local/bin；sudo 下 $HOME 可能漂移，需多路径重定位
    _uv_path=""
    for _try_uv in "${HOME}/.local/bin/uv" "/root/.local/bin/uv"; do
        if [[ -x "${_try_uv}" ]]; then
            _uv_path="${_try_uv}"
            break
        fi
    done
    # QUICKSTART.md §二明确要求：ARM64 平台确认 uv --version 能执行
    if [[ -z "${_uv_path}" ]] || ! "${_uv_path}" --version &>/dev/null; then
        log_error "uv 安装失败：安装完成后仍无法找到可执行的 uv"
        log_error "排错建议：① 重开终端或 hash -r 刷新 shell 缓存；② 手动执行 source ~/.bashrc；"
        log_error "          ③ 手动验证 ls -l ~/.local/bin/uv 与 ~/.local/bin/uv --version"
        exit 1
    fi
    UV_VERSION="$("${_uv_path}" --version | awk '{print $2}')"
    log_info "✓ uv 安装完成：${UV_VERSION}（${_uv_path}）"
fi

# 导出 PATH 至当前 shell，使后续步骤（uv venv / uv sync）可直接调用 uv（QUICKSTART 依赖行为）
_uv_dir="${_uv_path%/*}"
if [[ ":${PATH}:" != *":${_uv_dir}:"* ]]; then
    export PATH="${_uv_dir}:${PATH}"
fi

# uv 镜像源：默认尊重系统/uv 现有配置；--aliyun 时已在 setup_aliyun_mirror() 中临时 export
if [[ "$ALIYUN_MIRROR" -eq 1 ]]; then
    log_info "✓ uv 已使用阿里云源（临时，见步骤 2）"
else
    log_info "uv 镜像源沿用系统当前配置（如需阿里云加速，加 --aliyun 参数重跑）"
fi

# ==============================================================================
# 步骤 5：部署项目代码到 /opt/node_detect
# ==============================================================================
log_step "步骤 5/9：部署项目代码到 $INSTALL_DIR"

# 获取脚本所在目录（项目根目录的 scripts/ 下）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

mkdir -p "$INSTALL_DIR"
log_info "复制项目文件（覆盖已有配置，代码目录为单一配置源）..."
cp -r "$PROJECT_DIR"/* "$INSTALL_DIR/"
# 清理可能的 __pycache__ 与 logs
find "$INSTALL_DIR" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
rm -rf "$INSTALL_DIR/logs" 2>/dev/null || true

log_info "✓ 代码与配置已部署到 $INSTALL_DIR（同名配置文件已强制覆盖）"

cd "$INSTALL_DIR"

# ==============================================================================
# 步骤 6：创建 uv 虚拟环境（强制覆盖重建）并安装依赖
# ==============================================================================
log_step "步骤 6/9：创建 uv 虚拟环境（强制覆盖重建）并安装依赖"

# 强制覆盖重建：无条件删除已有 .venv 目录并重新创建（对齐 node_server §步骤 7"无情模式"）
log_info "强制覆盖重建 uv 虚拟环境: $INSTALL_DIR/.venv"
if [[ -d "$INSTALL_DIR/.venv" ]]; then
    log_warn "⚠️ 检测到已存在的 .venv 目录，正在强制删除并重建..."
    rm -rf "$INSTALL_DIR/.venv"
    log_info "  已删除旧环境：$INSTALL_DIR/.venv"
fi
# 双重保险：使用 --clear 参数让 uv 强制清除残留（无交互）
uv venv --python "$PYTHON_VERSION" --clear
log_info "✓ 虚拟环境已重建：$INSTALL_DIR/.venv（Python $PYTHON_VERSION）"

log_info "安装 Python 依赖（uv sync）..."
log_warn "⚠️ torch/torchvision 不在 requirements 中（通过后续 Jetson whl 安装）"

# 校验 pyproject.toml 存在（uv 项目模式必需）
if [[ ! -f "$INSTALL_DIR/pyproject.toml" ]]; then
    log_error "pyproject.toml 不存在：$INSTALL_DIR/pyproject.toml"
    log_error "请确认部署代码完整（含 pyproject.toml 与 requirements.txt）"
    exit 1
fi

# 直接 uv sync，失败即退出（torch 已不在 requirements 中，不会卡死）
uv sync || {
    log_error "uv sync 失败（退出码 $？）"
    log_error "排错：① 系统依赖是否安装（步骤 3）；② pyproject.toml 语法是否正确；③ 网络是否可达"
    exit 1
}
log_info "✓ Python 依赖安装完成（uv.lock 已生成/更新）"

# ---- Jetson 专用 PyTorch 安装（必须在 uv sync 之后）----
# requirements.txt 中 torch/torchvision 已注释，此处通过 NVIDIA 本地 whl 安装
# whl 路径：node_detect/source/torch-*-cp310-cp310-linux_aarch64.whl
TORCH_WHL="$(ls "$INSTALL_DIR"/source/torch-*-cp310-cp310-linux_aarch64.whl 2>/dev/null | head -1)"
if [[ -n "$TORCH_WHL" && -f "$TORCH_WHL" ]]; then
    log_info "安装 Jetson 专用 PyTorch wheel: $TORCH_WHL"
    log_warn "⚠️ 首次安装 PyTorch whl 可能需要 5-15 分钟（Jetson Nano CPU 编译部分依赖）..."
    # NVIDIA 定制 torch whl 文件名含简写版本号，内部 metadata 含 JetPack 构建号，需跳过 uv 版本校验
    UV_SKIP_WHEEL_FILENAME_CHECK=1 uv pip install "$TORCH_WHL" || {
        log_error "PyTorch whl 安装失败"
        log_error "排错：① 检查 whl 文件名是否匹配 JetPack 版本；② 检查 https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html"
        exit 1
    }
    # 验证 CUDA 可用
    if uv run python -c "import torch; assert torch.cuda.is_available(), 'CUDA 不可用'; print('torch OK, CUDA device:', torch.cuda.get_device_name(0))" 2>/dev/null; then
        log_info "✓ PyTorch + CUDA 安装验证通过"
    else
        log_error "PyTorch 已安装但 CUDA 不可用"
        log_error "排错：确认 JetPack 版本与 whl 匹配；执行：uv run python -c 'import torch; print(torch.cuda.is_available())'"
        exit 1
    fi

    # ② 防御性审计：确认 .venv 中的 torch 来自 Jetson whl、而非 PyPI cu13x 版本
    # （PyTorch 2.13+cu130 要求驱动 > 12060，当前 Jetson 驱动 12060 必触发 "driver too old"）
    # 注意：不能在此处使用 local（仅在函数内可用），直接赋值即可
    torch_actual="$(uv run python -c 'import torch; print(torch.__version__)' 2>/dev/null || echo 'N/A')"
    torch_source="$(uv run python -c 'import torch; print(torch.__file__)' 2>/dev/null || echo 'N/A')"
    log_info "torch 来源审计: version=$torch_actual  path=$torch_source"
    if printf '%s' "$torch_actual" | grep -qE 'cu13[0-9]'; then
        log_error ".venv 中的 torch 含 cu13x（PyPI 版本，要求 CUDA ≥ 13.0 / 驱动 > 12060）"
        log_error "当前 Jetson 驱动为 12060，与 cu13x 不兼容 → CUDA init 报 'driver too old'"
        log_error "根因：pyproject.toml 中 torch 仍被列为正式依赖，`uv sync` 在 whl 之前拉取了 PyPI 版"
        log_error "修复：确认 pyproject.toml 中 torch / torchvision 已被注释，然后重跑部署"
        exit 1
    fi

    # ③ 同步 uv.lock，锁定 whl 安装的 torch 版本，避免下次 uv sync 重置回 PyPI 2.13
    if [[ -f "$INSTALL_DIR/uv.lock" ]]; then
        log_info "同步 uv.lock 中的 torch 版本记录（锁定为 Jetson whl 版本）..."
        uv lock --upgrade-package torch 2>/dev/null || true
        log_info "✓ uv.lock 已更新（torch 版本锁定为 Jetson whl 版本）"
    fi
else
    log_error "未找到 Jetson PyTorch whl: $INSTALL_DIR/source/torch-*-cp310-cp310-linux_aarch64.whl"
    log_error "请在 Jetson 开发机按 QUICKSTART.md §2.2 下载 whl 并放置到 node_detect/source/ 目录后重跑部署"
    exit 1
fi

log_info "✓ Python 依赖（含 Jetson PyTorch）处理完成"

# ==============================================================================
# 步骤 7：编译 proto 文件
# ==============================================================================
log_step "步骤 7/9：编译 proto 文件"

cd "$INSTALL_DIR"

log_info "使用 uv run 调用 grpcio-tools 编译 proto..."
# 关键：--proto_path=. 让 protoc 把项目根作为 proto path，proto/rebar_inference.proto
# 处于 proto 子包下，protoc 识别 proto 为包，生成的 _pb2_grpc.py 用包内导入
# （from . import 或 from proto import），而非裸 import rebar_inference_pb2
uv run python -m grpc_tools.protoc \
    --proto_path=. \
    --python_out=. \
    --grpc_python_out=. \
    proto/rebar_inference.proto

if [[ ! -f proto/rebar_inference_pb2.py || ! -f proto/rebar_inference_pb2_grpc.py ]]; then
    log_error "proto 编译失败：未生成 _pb2.py 文件"
    exit 1
fi
log_info "✓ proto 编译完成"

# ==============================================================================
# 步骤 8：提示人工放置模型权重
# ==============================================================================
log_step "步骤 8/9：模型权重检查"

WEIGHTS_DIR="$INSTALL_DIR/weights"
mkdir -p "$WEIGHTS_DIR"

WEIGHTS_FILE="$WEIGHTS_DIR/Unet_resnet50.pth"
if [[ ! -f "$WEIGHTS_FILE" ]]; then
    log_warn "⚠️ 模型权重文件不存在：$WEIGHTS_FILE"
    log_warn "   权重文件不入版本控制（AGENTS.md §8.4 约定）"
    log_warn "   请人工放置到：$WEIGHTS_FILE"
    log_warn "   部署完成后可继续后续步骤，但服务启动前必须放置权重"
else
    log_info "✓ 模型权重已存在：$WEIGHTS_FILE"
fi

# ==============================================================================
# 步骤 9：注册 systemd 服务并启动
# ==============================================================================
log_step "步骤 9/9：注册 systemd 服务并启动"

cd "$INSTALL_DIR"

# 检查 service-manager.sh 是否存在
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
    log_warn "或查看应用日志：sudo tail -f $INSTALL_DIR/logs/node_detect.log"
fi

# ==============================================================================
# 部署完成
# ==============================================================================
echo ""
echo "==============================================================="
log_info "🎉 检测节点 部署完成！"
echo "==============================================================="
echo ""
echo "下一步操作："
echo ""
echo "1. 确认模型权重已放置："
echo "   ls -l $WEIGHTS_FILE"
echo ""
echo "2. 如未安装 Jetson 专用 PyTorch wheel，请按 QUICKSTART §2.2 安装"
echo ""
echo "3. 查看服务状态："
echo "   sudo systemctl status $SERVICE_NAME"
echo ""
echo "4. 查看实时日志："
echo "   sudo journalctl -u $SERVICE_NAME -f"
echo "   sudo tail -f $INSTALL_DIR/logs/node_detect.log"
echo ""
echo "5. 环境自检（独立验证）："
echo "   cd $INSTALL_DIR && sudo uv run main.py self-check"
echo ""
echo "6. 前台调试模式（可选）："
echo "   sudo systemctl stop $SERVICE_NAME"
echo "   cd $INSTALL_DIR && sudo uv run main.py run"
echo ""
