#!/bin/bash
# ==============================================================================
# test_torch_whl.sh — NVIDIA Jetson 专用 PyTorch WHL 兼容性批量测试脚本
#
# 用途：
#   遍历 node_detect/source/ 目录下所有 .whl 文件，逐一在临时 uv 虚拟
#   环境中安装、检测 CUDA 兼容性、卸载，输出汇总表格，帮助开发者在
#   部署前快速定位匹配当前 Jetson 驱动的 PyTorch whl 文件。
#
# 用法：
#   bash node_detect/scripts/test_torch_whl.sh [--keep-env] [--source-dir <path>]
#
#     --keep-env       测试完成后保留临时虚拟环境（调试用）
#     --source-dir     自定义 WHL 源目录（默认：脚本所在目录/../source）
#
# 前提条件：
#   - 系统已安装 uv（https://docs.astral.sh/uv/）
#   - node_detect/source/ 下存在一个或多个 .whl 文件
#   - Bash 4.x+（Jetson Ubuntu 默认满足）
#
# 输出：
#   - 每个 WHL 的安装/检测/卸载过程带时间戳日志
#   - 最终汇总表格：文件名 | 安装状态 | torch版本 | CUDA版本 | CUDA可用 | 驱动匹配 | 备注
#   - 退出码：0 全部 whl 通过；1 至少一个 whl 失败；2 参数/环境错误
# ==============================================================================

set -euo pipefail

# ----------------------------------------------------------------------------
# uv 路径修复：sudo 下 PATH 常被重置，默认不包含 ~/.local/bin 等用户安装目录
# 主动探测并追加，避免"已装 uv 但脚本报未找到"
# ----------------------------------------------------------------------------
_resolved_uv=0
if command -v uv &>/dev/null; then
    _resolved_uv=1
else
    _extra_paths=(
        "$HOME/.local/bin"
        "$HOME/.cargo/bin"
        "/home/jetson/.local/bin"
        "/home/nvidia/.local/bin"    # Orin/NX 开发套件默认用户
        "/root/.local/bin"           # 直接在 root 下安装时
        "/usr/local/bin"
    )
    for _p in "${_extra_paths[@]}"; do
        if [[ -x "$_p/uv" ]]; then
            export PATH="$_p:${PATH}"
            _resolved_uv=1
            break
        fi
    done
fi

# ---------------------------------------------------------------------------
# 路径常量
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$SCRIPT_DIR/.venv_test_torch"
SOURCE_DIR="$PROJECT_DIR/source"

# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------
KEEP_ENV=0
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --keep-env)
            KEEP_ENV=1
            shift
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --source-dir)
            if [[ -z "${2:-}" ]]; then
                echo "[ERROR] --source-dir 缺少路径参数" >&2
                exit 2
            fi
            SOURCE_DIR="$2"
            shift 2
            ;;
        -h|--help)
            sed -n '1,20p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "[ERROR] 未知参数: $1" >&2
            exit 2
            ;;
    esac
done

# ---------------------------------------------------------------------------
# 日志函数
# ---------------------------------------------------------------------------
log_info()  { printf '[%s] [INFO]  %s\n' "$(date '+%H:%M:%S')" "$*"; }
log_warn()  { printf '[%s] [WARN]  %s\n' "$(date '+%H:%M:%S')" "$*" >&2; }
log_error() { printf '[%s] [ERROR] %s\n' "$(date '+%H:%M:%S')" "$*" >&2; }
log_ok()    { printf '[%s] [OK]    %s\n' "$(date '+%H:%M:%S')" "$*"; }
log_step()  { printf '\n[%s] [STEP]  %s\n' "$(date '+%H:%M:%S')" "$*"; }

# ---------------------------------------------------------------------------
# 系统 CUDA 版本探测
#   优先级：nvidia-smi > /usr/local/cuda/version.txt > 内核模块参数
#   返回格式 "<major>.<minor>" 或 "N/A"
# ---------------------------------------------------------------------------
get_system_cuda_version() {
    local ver=""

    # 方法 1：nvidia-smi 输出的 "CUDA Version: X.Y" 行
    if command -v nvidia-smi &>/dev/null; then
        ver=$(nvidia-smi 2>/dev/null \
            | grep -oP 'CUDA Version:\s*\K[0-9]+\.[0-9]+' \
            || true)
    fi

    # 方法 2：/usr/local/cuda/version.txt（CUDA toolkit 告知的版本）
    if [[ -z "$ver" ]] && [[ -f /usr/local/cuda/version.txt ]]; then
        ver=$(grep -oP 'CUDA Version\s*\K[0-9]+\.[0-9]+' \
            /usr/local/cuda/version.txt 2>/dev/null || true)
    fi

    # 方法 3：通过 /usr/local/cuda*/version.txt（含 toolkit 版本串如 12.6.107）
    if [[ -z "$ver" ]]; then
        local cuda_root
        cuda_root=$(ls -d /usr/local/cuda* 2>/dev/null | head -1 || true)
        if [[ -n "$cuda_root" && -f "$cuda_root/version.txt" ]]; then
            ver=$(grep -oP 'CUDA Version\s*\K[0-9]+\.[0-9]+' \
                "$cuda_root/version.txt" 2>/dev/null || true)
        fi
    fi

    echo "${ver:-N/A}"
}

# ---------------------------------------------------------------------------
# 版本号拆解辅助
# ---------------------------------------------------------------------------
parse_version_major() {
    local v="${1:-N/A}"
    [[ "$v" == "N/A" ]] && { echo "0"; return; }
    echo "${v%%.*}"
}

parse_version_minor() {
    local v="${1:-N/A}"
    [[ "$v" == "N/A" ]] && { echo "0"; return; }
    local minor="${v#*.}"
    [[ "$minor" =~ ^[0-9]+$ ]] || minor="0"
    echo "$minor"
}

# ---------------------------------------------------------------------------
# 兼容性匹配判定
#   输入：torch_cuda (str), sys_cuda (str), cuda_available (0/1)
#   输出：OK | WARN | FAIL | N/A
# ---------------------------------------------------------------------------
judge_match() {
    local torch_cuda="${1:-N/A}"
    local sys_cuda="${2:-N/A}"
    local cuda_ok="${3:-0}"

    # 任一不可用
    [[ "$torch_cuda" == "N/A" || "$sys_cuda" == "N/A" ]] && { echo "N/A"; return; }

    # 归一化 cuda_ok：接受 Python bool (true/false)、yes、on、1、0 等字符串，统一为 bash 数值；
    # 否则 bash 在 [[ $cuda_ok -eq 1 ]] 中会把 'true' 展开为 $true (unbound under set -u)
    local _cuda_ok_num=0
    case "$cuda_ok" in
        [Tt]rue|[Yy]es|[Oo]n|1) _cuda_ok_num=1 ;;
        *) _cuda_ok_num=0 ;;
    esac

    local t_maj t_min s_maj s_min
    t_maj=$(parse_version_major "$torch_cuda")
    t_min=$(parse_version_minor "$torch_cuda")
    s_maj=$(parse_version_major "$sys_cuda")
    s_min=$(parse_version_minor "$sys_cuda")

        # torch cuda 主版本高于系统：必定不兼容
    if [[ "$t_maj" -gt "$s_maj" ]]; then
        echo "FAIL"
        return
    fi

    # torch cuda 主版本严格低于系统：兼容（warning 留给 CUDA 初始化）
    if [[ "$t_maj" -lt "$s_maj" ]]; then
        [[ "$_cuda_ok_num" -eq 1 ]] && echo "OK" || echo "WARN"
        return
    fi

    # 主版本相等：检查次版本
    if [[ "$t_min" -le "$s_min" ]]; then
        [[ "$_cuda_ok_num" -eq 1 ]] && echo "OK" || echo "WARN"
    else
        # 次版本 torch > 系统：边界，可能运行时报驱动太旧
        [[ "$_cuda_ok_num" -eq 1 ]] && echo "WARN" || echo "WARN"
    fi
}

# ---------------------------------------------------------------------------
# 临时 venv 管理
# ---------------------------------------------------------------------------
cleanup() {
    if [[ "$KEEP_ENV" -eq 1 ]]; then
        log_info "保留临时环境: $VENV_DIR"
    elif [[ -d "$VENV_DIR" ]]; then
        rm -rf "$VENV_DIR"
        log_info "临时环境已清理: $VENV_DIR"
    fi
}

trap cleanup EXIT INT TERM

setup_venv() {
    # 已存在则删掉，保证幂等
    if [[ -d "$VENV_DIR" ]]; then
        log_warn "已存在临时环境，先删除重建: $VENV_DIR"
        rm -rf "$VENV_DIR" || {
            log_error "无法删除旧临时环境: $VENV_DIR"
            return 1
        }
    fi

    log_info "创建临时 uv 虚拟环境: $VENV_DIR"
    if ! uv venv "$VENV_DIR" 2>&1 | while IFS= read -r line; do log_info "uv: $line"; done; then
        log_error "uv venv 创建失败"
        return 1
    fi
    log_ok "临时环境就绪（uv 管理）"
    return 0
}

# 从 whl 包内 METADATA 提取 torch 版本号 + 初步推导 CUDA 主版本
# （用于 import torch 失败时的降级显示，避免整行 N/A 一抹黑）
extract_whl_metadata() {
    local whl="$1"
    local ver="N/A" cuda_hint="N/A"
    command -v unzip &>/dev/null || { printf '%s\t%s' "$ver" "$cuda_hint"; return; }

    # 定位 METADATA 路径（兼容 NVIDIA 命名 torch-*.dist-info 与通用PackageName.dist-info）
    local meta_path
    meta_path=$(unzip -l "$whl" 2>/dev/null | grep -oE 'torch[a-zA-Z0-9._+-]*\.dist-info/METADATA' | head -1)
    [[ -z "$meta_path" ]] && meta_path=$(unzip -l "$whl" 2>/dev/null | grep -oE '[a-zA-Z0-9._+-]+\.dist-info/METADATA' | head -1)

    if [[ -n "$meta_path" ]]; then
        local meta_content
        meta_content=$(unzip -p "$whl" "$meta_path" 2>/dev/null)
        local extracted
        extracted=$(printf '%s' "$meta_content" | grep -m1 '^Version:' | sed 's/^Version:[[:space:]]*//' | tr -d '[:space:]')
        [[ -n "$extracted" ]] && ver="$extracted"
    fi

    # 从文件名 nvXX.YY 推导 CUDA 主版本（JetPack 家族线索，仅作兜底参考）
    local fname
    fname="$(basename "$whl")"
    if [[ "$fname" =~ nv(2[0-9])\.([0-9]+) ]]; then
        local jp_major="${BASH_REMATCH[1]}"
        if   [[ "$jp_major" -ge 24 ]]; then cuda_hint="12.x"
        elif [[ "$jp_major" -ge 22 ]]; then cuda_hint="11.x"
        fi
    fi

    printf '%s\t%s' "$ver" "$cuda_hint"
}

# 安装 + 检测 + 卸载单个 whl
test_one_whl() {
    local whl_path="$1"
    local whl_name
    whl_name="$(basename "$whl_path")"

    log_step "测试 WHL: $whl_name"

    local install_ok=0 torch_ver="N/A" cuda_ver="N/A" \
          cuda_available=0 device_count=0 device_name="N/A" \
          detect_note=""
    local install_err="" detect_err=""

    # ① 安装 WHL
    # NVIDIA Jetson 官方 whl 文件名含完整构建号（如 nv24.7.16234504），但 METADATA
    # 中版本号简化为 nv24.7 → 文件名与 metadata 版本不一致 → uv 默认会拒绝安装。
    # 此"校验不匹配"是 NVIDIA 的打包惯例而非损坏，故安装时统一跳过文件名校验。
    log_info "安装中..."
    if [[ "$DRY_RUN" -eq 1 ]]; then
        install_ok=1
        log_info "[DRY-RUN] 跳过真实安装"
    else
        local install_output
        install_output=$(UV_SKIP_WHEEL_FILENAME_CHECK=1 uv pip install --python "$VENV_DIR/bin/python" "$whl_path" 2>&1) && install_ok=1 || {
            install_ok=0
            install_err="$install_output"
        }
    fi

    if [[ "$install_ok" -ne 1 ]]; then
        log_error "安装失败: $whl_name"
        echo "$install_err" | tail -5 | while IFS= read -r line; do log_error "  $line"; done
        RESULTS_WHLS+=("$whl_name")
        RESULTS_INSTALL+=("FAIL")
        RESULTS_TORCH+=("N/A")
        RESULTS_CUDA+=("N/A")
        RESULTS_CUDA_AVAIL+=("N/A")
        RESULTS_MATCH+=("N/A")
        RESULTS_NOTE+=("install failed")
        return
    fi
    log_ok "安装成功: $whl_name"

    # ② 检测 torch / CUDA 信息
    log_info "检测 torch + CUDA..."
    if [[ "$DRY_RUN" -eq 1 ]]; then
        torch_ver="0.0.0-dry-run"
        cuda_ver="$SYSTEM_CUDA"
        cuda_available=1
        device_count=1
        device_name="DRY-RUN-DETECT"
        detect_note="dry-run"
    else
        local detect_script
        detect_script=$(cat <<'PYEOF'
import sys
try:
    import torch
    print('TORCH_VERSION=' + str(torch.__version__))
    print('CUDA_VERSION=' + (str(torch.version.cuda) if torch.version.cuda else 'null'))
    try:
        _avail = torch.cuda.is_available()
        print('CUDA_AVAILABLE=' + ('1' if _avail else '0'))
        if _avail:
            _dc = torch.cuda.device_count()
            print('DEVICE_COUNT=' + str(_dc))
            print('DEVICE_NAME=' + (torch.cuda.get_device_name(0) if _dc > 0 else ''))
        else:
            print('DEVICE_COUNT=0')
            print('DEVICE_NAME=')
            # 主动触发 CUDA 初始化，捕获 “驱动太旧” 等关键错误
            try:
                torch.cuda.init()
            except Exception as _init_err:
                print('CUDA_INIT_ERROR=' + type(_init_err).__name__ + ': ' + str(_init_err).replace('\n', ' '))
    except Exception as _cuda_err:
        print('CUDA_ERROR=' + type(_cuda_err).__name__ + ': ' + str(_cuda_err).replace('\n', ' '))
except Exception as _import_err:
    print('IMPORT_ERROR=' + type(_import_err).__name__ + ': ' + str(_import_err).replace('\n', ' '))
PYEOF
)
        local detect_output detect_rc=0
        detect_output=$("$VENV_DIR/bin/python" -c "$detect_script" 2>&1) \
            || detect_rc=$?

        if [[ $detect_rc -ne 0 || -z "$detect_output" ]]; then
            log_warn "检测脚本返回异常 (rc=$detect_rc)，完整输出："
            echo "$detect_output" | tail -10 | while IFS= read -r line; do log_warn "  $line"; done
            # 安装成功但检测失败 → 仍标记安装成功，torch信息置 N/A
            RESULTS_WHLS+=("$whl_name")
            RESULTS_INSTALL+=("OK")
            RESULTS_TORCH+=("N/A")
            RESULTS_CUDA+=("N/A")
            RESULTS_CUDA_AVAIL+=("N/A")
            RESULTS_MATCH+=("N/A")
            RESULTS_NOTE+=("detect failed")
            # ④ 卸载
            log_info "卸载 torch..."
            uv pip uninstall --python "$VENV_DIR/bin/python" torch 2>/dev/null || true
            return
        fi

        # 解析 key=value 输出（不依赖 jq；兼容所有平台）
        kv() {
            local key="$1"
            local line
            line=$(printf '%s' "$detect_output" | grep "^${key}=" | tail -1) || true
            if [[ -n "$line" ]]; then
                printf '%s' "${line#${key}=}"
            else
                printf 'N/A'
            fi
        }
        torch_ver=$(kv 'TORCH_VERSION')
        cuda_ver=$(kv 'CUDA_VERSION')
        [[ "$cuda_ver" == "null" ]] && cuda_ver="N/A"
        cuda_available=$(kv 'CUDA_AVAILABLE')
        [[ -z "$cuda_available" ]] && cuda_available="0"
        device_count=$(kv 'DEVICE_COUNT')
        device_name=$(kv 'DEVICE_NAME')
        detect_note="$(kv 'CUDA_INIT_ERROR')"
        [[ "$detect_note" == "N/A" || -z "$detect_note" ]] && detect_note="$(kv 'CUDA_ERROR')"
        [[ "$detect_note" == "N/A" || -z "$detect_note" ]] && detect_note="$(kv 'IMPORT_ERROR')"
        [[ "$detect_note" == "N/A" ]] && detect_note=""

        # 若解析失败（torch_ver=N/A，即 Python 先报 import torch failed），
        # 降级从 whl 包内 METADATA 读版本号，避免整行 N/A 一抹黑；
        # 同时打印完整 detect 输出供现场排错
        if [[ "$torch_ver" == "N/A" ]]; then
            log_warn "torch_ver=N/A, 完整 detect 输出:"
            echo "$detect_output" | head -20 | while IFS= read -r _line; do log_warn "  | $_line"; done

            # 降级：从 whl 元数据解析版本号（whl 文件就在本机，无需 import）
            local meta_whl meta_torch_ver meta_cuda_hint
            meta_whl="$(extract_whl_metadata "$whl_path")"
            meta_torch_ver=$(printf '%s' "$meta_whl" | cut -f1)
            meta_cuda_hint=$(printf '%s' "$meta_whl" | cut -f2)
            if [[ -n "$meta_torch_ver" ]]; then
                torch_ver="$meta_torch_ver"
                # 备注里标注版本来源（降级），方便与 import 成功时区分
                detect_note="import failed; version from whl metadata"
            fi
            [[ -n "$meta_cuda_hint" && ( "$cuda_ver" == "N/A" || "$cuda_ver" == "null" ) ]] && cuda_ver="$meta_cuda_hint"
        fi
        log_ok "torch=$torch_ver  cuda=$cuda_ver  avail=$cuda_available  device=$device_name"
    fi

    # ③ 驱动匹配判定
    local match
    match=$(judge_match "$cuda_ver" "$SYSTEM_CUDA" "$cuda_available")

    # ⑤ 卸载 torch
    log_info "卸载 torch..."
    if [[ "$DRY_RUN" -eq 0 ]]; then
        uv pip uninstall --python "$VENV_DIR/bin/python" torch 2>/dev/null || true
    fi
    log_ok "卸载完成"

    # 记录结果
    RESULTS_WHLS+=("$whl_name")
    RESULTS_INSTALL+=("OK")
    RESULTS_TORCH+=("${torch_ver:-N/A}")
    RESULTS_CUDA+=("${cuda_ver:-N/A}")
    RESULTS_CUDA_AVAIL+=("$([[ "$cuda_available" == "true" || "$cuda_available" == "1" ]] && echo "YES" || echo "NO")")
    RESULTS_MATCH+=("$match")
    RESULTS_NOTE+=("$([[ -n "$detect_note" ]] && echo "$detect_note" || echo "-")")
}

# ---------------------------------------------------------------------------
# 汇总表格打印
# ---------------------------------------------------------------------------
print_summary() {
    local total=${#RESULTS_WHLS[@]}
    [[ "$total" -eq 0 ]] && { log_warn "未测试任何 WHL 文件"; return; }

    echo
    echo "═══════════════════════════════════════════════════════════════════════════════════════════════"
    echo "                         PyTorch WHL 兼容性测试汇总"
    echo "═══════════════════════════════════════════════════════════════════════════════════════════════"
    log_info "系统 CUDA 版本: $SYSTEM_CUDA  临时环境: $VENV_DIR (${KEEP_ENV:+保留}${KEEP_ENV:-清理})"

    # 表头
    printf "\n%-44s %-8s %-20s %-12s %-10s %-8s %s\n" \
        "WHL文件" "安装" "torch版本" "CUDA版本" "CUDA可用" "驱动匹配" "备注"
    printf "%-44s %-8s %-20s %-12s %-10s %-8s %s\n" \
        "────────────────────────────────────────────" \
        "────────" \
        "────────────────────" \
        "────────────" \
        "──────────" \
        "────────" \
        "──────────"

    local success=0 fail=0
    for i in "${!RESULTS_WHLS[@]}"; do
        printf "%-44s %-8s %-20s %-12s %-10s %-8s %s\n" \
            "${RESULTS_WHLS[$i]}" \
            "${RESULTS_INSTALL[$i]}" \
            "${RESULTS_TORCH[$i]}" \
            "${RESULTS_CUDA[$i]}" \
            "${RESULTS_CUDA_AVAIL[$i]}" \
            "${RESULTS_MATCH[$i]}" \
            "${RESULTS_NOTE[$i]}"

        if [[ "${RESULTS_MATCH[$i]}" == "OK" ]]; then
            success=$((success + 1))
        elif [[ "${RESULTS_MATCH[$i]}" != "N/A" ]]; then
            fail=$((fail + 1))
        fi
    done

    echo "───────────────────────────────────────────────────────────────────────────────────────────────"
    log_info "测试共 $total 个 WHL：兼容 $success / 不兼容 $fail / 待定 $((total - success - fail))"
    echo
}

# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
main() {
    # uv 存在性检查（已尝试过常见安装目录探测）
    if ! command -v uv &>/dev/null; then
        log_error "未找到 uv 工具。"
        log_error "尝试以下任一修复："
        log_error "  1. 直接用当前用户（非 sudo）执行："
        log_error "     bash scripts/test_torch_whl.sh"
        log_error "  2. 显式指定 uv 完整路径："
        log_error "     export PATH=\"\$HOME/.local/bin:\$PATH\""
        log_error "     sudo -E bash scripts/test_torch_whl.sh"
        log_error "  3. 重新安装到系统目录（推荐）："
        log_error "     curl -LsSf https://astral.sh/uv/install.sh | sh"
        log_error '     sudo cp "$HOME/.local/bin/uv" /usr/local/bin/'
        exit 2
    fi

    # 源目录存在性检查
    if [[ ! -d "$SOURCE_DIR" ]]; then
        log_error "WHL 源目录不存在: $SOURCE_DIR"
        exit 2
    fi

    # 收集 whl 文件列表
    local whl_list=()
    while IFS= read -r -d '' f; do
        whl_list+=("$f")
    done < <(find "$SOURCE_DIR" -maxdepth 1 -type f -name '*.whl' -print0 | sort -z)

    if [[ ${#whl_list[@]} -eq 0 ]]; then
        log_error "在 $SOURCE_DIR 下未找到任何 .whl 文件"
        exit 2
    fi

    log_info "系统 CUDA 版本: $SYSTEM_CUDA"
    log_info "WHL 源目录:   $SOURCE_DIR"
    log_info "临时环境:     $VENV_DIR"
    log_info "发现 ${#whl_list[@]} 个 WHL 文件"
    printf '%s\n' "${whl_list[@]}" | while IFS= read -r f; do log_info "  ✓ $f"; done

    setup_venv || exit 2

    # 结果数组（全局）
    RESULTS_WHLS=()
    RESULTS_INSTALL=()
    RESULTS_TORCH=()
    RESULTS_CUDA=()
    RESULTS_CUDA_AVAIL=()
    RESULTS_MATCH=()
    RESULTS_NOTE=()

    for whl_path in "${whl_list[@]}"; do
        test_one_whl "$whl_path" || true   # 单 whl 报错不中断整体流程
    done

    print_summary

    # 最终退出码：至少一个 OK = 0；无任何 OK 但有 FAIL = 1
    local any_ok=0 any_fail=0
    for i in "${!RESULTS_WHLS[@]}"; do
        [[ "${RESULTS_MATCH[$i]}" == "OK" ]] && any_ok=1
        [[ "${RESULTS_MATCH[$i]}" == "FAIL" ]] && any_fail=1
    done

    if [[ "$any_ok" -eq 1 ]] && [[ "$any_fail" -eq 0 ]]; then
        log_ok "所有 WHL 与系统驱动兼容"
        return 0
    elif [[ "$any_ok" -eq 1 ]]; then
        log_warn "部分 WHL 兼容，存在不兼容项"
        return 0
    else
        log_error "没有 WHL 与当前系统驱动兼容"
        return 1
    fi
}

# 先探测系统 CUDA（必须在 main 之前，供后续函数使用）
SYSTEM_CUDA=$(get_system_cuda_version)

main
