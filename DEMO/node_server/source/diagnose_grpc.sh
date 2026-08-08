#!/bin/bash
# ==============================================================================
# diagnose_grpc.sh — gRPC 运行时与生成工具版本诊断脚本
#
# 用途：排查 grpcio / grpcio-tools 版本不匹配导致的
#       "AttributeError: module 'grpc' has no attribute '__version__'" 问题
#
# 使用方法（在开发板上）：
#   1. 将本脚本放入 /opt/node_server/source/
#   2. chmod +x diagnose_grpc.sh
#   3. 在项目根目录执行：sudo bash source/diagnose_grpc.sh
# ==============================================================================

set -euo pipefail

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "\n===== $* ====="; }

# 自动定位项目根目录（脚本所在目录的父目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

log_info "项目根目录: $PROJECT_DIR"
log_info "虚拟环境路径: $PROJECT_DIR/.venv"

# 检查虚拟环境是否存在
if [[ ! -f "$PROJECT_DIR/.venv/bin/python" ]]; then
    log_error "未找到 .venv 虚拟环境，请先运行 deploy_node_server.sh"
    exit 1
fi

PYTHON="$PROJECT_DIR/.venv/bin/python"
PIP="$PROJECT_DIR/.venv/bin/pip"

# ==============================================================================
# 步骤 1：检查 grpcio / grpcio-tools 实际安装版本
# ==============================================================================
log_step "步骤 1：grpcio / grpcio-tools 已安装版本"

echo ""
echo "--- grpcio 运行时 ---"
$PYTHON -c "
import grpc
ver = getattr(grpc, '__version__', 'NOT_FOUND')
print(f'  grpc.__version__ = {ver}')
print(f'  grpc.__file__   = {grpc.__file__}')
if ver == 'NOT_FOUND':
    import importlib.metadata
    meta_ver = importlib.metadata.version('grpcio')
    print(f'  importlib.metadata 版本 = {meta_ver}')
"

echo ""
echo "--- grpcio-tools 生成工具 ---"
$PIP show grpcio grpcio-tools 2>/dev/null | grep -E '^(Name|Version)' || {
    log_warn "grpcio / grpcio-tools 未安装"
}

echo ""
echo "--- protobuf 运行时 ---"
$PYTHON -c "
import google.protobuf
print(f'  protobuf.__version__ = {google.protobuf.__version__}')
" 2>/dev/null || log_warn "protobuf 未安装"

# ==============================================================================
# 步骤 2：检查 pb2_grpc.py 文件状态
# ==============================================================================
log_step "步骤 2：pb2_grpc.py 文件状态"

PB2_GRPC="utils/proto/rebar_inference_pb2_grpc.py"
echo ""
if [[ -f "$PB2_GRPC" ]]; then
    log_info "文件存在: $PB2_GRPC"
    
    # 提取 GRPC_GENERATED_VERSION
    GENERATED_VER=$(grep 'GRPC_GENERATED_VERSION' "$PB2_GRPC" | head -1 | grep -oP "'\K[^']+")
    log_info "生成工具版本 (GRPC_GENERATED_VERSION): ${GENERATED_VER:-无法提取}"
    
    # 提取前 12 行内容
    echo ""
    echo "--- 文件前 12 行 ---"
    head -12 "$PB2_GRPC" | sed 's/^/    /'
    echo ""
    
    # 文件修改时间
    echo "--- 文件元信息 ---"
    stat "$PB2_GRPC" | grep -E '(Modify|File)' | sed 's/^/    /'
else
    log_error "文件不存在: $PB2_GRPC"
fi

# ==============================================================================
# Step 3：手动验证 protoc 能否成功执行（并自动生成）
# ==============================================================================
log_step "步骤 3：验证 protoc 编译"

echo ""
PROTO_FILE="utils/proto/rebar_inference.proto"

if [[ ! -f "$PROTO_FILE" ]]; then
    log_error "proto 源文件不存在: $PROTO_FILE"
    exit 1
fi

BACKUP_DIR="/tmp/grpc_pb2_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"
log_info "备份旧 pb2 文件至: $BACKUP_DIR"

# 备份后再删除（保留原始状态供比对）
for f in utils/proto/rebar_inference_pb2.py utils/proto/rebar_inference_pb2_grpc.py; do
    if [[ -f "$f" ]]; then
        cp -p "$f" "$BACKUP_DIR/"
        rm -f "$f"
    fi
done

log_info "已清除旧 pb2 文件，开始重新编译..."
echo ""

# 执行 protoc 并捕获退出码
set +e
$PYTHON -m grpc_tools.protoc \
    --proto_path=. \
    --python_out=. \
    --grpc_python_out=. \
    "$PROTO_FILE"
PROTOC_EXIT=$?
set -e

echo ""
log_info "protoc 退出码: $PROTOC_EXIT"

if [[ $PROTOC_EXIT -ne 0 ]]; then
    log_error "protoc 编译失败！"
    log_error "请检查："
    log_error "  1. grpcio-tools 是否已安装: $PIP show grpcio-tools"
    log_error "  2. proto 文件语法是否有误: $PROTO_FILE"
    log_error "  3. 文件是否被其他进程占用"
    
    # 恢复备份
    log_warn "从备份恢复..."
    cp -p "$BACKUP_DIR"/* utils/proto/ 2>/dev/null || true
    exit 1
fi

# 验证生成结果
echo ""
GEN_OK=true
for f in utils/proto/rebar_inference_pb2.py utils/proto/rebar_inference_pb2_grpc.py; do
    if [[ -f "$f" ]]; then
        log_info "✓ 已生成: $f"
    else
        log_error "✗ 未生成: $f"
        GEN_OK=false
    fi
done

if [[ "$GEN_OK" != true ]]; then
    log_error "编译输出不完整"
    exit 1
fi

# 检查新生成文件的 GRPC_GENERATED_VERSION
echo ""
NEW_GEN_VER=$(grep 'GRPC_GENERATED_VERSION' utils/proto/rebar_inference_pb2_grpc.py | head -1 | grep -oP "'\K[^']+")
log_info "重新编译后 GRPC_GENERATED_VERSION: ${NEW_GEN_VER:-无法提取}"

# ==============================================================================
# Step 4：冒烟测试 — 验证新文件 import 可行
# ==============================================================================
log_step "步骤 4：冒烟测试 — import 验证"

echo ""
set +e
$PYTHON -c "
import sys
sys.path.insert(0, '.')
try:
    import utils.proto.rebar_inference_pb2 as pb2
    print(f'  ✓ rebar_inference_pb2 导入成功, __file__={pb2.__file__}')
except Exception as e:
    print(f'  ✗ rebar_inference_pb2 导入失败: {e}')

try:
    import utils.proto.rebar_inference_pb2_grpc as pb2_grpc
    print(f'  ✓ rebar_inference_pb2_grpc 导入成功')
except AttributeError as e:
    print(f'  ✗ rebar_inference_pb2_grpc 导入失败 (AttributeError): {e}')
except Exception as e:
    print(f'  ✗ rebar_inference_pb2_grpc 导入失败: {e}')
"
SMOKE_EXIT=$?
set -e

# ==============================================================================
# 诊断结论
# ==============================================================================
log_step "诊断结论"

echo ""
echo "关键信息汇总："
echo ""
echo "  1. grpcio 运行时版本:       $($PYTHON -c "import grpc; print(getattr(grpc, '__version__', 'NOT_FOUND'))" 2>/dev/null)"
echo "  2. grpcio-tools 生成版本:   $(grep 'GRPC_GENERATED_VERSION' utils/proto/rebar_inference_pb2_grpc.py 2>/dev/null | grep -oP "'\K[^']+" || echo 'N/A')"
echo ""
echo "判断规则："
echo "  - 若两个版本不同 → 需要锁定同版本（修复方向见下方）"
echo "  - 若运行时版本显示 NOT_FOUND → 已触发当前 bug，grpcio 版本过高"
echo "  - 若冒烟测试报错 → 版本不匹配确认"
echo ""
echo "修复建议（待确认后执行）："
echo "  1. 将 requirements.txt 和 pyproject.toml 中的"
echo "       grpcio>=1.50.0 / grpcio-tools>=1.50.0"
echo "     改为锁定版本，如："
echo "       grpcio==1.62.3 / grpcio-tools==1.62.3"
echo ""
echo "  2. 在 deploy_node_server.sh 步骤 8 中："
echo "     - protoc 执行前先 rm -f 旧的 *_pb2*.py"
echo "     - protoc 退出码非零时显式 exit 1"
echo ""
echo "  3. 修复 pyproject.toml packages 列表："
echo "     packages = [\"utils\"]"
echo ""
log_info "备份文件保留在: $BACKUP_DIR"
