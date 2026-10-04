#!/usr/bin/env bash
# ------------------------------------------------------------------
# J-Link 烧录脚本（Linux）—— 经 OpenOCD 的 jlink 适配器驱动 SEGGER J-Link
#
#   - 自动定位工程根目录与 build/<BUILD_TYPE>/<项目名>.elf，可在任意目录调用
#   - 对应 VS Code 任务：[🔌 JLink烧录Linux] → JLink_flash_linux
#
# 用法：bash Flash/JLink_flash.sh [Debug]
# ------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_NAME="$(basename "$PROJECT_DIR")"
BUILD_TYPE="${1:-Debug}"
BUILD_DIR="build/${BUILD_TYPE}"

cd "$PROJECT_DIR"

# 找 ELF：先按项目名，找不到再兜底搜索整个 build 目录
ELF_FILE="${BUILD_DIR}/${PROJECT_NAME}.elf"
if [ ! -f "$ELF_FILE" ]; then
    ELF_FILE="$(find build -name '*.elf' -type f 2>/dev/null | head -n 1 || true)"
fi
if [ -z "${ELF_FILE}" ] || [ ! -f "$ELF_FILE" ]; then
    echo "❌ 找不到 .elf，请先编译（任务：[🔨 编译 F7]）"
    exit 1
fi
ELF_FILE="$(realpath "$ELF_FILE")"

if ! command -v openocd >/dev/null 2>&1; then
    echo "❌ 未找到 openocd；本脚本通过 OpenOCD 的 jlink 适配器驱动 J-Link"
    exit 1
fi

echo "🔧 项目：$PROJECT_NAME（$BUILD_TYPE）"
echo "📦 ELF ：$ELF_FILE"
echo "⚡ 经 OpenOCD jlink 适配器烧录中（SWD 4 MHz）..."
openocd -f "$SCRIPT_DIR/jlink.cfg" \
        -c "gdb_port disabled" \
        -c "tcl_port disabled" \
        -c "telnet_port disabled" \
        -c "program \"$ELF_FILE\" verify reset exit"

echo "✅ 烧录完成"
