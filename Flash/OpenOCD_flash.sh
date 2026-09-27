#!/usr/bin/env bash
# ------------------------------------------------------------------
# OpenOCD + DAPLink 烧录脚本（Linux）
#
#   - 自动定位工程根目录与 build/Debug/<项目名>.elf，可在任意目录调用
#   - 对应 VS Code 任务：[⚡ DAP烧录Linux] → OpenOCD_flash_linux
#
# 用法：bash Flash/OpenOCD_flash.sh
#       ./Flash/OpenOCD_flash.sh
# ------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_NAME="$(basename "$PROJECT_DIR")"
cd "$PROJECT_DIR"

# 1) 找 ELF：先按项目名，找不到再兜底搜索整个 build 目录
ELF_FILE="build/Debug/${PROJECT_NAME}.elf"
if [ ! -f "$ELF_FILE" ]; then
    ELF_FILE="$(find build -name '*.elf' -type f 2>/dev/null | head -n 1 || true)"
fi
if [ -z "${ELF_FILE}" ] || [ ! -f "$ELF_FILE" ]; then
    echo "❌ 找不到 .elf，请先编译（任务：[🔨 编译 F7]）"
    exit 1
fi

# 2) 检查 openocd 是否可用
if ! command -v openocd >/dev/null 2>&1; then
    echo "❌ 未找到 openocd，请先安装：sudo apt install openocd"
    exit 1
fi

echo "🔧 项目：$PROJECT_NAME"
echo "📦 ELF ：$ELF_FILE"
echo "⚡ OpenOCD 烧录中（DAPLink / CMSIS-DAP）..."

openocd -f "$SCRIPT_DIR/daplink.cfg" \
        -c "program \"$ELF_FILE\" verify reset exit"

echo "✅ 烧录完成"
