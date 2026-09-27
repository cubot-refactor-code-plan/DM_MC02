#!/usr/bin/env bash
# ------------------------------------------------------------------
# J-Link 烧录脚本（Linux）
#
#   - 自动定位工程根目录与 build/Debug/<项目名>.elf，可在任意目录调用
#   - J-Link 命令文件写到 /tmp，脚本退出时自动删除，不污染工程目录
#   - 对应 VS Code 任务：[🔌 JLink烧录Linux] → JLink_flash_linux
#
# 用法：bash Flash/JLink_flash.sh
#       ./Flash/JLink_flash.sh
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
ELF_FILE="$(realpath "$ELF_FILE")" # J-Link 用绝对路径最稳

# 2) 找 J-Link 可执行文件（Linux 为 JLinkExe，Windows 为 JLink）
JLINK=""
for cmd in JLinkExe JLink; do
    if command -v "$cmd" >/dev/null 2>&1; then
        JLINK="$cmd"
        break
    fi
done
if [ -z "$JLINK" ]; then
    echo "❌ 未找到 JLinkExe/JLink，请安装 SEGGER J-Link 软件包并加入 PATH"
    exit 1
fi

# 3) 生成临时命令文件（/tmp，退出时自动清理）
TEMP_SCRIPT="$(mktemp -t jlink_flash_XXXXXX.jlink)"
trap 'rm -f "$TEMP_SCRIPT"' EXIT

cat > "$TEMP_SCRIPT" <<EOF
device STM32H723VG
if SWD
speed 4000
r
loadfile "$ELF_FILE"
r
go
exit
EOF

echo "🔧 项目：$PROJECT_NAME"
echo "📦 ELF ：$ELF_FILE"
echo "⚡ $JLINK 烧录中（SWD 4 MHz）..."

"$JLINK" -CommanderScript "$TEMP_SCRIPT"

echo "✅ 烧录完成"
