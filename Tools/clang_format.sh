#!/usr/bin/env bash
# 对 User/ 与 QSPI_Flash/ 做 clang-format 检查或格式化。
#
# 用法：
#   Tools/clang_format.sh check  [文件...]   # 只检查，不符合规范则退出码 1
#   Tools/clang_format.sh format [文件...]   # 原地格式化
#
# 不传文件时默认覆盖 User/ 与 QSPI_Flash/ 下的 .c/.h/.cpp/.hpp。
# Core/ 由 CubeMX 生成、第三方目录为上游代码，两者都不处理。

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# clang-format 优先取 VS Code C/C++ 扩展自带的版本（与编辑器格式化结果一致），
# 取不到再退回 PATH。
find_clang_format() {
  local candidate
  candidate="$(ls -1 "$HOME"/.vscode/extensions/ms-vscode.cpptools-*/LLVM/bin/clang-format 2>/dev/null |
    sort -V | tail -1)"
  if [ -n "$candidate" ] && [ -x "$candidate" ]; then
    printf '%s\n' "$candidate"
    return 0
  fi
  command -v clang-format
}

CLANG_FORMAT="$(find_clang_format)" || {
  echo "未找到 clang-format（既没有 C/C++ 扩展自带的，也没有 PATH 中的）" >&2
  exit 2
}

MODE="${1:-check}"
[ $# -gt 0 ] && shift

if [ $# -gt 0 ]; then
  FILES=("$@")
else
  mapfile -t FILES < <(find "$ROOT/User" "$ROOT/QSPI_Flash" -type f \
    \( -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.hpp' \) | sort)
fi

case "$MODE" in
  format)
    "$CLANG_FORMAT" -i "${FILES[@]}"
    echo "已格式化 ${#FILES[@]} 个文件（$CLANG_FORMAT）"
    ;;
  check)
    failed=0
    for f in "${FILES[@]}"; do
      if ! "$CLANG_FORMAT" --dry-run --Werror "$f" >/dev/null 2>&1; then
        echo "格式不符合 .clang-format: ${f#"$ROOT"/}"
        failed=1
      fi
    done
    if [ "$failed" -ne 0 ]; then
      echo "检查失败：运行 Tools/clang_format.sh format 统一格式后再提交"
      exit 1
    fi
    echo "格式检查通过：${#FILES[@]} 个文件"
    ;;
  *)
    echo "用法：$0 [check|format] [文件...]" >&2
    exit 2
    ;;
esac
