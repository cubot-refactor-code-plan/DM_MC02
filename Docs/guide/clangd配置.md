# clangd 配置

## 参数

`.vscode/settings.json` 中的 `clangd.arguments`：

```jsonc
"clangd.arguments": [
    "--compile-commands-dir=build/Debug",
    "--query-driver=**/arm-none-eabi-g++,/**/arm-none-eabi-gcc"
]
```

`--compile-commands-dir` 指向 CMake 导出的 `compile_commands.json`（相对工作区目录）。

## query-driver 的写法

clangd 需要执行编译器本身，才能得到该工具链的系统头文件搜索路径。出于安全考虑，clangd 不会自动执行任意程序，必须通过 `--query-driver` 给出允许执行的驱动白名单，取值为逗号分隔的 glob 列表。

glob 的语义（clangd `SystemIncludeExtractor.cpp` 中的 `convertGlobToRegex`）：

| 写法 | 匹配规则 |
| --- | --- |
| `*` | 任意字符，不跨越 `/` |
| `**` | 任意字符，可跨越 `/` |

匹配对象是编译命令中的驱动**绝对路径**，并且要求整串匹配。因此：

- 不要写死具体路径（如 `/home/<user>/...`）：换一台机器后 glob 不再命中，clangd 找不到 C++ 标准库，会报大量 `Use of undeclared identifier 'std'`。
- 写成 `**/arm-none-eabi-g++` 可以覆盖任意安装位置。

修改后需要重启语言服务器（命令面板执行 `clangd.restart`）。

## 间接包含的头文件

clangd 的 include-cleaner 会检查头文件是否被直接使用。间接使用的头文件在行尾加标记保留：

```cpp
#include "FreeRTOS.h" // IWYU pragma: keep
```

## 条件编译的文件

整体被 `#if APP_TEST_XXX_ENABLED` 包住的测试文件，include 也要放在 `#if` 内部：否则宏为 0 时翻译单元为空，include-cleaner 会把每一个 include 都判为多余。

## 代码格式化

格式化只由 `.clang-format` 决定，`.clangd` 不参与格式化。

本机没有单独安装 `clang-format`，用的是 VS Code C/C++ 扩展自带的那份（`~/.vscode/extensions/ms-vscode.cpptools-*/LLVM/bin/clang-format`）。编辑器与命令行走同一份配置，因此结果一致；配置里已不再使用会随版本改变行为的旧写法（如 `ConstructorInitializerAllOnOneLineOrOnePerLine`），所以换扩展版本也不会改变格式。

用法：

- 保存时自动格式化已开启（`[c]` / `[cpp]` → `ms-vscode.cpptools`）。
- 提交前检查：`Tools/clang_format.sh check`（Windows 用 `Tools/clang_format.bat check`），不符合规范时列出文件并以退出码 1 结束。
- 统一格式：把 `check` 换成 `format`。
- VS Code 任务里对应 `Format_Check_linux` / `Format_Apply_linux` / `Format_Check_win` / `Format_Apply_win`。

范围是 `User/` 与 `QSPI_Flash/`。`Core/` 由 CubeMX 生成、第三方目录是上游代码，两者都不格式化。

### 必须靠前的头文件

FreeRTOS 要求 `FreeRTOS.h` 出现在其它 FreeRTOS 头文件之前，否则 `task.h`、`event_groups.h` 会直接 `#error`。`.clang-format` 已用 `IncludeCategories` 给 `FreeRTOS.h` 最高优先级，排序不会把它挤到后面；新增 FreeRTOS 头文件时不需要手动调整顺序。

