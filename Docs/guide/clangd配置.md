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
