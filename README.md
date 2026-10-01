# CUBOT Code Rebuild

STM32H723 机器人主控固件，使用 CMake + Ninja + Arm GNU Toolchain 构建，运行在 FreeRTOS 上。

代码按分层架构组织，依赖方向单向：

```txt
App → Module → Device → Bsp
```

`Service` / `Algorithm` / `Protocol` 不属于这条主链，以工具库、服务的形式贯穿整个工程。

仓库包含板载驱动、设备驱动与上位机通讯协议，并附带 CubeMX 工程、烧写脚本和开发文档；目标是提供基于全开源工具链的一套开源、便于跨兵种/跨比赛复用的成熟工程样板。

TODO：实机测试部分已存在，但不完善、可读性不足。

## 目录总览

| 目录 | 说明 |
| --- | --- |
| `Core/` | CubeMX 生成的 HAL 初始化与中断服务 |
| `User/` | 业务代码，按分层组织 |
| `cmake/` | 工具链文件与 CubeMX 的 CMake 集成 |
| `Docs/` | 开发文档、硬件图纸与器件手册 |
| `Flash/` | 烧写脚本与调试配置 |
| `QSPI_Flash/` | 外置 Flash 驱动与 XIP |
| `Drivers/`、`Middlewares/`、`tinyusb-0.20.0/` | 第三方库（HAL、FreeRTOS、TinyUSB） |

各目录与分层的详细说明见 [Docs/guide/项目结构.md](Docs/guide/项目结构.md)。

## 编译与烧写

编译使用 VS Code 的 CMake 插件，烧录使用命令行脚本（OpenOCD / J-Link），调试使用 Cortex-Debug 或 Ozone。

环境要求、命令行细节，以及 VS Code / Cortex-Debug / Ozone 的用法见 [Docs/guide/开发环境与烧录调试.md](Docs/guide/开发环境与烧录调试.md)。

## 代码规范

代码遵循 [Docs/spec/编码规范.md](Docs/spec/编码规范.md)：分层依赖单向、命名与注释有固定写法，格式由 `.clang-format` 固定。格式化为**手动执行**，工程里没有任何自动格式化；格式化范围是 `User/` 与 `QSPI_Flash/`，`Core/`（CubeMX 生成）与第三方目录不参与，细节见 [Docs/guide/clangd配置.md](Docs/guide/clangd配置.md)。

## 文档

开发文档总入口（AI 开发必读）：[Docs/README.md](Docs/README.md)

| 文档 | 内容 |
| --- | --- |
| [项目结构](Docs/guide/项目结构.md) | 目录结构与各层职责 |
| [开发环境与烧录调试](Docs/guide/开发环境与烧录调试.md) | 环境、编译、烧写、调试、验证记录 |
| [进度与待办](Docs/plan/进度与待办.md) | 完成情况、待办、设计取舍 |
| [编码规范](Docs/spec/编码规范.md) | 命名、注释、类设计、分层、C/C++ 混编 |
| [分层架构](Docs/spec/分层架构.md) | 分层原则与分层规划图 |
| [clangd 配置](Docs/guide/clangd配置.md) | clangd 参数、query-driver、IWYU |
| [引脚分配与冲突](Docs/hardware/引脚分配与冲突.md) | 引脚复用冲突与最终分配 |
| [构建系统改动](Docs/build/构建系统改动.md) | CMakeLists 与 TinyUSB 接入 |
| [链接脚本改动](Docs/build/链接脚本改动.md) | `.dma_buffer` 段与 DMA 内存布局 |

