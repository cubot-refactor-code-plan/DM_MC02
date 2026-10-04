# CUBOT Code Rebuild

STM32H723 机器人主控固件，使用 CMake + Ninja + Arm GNU Toolchain 构建，运行在 FreeRTOS 上。

代码按分层架构组织，依赖方向单向：

```txt
App → Module → Device → Bsp
```

`Service` / `Algorithm` 不属于这条主链，以工具库、服务的形式贯穿整个工程；`Protocol` 目录已预留，当前为空。

## CAN 分层

```txt
Bsp     BspCan                    纯硬件：收发 / 恢复 / 诊断 / ISR
Service CanBus                    节点注册 / 按 ID 分发 / 槽位调度 / 回退缓冲 / 1 kHz 收发任务
Device  DjiMotor / DmMotor / DmImu 注册到 CanBus
App     api_main                  sys_flag_init → bsp_init → can_bus_init → device_init → 建任务
```

## 目录总览

| 目录 | 说明 |
| --- | --- |
| `Core/` | CubeMX 生成的 HAL 初始化与中断服务 |
| `User/` | 业务代码，按分层组织（Bsp / Service / Device / Algorithm / App） |
| `cmake/` | 工具链文件与 CubeMX 的 CMake 集成 |
| `Docs/` | 开发文档、硬件图纸与器件手册 |
| `Flash/` | 烧写脚本、SVD 与调试配置 |
| `Drivers/`、`Middlewares/` | 第三方库（STM32 HAL、CMSIS、FreeRTOS） |
| `Example/` | 参考代码与阅读笔记；`can_backup/` 为旧 CAN 链路的归档 |

各目录与分层的详细说明见 [Docs/guide/项目结构.md](Docs/guide/项目结构.md)。

## 编译与烧写

编译使用 VS Code 的 CMake 插件（或 `cmake --preset Debug` + `cmake --build --preset Debug`），烧录使用命令行脚本（OpenOCD / J-Link），调试使用 Cortex-Debug 或 Ozone。

环境要求、命令行细节，以及 VS Code / Cortex-Debug / Ozone 的用法见 [Docs/guide/开发环境与烧录调试.md](Docs/guide/开发环境与烧录调试.md)。

## 代码规范

代码遵循 [Docs/spec/编码规范.md](Docs/spec/编码规范.md)：分层依赖单向、命名与注释有固定写法，格式由 `.clang-format` 固定。格式化为**手动执行**，工程里没有任何自动格式化；格式化范围是 `User/`，`Core/`（CubeMX 生成）与第三方目录不参与，细节见 [Docs/guide/clangd配置.md](Docs/guide/clangd配置.md)。

## 文档

开发文档总入口（AI 开发必读）：[Docs/README.md](Docs/README.md)

改动清单与验证状态：[Docs/CHANGELOG.md](Docs/CHANGELOG.md)

| 文档 | 内容 |
| --- | --- |
| [CHANGELOG](Docs/CHANGELOG.md) | 改动清单与实机验证的真实状态 |
| [问题记录](Docs/问题记录.md) | AI 协作复盘：失实汇报、不可核查的产出、约束清单 |
| [项目结构](Docs/guide/项目结构.md) | 目录结构与各层职责 |
| [开发环境与烧录调试](Docs/guide/开发环境与烧录调试.md) | 环境、编译、烧写、调试、验证记录 |
| [进度与待办](Docs/plan/进度与待办.md) | 完成情况、待办、设计取舍 |
| [编码规范](Docs/spec/编码规范.md) | 命名、注释、类设计、分层、C/C++ 混编 |
| [分层架构](Docs/spec/分层架构.md) | 分层原则与分层规划图 |
| [clangd 配置](Docs/guide/clangd配置.md) | clangd 参数、query-driver、IWYU |
| [引脚分配与冲突](Docs/hardware/引脚分配与冲突.md) | 引脚复用冲突与最终分配 |
| [构建系统改动](Docs/build/构建系统改动.md) | CMakeLists 改动记录 |
| [链接脚本改动](Docs/build/链接脚本改动.md) | `.dma_buffer` 段与 DMA 内存布局 |
| [CAN 分层合并计划](Docs/plan/CAN分层合并计划.md) | CAN 分层的方案与实施记录 |

