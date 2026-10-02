# 改动说明（`origin/main` 之后，尚未推送）

面向合作者：本文汇总 `origin/main` 之后本地的全部改动与验证状态，便于对比自己的分支。
内容取自 `git log origin/main..HEAD`，推送后本文可并入正式历史。

## 1. 验证状态（先看这里）

**代码层**

| 项目 | 状态 |
| --- | --- |
| 编译 | 通过，0 error / 0 warning（`cmake --build build/Debug`，当前 `User/App/app_test.hpp` 中 8 个测试宏全为 0） |
| 宿主测试 | 通过，`User/App/test/can/test_recovery_host.py`（在 PC 上编译 `bsp_can.cpp` 的恢复状态机：1001 次恢复循环、持续故障、取消闸门、重试延迟、tick 回绕、未初始化） |

**实机层**

| 项目 | 状态 |
| --- | --- |
| **`Service/can_bus`（`CanBus` / `CanRxNode` / `CanTxNode` / `service_cfg`）** | **❌ 完全没有测试**——既没有实机测试也没有单元测试；三路 CAN 收发、按 ID 分发、回退缓冲、发送槽位调度全部未验证 |
| Bus-Off 自动恢复的实机表现 | ❌ 未验证（只有上面的 PC 状态机测试） |
| `dji_motor` 改用 `CanBus` + `Config` 之后 | ❌ 未实机回归（历史提交里的"已测试通过"针对旧的总线系统，不适用于现在的分层） |
| QSPI Flash / USB / Online | ❌ 本轮未跑，测试宏当前都是 0 |
| `sys_task` 的 10 ms 节拍与 UART 巡检、`Online` 的毫秒超时 | ❌ 未实机验证（改动前 `sys_task` 为 1 ms，`Online` 阀值按调用次数计） |
| `key_task`（按键事件与蜂鸣器提示） | ✅ **已实机验证（2026-10-02）**：按键三档提示音都能正常发声并自动关闭 |
| `bsp_pwm_buzzer`（TIM12_CH2）+ `device_buzzer` | ✅ **已实机验证（2026-10-02）**：`set_freq()` + `set_duty()` 起振、`off()` 停振均正常 |

结论：**这批改动只保证"能编译 + 宿主测试通过"，不保证硬件行为正确**。实机回归清单见第 5 节。

## 2. 分层与结构

### CAN：Bsp 只留纯硬件，总线系统移到 Service

| 条目 | 说明 |
| --- | --- |
| `User/Bsp/bsp_can.*` | 纯硬件驱动：Message Buffer 收发、`service_recovery()` 恢复、诊断计数、ISR 入口 |
| `User/Service/can_bus.*`（新增） | 一条 CAN 总线：节点注册 + 按 ID 分发 + 槽位发送调度 + 回退缓冲 + 收发任务 |
| `User/Service/service_cfg.{hpp,cpp}`（新增） | 全局实例 `bus_can1/2/3`（原先在 `can_bus.cpp` 定义、`can_bus.hpp` 声明） |
| 删除 | `User/Bsp/bsp_can_recovery.cpp`（内容并入 `bsp_can.cpp`）、`User/Bsp/merge/`（未入库的重复实现） |
| 删除 | `User/App/task/can_rx_task.cpp`、`can_tx_task.cpp`（逻辑并入 `can_bus.cpp`） |
| 删除 | `User/Device/lcd.*`、`User/Module/menu.*`、`User/App/app_message.*`、`User/App/task/msg_task.cpp`、`task_menu.cpp` |

### 其他

- `dji_motor`：接收 `CanBus&` 而不是 `BspCan&`，构造参数收进 `Config`
- `Service/status.*`：去掉 CMSIS-RTOS2，改用原生 FreeRTOS EventGroup（全工程已无 `cmsis_os2` 引用）
- `sys_task`：周期由 1 ms 改为 **10 ms**，并把 UART 巡检（`tx_recover()` / `rx_recover()`，遍历 `bsp_cfg` 中全部串口实例）并入；CAN 的正常收发仍由 `can_rx_task` / `can_tx_task` 以 1 kHz 负责，`sys_task` 只做 10 ms 级的补救
- `Online`：离线阈值改为**毫秒**语义（按 tick 差值判定，与 `update()` 的调用周期解耦），默认 30 ms 不变；`update()` 的调用周期只影响判定延迟
- 新增 `User/Bsp/bsp_dwt.{hpp,cpp}`（内核 CYCCNT 计时）、`User/App/task/key_task.cpp`、`User/App/test/dwt/dwt_test.cpp`
- `key_task` 接入 `all_init()`：任务名 `key`、256 words、`idle+2`，200 ms 轮询 `key_user`（对应 debounce 1 / long_press 5），按事件驱动蜂鸣器；轮询周期用 `pdMS_TO_TICKS(200U)` 表达
- 新增 `User/Bsp/bsp_pwm.{hpp,cpp}`：PWM 通道驱动，句柄 / 通道 / 定时器时钟 / PSC / ARR 全部由
  `Config` 手动传入（取自 CubeMX），驱动内不写死板级常量；`init()` **先清 CCR 再启动** →
  `bsp_pwm1~4`（TIM1_CH3 / TIM1_CH1 / TIM2_CH3 / TIM2_CH1，排针预留舵机）、`bsp_pwm_gyro`
  （TIM3_CH4）、`bsp_pwm_buzzer`（TIM12_CH2）全部上电 **0% 占空比**；接口 `set_duty(float)`
  （0~100）与 `set_pulse_us(float)` 并存，另有 `set_freq()` / `off()` 与频率、脉宽查询
- 蜂鸣器从 Bsp 移到 Device：新增 `User/Device/device_buzzer.{hpp,cpp}`（`DeviceBuzzer` 持有
  `BspPwm&`，负责音调限幅、音量、鸣叫时长语义），实例 `buzzer` 在 `device_cfg`；
  **删除** `User/Bsp/bsp_buzzer.{hpp,cpp}`；`key_task` 与 `bsp_key.hpp` 示例改用 `buzzer`
- 修掉蜂鸣器的时钟换算错误：原 `BspBuzzer::Config::base_clk` 写死 6 MHz，与 TIM12 实际输入
  时钟（275 MHz / 24 ≈ 11.458 MHz）不符，导致实际发声约为请求频率的 1.9 倍；现在频率换算
  统一由 `BspPwm` 按 `timer_clk_hz / (prescaler + 1)` 计算

## 3. 命名与代码规范（改动面最大，合并分支时最容易冲突）

| 规则 | 说明 |
| --- | --- |
| 分区横幅 | `// ---------------- 名 ----------------` 开始 + 单独一行 `// ----------------` 结束；**不可嵌套**；函数体内不用横幅 |
| 私有函数 | 统一加 `_` 前缀（35 个函数、170 处） |
| 方法名 | `DjiMotor::DataUnpack → data_unpack`、`FillData → fill_data`、`Online::isOnline → is_online` |
| 构造参数 | 参数 ≥3 个的类改用类内 `Config`（`DjiMotor`、`DmMotor`） |
| `friend` | 全部移除；需要类外访问就直接设 `public` |
| RTOS API | 一律原生 FreeRTOS（`xTaskCreate` / `xSemaphore*` / `xEventGroup*` / `vTaskDelay`） |
| 测试 | 不再越层直查 HAL 寄存器，改用 `bsp_can1.diagnostics`（`HAL_NVIC_*` 故障注入保留） |

## 4. 工具链与工程配置

- **`.gitattributes`（新增）**：入库与检出统一 LF；`Drivers/`、`Middlewares/`、`tinyusb-0.20.0/` 保持原样。69 个 CRLF 文件已归一
- **`.clang-format`**：迁移到 clang-format 18+ 的正式写法（原配置有 1 个已废弃选项会被静默忽略、改变输出）；`DerivePointerAlignment: false` 使指针固定靠右；新增 `IncludeCategories` 保证 `FreeRTOS.h` 排在其它 FreeRTOS 头文件之前（否则自动排序会让 `task.h` / `event_groups.h` 直接 `#error`）
- **已统一格式化** `User/` 与 `QSPI_Flash/` 共 76 个文件；`Core/`（CubeMX 生成）与第三方目录不参与
- 格式化为**手动执行**，工程里没有自动格式化（编辑器里对当前文件 Format Document 即可）
- **`.clangd`**：按路径屏蔽 `Middlewares/`、`Drivers/` 的诊断（单独打开其中的头文件时会成片误报）；`tinyusb-0.20.0/` 不屏蔽
- 根 `CMakeLists.txt`：删除末尾多余的 `##### User #####` 标记
- 文档目录 `docs/` 改为 `Docs/`，内容全部重写；新增 `Docs/spec/分层架构.md`

## 5. 实机回归清单（未执行）

1. 三路 CAN 正常收发
2. 命中节点 → 回调消费（`DjiMotor` 反馈解包、`Online` 刷新）
3. 未命中 → `CanBus::receive()` 取到
4. 突发 256 帧 → 触发 `Status::FULL`，`diagnostics.tx_dropped` 计数
5. 断连 → Bus-Off → `service_recovery()` 自动恢复，`bus_off_events` / `recovery_successes` 增长
6. `sys_task` 10 ms 周期不阻塞（`sys_task_loop_count` 持续推进）
7. Live Watch 可读 `diagnostics` 全字段
8. `can_tx_task` 无节点时正常退出
9. `dji_motor` 析构期 `unregist` 返回 `NOT_SUPPORTED` 的路径不变

## 6. 合作者注意事项

- 拉取后建议执行一次 `git add --renormalize .`，让本地文件行尾与 `.gitattributes` 一致
- 文档目录由 `docs/` 改为 `Docs/`；Windows / macOS 上若 git 配置了 `core.ignorecase=true`，大小写改名可能不生效
- 本轮重命名范围较大（私有函数、方法名、全局实例位置、设备层接口），与未合并的分支会有较多冲突，建议先对齐本轮再继续开发
- 新代码请按 `Docs/spec/编码规范.md` 写，分层原则见 `Docs/spec/分层架构.md`
