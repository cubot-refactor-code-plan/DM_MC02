# 改动说明

> 只记**改动清单 + 实机验证的真实状态**；其他专题见 `Docs/` 下对应文档。

## 1. 改动清单

| 区域 | 内容 |
| --- | --- |
| Bsp | `bsp_can` 为纯硬件驱动（收发 / 恢复 / 诊断 / ISR）；`bsp_pwm` 提供 6 路 PWM 通道（`bsp_pwm1~4` / `bsp_pwm_gyro` / `bsp_pwm_buzzer`，上电 0% 占空比） |
| Service | `can_bus`（节点注册、按 ID 分发、槽位调度、回退缓冲）、`can_rx_node`、`can_tx_node`、`service_cfg`、`motor_definition`、`status`（FreeRTOS 原生 EventGroup）、`online_check` |
| Device | `dji_motor`、`dm_motor`、`dm_imu`、`device_buzzer`（蜂鸣器，基于 `bsp_pwm_buzzer`）；`device_cfg` 实例化并由 `device_init()` 注册 |
| App | `api_main`：`sys_flag_init` → `bsp_init` → `can_bus_init` → `device_init` → 建任务 → `sys_complete_init`；任务为 `sys_task`（10 ms）、`can_rx_task` / `can_tx_task`（1 kHz）、`key_task`（200 ms） |
| 测试 | `User/App/test/{can,motor,online_check,dwt}/`；开关集中在 `app_test.hpp`，CAN 相关默认全为 0 |
| 文档 | `Docs/` 全部文档与根 `README.md` |

## 2. 验证状态

| 项目 | 真实状态 |
| --- | --- |
| 编译 | ✅ 已验证（2026-10-03）：`cmake --build --preset Debug`，0 error，生成 `build/Debug/DM_MC02.elf`，含 `can_bus_init` / `can_rx_task` / `can_tx_task` / `CanBus::*` / `dm_imu` / `m2006` / `gm6020` 符号 |
| CAN 分层（注册 / 分发 / 槽位调度 / 回退缓冲） | ❌ 未实机验证 |
| `dji_motor` / `dm_motor` / `dm_imu` | ❌ 未实机验证 |
| Bus-Off 恢复 / 回退缓冲溢出 | ❌ 未实机验证 |
| `sys_task` 10 ms 节拍 | ❌ 未实机验证 |
| 各测试任务（开关默认全为 0） | ❌ 未运行 |

> 按“未实机跑过一律写未验证”的约定填写。

## 3. 待办

- CAN 三路收发、节点回调、Bus-Off 恢复、回退缓冲溢出的实机回归，清单见
  [plan/CAN分层合并计划.md](plan/CAN分层合并计划.md)
- `dm_imu` 主动上报（需先从模块 USB 口进设置模式配置）
- `User/Protocol/`：目录已预留，尚无代码
