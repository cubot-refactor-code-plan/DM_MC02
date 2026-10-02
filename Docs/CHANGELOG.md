# 改动说明（`origin/main` 之后，尚未推送）

> 只记**改动清单 + 实机验证的真实状态**；其他专题见 `Docs/` 下对应文档。

## 1. 验证状态（先看这里）

| 项目 | 真实状态 |
| --- | --- |
| `Service/can_bus`（注册 / 分发 / 槽位调度 / 回退缓冲） | ✅ 实机（2026-10-02）：CAN3 上 1 kHz 反馈分发、`Online` 刷新、诊断计数正常 |
| `dji_motor`（改用 `CanBus`） | ✅ 实机（2026-10-02）：M2006 0x200/0x201、GM6020 电流模式 0x1FE/0x205，反馈电流跟随指令 |
| CAN3 电机自检 `can3_device_test.cpp` | ⚠️ **只验证到"总线能发、电机会转、转向跟随电流符号"**。最终版是**开环**正弦电流（2026-10-03）；转速闭环从未实现、从未验证 |
| `dm_imu`（达妙 DM-IMU-L1） | ⚠️ 应答模式 ✅（`ack_id=0x59`、欧拉角随姿态刷新）；**主动上报 ❌ 未打通**——设间隔 / 切主动 / 保存 flash / 重启四招均无效（`fb=0 RXd=0 RXl=0`）。卡在模块侧「输出数据选择」（手册注明不支持 CAN 修改），需从 USB 进设置模式配置 |
| `sys_task` 10 ms 节拍 | ✅ 实机（2026-10-02）：单轮 ≤30 µs、唤醒间隔恒 10 ms；IMU 上线检查已于 2026-10-03 删除 |
| Bus-Off 恢复 / QSPI / USB / Online / UART / DWT | ❌ 未实机验证（测试宏全为 0） |

**真正跑通的只有一条：CAN 总线能收发 + 电机正反转。**

## 2. 改动清单

| 区域 | 改动 |
| --- | --- |
| CAN 分层 | Bsp 只留纯硬件；新增 `Service/can_bus.*`（注册 + 分发 + 槽位调度 + 回退缓冲 + 收发任务）与 `service_cfg.*`；`dji_motor` 收 `CanBus&` + `Config`；`Online` 离线阈值改为毫秒语义；删除 `bsp_can_recovery.cpp`、`Bsp/merge/`、`can_rx_task.cpp`、`can_tx_task.cpp`、`lcd.*`、`menu.*`、`app_message.*`、`msg_task.cpp`、`task_menu.cpp` |
| `sys_task` | 周期 1 ms → 10 ms 并并入 UART 巡检；2026-10-03 移除 IMU 上线检查（含 `sys_task_imu_check` 与 `device_cfg.hpp` 依赖） |
| CAN3 设备与自检 | `device_init()` 注册 `dm_imu`(0x58/0x59) / `m2006`(ID1) / `gm6020`(ID1 电流模式)；`can3_device_test.cpp`（开环正弦电流）、`can3_imu_test.cpp`（逐帧打印 + 开机自检 + 无数据告警），两个测试宏当前都是 0 |
| `dm_imu` | 重写：依赖 `CanBus&`、指令单发不占 `CanTxNode` 槽位、01~04 数据帧全解析、新增 `probe_read()` / `data_frames()` / `Config::save_params`。修复映射值解析（原按 `int16_t` 导致正角度偏 −360°，如 `roll=-359.65°` 实为 `+0.35°`，改 `uint16_t`） |

## 3. Git 与回归

改动**全部未推送、未提交**，停留在工作区；提交前必须**手动格式化**（工程禁用自动格式化）；本文件已从工程根移到 `Docs/`。

未执行的实机回归：CAN 三路收发 / 节点回调 / Bus-Off 恢复 / 回退缓冲溢出；QSPI 擦写与 XIP；USB HID 与 CDC；`Online` 毫秒边界；`dm_imu` 主动上报（需先做模块侧 USB 配置）。
