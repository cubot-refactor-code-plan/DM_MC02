# CAN1 Bus-Off 恢复压力测试

> ⚠️ 本测试尚未实机运行（`APP_TEST_CAN_RECOVERY_ENABLED` 当前为 0）。

## 固件与状态

`APP_TEST_CAN_RECOVERY_ENABLED=1` 启用测试。
CAN1 连接 C620 ID2，所有发送指令都是零电流。复位后等待 GDB 写
`can_recovery_test_arm=1`，不会自动注入故障。

自动测试依次执行：

1. 确认至少 10 个新的 0x202 反馈。
2. 以不同临时错误波特率注入故障，每次维持 300 ms，然后恢复原 NBTP。
   最多尝试 100 次，只有实际观察到 BO 且反馈恢复的场景才计入 20 轮。
   每轮检测真正的 PSR.BO/恢复事件，再收到至少 10 个新反馈；恢复后保持正常时序 500 ms。
3. 屏蔽 CAN1 接收 IRQ 50 ms，制造硬件 FIFO 与软件缓冲积压。
   必须观察到软件丢帧并恢复反馈。
4. 连续发送 256 个零控制帧，必须观察到 FULL，并恢复正常反馈。

`can_recovery_test_stage=8 / passed=1 / failure=0` 才表示全部通过。
失败码：1 初始无反馈；2 测试时序配置失败；3 Bus-Off 后未恢复；
4 未实际进入 Bus-Off；5 接收积压后未恢复；6 未制造出接收溢出；
7 发送满载测试失败。9 是失败阶段，不代表生产恢复任务停止。

观测 `bsp_can1.diagnostics`：Bus-Off 次数、恢复尝试/成功次数、
最长恢复时间、RX/TX 丢帧及 recovering 状态。恢复成功计数表示控制器退出
Bus-Off 且旧 TX 请求已取消；端到端恢复由测试的新反馈验证。

`arm=2` 是物理故障模式：先有正常反馈，再等待用户断连/恢复接线。
单纯拔线缺 ACK 可能只达到 error-passive，测试不能将它冒充 Bus-Off。
若自动错误波特率未制造真实 BO，需要另行确认故障注入方式后再测试。

## 主机状态机测试

运行 `python3 User/App/test/can/test_recovery_host.py`。
它从 `User/Bsp/bsp_can.cpp` 中抽取 `service_recovery()` / `tx_available()` 的函数体，
配上模拟寄存器和 RTOS 时钟，验证 1001 次恢复循环、持续故障节流、TX 取消门控、
tick 回绕和未初始化状态。
主机模拟不验证真实硬件的 TX 取消或 CAN 物理恢复时序；仍须执行实机测试。

## 生产行为与边界

sys_task 每 10 ms 服务三条总线（下面列出的实测数据是 1 ms 节拍下测得的，
节拍放宽后恢复耗时上限会相应变大）。Bus-Off 后清软件 TX/原始 RX 缓冲，取消
硬件 TXBRP 中的旧请求，清 CCCR.INIT 启动硬件恢复。若再次 Bus-Off，
最短 100 ms 再尝试；没有忙等、动态分配或共享外设 RCC 复位。
恢复过程禁止新的发送；退出 BO 且 TXBRP 清零后重新允许发送。
注册节点保留，业务应持续更新控制意图，以恢复最新指令。

恢复不消除仍然存在的断线、短路、错误波特率或外设硬件故障；它保证
软件继续尝试，而不要求用户复位 MCU。总线恢复不意味着离线期间执行器
仍可控制，执行器自身的通信超时策略仍有必要。

## 2026-09-30 实机验证

CAN1 / C620 ID2 / M3508，电源限流 2 A，全程零电流指令。
内外 Flash 烧写及校验通过。自动测试结果：

- stage=8、passed=1、failure=0。
- 29 次故障注入，完成 20 轮真实 Bus-Off 及端到端反馈恢复。
- 共发生 259 次 Bus-Off，259 次恢复成功，271 次恢复尝试。
- 最长恢复耗时 295 个 1 ms tick（当时 sys_task 为 1 ms 节拍）；持续故障阶段会多次重新进入 BO。
- 接收积压测试产生 9 次软件溢出丢帧，然后反馈正常恢复。
- 发送 FULL 累计 2714 次（含故障阶段积压），突发满载后反馈恢复。
- 最终 recovering=false、CCCR.INIT=0，CAN2/3 没有恢复事件。
- 主机状态机测试再次通过 1001 次恢复循环。

该测试验证错误位时序、持续/重复 BO、收发积压后的恢复。
未进行物理短路、断线和电调断电测试，不将这些情况写为已验证。

## CAN 分层重构（2026-10-01）

> ⚠️ 本节引入的 `CanBus` 分发层已于 2026-10-04 归档，见文末「架构变更」。

- Bsp 层 `BspCan` 只保留硬件收发 / 恢复 / 诊断，去掉节点链表、`friend` 与回退缓冲；
  按 ID 分发、槽位发送调度与回退缓冲当时移到了 Service 层 `CanBus`（`bus_can1/2/3`）。
- `can_rx_task` / `can_tx_task` 从 `User/App/task/` 移到 `User/Service/can_bus.cpp`。
- 应用与设备层一律通过 `CanBus` 收发：`bus_can1.send()` / `bus_can1.receive()`；
  直接调 `BspCan::receive()` 会与分发任务争抢同一个 MessageBuffer。
- 恢复接口：`BspCan::service_recovery()`（Bus-Off）与 `BspCan::tx_recover()`（丢唤醒补发），
  由 `sys_task` 每 10 ms 对三条总线各调一次。
- 系统状态改用 FreeRTOS 原生事件组，封装成可多实例的 `EventState` 类（全局实例 `sys_state`）；
  CAN 相关 RTOS 资源统一用原生 API（`xSemaphore*` / `xMessageBuffer*`）。
- 诊断量集中在 `BspCan::diagnostics`，新增 `rx_lost` / `rx_len_drop` / `tx_buf_full` /
  `tx_stall_recover` / `tx_it_fail` / `err_passive` / `err_warning`。

实机回归结果：待补充。

## 2026-10-01 发送链表重构后的实机回归

每条 BspCan 独立持有 rx_head / tx_head 后，重新编译、烧写并验证：

- 故障恢复测试 passed=1、failure=0；28 次注入完成 20 轮真实 BO 场景。
- 累计 245 次 BO / 245 次恢复成功，最长 295 ms。
- 接收积压丢帧 9 次，发送 FULL 2454 次，之后反馈恢复。
- 随后切换为 DJI 电机测试固件并重新烧写；CAN1 收发节点注册成功，CAN2/3 TX 链表为空。
- CAN1 ID2 M3508 双向测试 passed=1、failure=0：正向峰值 1879 rpm，反向 -1988 rpm。
- 输出轴位移 +6.9367 / -7.7443 rad；最终转速 0、0x200 全零输出。
- 反馈在线，反馈拒收与 CAN 收发错误计数均为 0。

当前各测试开关均已关闭（`APP_TEST_CAN_RECOVERY_ENABLED=0`、
`APP_TEST_ONLINE_CHECK_ENABLED=0`、`APP_TEST_DJI_GROUP_ENABLED=0`）；
Bus-Off 生产恢复逻辑持续由 sys_task 服务。

## CAN3 电机自检（2026-10-02，已归档）

当时的测试开关 `APP_TEST_CAN3_DEVICE_ENABLED`（任务 `can3_test`）依赖已归档的
`CanBus` 与节点注册表，文件已移到 `Example/can_backup/can3_device_test.cpp`。

- 被测设备：`m2006`（C610，ID 1 → 0x200 槽位 0 / 反馈 0x201）、`gm6020`（电流模式，
  ID 1 → 0x1FE 槽位 0 / 反馈 0x205），都在 CAN3。

## 2026-10-04 架构变更：设备直连 BspCan

- `CanBus` / `CanRxNode` / `CanTxNode` / `DjiMotor<型号>` 整体归档到 `Example/can_backup/`。
  归档后**不再有总线级分发层**：`BspCan` 取出的帧由设备自己的任务处理。
- `can_rx_task` / `can_tx_task` 删除；`sys_task` 改为直接对 `bsp_can1/2/3` 调用
  `tx_recover()` / `service_recovery()`。
- `sys_task` 的注释与 `BspCan` 的文件头同步删掉了对 `CanBus` 的引用。
  `test_recovery_host.py` 只依赖 `bsp_can.cpp`，不受影响（仍为 PASS）。

> ⚠️ **`BspCan` 的接收缓冲只允许一个消费者**：同一条总线上如果有两个任务都调
> `receive()`，会互相抢帧。每条总线请只在一个任务里取帧。
- 控制方式：**开环**——电流指令直接给 `AMP·sin(2π·f·t)`（不用转速反馈），默认 ±3000 / 0.25 Hz；
  电流正负交替，电机随之正反往复；指令带斜率限制，转速只做 1000 rpm 兜底（等于不限制）。
- 打印：每轮给出 `rpm` 极值、`given`（电调反馈电流）极值、`temp`、`limit_hits`，
  以及 `sys_task` 的单轮耗时与唤醒抖动。

实机验证：两路反馈 `samples=1000`、诊断计数全 0、链路通、方向正确；**只验证了链路是否通，
未做转速闭环**。开环版已实机确认可用（2026-10-03）；`sys_task` 单轮最大 30 µs、
唤醒间隔恒为 10 ms。

