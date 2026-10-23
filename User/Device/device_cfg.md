# device_cfg

> 代码：`device_cfg.hpp` / `device_cfg.cpp`

设备层全局实例、`device_init()` 与本层通用规则。

## 层规则

Device 把帧 / 字节 / 占空比换成物理量，或把物理量换成协议帧。
一个实例 = 一台设备（或一条总线上的设备组）。

| 可以放进来 | 不能放进来 |
| --- | --- |
| 协议帧解析与组装、单位换算、限幅 | 外设寄存器、HAL 调用、中断回调 |
| 设备级配置（地址、减速比、零位、通道） | `sys_state` / `EventState` |
| 在线判定节点、内部接收任务（确有需要时） | 业务决策 |
| 返回 `Status` 给调用方 | 吞掉错误、自己把整机置为失败 |

1. 设备不引用 `sys_state` / `EventState`：失败只 `return Status`，聚合由 `device_init()` 做。
2. 设备不拥有 BSP 资源，只持引用 / 指针。
3. 一条总线 / 一路串口只能有一个接收消费者：`DeviceEmmv5GroupT` / `DjiMotorGroupT` /
   `DeviceR9dsT` 都由外部任务调 `update()` 驱动，设备类不建任务。
4. 对象不可复制：`Online` 节点靠地址挂链表。

## 实例

模板实例化位置（全量）：

| 文件 | 行 |
| --- | --- |
| `dji_motor_group.cpp` | `template class DjiMotorGroupT<BspCanT<16, 16>>;` |
| `device_emmv5_group.cpp` | `template class DeviceEmmv5GroupT<BspUartT<uart_detail::DmaTransport, 128, 128>>;` |
| `device_r9ds.cpp` | `template class DeviceR9dsT<BspUartT<uart_detail::ItTransport, 128, 32>>;` |
| `device_r9ds.cpp` | `template class DeviceR9dsT<BspUartT<uart_detail::DmaTransport, 128, 128>>;` |
| servo / buzzer | 非模板，无实例化 |

| 实例 | 类 | 底层资源 | 接收方 | 在线判定 |
| --- | --- | --- | --- | --- |
| `can3_dji_group` | `DjiMotorGroupT<decltype(bsp_can3)>` | `bsp_can3` | 调用方任务 | `Online`，30 ms |
| `r9ds` | `DeviceR9dsT<decltype(bsp_uart_it5)>` | UART5（SBUS） | 调用方任务 | 自算 tick，100 ms |
| `emm_group` | `DeviceEmmv5GroupT<decltype(bsp_uart2)>` | USART2 RS485，115200 | 调用方任务（`corexy_io_task`） | 每台一个 `Online`，100 ms |
| `servo1~4` | `DeviceServo` | `bsp_pwm1~4`（50 Hz） | 无（开环） | 无 |
| `buzzer` | `DeviceBuzzer` | `bsp_pwm_buzzer` | 无 | 无 |

## 三种驱动骨架

**外部任务驱动**（DJI / R9DS）：

```cpp
CanRxMsg rx = {};
for (uint32_t i = 0; i < 8 && bsp_can3.receive(&rx, 0) == Status::OK; ++i)
  (void)can3_dji_group.update(rx);
(void)can3_dji_group.set_output(4, 5000);
(void)can3_dji_group.poll();
```

**外部驱动**（EmmV5）：`init()` 只建总线锁；由调用任务每 10 ms 调一次 `update()`
把串口字节喂给扫描器。**同一路串口再让第二个任务调 `update()` 就是两个消费者**。

**单写者**（Servo）：`update()` 是唯一 PWM 写入者；`set_angle()` / `off()` 可从任意任务调用。

## `device_init()` 顺序

```cpp
void device_init(void)
{
  configASSERT(can3_dji_group.init() == Status::OK);
  configASSERT(r9ds.init() == Status::OK);
  configASSERT(emm_motor1.init() == Status::OK);
  configASSERT(emm_group.init() == Status::OK);
  configASSERT(servo1.init() == Status::OK); // …servo4
}
```

必须在 `bsp_init()` 之后（设备持有 BSP 实例；`DeviceServo::init()` 还读 PWM 计数器时钟）。
`buzzer` 没有 `init()`。

## 新增一台设备

1. 建 `device_xxx.hpp/.cpp`，参数收进 `Config`（字段 < 4 个可不建）。
2. 模板类在 `.cpp` 顶部写**显式实例化**，覆盖 `device_cfg.cpp` 用到的类型；
   漏写会在链接期报 undefined reference。
3. `device_cfg.hpp` 加 `extern`（模板用 `decltype(bsp_xxx)`），`device_cfg.cpp` 加实例 +
   `device_init()` 一行断言。
4. 需要测试：`User/App/test/<模块>/` + `app_test.hpp` 开关 + `api_main.cpp` 的 `#if` 创建块。
5. 明确 `update()` 由谁调（外部任务或类内自建任务），并写清周期与优先级。

## 参数怎么填

| 设备 | 参数 | 位置 | 约束 |
| --- | --- | --- | --- |
| DJI 电机组 | `Member{model, ratio, offset}` | `device_cfg.cpp` 静态常量 | 实参顺序 = 位置 0~10；空位 `nullptr`；`ratio = 0` 用型号默认 |
| DJI 电机组 | 限幅 | `MODEL_TRAITS` | 16384 / 10000 / 16384；改型号同步 `MODEL_NUM` |
| EmmV5 单机 | 地址 | `Config{uart, addr}` | 1~255；0 是广播 |
| EmmV5 多机 | 地址表 | `Config{uart, {a, b, …}}` | 1~8 个、不重复、非 0 |
| EmmV5 | 波特率 | CubeMX + 电机菜单 `UartBaud` | 两边一致（UART7 = 921600，USART2 = 115200） |
| R9DS | 串口格式 | CubeMX `huart5` | 100000 / 9B / Even / 2；SBUS 模式；硬件反相 |
| R9DS | 在线阈值 | `ONLINE_TIMEOUT_MS` / `LOST_STREAK_TO_OFFLINE` | 100 ms / 5 帧 |
| 舵机 | 脉宽 / 行程 / 限速 / 反向 / 微调 | `device_cfg.cpp` | `pulse_max + trim` < 20 ms；限速 0 = 不限 |
| 蜂鸣器 | `Config{default_freq, freq_min, freq_max, short_ms, volume_pct}` | `DeviceBuzzer` 构造 | 50~20000 Hz；音量 = 占空比 |

换绑串口 / 总线：改模板实参类型 → 在设备 `.cpp` 补显式实例化 → 改 `device_cfg.cpp` 构造参数。
三处类型必须一致。

## 反模式

| 写法 | 后果 |
| --- | --- |
| 设备里写 `sys_state` / `EventState` | 单点故障拖停整机 |
| 业务任务再调 `DeviceEmmv5*::update()` | 抢字节，帧被切坏 |
| 两任务共用一条串口/总线接收 | 帧被瓜分 |
| MMCL 跨任务 `queue_*` | 子命令互相拼接 |
| 保存 `get_data()` / `rc()` 的引用跨周期用 | 引用指向内部缓冲 |
| 复制设备对象 | `Online` 链表错乱 |
| 假设舵机有位置反馈 | 开环器件无从判断 |
| 周期任务里调 `beep()` | 阻塞 `vTaskDelay`，周期被拉长 |
