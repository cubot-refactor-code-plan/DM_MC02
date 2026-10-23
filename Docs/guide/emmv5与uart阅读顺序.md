# EmmV5 与 UART 阅读顺序

## 先建立一张图

```text
业务任务 / App test
        │  enable()、set_position()、get_position() …
        ▼
DeviceEmmv5GroupT                         设备层：命令、接收任务、Online
        │                                  │
        │ emmv5::Frame                     │ BspUartT::send()/receive()
        ▼                                  ▼
emmv5_protocol.cpp                     BspUartT<Transport, RX, TX>
纯编码 / 扫描器                        流缓冲区 + 发送状态机 + 恢复
                                           │
                                           ▼
                                  HAL UART + DMA 或 IT
```

分工原则：

- `emmv5_protocol` 只负责“语义参数 → 字节帧”和“字节流 → 应答”，不碰串口。
- `BspUartT` 只负责“字节流收发 + 诊断/恢复”，不认识 EmmV5 协议。
- `DeviceEmmv5GroupT` 是单路串口多电机 + MMCL；只挂一台时地址表写单元素即可，
  不再保留单独的单电机类。
- `bsp_cfg.cpp` 决定实例用 DMA 还是 IT，以及缓冲区大小；`device_cfg.cpp` 把设备绑定到具体串口。

## 推荐阅读顺序

### 第一遍：只弄清“谁调用谁”

1. `User/Bsp/bsp_uart.hpp` 顶部 `@details`、`Config`、`send()`、`receive()`、`RxDiag/TxDiag`。
2. `User/Protocol/emmv5_protocol.hpp` 的 `Frame`、三个 Scanner、`build_multi_frame`。
3. `User/Device/device_emmv5_group.hpp` 顶部说明、`Config`、`init()`、常用命令、`get_*()`。
4. 同文件的 MMCL 三件套 `multi_clear()` / `queue_*()` / `multi_send()` 与总线锁。

这一遍不要钻 `bsp_uart.cpp` 的 ISR 和恢复逻辑，先知道数据从哪进、从哪出。

### 第二遍：看收发路径

按这个顺序读 `bsp_uart.cpp`：

1. `init()`：创建流缓冲区和锁，启动接收。
2. `send()` / `receive()`：任务侧只和流缓冲区打交道。
3. `on_idle_isr()` / `start_transmission_from_isr()`：中断只搬字节、续发。
4. `tx_recover()` / `rx_recover()`：最后看 10 ms 巡检如何补救。
5. HAL 回调分发：按 `huart` 指针找到全局实例。

读 ISR 时始终记住：**中断不解析协议，只把字节块推进流缓冲区**。

### 第三遍：看设备层如何切帧

1. `device_emmv5_group.cpp` 的 `feed_rx()`：同一块数据交给组内每台电机的扫描器，
   由扫描器里的“地址 + 功能码”决定归属。
3. `get_position()` / `get_status()` / `get_in_position()`：控制任务如何取走结果。

### 第四遍：按目标选读

| 你的目标             | 先看                                                                           |
| -------------------- | ------------------------------------------------------------------------------ |
| 只想用电机           | `device_emmv5_group.md` 的 §0 快速使用 + MMCL 示例                          |
| 想加一条协议命令     | `emmv5_protocol.hpp` 对应声明 → `.cpp` 里同组命令的帧布局注释             |
| 想换串口 / 换 DMA/IT | `bsp_cfg.hpp/.cpp`、`bsp_uart.cpp` 的显式实例化和回调分发                  |
| 想排查丢包           | `get_rx_diag()/get_tx_diag()` → `on_error_isr()` → `tx_recover()/rx_recover()` |
| 想跑极限测试         | `User/App/test/uart/uart_stress_test.cpp`                                    |

## 必须记住的约束

1. `Config::baudrate` 只用于估算 `tx_recover()` 的卡死阈值；**真正波特率在 CubeMX 的
   `huartX.Init.BaudRate`**，两者必须一致。
2. `send()` 返回 `OK` 只表示“整帧进了发送流缓冲区”，不表示已经发到线上。
3. `receive()` 是**字节流**，不保证一次读到完整帧；切帧由设备层/协议层负责。
4. `DeviceEmmv5GroupT` **不建任务**，由调用任务周期调 `update()`；
   一条总线只能有一个任务调它，否则会变成两个消费者。
5. `DeviceEmmv5GroupT` 的 `multi_clear()` / `queue_*()` / `multi_send()` 要么同一任务串行，
   要么用 `lock_bus()` / `unlock_bus()` 把整段包起来。
6. `get_position()` / `get_status()` 读后清有效标志；`get_in_position()` 读后清累计到位位图。
7. `DeviceEmmv5GroupT` 的 MMCL 上限按 128 字节发送缓冲区算（`UART_BUFFER_SIZE = 128`）；
   换更小的串口发送缓冲区时要同步改这个常量。
8. Online 阈值是 100 ms，应用层查询周期应小于它，否则“没被问”的电机也会显示离线。
9. UART8 当前是 IT + FIFO；FIFO 模式下 HAL 用 `CR3.RXFTIE`，不是 `CR1.RXNEIE`。
   健康检查必须兼容两种，否则 `sys_task` 会每 10 ms 误重建 RX。
10. DMA 实例的收发缓冲区需要放 `.dma_buffer`（RAM_D1）；IT 实例不强制，但本工程
    UART8 测试实例仍保留该段。

## 本次 review 的实际改动

- `BspUartT::tx_recover()`：原来只看发送流缓冲区是否有数据，会漏掉“数据已经在途、
  流缓冲区已空”的卡死；现在同时检查 `_tx_inflight_len` 和流缓冲区。
- `BspUartT` 的 FIFO 健康检查：兼容 `CR3.RXFTIE` 与 `CR1.RXNEIE` 两条 HAL 路径。
- `DeviceEmmv5GroupT::in_position` / `get_in_position()`：到位标志的写、读、清零统一放进
  短临界区，避免接收任务和控制任务竞争。
- UART / EmmV5 / Protocol 三个头文件补充了“建议阅读顺序”，UART 私有区按
  “成员变量 / 内部实现”重新分区。
