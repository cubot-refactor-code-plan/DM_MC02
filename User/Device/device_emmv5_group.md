尖括号

# device_emmv5_group

> 代码：`device_emmv5_group.hpp` / `device_emmv5_group.cpp`
> 单串口多电机 + MMCL 多机命令。本工程实际绑定的串口与地址见 [`device_cfg.md`](device_cfg.md)。

## 0. 如何快速完整的使用（不需要了解实际实现）

```cpp
// 1) 头文件：业务侧一般只需 device_cfg.hpp
#include "device_cfg.hpp"

// 2) 实例：device_cfg.cpp —— 一条总线 + 地址表；只挂一台就写单元素
DeviceEmmv5GroupT<decltype(bsp_uartX)> group({bsp_uartX, {addr1, addr2}});

// 3) 声明：device_cfg.hpp
extern DeviceEmmv5GroupT<decltype(bsp_uartX)> group;

// 4) 显式实例化：device_emmv5_group.cpp 顶部（串口类型变了才需要改）尖括号中为上述参数类型
template class DeviceEmmv5GroupT<BspUartT<uart_detail::DmaTransport, 128, 128>>;

// 5) 初始化：device_cfg.cpp 的 device_init() —— 只建总线锁，不建任务
configASSERT(group.init() == Status::OK);

// 6) 串口：CubeMX + bsp_cfg.cpp —— 半双工；RS485 要有硬件 DE，波特率与驱动器一致

// 7) 周期驱动：调用方任务每 10 ms 一次，它是这条总线唯一的消费者
(void)group.update();
```

**调用示例**

```cpp
#include "device_cfg.hpp"

emm_group.multi_clear();
emm_group.queue_position(1, emmv5::Direction::CW, 500, 50, 3200, emmv5::PositionMode::Relative);
emm_group.queue_position(2, emmv5::Direction::CCW, 500, 50, 3200, emmv5::PositionMode::Relative);
(void)emm_group.multi_send();

uint32_t arrived = emm_group.get_in_position();   // 读后清零
int32_t  pos     = 0;
if (emm_group.get_position(1, pos)) { /* counts */ }
```

## 地址表

`Config{Uart&, std::initializer_list<uint8_t> addrs}`，最多 8 台。`init()` 校验：地址数 1~8、
地址非 0、不重复。每个电机有独立的三个扫描器 + 一个 `Online` 节点（100 ms）。

## 按地址分发

`update()` 读到的每块字节交给组内**每一台**电机的扫描器各喂一遍，归属由扫描器内部的
「地址 + 功能码」决定；设备层不写协议解析、不拆字节流。

## MMCL

```
总帧 = target(1) + 0xAA(1) + 总长度(2, 大端，含头尾 5 字节) + 子命令串(N) + 0x6B(1)
子命令 = 各自地址 + 功能码 + 数据 + 0x6B
```

- `_multi[128]` 前 4 字节给总帧头 → 子命令上限 `128 − 5 = 123`（`MULTI_PAYLOAD_MAX`）。
- `_queue_for()` 先判断整帧放得下再拷贝，放不下返回 `FULL`，不留半条子命令。
- `multi_send()` 就地把总帧封进 `_multi`（`payload` 与 `out` 重叠，必须 `memmove`）；
  发送后无论成败都清空缓冲（超时可能有半帧已进流缓冲，重发会出现“半帧 + 完整帧”）。
- `multi_clear() / queue_*() / multi_send()` 必须同一任务串行。

## 结果接口

| 接口                        | 语义                                               |
| --------------------------- | -------------------------------------------------- |
| `get_in_position()`       | 累计到位位图（bit i = 下标 i），**读后清零** |
| `in_position(index)`      | 只查询，不清零                                     |
| `get_position(addr, out)` | 取走 0x36 最近值，读后清标志                       |
| `get_status(addr, out)`   | 取走 0x3A 状态字节，读后清标志                     |
| `is_online(addr)`         | 100 ms 内有过有效应答                              |

写入 / 读取 / 清零都用同一个短临界区。

## 约束

- **控制命令没有应答**（位置 / 速度 / 使能 / 停止 / 清零都只回“无”），只有 `read` 类
  （`request_position` / `request_status`）才有应答；因此判断“能否发下一条”只能靠固定延时，
  或者用后续读数变化间接确认（`zero_position` 就是靠读数复核的）。
- `update()` 由**外部任务**周期调用（建议 10 ms），且整个组只能有一个任务调：
  接收缓冲区只有一个消费者。本类不建任务、不隐藏优先级。
- 业务任务可调 `get_*()` 与发送接口（发送一律经实例自己的递归总线锁）；MMCL 多条子命令拼一帧时要自己用 `lock_bus()` … `unlock_bus()` 把整段包起来。
- 对象必须是全局/静态生命周期。
