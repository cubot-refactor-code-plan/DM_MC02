# device_r9ds

> 代码：`device_r9ds.hpp` / `device_r9ds.cpp`　实例：`r9ds`（UART5 / `bsp_uart_it5`）
> 在 `BspUartT` 之上把 SBUS 字节流解成通道值。

## 0. 如何快速完整的使用（不需要了解实际实现）

```cpp
// 1) 头文件：业务侧一般只需 device_cfg.hpp
#include "device_cfg.hpp"

// 2) 实例：device_cfg.cpp
DeviceR9dsT<decltype(bsp_uart_it5)> r9ds(bsp_uart_it5);

// 3) 声明：device_cfg.hpp
extern DeviceR9dsT<decltype(bsp_uart_it5)> r9ds;

// 4) 显式实例化：device_r9ds.cpp 顶部
template class DeviceR9dsT<BspUartT<uart_detail::ItTransport, 128, 32>>;

// 5) 初始化：device_cfg.cpp 的 device_init()（校验 100000/9B/Even/2）
configASSERT(r9ds.init() == Status::OK);

// 6) 串口：UART5 SBUS（仅收），接收机拨到 SBUS，硬件反相

// 7) 周期调用：调用方任务周期调 update()，消化串口里攒的字节
(void)r9ds.update();
```

**调用示例**

```cpp
#include "device_cfg.hpp"

r9ds.update();                                  // 把串口里攒的字节全部消化
if (r9ds.is_online() && !r9ds.failsafe())
{
  const auto &rc = r9ds.rc();
  const float fwd  = rc.left_vertical * 0.01f;  // -1.0 ~ +1.0
  const bool  a_up = (rc.switch_a == decltype(r9ds)::Switch::UP);

  const uint16_t *raw = r9ds.raw_channels();
  (void)fwd; (void)a_up; (void)raw;
}
```

## 协议

100000 bps、8 数据位 + 偶校验 + 2 停止位（每字节 12 位）、**反相 TTL**。
一帧 25 字节 = `0x0F` + 22 字节通道数据（16 × 11 bit，小端）+ flags + `0x00`。
`init()` 用 `check_format(100000, 9B, Even, 2)` 兜底校验，不一致返回 `BAD_ARG`；串口配置在 CubeMX。

`flags = frame[23]`：bit0/bit1 = 通道 17/18，bit2 = 本帧丢失，bit3 = 失控保护。

## `update()`：逐字节同步

找 `0x0F` → 拼满 25 字节 → 验帧尾 `0x00`（否则 `bad_footer_cnt++`）。
逐字节读而不是一次读 25 字节：`receive()` 有多少给多少，凑不满会把帧切歪。

通道提取：`bit = i × 11`，`byte = bit / 8 + 1`，`shift = bit % 8`，
`(frame[byte] | frame[byte+1] << 8 | frame[byte+2] << 16) >> shift & 0x7FF`。

## 两套量

- `raw_channels()`：协议原始值（上 200 / 中 1000 / 下 1800）。
- `rc()`：遥控器语义值（连续通道 −100 ~ +100，`RAW_PER_UNIT = 8`），**控制逻辑用这套**。

| 通道 | 物理部件 | 类型 | `Rc` 字段 |
| --- | --- | --- | --- |
| CH1 | 右摇杆 上下 | 连续 | `right_vertical` |
| CH2 | 右摇杆 左右 | 连续 | `right_horizontal` |
| CH3 | 左摇杆 上下 | 连续 | `left_vertical` |
| CH4 | 左摇杆 左右 | 连续 | `left_horizontal` |
| CH5 | C 三挡开关 | 三挡 | `switch_c` |
| CH6 | 正面左旋钮 | 连续 | `knob_left` |
| CH7 | 背面左推杆 | 连续 | `slider_left` |
| CH8 | 正面右旋钮 | 连续 | `knob_right` |
| CH9 | 正面 B 开关 | 两挡 | `switch_b` |
| CH10 | 正面 A 开关 | 两挡 | `switch_a` |

开关按「离 200 / 1000 / 1800 哪个最近」判挡；整数截断使 raw 993~1007 都算 0。

## 在线判定

三个条件同时成立：

1. 收到过帧，且最近一帧在 100 ms 内；
2. `!failsafe`；
3. `lost_streak < 5`。

只看“有没有帧”不够：遥控器关机时接收机照发帧，只把 flags 置位。实测 `lost` 比 `fs` 先置 1，
所以用连续 lost 当提前量（约 40 ms）。

## 诊断

`get_diag()`：`byte_cnt == 0` → 接线 / 波特率 / 反相；`byte_cnt` 在涨而 `frame_cnt` 不动 → 帧格式或反相。

## 约束

- 不是线程安全：`update()` 只允许一个任务调；`rc()` 返回实时引用。
- 在线判定不走 `Online` 节点，由本类按 tick 现算。
