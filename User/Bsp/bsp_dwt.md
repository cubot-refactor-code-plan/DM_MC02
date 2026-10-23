# bsp_dwt

> 代码：`bsp_dwt.hpp` / `bsp_dwt.cpp`　实例：`bsp_dwt`（`bsp_cfg.cpp`）

## 0. 如何快速完整的使用（不需要了解实际实现）

```cpp
// 1) 头文件：业务侧一般只需 bsp_cfg.hpp
#include "bsp_cfg.hpp"

// 2) 实例：bsp_cfg.cpp
BspDwt bsp_dwt;

// 3) 声明：bsp_cfg.hpp
extern BspDwt bsp_dwt;

// 4) 初始化：bsp_cfg.cpp 的 bsp_init() 第一句，后续计时都依赖它
configASSERT(bsp_dwt.init() == Status::OK);

// 5) 心跳：sys_task.cpp 每 200 拍（约 2 s）调一次，兼作回绕心跳
(void)bsp_dwt.get_time_s();
```

无模板实例化、无中断。

**调用示例**

```cpp
#include "bsp_cfg.hpp"

uint32_t mark = 0U;
(void)bsp_dwt.get_delta_s(&mark);                  // 首次只取基准，结果丢弃
/* 被测代码 */
const double dt_us = bsp_dwt.get_delta_s(&mark) * 1e6;

const double now_ms = bsp_dwt.get_time_ms();       // 绝对时间（需 < 7.809 s 心跳）
bsp_dwt.delay_us(500.0);                           // 忙等 500 µs
```

内核 CYCCNT 计时：不占外设、不依赖中断，临界区内也能计时；1 计数 ≈ 1.818 ns（550 MHz）。

## 初始化

`DEMCR.TRCENA`（脱机运行必须自己置位）→ `DWT->LAR = 0xC5ACCE55` → `CYCCNT = 0` →
使能 `CYCCNTENA` → 等约 8 周期 → 再清零。

## 约束

- 基准 = `SYSCLK / D1CPRE`。32 位计数器 **7.809 s 回绕一次**。
- `get_delta_s(&last)`：无符号差值，间隔 < 7.809 s 恒正；无状态，可在 ISR 调用。
- `get_time_*`：唯一状态更新点是 `update_timeline()`，**先调用再读 `_base_s`**。
  写成 `_base_s + update_timeline()/...` 会因求值顺序在回绕时少算一圈（倒退 7.809 s）。
- 任意两次绝对时间调用间隔必须 < 7.809 s；`sys_task` 每 200 拍（约 2 s）调一次 `get_time_s()`
  打印 `[sys] up=...s`，同时充当回绕心跳。
- `delay_*` 纯忙等，临界区可用；毫秒级以上用 `vTaskDelay()`。
- CPU 进 sleep/stop 时停止计数。
