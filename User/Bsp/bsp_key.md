# bsp_key

> 代码：`bsp_key.hpp` / `bsp_key.cpp`　实例：`key_user`（`bsp_cfg.cpp`）

## 0. 如何快速完整的使用（不需要了解实际实现）

```cpp
// 1) 头文件：业务侧一般只需 bsp_cfg.hpp
#include "bsp_cfg.hpp"

// 2) 实例：bsp_cfg.cpp
BspKey key_user;

// 3) 声明：bsp_cfg.hpp
extern BspKey key_user;

// 4) 配置 + 初始化：bsp_cfg.cpp 的 bsp_init()
configASSERT(key_user.init({KEY_GPIO_Port, KEY_Pin, true, 1U, 5U}) == Status::OK);

// 5) 周期轮询：App/task/key_task.cpp 每 200 ms 一次
switch (key_user.update()) { /* PRESS / SHORT / LONG */ }

// 6) CubeMX：PA15 配输入 + 上拉
```

**调用示例**

```cpp
for (;;)
{
  switch (key_user.update())
  {
    case BspKey::Event::PRESS: /* ... */ break;
    case BspKey::Event::SHORT: /* ... */ break;
    case BspKey::Event::LONG:  /* ... */ break;
    default: break;
  }
  vTaskDelay(pdMS_TO_TICKS(200));
}
```

纯软件轮询消抖，无中断、无 RTOS 依赖。

## 状态机

```
        电平跳变                     稳定按住 ≥ long_press_cnt × T
 NONE ──────────► 计数 cnt ──(cnt ≥ debounce_cnt)──► PRESS ──────────────► LONG（只触发一次）
                                                       │
                                                       └─(阈值内松手)─► SHORT
```

- `消抖时间 = debounce_cnt × 轮询周期`，`长按时间 = long_press_cnt × 轮询周期`。
  本工程 `{true, 1, 5}` + 200 ms → 200 ms 消抖 / 1 s 长按。
- `init()` 以当前电平作 `_last_stable`：上电已按住不产生 `PRESS`。
- 确认按下时把已累计消抖次数作为 `_hold_cnt` 初值，长按从**电平跳变那一刻**起算。
- 长按后松手不产生 `SHORT`（`_long_fired`），避免一次长按被当成两次操作。
- `update()` 必须同一任务严格等周期；非线程安全。
- 一次「按下→松手」只得 `PRESS + SHORT` 或 `PRESS + LONG`。

配置：`key_user.init({KEY_GPIO_Port, KEY_Pin, true, 1U, 5U})`（`src/bsp_cfg.cpp`）；
引脚方向与上下拉由 `MX_GPIO_Init()` 配置。
