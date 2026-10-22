# online_check

> 代码：`online_check.hpp` / `online_check.cpp`

`Online`：可嵌入任意设备对象的在线检查节点，内部是一条静态单向链表。

## 结构

- 每个对象在**构造时**挂到链表（`_head` / `_tail`），析构时摘除。
- 对象**不可复制 / 移动**，必须是全局或设备对象的成员。
- 构造时 `_last_refresh_tick = now − timeout`，初始 `TIMEOUT`；收到第一帧有效数据才算在线。

## 三件套

```cpp
class Foo
{
  Online _online{100};                       // 1) 成员：阈值 100 ms（默认 30）
};

// 2) 收到有效数据时续命
_online.refresh_task();                      // 任务上下文
_online.refresh_isr();                       // 中断上下文

// 3) 周期推进（只能一个任务做，本工程是 sys_task，10 ms）
Online::update();
```

- `refresh_*` 只写 `_last_refresh_tick` 与 `Status::OK`；任务侧 `taskENTER_CRITICAL()`，
  中断侧 `taskENTER_CRITICAL_FROM_ISR()`。
- `update()` 遍历链表：`now − last_refresh ≥ timeout_gap` → `TIMEOUT`，否则 `OK`；tick 差值用无符号减法。
- `is_online()` 返回最近一次 `update()` 算出的 `Status`。

## 时间语义

- `timeout_gap` 单位是**毫秒**，与 `update()` 调用周期无关。
- 调用周期只决定判定延迟上限（10 ms 调用 → 最坏晚 10 ms 判出离线）。
- 调用周期应不大于最小的 `timeout_gap`。
- 依赖 `configTICK_RATE_HZ == 1000`（tick = 1 ms）。

## 调度器启动前构造

链表操作包在 `ScopedTaskCritical` 里：调度器未启动（`taskSCHEDULER_NOT_STARTED`）时不进临界区。
所以 `device_cfg.cpp` 的全局设备对象可以在 `main()` 之前静态构造。

## 与 `EventState` 的分工

`Online` 只回答“最近有没有收到数据”，不登记错误、不参与阶段判断。
需要系统级错误时由持有者把 `Status` 往上抛，不要让 `Online` 自己写 `EventState`。
