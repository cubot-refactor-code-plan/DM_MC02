# device_servo

> 代码：`device_servo.hpp` / `device_servo.cpp`　实例：`servo1~4`（`device_cfg.cpp`）
> 在 `BspPwm` 之上提供角度语义、限速与梯形规划。

## 0. 如何快速完整的使用（不需要了解实际实现）

| 步骤 | 文件 | 写什么 |
| --- | --- | --- |
| 1. 头文件 | 使用处 | `#include "device_servo.hpp"`；业务侧一般只需 `#include "device_cfg.hpp"` |
| 2. 全局实例 | `device_cfg.cpp` | `DeviceServo servoX({&bsp_pwmX, pulse_min, pulse_max, range_deg, max_speed, max_accel});` |
| 3. 声明 | `device_cfg.hpp` | `extern DeviceServo servoX;` |
| 4. 初始化 | `device_cfg.cpp` 的 `device_init()` | `configASSERT(servoX.init() == Status::OK);`（只校验配置，不动硬件） |
| 5. 周期调用 | 自己的任务 | 固定一个任务周期调 `servoX.update()`（当前在 `App/test/servo/servo_test.cpp`） |
| 6. 前置 | `bsp_cfg.cpp` / `bsp_init()` | 对应的 `bsp_pwmX` 已实例化并初始化 |

无模板实例化、无中断。

```cpp
// device_cfg.cpp
DeviceServo servo1({&bsp_pwm1, 600.0f, 2400.0f, 270.0f, 0.0f, 0.0f});
// device_cfg.hpp
extern DeviceServo servo1;
// device_init()
configASSERT(servo1.init() == Status::OK);
```

**调用示例**

```cpp
#include "device_cfg.hpp"

// 任务 A，周期调用
servo1.update();

// 任意任务
servo1.set_angle(120.0f);          // 只记目标
servo1.off();                      // 请求关断
```

## 换算

```
t      = angle / range_deg
t      = reversed ? 1 − t : t
pulse  = pulse_min_us + t × (pulse_max_us − pulse_min_us) + trim_us
```

算例：`range_deg = 180`、`500 ~ 2500 µs` → 90° → `t = 0.5` → 1500 µs。

## `update()` 四步

1. 没有「要输出」请求 → `pwm->off()`（只在还亮着时写一次寄存器），返回。
2. 首次生效那一拍只登记、不积分（避免长时间没更新后第一拍跳变）。
3. `max_speed_dps == 0` → 直接落到目标并写脉宽。
4. 否则梯形规划：
   - `dt = tick 差值 / configTICK_RATE_HZ`，上限 `MAX_DT_S = 0.1 s`。
   - `v_cmd = ±v_max`（由剩余角度符号决定）。
   - `|Δv| ≤ a·dt`；`|v| ≤ sqrt(2·a·|rem|)`（保证恰好停在目标点）。
   - `|v·dt| ≥ |rem|` 时直接落位并把速度清零。

## 线程模型

| 接口 | 谁调 | 原因 |
| --- | --- | --- |
| `update()` | 固定一个任务、周期调用 | 唯一 PWM 写入者；不调就没有输出，`off()` 也不生效 |
| `set_pulse_us()` | 须与 `update()` 同任务 | 要立即接管输出 |
| `set_angle()` | 任意任务 | 只写 `_target_deg` + `_enabled_req` 两个 32 位变量 |
| `off()` | 任意任务 | 只清 `_enabled_req`，下一拍关断 |
| `get_*()` | 任意任务 | 32 位对齐读，可能晚一拍 |

前提：Cortex-M7 单核、32 位对齐读写单条指令、D-Cache 未开。

## 约束

- 开环器件：写出去的是“希望它到哪”，不注册 `Online`。
- `init()` 只校验配置、不写脉宽（上电不输出）。
- 脉宽端点、限速、加速度都是待实机标定的占位值。
- 每条 PWM 通道只由一个实例驱动；同定时器通道共用 ARR。
