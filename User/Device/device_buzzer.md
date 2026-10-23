# device_buzzer

> 代码：`device_buzzer.hpp` / `device_buzzer.cpp`　实例：`buzzer`（`bsp_pwm_buzzer`）

在 `BspPwm` 之上提供音调 / 音量 / 鸣叫时长语义。不拥有 PWM、不创建 RTOS 资源、没有 `init()`。

## 0. 如何快速完整的使用（不需要了解实际实现）

```cpp
// 1) 头文件：业务侧一般只需 device_cfg.hpp
#include "device_cfg.hpp"

// 2) 实例：device_cfg.cpp —— 绑定一个 PWM 通道（可传第二参数 Config）
DeviceBuzzer buzzer(bsp_pwm_buzzer);

// 3) 声明：device_cfg.hpp
extern DeviceBuzzer buzzer;

// 4) 初始化：本类没有 init()，只需保证 bsp_pwm_buzzer 已初始化
```

无模板实例化、无中断。

**调用示例**

```cpp
#include "device_cfg.hpp"

buzzer.tone(3000U, 50.0f);   // 3 kHz，50% 占空比（非阻塞）
buzzer.beep(3000U, 40U);     // 响 40 ms 后自动关（阻塞）
buzzer.off();
```

- 音调 = PWM 频率（`set_freq`），音量 = 占空比。
- `tone()` 非阻塞；`off()` 占空比 0；`beep()` = `tone()` + `vTaskDelay()` + `off()`，**阻塞**。

约束：`beep()` 不能与「严格等周期轮询」同任务（周期会多出响铃时长）。
