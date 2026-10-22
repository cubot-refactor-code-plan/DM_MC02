# bsp_pwm

> 代码：`bsp_pwm.hpp` / `bsp_pwm.cpp`　实例：`bsp_cfg.cpp`

## 0. 如何快速完整的使用（不需要了解实际实现）

| 步骤 | 文件 | 写什么 |
| --- | --- | --- |
| 1. 头文件 | 使用处 | `#include "bsp_pwm.hpp"`；业务侧一般只需 `#include "bsp_cfg.hpp"` |
| 2. 全局实例 | `bsp_cfg.cpp` | `BspPwm bsp_pwmX({&htimX, TIM_CHANNEL_y, 275000000UL, PSC, ARR, 0.0f});` |
| 3. 声明 | `bsp_cfg.hpp` | `extern BspPwm bsp_pwmX;` |
| 4. 初始化 | `bsp_cfg.cpp` 的 `bsp_init()` | `configASSERT(bsp_pwmX.init() == Status::OK);` |
| 5. CubeMX | `DM_MC02.ioc` | TIMx 通道设为 PWM Generation CHy，引脚 AF |

无模板实例化、无中断、无回调。

**调用示例**

```cpp
#include "bsp_cfg.hpp"

bsp_pwm1.set_duty(7.5f);          // 占空比 7.5%
bsp_pwm1.set_pulse_us(1500.0f);   // 舵机中位（本板 1 计数 = 1 µs）
bsp_pwm2.set_freq(3000U);         // 改频率会同时影响 TIM1 全部通道
bsp_pwm1.off();                   // CCR = 0，引脚保持低电平
```

一个实例 = 一个定时器的一个 PWM 通道，只碰 CCR / ARR，无中断、无 RTOS。

## 换算

- `counter_clk = timer_clk_hz / (prescaler + 1)`。本板 275 MHz，舵机 `PSC=274` → 1 MHz，
  `ARR=19999` → 20000 计数 = 20 ms = 50 Hz；1 计数 = 1 µs。
- `ccr = round(duty% × (ARR+1) / 100)`，限幅 `[0, ARR+1]`。
- 脉宽：`period_us = (ARR+1) × 1e6 / counter_clk`，`duty = width_us × 100 / period_us`。
- 改频率：`ARR = counter_clk / freq − 1`，PSC 不动；上限按 `IS_TIM_32B_COUNTER_INSTANCE()`
  取 32 位 / 16 位；随后重算 CCR 保持占空比。

## 约束

- `init()` 幂等（`_initialized`）：`HAL_TIM_PWM_Start()` 置 BUSY 且不复位，重复调用返回 `HAL_ERROR`；
  启动前先写 CCR=0，保证上电 0%。
- 通道合法性用 `IS_TIM_CCX_INSTANCE()`。**不能写 `channel == 0`**：`TIM_CHANNEL_1 == 0`，
  会误杀 CH1（本工程 pwm2/pwm4 上电断言过）。
- 同定时器共用计数器与 ARR：`bsp_pwm1`/`bsp_pwm2` 必须同频，`bsp_pwm3`/`bsp_pwm4` 同理。
- 含浮点，只在任务上下文调用。

## 新增一路 PWM

先定频率 → 选 PSC → 算 ARR → 填 Config。

1. `timer_clk_hz` 填定时器内核时钟 **275 MHz**（APB 137.5 × 2，因为 APB 预分频 ≠ 1），不是 137.5 MHz。
2. `counter_clk = 275 MHz / (PSC + 1)`；PSC 越大计数越慢、可用 ARR 越大。
3. `ARR = round(counter_clk / f) − 1`；16 位定时器 `ARR ≤ 65535`，TIM2/TIM5 是 32 位。

| 用途 | 频率 | PSC | counter_clk | ARR | 分辨率 |
| --- | --- | --- | --- | --- | --- |
| 舵机 | 50 Hz | 274 | 1 MHz | 19999 | 1 µs/计数 |
| 蜂鸣器 | `tone()` 动态改 | 23 | 11.4583 MHz | 1999（初始） | 87 ns/计数 |
| 陀螺仪 | 按器件手册 | 23 | 11.4583 MHz | 9999 | 87 ns/计数 |

4. `Config{&htimX, TIM_CHANNEL_y, 275000000UL, PSC, ARR, 上电占空比}`（参数顺序 = 字段顺序）；
   `init()` 先 CCR=0 再启动，随后按该占空比输出（默认 0）。
5. 同定时器共用 PSC 与 ARR：要不同频率必须换定时器；同频多通道只差 CCR。

**步骤**：CubeMX 把 TIMx CHy 设为 PWM Generation CHy（引脚自动 AF，Clock Source = Internal）；
`bsp_cfg.hpp` 加 `extern`；`bsp_cfg.cpp` 加实例 + `bsp_init()` 断言。无中断、无回调、无模板实例化。

**确认**：`get_freq_hz()` / `get_duty_pct()` 是内部计算值；真实波形用示波器量。上电应 0% 占空比。
