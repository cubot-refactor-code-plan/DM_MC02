# bsp_gpio

> 代码：`bsp_gpio.hpp`（仅头文件）　实例：`bsp_cfg.cpp`

## 0. 如何快速完整的使用（不需要了解实际实现）

```cpp
// 1) 头文件：业务侧一般只需 bsp_cfg.hpp
#include "bsp_cfg.hpp"

// 2) 实例：bsp_cfg.cpp —— 构造时绑定 {port, pin}，只做赋值
inline BspGpio xxx({ .port = GPIOx, .pin = GPIO_PIN_y });   // bsp_cfg.hpp

// 3) 声明：bsp_cfg.hpp
extern BspGpio xxx;

// 4) 初始化：不需要，方向与上下拉由 CubeMX 的 MX_GPIO_Init() 配置
```

无模板实例化、无中断、无 `init()`。

**调用示例**

```cpp
#include "bsp_cfg.hpp"

power_24v_1.set();                        // 高电平
power_24v_1.reset();                      // 低电平
btb_pa5.toggle();                         // 翻转
const bool level = gyro_acc_cs.get_level();
```

构造绑定 `{port, pin}`；方向与上下拉由 `MX_GPIO_Init()` 配置，本类不做初始化。

接口：`set()` / `reset()` / `toggle()` / `set_level(bool)` / `get_level()`。

现有实例与引脚见 `bsp_cfg.cpp` 与 `Docs/hardware/引脚分配与冲突.md`。
